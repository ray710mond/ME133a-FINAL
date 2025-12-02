'''pirouetteandwave.py

   This is a demo for moving/placing an ungrounded robot and moving joints.

   Multi-robot usage:
     - Run this node in a namespace (e.g. atlas2).
     - robot_name parameter must match the frame_prefix used in
       robot_state_publisher (e.g. "atlas2" -> frames "atlas2/pelvis", ...).
     - Publishes <ns>/joint_states.
     - Broadcasts world -> <robot_name>/pelvis.
     - Publishes saber tip position on /<robot_name>_pos.
     - Subscribes to /<other_robot_name>_pos for "fight" behavior.
'''

import rclpy
import numpy as np

from math                        import pi, sin, cos
from asyncio                    import Future
from rclpy.node                 import Node
from tf2_ros                    import TransformBroadcaster

from geometry_msgs.msg          import TransformStamped, Twist
from sensor_msgs.msg            import JointState
from std_msgs.msg               import Header

from utils.TransformHelpers     import *
from visualization_msgs.msg     import Marker


#
#   Atlas Joint Names
#
atlasnames = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
              'l_leg_kny',
              'l_leg_akx', 'l_leg_aky',

              'r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz',
              'r_leg_kny',
              'r_leg_akx', 'r_leg_aky',

              'back_bkx', 'back_bky', 'back_bkz',
              'neck_ry',

              'l_arm_elx', 'l_arm_ely',
              'l_arm_shx', 'l_arm_shz',
              'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',

              'r_arm_elx', 'r_arm_ely',
              'r_arm_shx', 'r_arm_shz',
              'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2']


class TrajectoryNode(Node):
    # Initialization.
    def __init__(self, name, future):
        super().__init__(name)
        self.future = future

        ##############################################################
        # PARAMETERS TO SELECT WHICH ROBOT WE ARE CONTROLLING

        self.robot_name = self.declare_parameter(
            'robot_name', 'atlas1'
        ).get_parameter_value().string_value

        self.other_robot_name = self.declare_parameter(
            'other_robot_name', 'atlas2'
        ).get_parameter_value().string_value

        # pelvis frame must match frame_prefix + "pelvis"
        self.pelvis_frame = f'{self.robot_name}/pelvis'
        self.world_frame  = 'world'

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        self.jointnames = atlasnames
        
        self.qc   = np.zeros(len(self.jointnames))
        self.qdot = np.zeros(len(self.jointnames))

        self.arm_joint_names = ['r_arm_shz', 'r_arm_shx',
                                'r_arm_ely', 'r_arm_elx',
                                'r_arm_wry', 'r_arm_wrx']
        self.arm_idx = [self.jointnames.index(n) for n in self.arm_joint_names]

        self.lam     = 1.0
        self.dq_num  = 1e-4

        # target in WORLD frame (saber tip target)
        self.p_target            = np.array([1.0, 0.0, 0.5])
        self.has_external_target = False

        ##############################################################
        # Setup the logistics of the node:
        # Add publishers and a TF broadcaster.

        # IMPORTANT: no leading '/' so this becomes <namespace>/joint_states
        self.pubjoint = self.create_publisher(JointState, 'joint_states', 10)
        self.tfbroad  = TransformBroadcaster(self)

        self.get_logger().info("Waiting for a joint_states subscriber...")
        while not self.count_subscribers('joint_states'):
            pass

        self.dt    = 0.01
        self.t     = -self.dt
        self.now   = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))
                               
        self.pubmarker = self.create_publisher(Marker, 'marker', 10)

        # Robot-specific position topic (global, for cross-robot comms)
        my_pos_topic     = f'/{self.robot_name}_pos'
        other_pos_topic  = f'/{self.other_robot_name}_pos'

        self.get_logger().info(f"Publishing my saber tip position on {my_pos_topic}")
        self.get_logger().info(f"Subscribing to other robot position on {other_pos_topic}")

        self.pubpos = self.create_publisher(Twist, my_pos_topic, 10)
        self.subpos = self.create_subscription(
            Twist, other_pos_topic, self.newTarget, 10
        )
    
    def newTarget(self, other_pos_msg):
        # Use the other robot's saber tip as new target (WORLD frame)
        self.p_target = np.array([
            other_pos_msg.linear.x,
            other_pos_msg.linear.y,
            other_pos_msg.linear.z
        ])
        self.has_external_target = True

    def _T(self, R, p):  # t matrix function
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3]   = p
        return T

    def fk_right_hand(self, q):  # forward kinematics to SABER TIP (in pelvis frame)
        q_shz = q[0]
        q_shx = q[1]
        q_ely = q[2]
        q_elx = q[3]
        q_wry = q[4]
        q_wrx = q[5]

        # pelvis frame origin
        T = self._T(Reye(), np.array([-0.0125, 0.0, 0.212]))

        # 1) r_arm_shz
        T = T @ self._T(Rotz(q_shz), np.array([0.1406, -0.2256, 0.4776]))
        # 2) r_arm_shx
        T = T @ self._T(Rotx(q_shx), np.array([0.0, -0.11, -0.245]))
        # 3) r_arm_ely
        T = T @ self._T(Roty(q_ely), np.array([0.0, -0.187, -0.016]))
        # 4) r_arm_elx
        T = T @ self._T(Rotx(q_elx), np.array([0.0, -0.119, 0.0092]))
        # 5) r_arm_wry
        T = T @ self._T(Roty(q_wry), np.array([0.0, -0.29955, -0.00921]))
        # 6) r_arm_wrx
        T = T @ self._T(Rotx(q_wrx), np.array([0.0, 0.0, 0.0]))

        # 7) fixed wrist2 + hand link
        T = T @ self._T(Reye(), np.array([0.0, -0.06, 0.0]))

        # 8) hand to saber tip offset
        saber_offset = np.array([0.0, 0.0, 0.41 * 2])
        T = T @ self._T(Reye(), saber_offset)

        # saber tip in pelvis frame
        return T[0:3, 3]

    def jacobian_right_hand(self, q):
        J  = np.zeros((3, 6))
        dq = self.dq_num

        for i in range(6):
            q_plus  = q.copy()
            q_minus = q.copy()
            q_plus[i]  += dq
            q_minus[i] -= dq

            p_plus  = self.fk_right_hand(q_plus)
            p_minus = self.fk_right_hand(q_minus)

            J[:, i] = (p_plus - p_minus) / (2.0 * dq)

        return J

    # Shutdown
    def shutdown(self):
        self.timer.destroy()
        self.destroy_node()

    # Update function
    def update(self):
        self.t   = self.t + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        # pelvis pose in world (this is atlas2: 1.5 m away, rotated 180 deg)
        ppelvis = pxyz(0, 0.0, 1.0)
        Rpelvis = Reye()

        # right-arm joint vector
        q_arm = np.array([self.qc[i] for i in self.arm_idx])

        # circular default target in world frame (used until we see the other robot)
        Cx = 1.0
        Cy = 0.0
        Cz = 0.5
        R  = 0.25
        omega = 0.1
        theta = omega * self.t

        circle_target = np.array([
            Cx,
            Cy + R * np.cos(theta),
            Cz + R * np.sin(theta)
        ])

        if not self.has_external_target:
            self.p_target = circle_target

        # current saber tip in pelvis frame
        p_tip_pelvis = self.fk_right_hand(q_arm)

        # convert to WORLD frame: p_w = Rpelvis * p_pelvis + ppelvis
        p_tip_world = Rpelvis @ p_tip_pelvis + ppelvis

        # task-space error in world frame
        e     = self.p_target - p_tip_world
        xdot  = self.lam * e 

        # numerical jacobian in pelvis frame (OK; rotation doesn't change norms)
        J = self.jacobian_right_hand(q_arm)  # 3x6

        # under-determined: 3 eqns, 6 unknowns, least squares
        qdot_arm, *_ = np.linalg.lstsq(J, xdot, rcond=None)

        # integrate arm joints
        q_arm_new = q_arm + self.dt * qdot_arm

        # write arm joints back into full Atlas joint vector
        qc    = self.qc.copy()
        qcdot = np.zeros_like(self.qc)

        for k, idx in enumerate(self.arm_idx):
            qc[idx]    = q_arm_new[k]
            qcdot[idx] = qdot_arm[k]

        self.qc   = qc
        self.qdot = qcdot
        
        # Publish joint states and TF
        header = Header(stamp=self.now.to_msg(), frame_id=self.world_frame)

        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=qc.tolist(),
            velocity=qcdot.tolist()))

        # world -> <robot_name>/pelvis
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id=self.pelvis_frame,
            transform=Transform_from_Rp(Rpelvis, ppelvis)))

        # Publish marker at target (world frame)
        marker = Marker()
        marker.header = Header(stamp=self.now.to_msg(), frame_id=self.world_frame)
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.p_target[0])
        marker.pose.position.y = float(self.p_target[1])
        marker.pose.position.z = float(self.p_target[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pubmarker.publish(marker)

        # Publish this robot's saber tip *actual* world position
        my_pos_msg = Twist()
        my_pos_msg.linear.x = float(p_tip_world[0])
        my_pos_msg.linear.y = float(p_tip_world[1])
        my_pos_msg.linear.z = float(p_tip_world[2])
        self.pubpos.publish(my_pos_msg)


#
#  Main Code
#
def main(args=None):
    rclpy.init(args=args)
    future = Future()
    node = TrajectoryNode('pirouette', future)
    rclpy.spin_until_future_complete(node, future)

    if future.done():
        node.get_logger().info("Stopping: " + future.result())
    else:
        node.get_logger().info("Stopping: Interrupted")

    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

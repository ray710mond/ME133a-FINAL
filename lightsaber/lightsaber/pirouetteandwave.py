'''pirouetteandwave.py

   This is a demo for moving/placing an ungrounded robot and moving joints.

   In particular, imagine a humanoid robot.  This moves/rotates the
   pelvis frame relative to the world.  And waves an arm.

   Node:        /pirouette
   Publish:     /joint_states           sensor_msgs.msg.JointState
   Broadcast:   'world' -> 'pelvis'     geometry_msgs.msg.TransformStamped

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from asyncio                    import Future
from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster

from geometry_msgs.msg          import TransformStamped
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


#
#   Trajectory Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Arguments are the node name and a future object (to force a shutdown).
#
class TrajectoryNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        # Define the list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        self.jointnames = atlasnames
        
        # State for Atlas joints (all joints, but we only move right arm)
        self.qc   = np.zeros(len(self.jointnames))
        self.qdot = np.zeros(len(self.jointnames))

        # Right arm joint set (subset of atlasnames)
        self.arm_joint_names = ['r_arm_shz', 'r_arm_shx',
                                'r_arm_ely', 'r_arm_elx',
                                'r_arm_wry', 'r_arm_wrx']
        self.arm_idx = [self.jointnames.index(n) for n in self.arm_joint_names]

        # inverse kinematics
        self.lam     = 1.0
        self.dq_num  = 1e-4
        self.p_target = np.array([1.0, 0.0, 0.5])   # target hand position


        ##############################################################
        # Setup the logistics of the node:
        # Add publishers and a TF broadcaster.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.tfbroad  = TransformBroadcaster(self)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up the timer to update at 100Hz, with (t=0) occuring in
        # the first update cycle (dt) from now.
        self.dt    = 0.01                       # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()     # ROS time since 1970
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))
                               
        self.pubmarker = self.create_publisher(Marker, '/marker', 10)
    
    
    def _T(self, R, p): # t matrix function
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3]   = p
        return T

    def fk_right_hand(self, q): # forward kinematics
        # joints
        q_shz = q[0]
        q_shx = q[1]
        q_ely = q[2]
        q_elx = q[3]
        q_wry = q[4]
        q_wrx = q[5]

        # with pelvis at world (0,0,0), utorso offset by this
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

        # 8) hand to saber middle offset from URDF
        saber_offset = np.array([0.0, 0.0, 0.41*2])
        T = T @ self._T(Reye(), saber_offset)

        # return saber tip position
        return T[0:3, 3]


    def jacobian_right_hand(self, q):
        J = np.zeros((3, 6))
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
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Update function
    def update(self):
        # increment time
        self.t   = self.t + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        # fix pelvis in the world frame
        ppelvis = pxyz(0.0, 0.0, 0.0)
        Rpelvis = Reye()

        # extract current right-arm joint vector
        q_arm = np.array([self.qc[i] for i in self.arm_idx])

        # compute current hand position and task-space error (circle)
        Cx = 1.0
        Cy = 0.0
        Cz = 0.5
        R  = 0.25
        omega = 0.1   # rad/sec

        theta = omega * self.t

        # target trajectory in world frame
        self.p_target = np.array([
            Cx,
            Cy + R*np.cos(theta),
            Cz + R*np.sin(theta)
        ])

        # compute current end effector position and task error
        p_hand = self.fk_right_hand(q_arm)
        e      = self.p_target - p_hand

        # simple first-order reference velocity towards the goal
        xdot = self.lam * e 

        # numerical jacobian and joint velocity 
        J = self.jacobian_right_hand(q_arm) # 3x6

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

        # save
        self.qc   = qc
        self.qdot = qcdot
        
        
        # Finish by publishing the data
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=qc.tolist(),
            velocity=qcdot.tolist()))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='pelvis',
            transform=Transform_from_Rp(Rpelvis,ppelvis)))

        # Publish a small marker at (1,0,0)
        marker = Marker()
        marker.header = Header(stamp=self.now.to_msg(), frame_id='world')
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # Position
        marker.pose.position.x = float(self.p_target[0])
        marker.pose.position.y = float(self.p_target[1])
        marker.pose.position.z = float(self.p_target[2])

        # Orientation (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # Size (very small)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pubmarker.publish(marker)

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the node ends.
    future = Future()

    # Initialize the trajectory generator node.
    node = TrajectoryNode('pirouette', future)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the node is
    # complete (as signaled by the future object).
    rclpy.spin_until_future_complete(node, future)

    # Report the reason for shutting down.
    if future.done():
        node.get_logger().info("Stopping: " + future.result())
    else:
        node.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

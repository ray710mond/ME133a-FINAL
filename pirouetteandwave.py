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

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Update - send a new joint command every time step.
    def update(self):
        # increment time
        self.t   = self.t + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        # pelvis fixed at world
        ppelvis = pxyz(0.0, 0.0, 0.0)
        Rpelvis = Rotz(0.0)

        # des saber target
        p_target = np.array([1.0, 0.0, 0.0])
        
        # cubic spline
        T = 40
        if self.t < 0:
            s = 0.0
        elif self.t < T:
            tau = self.t / T
            s = 3*tau**2 - 2*tau**3
        else:
            s = 1.0

        # Desired point in world frame
        p_des = s * p_target

        # Allocate joint arrays
        qc    = np.zeros(len(self.jointnames))
        qcdot = np.zeros(len(self.jointnames))

        # Joint indices
        j_shz = self.jointnames.index('r_arm_shz')
        j_shx = self.jointnames.index('r_arm_shx')
        j_ely = self.jointnames.index('r_arm_ely')
        j_elx = self.jointnames.index('r_arm_elx')

        ###############################################################
        # NEED IN THIS SECTION BELOW
        # WE CAN ALSO CALCULATE THE FORWARD KIN, 
        # MIGHT JUST SCRAP THIS BELOW
        # shoulder origin from URDF
        # could also make pelvis not fixed at world
        shoulder = np.array([
            -0.0581 + 0.1406,   # x
             0.0     - 0.2256,  # y
             0.162 + 0.05 + 0.3056 + 0.4776  # z
        ])

        # Tip offset
        tip_offset = np.array([0.0, -0.09, 0.40])

        # Desired wrist position
        p_des_hand = p_des - tip_offset

        # IK vector shoulder → hand
        v = p_des_hand - shoulder
        dist = np.linalg.norm(v)

        # Arm link lengths
        L1 = 0.30
        L2 = 0.30

        # Elbow IK with clamping
        dist_clamped = max(min(dist, L1+L2-1e-6), abs(L1-L2)+1e-6)
        cos_elbow = (L1**2 + L2**2 - dist_clamped**2) / (2*L1*L2)
        elbow = -(pi - acos(cos_elbow))
        qc[j_elx] = elbow

        # Shoulder yaw
        shz = atan2(v[1], v[0])
        qc[j_shz] = shz

        # Shoulder pitch
        horiz = sqrt(v[0]**2 + v[1]**2)
        shx = atan2(v[2], horiz)
        qc[j_shx] = shx

        # No forearm twist
        qc[j_ely] = 0.0
        #############################################################


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
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
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

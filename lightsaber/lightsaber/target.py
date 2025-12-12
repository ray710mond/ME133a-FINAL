#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from random import uniform

from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header


class SharedTargetPublisher(Node):
    def __init__(self):
        super().__init__("shared_target")

        self.pub_target = self.create_publisher(Twist, '/shared_target', 10)
        self.pub_marker = self.create_publisher(Marker, '/shared_target_marker', 10)

        self.timer_period = 4.0  # seconds between new random targets
        self.timer = self.create_timer(self.timer_period, self.update_target)

        self.get_logger().info("Shared target publisher started")
        self.update_target()  # publish one immediately

    def sample_point(self):
        # vertical plane between robots (x = 0 plane)
        x = 0.0
        y = uniform(-0.75, 0.75)
        z = uniform(1.0, 2.25)
        return np.array([x, y, z])

    def update_target(self):
        p = self.sample_point()

        # Publish numeric target
        msg = Twist()
        msg.linear.x = float(p[0])
        msg.linear.y = float(p[1])
        msg.linear.z = float(p[2])
        self.pub_target.publish(msg)

        # Publish visualization marker
        marker = Marker()
        marker.header = Header(frame_id='world')
        marker.ns = "shared_random_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(p[0])
        marker.pose.position.y = float(p[1])
        marker.pose.position.z = float(p[2])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.pub_marker.publish(marker)

        self.get_logger().info(f"New target at {p}")


def main(args=None):
    rclpy.init(args=args)
    node = SharedTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from builtin_interfaces.msg import Time


class ZeroOdom(Node):

    def __init__(self):
        super().__init__("zero_odom")
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.timer = self.create_timer(0.5, self.publish_zero_odom)  # 2 Hz

    def publish_zero_odom(self):
        odom_msg = Odometry()
        now = self.get_clock().now().to_msg()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "os_senosr"
        odom_msg.child_frame_id = "os_imu"

        # Position und Orientierung = 0
        odom_msg.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        # Geschwindigkeit = 0
        odom_msg.twist.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )

        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZeroOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

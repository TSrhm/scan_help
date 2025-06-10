#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

class ScanNode(Node):

    def __init__(self):
        super().__init__("scan_help_now")
        self.scan_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.scan_subscriber = self.create_subscription(LaserScan, "/ouster/scan", self.scan_callback, qos)
        self.latest_scan = None  
        self.timer = self.create_timer(0.5, self.send_scan)  

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg  

    def send_scan(self):
        if self.latest_scan:
            self.scan_publisher.publish(self.latest_scan) 

def main(args=None):
    rclpy.init(args=args)
    node = ScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

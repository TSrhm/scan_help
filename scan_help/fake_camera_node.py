import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
from builtin_interfaces.msg import Time

class FakeCameraNode(Node):
    def __init__(self):
        super().__init__('fake_camera_node')
        self.bridge = CvBridge()

        # Publisher
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        # Timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_images)

        # Lade Beispielbilder
        rgb_path = os.path.expanduser('~/Downloads/basement_0001c/t1.ppm')
        depth_path = os.path.expanduser('~/Downloads/basement_0001c/t1.pgm')
        self.rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        self.depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # 16-bit depth

        if self.rgb_img is None or self.depth_img is None:
            self.get_logger().error("Konnte Bilder nicht laden! Pfade prüfen.")
            return

        self.get_logger().info('FakeCameraNode gestartet.')

    def publish_images(self):
        now = self.get_clock().now().to_msg()

        rgb_msg = self.bridge.cv2_to_imgmsg(self.rgb_img, encoding='bgr8')
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'

        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_img, encoding='mono16')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'

        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = 'camera_depth_optical_frame'
        info_msg.height = self.depth_img.shape[0]
        info_msg.width = self.depth_img.shape[1]
        info_msg.k = [525.0, 0.0, 319.5,
                      0.0, 525.0, 239.5,
                      0.0, 0.0, 1.0]  # Beispiel-Intrinsics (z.B. RealSense)
        info_msg.p = [525.0, 0.0, 319.5, 0.0,
                      0.0, 525.0, 239.5, 0.0,
                      0.0, 0.0, 1.0, 0.0]

        # Publish
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info_msg)

        self.get_logger().info('RGB, Depth und CameraInfo veröffentlicht.')

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.br = CvBridge()
        timer_period = 1.0  # Sekunden
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Image publisher node started.')

    def timer_callback(self):
        # Pfad zum Bild anpassen
        img = cv2.imread('/home/imech/data/Data/0/000001.png')
        if img is None:
            self.get_logger().error('Bild konnte nicht geladen werden!')
            return
        msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Bild gesendet.')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Knoten wird beendet...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

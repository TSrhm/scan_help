import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
from cv_bridge import CvBridge
import cv2

class ResizeAndScaleBbox(Node):
    def __init__(self):
        super().__init__('resize_and_scale_bbox_node')
        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(Image, '/spot/camera/left/image', self.rgb_callback, 10)
        self.yolo_sub = self.create_subscription(DetectionArray, '/yolo/tracking', self.detection_callback, 10)

        self.rgb_image = None
        self.detections = None

        # Zielgröße: Größe des Tiefenbildes
        self.target_width = 424
        self.target_height = 240

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def detection_callback(self, msg):
        self.detections = msg
        self.process()

    def process(self):
        if self.rgb_image is None or self.detections is None:
            return

        scale_x = self.target_width / self.rgb_image.shape[1]
        scale_y = self.target_height / self.rgb_image.shape[0]

        resized_image = cv2.resize(self.rgb_image, (self.target_width, self.target_height))

        for det in self.detections.detections:
            bbox = det.bbox
            cx = bbox.center.position.x * scale_x
            cy = bbox.center.position.y * scale_y
            w = bbox.size.x * scale_x
            h = bbox.size.y * scale_y

            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            cv2.rectangle(resized_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(resized_image, (int(cx), int(cy)), 3, (0, 0, 255), -1)

        cv2.imshow("Resized RGB + Scaled YOLO BBoxes", resized_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ResizeAndScaleBbox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


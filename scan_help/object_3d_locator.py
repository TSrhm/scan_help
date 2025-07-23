import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import numpy as np
import cv2

class Object3DLocator(Node):
    def __init__(self):
        super().__init__('object_3d_locator')
        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(Image, '/camera/left/image', self.rgb_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/depth/left/image', self.depth_callback, 10)
        self.sub_detections = self.create_subscription(DetectionArray, '/yolo/tracking', self.detection_callback, 10)
        self.sub_camera_info = self.create_subscription(CameraInfo, '/camera/left/camera_info', self.camera_info_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.rgb_image = None
        self.depth_image = None
        self.detections = None
        self.camera_info = None

        self.depth_width = 424
        self.depth_height = 240

        self.registered_objects = []  # (class_name, np.array([x, y, z]), bbox)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def detection_callback(self, msg):
        self.detections = msg
        self.process()

    def process(self):
        if any(x is None for x in [self.rgb_image, self.depth_image, self.detections, self.camera_info]):
            return

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        scale_x = self.depth_width / self.rgb_image.shape[1]
        scale_y = self.depth_height / self.rgb_image.shape[0]
        vis_image = cv2.resize(self.rgb_image, (self.depth_width, self.depth_height))

        for det in self.detections.detections:
            bbox = det.bbox
            u = int(bbox.center.position.x * scale_x)
            v = int(bbox.center.position.y * scale_y)

            if not (0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]):
                continue

            depth_raw = self.depth_image[v, u]
            if depth_raw == 0:
                self.get_logger().warn(f"Kein Tiefenwert bei ({u}, {v})")
                continue

            Z = depth_raw / 1000.0
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            # Bounding Box in Depth-Auflösung
            w = bbox.size.x * scale_x
            h = bbox.size.y * scale_y
            x1 = int(u - w / 2)
            y1 = int(v - h / 2)
            x2 = int(u + w / 2)
            y2 = int(v + h / 2)
            bbox_rect = (x1, y1, x2, y2)

            try:
                tf_cam_to_map = self.tf_buffer.lookup_transform('map', self.camera_info.header.frame_id, rclpy.time.Time())
                pt = tf2_geometry_msgs.PointStamped()
                pt.header.frame_id = self.camera_info.header.frame_id
                pt.point.x = X
                pt.point.y = Y
                pt.point.z = Z
                pt.header.stamp = self.get_clock().now().to_msg()

                pt_world = tf2_geometry_msgs.do_transform_point(pt, tf_cam_to_map)
                xyz = np.array([pt_world.point.x, pt_world.point.y, pt_world.point.z])

                if not self.is_duplicate(det.class_name, xyz, bbox_rect):
                    self.registered_objects.append((det.class_name, xyz, bbox_rect))
                    self.publish_static_tf(det.class_name, xyz)
                else:
                    self.get_logger().info(f"{det.class_name} in Nähe bereits registriert.")

                cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(vis_image, f"{det.class_name}", (u + 5, v), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.circle(vis_image, (u, v), 3, (0, 0, 255), -1)

            except Exception as e:
                self.get_logger().warn(f"TF Transform fehlgeschlagen: {e}")

        cv2.imshow("3D Objekte mit Filter", vis_image)
        cv2.waitKey(1)

    def is_duplicate(self, obj_class, xyz, new_bbox, threshold=0.3, iou_thresh=0.5):
        for cls, pos, bbox in self.registered_objects:
            if cls == obj_class:
                dist = np.linalg.norm(pos - xyz)
                if dist < threshold and self.iou(bbox, new_bbox) > iou_thresh:
                    return True
        return False

    def iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = max(1, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
        boxBArea = max(1, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))

        iou = interArea / float(boxAArea + boxBArea - interArea)
        return iou

    def publish_static_tf(self, obj_class, xyz):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = f"object_{obj_class}_{len(self.registered_objects)}"

        tf_msg.transform.translation.x = float(xyz[0])
        tf_msg.transform.translation.y = float(xyz[1])
        tf_msg.transform.translation.z = float(xyz[2])
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(f"Neuer Frame: {tf_msg.child_frame_id}")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Object3DLocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



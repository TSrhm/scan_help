from sensor_msgs.msg import CameraInfo

class Object3DLocator(Node):
    def __init__(self):
        super().__init__('object_3d_locator')
        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(Image, '/spot/camera/left/image', self.rgb_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/spot/depth/left/image', self.depth_callback, 10)
        self.sub_detections = self.create_subscription(DetectionArray, '/yolo/tracking', self.detection_callback, 10)
        self.sub_camera_info = self.create_subscription(CameraInfo, '/spot/camera/left/camera_info', self.camera_info_callback, 10)

        self.rgb_image = None
        self.depth_image = None
        self.detections = None
        self.camera_info = None

        self.depth_width = 424
        self.depth_height = 240

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

            if 0 <= u < self.depth_image.shape[1] and 0 <= v < self.depth_image.shape[0]:
                depth_raw = self.depth_image[v, u]
                if depth_raw == 0:
                    self.get_logger().warn(f"Kein Tiefenwert bei ({u}, {v})")
                    continue
                Z = depth_raw / 1000.0  # mm → m
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy

                self.get_logger().info(f"Objekt: {det.class_name}, 3D: X={X:.2f} Y={Y:.2f} Z={Z:.2f}")
                cv2.putText(vis_image, f"{det.class_name}: ({X:.2f},{Y:.2f},{Z:.2f})", (u+5, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.circle(vis_image, (u, v), 4, (0, 0, 255), -1)

        cv2.imshow("3D Positionen", vis_image)
        cv2.waitKey(1)


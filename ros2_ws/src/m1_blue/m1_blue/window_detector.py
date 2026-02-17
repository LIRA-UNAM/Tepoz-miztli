import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import os

class RealSenseWindowDetector(Node):
    def __init__(self):
        super().__init__('m1_blue_realsense_detector')

        # CONFIGURATION REALSENCE

        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.info_topic = '/camera/camera/color/camera_info'
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic = '/m1/blue/coordinates'
        
        # Load Yolo Model

        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')
        
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        # Camera variables

        self.intrinsics = None
        self.bridge = CvBridge()

        # SUSCRIPTIONS
        
        # Get calibration info
        self.info_sub = self.create_subscription(
            CameraInfo, 
            self.info_topic, 
            self.info_callback, 
            1
        )

        # Synchronize image and depth
        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # Publishers
        self.image_pub = self.create_publisher(Image, self.image_pub_topic, 10)
        self.coord_pub = self.create_publisher(Point, self.coord_topic, 10)

        self.get_logger().info("Node start. Waiting frames...")

    def info_callback(self, msg):
        # Save intrinsic matrix K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        if self.intrinsics is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics = msg
            self.get_logger().info(f"Calibración recibida: fx={self.fx:.2f}, fy={self.fy:.2f}")

    def sync_callback(self, rgb_msg, depth_msg):
        if self.intrinsics is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # YOLO parameter
            results = self.model(frame, conf=0.5, verbose=False)

            if not results or len(results[0].boxes) == 0:
                return

            # Use best detection
            box = results[0].boxes[0]
            x_center, y_center, w, h = box.xywh[0].cpu().numpy()

            #Area in pixels
            area_px = w * h

            #Formula distance using Area
            distance_px = 1038.33 / (area_px ** 0.5)
            
            # Coordenadas de la caja en enteros para recortar
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Image limits
            h_img, w_img = depth_frame.shape
            x1 = max(0, x1); y1 = max(0, y1)
            x2 = min(w_img, x2); y2 = min(h_img, y2)

            self.get_logger().info(
                f"Area: {area_px:.2f}m | Distancia: {distance_px:.2f}m"
            )

            # Write distance
            annotated = results[0].plot()
            cv2.putText(annotated, f"{distance_px:.2f}m", (int(x1), int(y1)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header = rgb_msg.header
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error en procesamiento: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseWindowDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

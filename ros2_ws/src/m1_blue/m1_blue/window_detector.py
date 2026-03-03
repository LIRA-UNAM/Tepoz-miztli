#!/usr/bin/env python3

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

        # ===== TOPICS =====
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.info_topic = '/camera/camera/color/camera_info'
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic = '/m1/blue/coordinates'

        # ===== LOAD YOLO MODEL =====
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        # ===== VARIABLES =====
        self.bridge = CvBridge()
        self.intrinsics = None

        self.last_frame = None
        self.last_depth = None
        self.last_detection = None

        # ===== SUBSCRIPTIONS =====
        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.info_callback,
            1
        )

        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub],
            queue_size=10,
            slop=0.1
        )

        self.ts.registerCallback(self.sync_callback)

        # ===== PUBLISHERS =====
        self.image_pub = self.create_publisher(Image, self.image_pub_topic, 10)
        self.coord_pub = self.create_publisher(Point, self.coord_topic, 10)

        # ===== YOLO TIMER (Hz) =====
        self.timer = self.create_timer(0.2, self.yolo_process)

        self.get_logger().info("Node started. Camera running...")

    # ==========================================================
    # CAMERA INFO CALLBACK
    # ==========================================================
    def info_callback(self, msg):
        if self.intrinsics is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics = msg

            self.get_logger().info(
                f"Calibration received: fx={self.fx:.2f}, fy={self.fy:.2f}"
            )

    # ==========================================================
    # IMAGE CALLBACK (FLUID STREAM)
    # ==========================================================
    def sync_callback(self, rgb_msg, depth_msg):

        if self.intrinsics is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding='bgr8'
            )

            # Guardamos último frame para YOLO
            self.last_frame = frame

            annotated = frame.copy()

            # Dibujar última detección si existe
            if self.last_detection is not None:
                x1, y1, x2, y2, distance = self.last_detection

                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    annotated,
                    f"{distance:.2f}m",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2
                )

            # PUBLICAR SIEMPRE (fluido tipo webcam)
            img_msg = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8'
            )
            img_msg.header = rgb_msg.header
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Stream error: {e}")

    # ==========================================================
    # YOLO PROCESS (BACKGROUND)
    # ==========================================================
    def yolo_process(self):

        if self.last_frame is None:
            return

        frame = cv2.resize(self.last_frame, (640, 480))

        results = self.model(frame, conf=0.5, verbose=False)

        if results and len(results[0].boxes) > 0:

            box = results[0].boxes[0]

            x_center, y_center, w, h = box.xywh[0].cpu().numpy()

            area_px = w * h
            distance_px = 1038.33 / (area_px ** 0.5)

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Guardamos detección
            self.last_detection = (x1, y1, x2, y2, distance_px)

        else:
            # Si no detecta nada, eliminar detección
            self.last_detection = None


# ==========================================================
# MAIN
# ==========================================================
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

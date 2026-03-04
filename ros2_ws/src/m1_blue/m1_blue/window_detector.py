#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import threading
import time


class RealSenseWindowDetector(Node):

    def __init__(self):
        super().__init__('m1_blue_realsense_detector')

        # TOPICS
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic = '/m1/blue/coordinates'

        # YOLO MODEL
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        # VARIABLES
        self.bridge = CvBridge()
        self.last_frame = None
        self.last_detection = None

        # Lock para evitar conflictos entre hilos
        self.lock = threading.Lock()

        # RGB SUBSCRIPTION
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            10
        )

        # PUBLISHERS
        self.image_pub = self.create_publisher(Image, self.image_pub_topic, 10)
        self.coord_pub = self.create_publisher(Point, self.coord_topic, 10)

        # HILO DE YOLO
        self.running = True
        self.yolo_thread = threading.Thread(target=self.yolo_loop)
        self.yolo_thread.daemon = True
        self.yolo_thread.start()

        self.get_logger().info("RGB detector with threaded YOLO started.")

    # CALLBACK DE IMAGEN (ULTRA LIGERO)
    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )

            with self.lock:
                self.last_frame = frame.copy()
                detection = self.last_detection

            annotated = frame.copy()

            if detection is not None:
                x1, y1, x2, y2, distance = detection

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

            img_msg = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8'
            )
            img_msg.header = msg.header
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Stream error: {e}")

    # LOOP INDEPENDIENTE DE YOLO
    def yolo_loop(self):

        while self.running:

            frame = None

            with self.lock:
                if self.last_frame is not None:
                    frame = self.last_frame.copy()

            if frame is not None:
                self.yolo_process(frame)

            time.sleep(0.2)  # misma frecuencia que tu timer

    # TU LOGICA EXACTA (NO MODIFICADA)
    def yolo_process(self, frame):

        frame = cv2.resize(frame, (640, 480))

        results = self.model(frame, conf=0.5, verbose=False)

        if results and len(results[0].boxes) > 0:

            box = results[0].boxes[0]

            x_center, y_center, w, h = box.xywh[0].cpu().numpy()

            area_px = w * h
            distance_px = 1038.33 / (area_px ** 0.5)

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            with self.lock:
                self.last_detection = (x1, y1, x2, y2, distance_px)

            center = Point()
            center.x = float((x1 + x2) / 2)
            center.y = float((y1 + y2) / 2)
            center.z = float(distance_px)

            self.coord_pub.publish(center)

        else:
            with self.lock:
                self.last_detection = None


def main(args=None):

    rclpy.init(args=args)
    node = RealSenseWindowDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
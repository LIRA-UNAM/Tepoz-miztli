#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

import cv2
import os
import torch

from ultralytics import YOLO


class RealSenseWindowDetector(Node):

    def __init__(self):
        super().__init__('m1_blue_realsense_detector')

        # TOPICS

        self.rgb_topic = '/camera/camera/color/image_raw'
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic = '/m1/blue/coordinates'

        # LOAD MODEL

        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Loading YOLO model: {model_path}")

        self.model = YOLO(model_path)
        self.model.to("cuda")

        torch.set_num_threads(1)

        # VARIABLES

        self.bridge = CvBridge()

        # SUBSCRIBER

        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            10
        )

        # PUBLISHERS

        self.image_pub = self.create_publisher(
            Image,
            self.image_pub_topic,
            10
        )

        self.coord_pub = self.create_publisher(
            Point,
            self.coord_topic,
            10
        )

        self.get_logger().info("RGB detector started.")

    def image_callback(self, msg):

        try:

            # Convertir imagen ROS → OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame_resized = cv2.resize(frame, (640, 480))

            # INFERENCIA YOLO (GPU)

            results = self.model(
                frame_resized,
                conf=0.5,
                device=0,
                verbose=False
            )

            detection = None

            if results and results[0].boxes is not None and len(results[0].boxes) > 0:

                box = results[0].boxes[0]

                x_center, y_center, w, h = box.xywh[0].cpu().numpy()

                area_px = w * h
                distance_px = 1038.33 / (area_px ** 0.5)

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                detection = (x1, y1, x2, y2, distance_px)

                center = Point()

                center.x = float((x1 + x2) / 2)
                center.y = float((y1 + y2) / 2)
                center.z = float(distance_px)

                self.coord_pub.publish(center)

            # DIBUJAR DETECCIÓN

            annotated = frame_resized.copy()

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

            # PUBLICAR IMAGEN

            img_msg = self.bridge.cv2_to_imgmsg(
                annotated,
                encoding='bgr8'
            )

            img_msg.header = msg.header

            self.image_pub.publish(img_msg)

        except Exception as e:

            self.get_logger().error(f"Processing error: {e}")

# MAIN

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
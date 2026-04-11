#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point

import cv2
import os
import numpy as np
import torch

from ultralytics import YOLO


class LandingUSBDetector(Node):

    def __init__(self):
        super().__init__('landing_usb_detector')

        # TOPICS
        self.rgb_topic = '/image_raw/compressed'
        self.image_pub_topic = '/m4/landing/detections'

        # YOLO MODEL
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Loading YOLO model: {model_path}")

        self.model = YOLO(model_path)

        # mover modelo a GPU
        self.model.to("cuda")

        torch.backends.cudnn.benchmark = True

        # VARIABLES
        self.last_frame = None
        self.last_detection = None

        self.min_area = 1500
        self.margin = 20

        # SUBSCRIPTION
        self.rgb_sub = self.create_subscription(
            CompressedImage,
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

        self.landing_coord_pub = self.create_publisher(
            Point,
            '/m4/landing/coordinates',
            10
        )

        # YOLO TIMER
        self.timer = self.create_timer(0.1, self.yolo_process)

        self.get_logger().info("Landing USB detector started (GPU mode).")

    # ---------- MJPEG -> OpenCV ----------
    def compressed_to_numpy(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        return frame

    # ---------- OpenCV -> ROS Image ----------
    def numpy_to_rosimg(self, img, header):

        msg = Image()

        msg.header = header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()

        return msg

    # ---------- IMAGE CALLBACK ----------
    def image_callback(self, msg):

        try:

            frame = self.compressed_to_numpy(msg)

            if frame is None:
                return

            self.last_frame = frame
            annotated = frame.copy()

            if self.last_detection is not None:

                x1, y1, x2, y2, distance, class_name = self.last_detection

                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    annotated,
                    f"{class_name} {distance:.2f}m",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2
                )

            img_msg = self.numpy_to_rosimg(annotated, msg.header)

            self.image_pub.publish(img_msg)

        except Exception as e:

            self.get_logger().error(f"Stream error: {e}")

    # ---------- YOLO PROCESS ----------
    def yolo_process(self):

        if self.last_frame is None:
            return

        with torch.no_grad():

            results = self.model(self.last_frame, conf=0.7, device="cuda", half=True, verbose=False)

        if results and len(results[0].boxes) > 0:

            boxes = results[0].boxes

            box = max(
                boxes,
                key=lambda b: (b.xyxy[0][2]-b.xyxy[0][0])*(b.xyxy[0][3]-b.xyxy[0][1])
            )

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]

            if class_name != "Landing_home":
                return

            w_box = x2 - x1
            h_box = y2 - y1
            area_px = w_box * h_box

            h, w, _ = self.last_frame.shape

            if area_px < self.min_area:
                return

            if x1 < self.margin or x2 > (w - self.margin):
                return

            distance = 605.86376 / (area_px ** 0.5)

            self.last_detection = (x1, y1, x2, y2, distance, class_name)

            self.get_logger().info(
                f"Detected: {class_name} | Distance: {distance:.2f}m | Area_px: {area_px}"
            )

            center = Point()

            center.x = float((x1 + x2) / 2 - w/2)
            center.y = float((y1 + y2) / 2 - h/2)
            center.z = float(distance)

            self.landing_coord_pub.publish(center)

        else:

            self.last_detection = None


# ---------- MAIN ----------
def main(args=None):

    rclpy.init(args=args)

    node = LandingUSBDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
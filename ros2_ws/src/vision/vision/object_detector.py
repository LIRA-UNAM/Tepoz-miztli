#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import cv2
import os
import numpy as np
import torch

from ultralytics import YOLO


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

        # mover modelo a GPU
        self.model.to("cuda")

        # optimización cudnn
        torch.backends.cudnn.benchmark = True

        # VARIABLES
        self.last_frame = None
        self.last_detection = None
        self.gate_detect_counter = 0
        self.required_detections = 10
        self.min_area = 2000
        self.margin = 30

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

        # YOLO TIMER
        self.timer = self.create_timer(0.1, self.yolo_process)

        self.get_logger().info("RGB detector started (GPU mode).")

    # ---------- ROS Image -> OpenCV ----------
    def rosimg_to_numpy(self, msg):

        img = np.frombuffer(msg.data, dtype=np.uint8)

        img = img.reshape(msg.height, msg.width, -1)

        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]

        return img

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

            frame = self.rosimg_to_numpy(msg)

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

            results = self.model(self.last_frame, conf=0.7, verbose=False)

        if results and len(results[0].boxes) > 0:

            boxes = results[0].boxes

            box = max(
                boxes,
                key=lambda b: (b.xyxy[0][2]-b.xyxy[0][0])*(b.xyxy[0][3]-b.xyxy[0][1])
            )

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]

            w_box = x2 - x1
            h_box = y2 - y1
            area_px = w_box * h_box
            h, w, _ = self.last_frame.shape

            if class_name == "Blue_gates":

                if area_px < self.min_area:
                    return
            
                if x1 < self.margin or x2 > (w - self.margin):
                    return
                
                self.gate_detect_counter += 1

                if self.gate_detect_counter < self.required_detections:
                    return

                distance = 1038.33 / (area_px ** 0.5)
                distance_text = f"{distance:.2f}m"

            elif class_name == "Green_gates":
                self.gate_detect_counter = 0
                distance = -1.0
                distance_text = "wait"

            elif class_name == "Landing_home":
                self.gate_detect_counter = 0
                distance = 605.86376 / (area_px ** 0.5)
                distance_text = f"{distance:.2f}m"

            else:
                self.gate_detect_counter = 0 
                distance = -1.0
                distance_text = "x"
            
            self.last_detection = (x1, y1, x2, y2, distance, class_name)

            self.get_logger().info(
                f"Detected: {class_name} | Distance: {distance_text} | Area_px: {area_px}" 
            )

            center = Point()

            center.x = float((x1 + x2) / 2 - w/2)
            center.y = float((y1 + y2) / 2 - h/2)
            center.z = float(distance)

            self.coord_pub.publish(center)

        else:
            self.last_detection = None
            self.gate_detect_counter = 0


# ---------- MAIN ----------
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
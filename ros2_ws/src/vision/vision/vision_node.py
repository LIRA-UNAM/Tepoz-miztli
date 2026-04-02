#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import torch
import os

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Header

from cv_bridge import CvBridge
from ultralytics import YOLO


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # ---------- TOPICS ----------
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.camera_info_topic = '/camera/camera_info'

        # ---------- YOLO MODEL ----------
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Loading YOLO model: {model_path}")

        self.model = YOLO(model_path)
        self.model.to("cuda")

        torch.backends.cudnn.benchmark = True

        # ---------- ARUCO ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.marker_size = 0.15

        self.K = None
        self.D = None

        # ---------- BRIDGE ----------
        self.bridge = CvBridge()

        # ---------- VARIABLES ----------
        self.last_frame = None

        # ---------- SUBSCRIBERS ----------
        self.create_subscription(Image, self.rgb_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        # ---------- PUBLISHERS ----------
        self.image_pub = self.create_publisher(Image, '/vision/image', 10)
        self.coord_pub = self.create_publisher(Point, '/vision/blue_gate', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/vision/aruco', 10)

        # ---------- TIMER ----------
        self.timer = self.create_timer(0.3, self.process_frame)

        self.get_logger().info("Vision node started")

    # ---------- CAMERA INFO ----------
    def camera_info_cb(self, msg):

        self.K = np.array(msg.k).reshape(3,3)
        self.D = np.array(msg.d)

    # ---------- IMAGE CALLBACK ----------
    def image_callback(self, msg):

        self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_header = msg.header

    # ---------- MAIN PROCESS ----------
    def process_frame(self):

        if self.last_frame is None:
            return

        frame = self.last_frame.copy()

        # ==============================
        # YOLO DETECTION
        # ==============================

        with torch.no_grad():
            results = self.model(frame, conf=0.5, verbose=False)

        if results:

            for box in results[0].boxes:

                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                x1,y1,x2,y2 = box.xyxy[0].cpu().numpy().astype(int)

                # Dibujar bbox
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

                cv2.putText(frame,label,(x1,y1-10),
                cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)

                # SOLO DISTANCIA PARA BLUE GATE
                if label == "Blue_gates":

                    w = x2-x1
                    h = y2-y1

                    area = w*h
                    distance = 1038.33/(area**0.5)

                    cv2.putText(frame,
                        f"{distance:.2f}m",
                        (x1,y2+20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,(255,0,0),2)

                    center = Point()

                    center.x = float((x1+x2)/2)
                    center.y = float((y1+y2)/2)
                    center.z = float(distance)

                    self.coord_pub.publish(center)

        # ==============================
        # ARUCO DETECTION
        # ==============================

        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        poses_msg = PoseArray()
        poses_msg.header = Header(
            stamp=self.last_header.stamp,
            frame_id=self.last_header.frame_id
        )

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if self.K is not None:

                rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.K,
                    self.D
                )

                for rvec,tvec in zip(rvecs,tvecs):

                    cv2.drawFrameAxes(
                        frame,
                        self.K,
                        self.D,
                        rvec,
                        tvec,
                        self.marker_size*0.5
                    )

                    pose = Pose()

                    pose.position.x = float(tvec[0][0])
                    pose.position.y = float(tvec[0][1])
                    pose.position.z = float(tvec[0][2])

                    poses_msg.poses.append(pose)

        if poses_msg.poses:
            self.pose_pub.publish(poses_msg)

        # ==============================
        # PUBLICAR IMAGEN
        # ==============================

        img_msg = self.bridge.cv2_to_imgmsg(frame,"bgr8")
        img_msg.header = self.last_header

        self.image_pub.publish(img_msg)


# ---------- MAIN ----------
def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

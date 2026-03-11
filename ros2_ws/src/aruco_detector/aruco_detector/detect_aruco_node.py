#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # ===== PARAMETERS =====
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_1000')
        self.declare_parameter('marker_size_m', 0.20)

        img_topic = self.get_parameter('image_topic').value
        caminfo_topic = self.get_parameter('camera_info_topic').value
        dict_name = self.get_parameter('aruco_dictionary').value
        self.marker_size = float(self.get_parameter('marker_size_m').value)

        # ===== ARUCO DICTIONARY =====
        try:
            dict_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().warn(f"Dictionary {dict_name} invalid, using DICT_5X5_1000")
            dict_id = cv2.aruco.DICT_5X5_1000

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()

        self.K = None
        self.D = None

        self.create_subscription(Image, img_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, caminfo_topic, self.camera_info_callback, 10)

        self.pose_pub = self.create_publisher(PoseArray, 'aruco/poses', 10)
        self.image_pub = self.create_publisher(Image, 'aruco/image_annotated', 10)

        self.get_logger().info("=== ARUCO DETECTOR READY ===")

    def camera_info_callback(self, msg: CameraInfo):

        if self.K is None:
            # matriz intrínseca
            self.K = [
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ]

            self.D = list(msg.d)

            self.get_logger().info("Camera intrinsics received.")

    def image_callback(self, msg: Image):

        if self.K is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated = frame.copy()

        corners, ids, _ = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        poses_msg = PoseArray()
        poses_msg.header = Header(
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id
        )

        if ids is not None and len(ids) > 0:

            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.K,
                self.D
            )

            for i in range(len(ids)):

                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                marker_id = int(ids[i][0])

                cv2.drawFrameAxes(
                    annotated,
                    self.K,
                    self.D,
                    rvec,
                    tvec,
                    self.marker_size * 0.5
                )

                pose = Pose()

                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])

                qx, qy, qz, qw = self.rodrigues_to_quaternion(rvec)

                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw

                poses_msg.poses.append(pose)

                dist = math.sqrt(
                    tvec[0]*tvec[0] +
                    tvec[1]*tvec[1] +
                    tvec[2]*tvec[2]
                )

                self.get_logger().info(
                    f"ID {marker_id} | Distance: {dist:.3f} m"
                )

        if len(poses_msg.poses) > 0:
            self.pose_pub.publish(poses_msg)

        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        )

    @staticmethod
    def rodrigues_to_quaternion(rvec):

        R, _ = cv2.Rodrigues(rvec)

        tr = R[0][0] + R[1][1] + R[2][2]

        qw = math.sqrt(max(0.0, 1.0 + tr)) / 2.0
        qx = (R[2][1] - R[1][2]) / (4.0 * qw + 1e-12)
        qy = (R[0][2] - R[2][0]) / (4.0 * qw + 1e-12)
        qz = (R[1][0] - R[0][1]) / (4.0 * qw + 1e-12)

        return float(qx), float(qy), float(qz), float(qw)


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
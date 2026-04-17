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

# ─── Editable Parameters ─────────────────────────────────────────────────────
CONF_THRESHOLD  = 0.70
MIN_AREA_PX     = 2000
MARGIN_PX       = 30
CLASS_NAME      = "Blue_gates"
REQUIRED_DETECTIONS = 8

BLUE_HSV_LOW   = np.array([ 90,  80,  50])
BLUE_HSV_HIGH  = np.array([130, 255, 255])
MIN_MASK_RATIO = 0.25

# ─── Distance Boundaries ─────────────────────────────────────────────────────
MAX_DEPTH_M    = 6.0    # Discard depth readings greater than this [m]
MIN_DEPTH_M    = 0.3    # Discard very close readings (noise) [m]

class GateDetector(Node):
    def __init__(self):
        super().__init__('blue_gate_detector')

        # ── RealSense D435i Topics ───────────────────────────────────────────
        self.rgb_topic       = '/camera/camera/color/image_raw'
        self.depth_topic     = '/camera/camera/depth/image_rect_raw'
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic     = '/m1/blue/coordinates'

        # ── YOLO Model ───────────────────────────────────────────────────────
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path  = os.path.join(weights_dir, 'best.pt')
        self.get_logger().info(f"Loading YOLO model: {model_path}")

        self.model = YOLO(model_path)
        self.model.to("cuda")
        torch.backends.cudnn.benchmark = True

        # ── Subscribers ─────────────────────────────────────────────────────
        self.rgb_sub   = self.create_subscription(Image, self.rgb_topic,   self.rgb_cb,   1)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_cb, 1)

        # ── Publishers ──────────────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, self.image_pub_topic, 10)
        self.coord_pub = self.create_publisher(Point, self.coord_topic,      1)

        # ── Internal State ────────────────────────────────────────────────────
        self.last_frame      = None
        self.last_depth      = None   # Depth image 16UC1 in mm
        self.last_detection  = None   # (x1, y1, x2, y2, conf, dist_m, method)
        self.gate_detect_counter = 0

        # ── YOLO Timer 10 Hz ──────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.yolo_process)
        self.get_logger().info("GateDetector D435i started with Mask-Guided Depth.")

    # ===================== HELPERS =====================

    def rosimg_to_numpy(self, msg: Image) -> np.ndarray:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1].copy()
        return img

    def numpy_to_rosimg(self, img: np.ndarray, header) -> Image:
        msg          = Image()
        msg.header   = header
        msg.height   = img.shape[0]
        msg.width    = img.shape[1]
        msg.encoding = "bgr8"
        msg.step     = img.shape[1] * 3
        msg.data     = img.tobytes()
        return msg

    def get_masked_depth(self, depth_img: np.ndarray, x1, y1, x2, y2, mask: np.ndarray) -> float:
        """
        Extracts median depth using ONLY the pixels where the HSV mask is active.
        This prevents reading the background through the hollow center of the gate.
        """
        # Crop depth image to match the bounding box
        depth_roi = depth_img[y1:y2, x1:x2]

        # Safety check to ensure dimensions match before bitwise operations
        if depth_roi.shape != mask.shape:
            return -1.0

        # Create a condition where mask is > 0 AND depth is within physical limits
        valid_condition = (mask > 0) & (depth_roi > MIN_DEPTH_M * 1000) & (depth_roi < MAX_DEPTH_M * 1000)
        valid_depths = depth_roi[valid_condition]

        if valid_depths.size == 0:
            return -1.0  # No valid depth data on the frame itself

        return float(np.median(valid_depths)) / 1000.0  # mm → meters

    # ===================== CALLBACKS =====================

    def rgb_cb(self, msg: Image):
        try:
            self.last_frame = self.rosimg_to_numpy(msg)
            annotated = self.last_frame.copy()
            h, w, _ = annotated.shape
            img_cx, img_cy = w / 2, h / 2

            if self.last_detection is not None:
                x1, y1, x2, y2, conf, dist_m, method = self.last_detection
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                err_x = cx - img_cx
                err_y = cy - img_cy

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 100, 0), 2)
                cv2.circle(annotated, (cx, cy), 5, (0, 255, 255), -1)
                
                cv2.putText(annotated,
                            f"Blue Gate {conf:.0%} | {dist_m:.2f}m ({method})",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)
                cv2.putText(annotated,
                            f"err=({err_x:+.0f}, {err_y:+.0f}) px",
                            (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            self.image_pub.publish(self.numpy_to_rosimg(annotated, msg.header))

        except Exception as e:
            self.get_logger().error(f"rgb_cb error: {e}")

    def depth_cb(self, msg: Image):
        try:
            self.last_depth = np.frombuffer(
                msg.data, dtype=np.uint16
            ).reshape(msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f"depth_cb error: {e}")

    # ===================== YOLO PROCESS =====================

    def yolo_process(self):
        if self.last_frame is None:
            return

        h, w, _ = self.last_frame.shape
        img_cx = w / 2
        img_cy = h / 2

        with torch.no_grad():
            results = self.model(self.last_frame, conf=CONF_THRESHOLD, verbose=False)

        if not results or len(results[0].boxes) == 0:
            self.last_detection = None
            self.gate_detect_counter = 0
            return

        best_box  = None
        best_area = 0
        best_mask = None

        for box in results[0].boxes:
            class_name = self.model.names[int(box.cls[0])]
            conf       = float(box.conf[0])

            if class_name != CLASS_NAME:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            
            # Prevent out-of-bounds slicing which causes shape mismatches later
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            area = (x2 - x1) * (y2 - y1)

            # ── Filter 1: Minimum Area ─────────────────────────────────────────
            if area < MIN_AREA_PX:
                continue

            # ── Filter 2: Edge Margin ──────────────────────────────────────────
            if (x1 < MARGIN_PX or y1 < MARGIN_PX or
                    x2 > w - MARGIN_PX or y2 > h - MARGIN_PX):
                continue

            # ── Filter 3: HSV Mask ─────────────────────────────────────────────
            roi        = self.last_frame[y1:y2, x1:x2]
            hsv        = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask       = cv2.inRange(hsv, BLUE_HSV_LOW, BLUE_HSV_HIGH)
            blue_ratio = np.count_nonzero(mask) / (area + 1e-6)

            if blue_ratio < MIN_MASK_RATIO:
                continue

            if area > best_area:
                best_area = area
                best_box  = (x1, y1, x2, y2, conf, area)
                best_mask = mask.copy()  # Save the mask of the best box

        if best_box is None:
            self.last_detection = None
            self.gate_detect_counter = 0
            return

        # ── Counter Logic (Debounce) ────────────────────────────────────────
        self.gate_detect_counter += 1
        if self.gate_detect_counter < REQUIRED_DETECTIONS:
            return  

        x1, y1, x2, y2, conf, area = best_box

        # ── Distance Calculation (Mask-Guided Hybrid) ───────────────────────
        dist_m = -1.0
        method = "None"

        # 1. Try D435i Depth Sensor USING THE HSV MASK
        if self.last_depth is not None and best_mask is not None:
            dist_m = self.get_masked_depth(self.last_depth, x1, y1, x2, y2, best_mask)
            method = "Sensor"

        # 2. Fallback to Camera Calibration if Sensor fails (returns -1.0)
        if dist_m == -1.0:
            dist_m = 1038.33 / (area ** 0.5)
            method = "Calib"

        self.last_detection = (x1, y1, x2, y2, conf, dist_m, method)

        err_x = (x1 + x2) / 2 - img_cx
        err_y = (y1 + y2) / 2 - img_cy

        coord   = Point()
        coord.x = float(err_x)
        coord.y = float(err_y)
        coord.z = float(dist_m)
        self.coord_pub.publish(coord)

        self.get_logger().info(
            f"Gate | conf={conf:.0%} area={area}px² | "
            f"err_x={err_x:+.0f} err_y={err_y:+.0f} px | "
            f"dist={dist_m:.2f} m ({method})"
        )

# ===================== MAIN =====================

def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Closing node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
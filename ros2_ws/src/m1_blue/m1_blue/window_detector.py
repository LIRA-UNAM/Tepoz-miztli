import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
import os
import yaml
import numpy as np

from ament_index_python.packages import get_package_share_directory


class WindowDetector(Node):
    def __init__(self):
        super().__init__('m1_blue_window_detector')

        # ===============================
        # CARGA DE CALIBRACIÓN DE CÁMARA
        # ===============================
        pkg_path = get_package_share_directory('m1_blue')
        config_path = os.path.join(pkg_path, 'config', 'laptop_cam.yaml')

        self.get_logger().info(f"Cargando calibración desde: {config_path}")

        with open(config_path, 'r') as file:
            cam_data = yaml.safe_load(file)

        self.fx = cam_data['camera_matrix']['data'][0]
        self.fy = cam_data['camera_matrix']['data'][4]
        self.cx = cam_data['camera_matrix']['data'][2]
        self.cy = cam_data['camera_matrix']['data'][5]

        # ===============================
        # DIMENSIONES REALES DE LA GATE
        # ===============================
        self.gate_width_real = 1.5   # metros
        self.gate_height_real = 1.5  # metros
        self.gate_area_real = self.gate_width_real * self.gate_height_real  # 2.25 m²

        # ===============================
        # CARGA DEL MODELO YOLO
        # ===============================
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Cargando modelo YOLO desde: {model_path}")
        self.model = YOLO(model_path)

        # ===============================
        # ROS TOPICS
        # ===============================
        self.camera_topic = '/image'
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic = '/m1/blue/coordinates'
        self.dist_topic = '/m1/blue/distance'

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            1
        )

        self.image_pub = self.create_publisher(Image, self.image_pub_topic, 10)
        self.coord_pub = self.create_publisher(Point, self.coord_topic, 10)
        self.dist_pub = self.create_publisher(Point, self.dist_topic, 10)

        self.get_logger().info(" Nodo m1_blue_window_detector iniciado correctamente")

    # ===============================
    # CALLBACK DE IMAGEN
    # ===============================
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.model(frame, conf=0.5, verbose=False)

            if not results or len(results[0].boxes) == 0:
                return

            # Tomamos la detección más confiable
            box = results[0].boxes[0]

            x_center, y_center, w, h = box.xywh[0].cpu().numpy()

            # ===============================
            # DISTANCIA POR ÁREA (ROBUSTA)
            # ===============================
            area_px = w * h

            # Filtro simple para evitar ruido
            if area_px < 800:
                return

            distance = np.sqrt(
                (self.gate_area_real * self.fx * self.fy) / area_px
            )

            # ===============================
            # PUBLICAR COORDENADAS
            # ===============================
            coord_msg = Point()
            coord_msg.x = float(x_center)
            coord_msg.y = float(y_center)
            coord_msg.z = float(area_px)
            self.coord_pub.publish(coord_msg)

            # ===============================
            # PUBLICAR DISTANCIA
            # ===============================
            dist_msg = Point()
            dist_msg.x = float(distance)
            dist_msg.y = 0.0
            dist_msg.z = 0.0
            self.dist_pub.publish(dist_msg)

            self.get_logger().info(
                f"Área(px): {area_px:.0f} | Distancia: {distance:.2f} m"
            )

            # ===============================
            # PUBLICAR IMAGEN ANOTADA
            # ===============================
            annotated = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header = msg.header
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error en procesamiento: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WindowDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



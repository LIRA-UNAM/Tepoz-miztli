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

        # ===============================
        # CONFIGURACIÓN
        # ===============================
        # Ajusta esto según tu setup de RealSense
        # Es crucial usar 'aligned_depth_to_color' para que los pixeles coincidan
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.info_topic = '/camera/camera/color/camera_info'
        
        self.image_pub_topic = '/m1/blue/detections'
        self.coord_topic = '/m1/blue/coordinates' # Enviaremos X, Y, Z reales
        
        # ===============================
        # CARGA DEL MODELO YOLO
        # ===============================
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')
        
        self.get_logger().info(f"Cargando modelo YOLO desde: {model_path}")
        self.model = YOLO(model_path)

        # ===============================
        # VARIABLES DE CÁMARA
        # ===============================
        self.intrinsics = None # Se llenará automáticamente
        self.bridge = CvBridge()

        # ===============================
        # SUSCRIPCIONES
        # ===============================
        
        # 1. Obtener info de calibración una vez (o actualizar si cambia)
        self.info_sub = self.create_subscription(
            CameraInfo, 
            self.info_topic, 
            self.info_callback, 
            10
        )

        # 2. Sincronización de Imagen y Profundidad
        # Usamos ApproximateTimeSynchronizer porque los timestamps de RGB y Depth 
        # pueden variar por milisegundos
        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1 # Margen de error de tiempo en segundos
        )
        self.ts.registerCallback(self.sync_callback)

        # Publicadores
        self.image_pub = self.create_publisher(Image, self.image_pub_topic, 10)
        self.coord_pub = self.create_publisher(Point, self.coord_topic, 10)

        self.get_logger().info("Nodo RealSense Detector iniciado. Esperando frames...")

    def info_callback(self, msg):
        # Guardamos la matriz intrínseca K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        if self.intrinsics is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics = msg
            self.get_logger().info(f"Calibración recibida: fx={self.fx:.2f}, fy={self.fy:.2f}")

    def sync_callback(self, rgb_msg, depth_msg):
        # Si no tenemos calibración aún, no procesamos
        if self.intrinsics is None:
            return

        try:
            # Convertir mensajes ROS a OpenCV
            # RealSense depth suele ser 16UC1 (milímetros en uint16)
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # Inferencia YOLO
            results = self.model(frame, conf=0.5, verbose=False)

            if not results or len(results[0].boxes) == 0:
                return

            # Tomar la mejor detección
            box = results[0].boxes[0]
            x_center, y_center, w, h = box.xywh[0].cpu().numpy()
            
            # Coordenadas de la caja en enteros para recortar
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Asegurar límites dentro de la imagen
            h_img, w_img = depth_frame.shape
            x1 = max(0, x1); y1 = max(0, y1)
            x2 = min(w_img, x2); y2 = min(h_img, y2)

            # ===============================
            # OBTENCIÓN DE PROFUNDIDAD REAL
            # ===============================
            # Recortamos la zona de profundidad correspondiente a la detección
            depth_roi = depth_frame[y1:y2, x1:x2]

            # Calculamos la distancia. 
            # IMPORTANTE: RealSense da 0 cuando no puede leer la profundidad.
            # Filtramos los 0 y calculamos la mediana para evitar ruido.
            valid_depths = depth_roi[depth_roi > 0]

            if len(valid_depths) == 0:
                self.get_logger().warn("Objeto detectado pero sin datos de profundidad válidos")
                return

            # Distancia en milímetros (promedio o mediana)
            dist_mm = np.median(valid_depths)
            dist_m = dist_mm / 1000.0  # Convertir a metros

            # ===============================
            # CÁLCULO DE COORDENADAS 3D (X, Y, Z)
            # ===============================
            # Usamos el modelo pinhole: X = (u - cx) * Z / fx
            real_x = (x_center - self.cx) * dist_m / self.fx
            real_y = (y_center - self.cy) * dist_m / self.fy
            real_z = dist_m

            # ===============================
            # PUBLICAR COORDENADAS
            # ===============================
            coord_msg = Point()
            coord_msg.x = float(real_x) # Metros a la derecha/izquierda del centro de la cámara
            coord_msg.y = float(real_y) # Metros arriba/abajo
            coord_msg.z = float(real_z) # Metros de profundidad
            self.coord_pub.publish(coord_msg)

            self.get_logger().info(
                f"Pos 3D -> X: {real_x:.2f}m | Y: {real_y:.2f}m | Dist: {real_z:.2f}m"
            )

            # ===============================
            # PUBLICAR IMAGEN ANOTADA
            # ===============================
            # Dibujamos la distancia en la imagen
            annotated = results[0].plot()
            cv2.putText(annotated, f"{dist_m:.2f}m", (int(x1), int(y1)-10), 
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

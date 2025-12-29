import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import os
import numpy as np

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        #Cargar el modelo
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Cargando el modelo YOLO de: {model_path}")

        self.model = YOLO(model_path)

        # Configuración
        self.camera_topic = '/front_camera/image_raw'
        self.detection_topic = 'yolo/detections'

        #Comunicación
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)

        self.publisher = self.create_publisher(Image, self.detection_topic, 10)

        self.get_logger().info(f"Subscrito a: {self.camera_topic}")
        self.get_logger().info(f"Detecciones publicadas en: {self.detection_topic}")

    def image_callback(self, msg):
        if not hasattr(self, 'model'):
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(cv_image, verbose=False, conf=0.5)

            annotated_frame = results[0].plot()

            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            output_msg.header = msg.header

            self.publisher.publish(output_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import os
import numpy as np

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Cargar el modelo
        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')

        self.get_logger().info(f"Cargando el modelo YOLO de: {model_path}")

        self.model = YOLO(model_path)

        # Configuración
        # ros2 run ... --ros-args -r /front_camera/image_raw:=/camera/camera/color/image_raw
        self.camera_topic = '/front_camera/image_raw'
        self.detection_topic = 'yolo/detections'
        self.coord_topic = 'yolo/coordinates'

        # Comunicación
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, 
            self.camera_topic, 
            self.image_callback, 
            qos_profile_sensor_data
        )

        self.img_publisher = self.create_publisher(Image, self.detection_topic, 10)
        self.coord_publisher = self.create_publisher(Point, self.coord_topic, 10)

        self.get_logger().info(f"Subscrito a: {self.camera_topic}")

    def image_callback(self, msg):
        if not hasattr(self, 'model'):
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(cv_image, verbose=False, conf=0.5)

            if len(results) > 0 and len(results[0].boxes) > 0:
                best_box = results[0].boxes[0]
                coords = best_box.xywh[0].cpu().numpy()
                x_center, y_center, width, height = coords

                point_msg = Point()
                point_msg.x = float(x_center)
                point_msg.y = float(y_center)
                point_msg.z = float(width * height)
                self.coord_publisher.publish(point_msg)
                
                self.get_logger().info(f"Gate: X={x_center:.0f}, Y={y_center:.0f}") #Log en terminal

            annotated_frame = results[0].plot()

            cv2.imshow("YOLO RealSense", annotated_frame)
            cv2.waitKey(1) 

            # Publicar imagen anotada (para RQT si lo usas despues)
            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            output_msg.header = msg.header
            self.img_publisher.publish(output_msg)

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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
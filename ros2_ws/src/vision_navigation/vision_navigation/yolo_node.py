import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point 
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import os
import numpy as np
import message_filters

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        weights_dir = os.path.expanduser('~/Tepoz-miztli/ros2_ws/weights')
        model_path = os.path.join(weights_dir, 'best.pt')
        self.model = YOLO(model_path)

        self.color_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        
        self.detection_topic = 'yolo/detections'
        self.coord_topic = 'yolo/coordinates'

        self.bridge = CvBridge()

        self.color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        self.img_publisher = self.create_publisher(Image, self.detection_topic, 10)
        self.coord_publisher = self.create_publisher(Point, self.coord_topic, 10)


    def image_callback(self, color_msg, depth_msg):
        if not hasattr(self, 'model'):
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

            results = self.model(cv_image, verbose=False, conf=0.5)
            annotated_frame = results[0].plot()

            if len(results) > 0 and len(results[0].boxes) > 0:
                # Se toma la mejor probabilidad
                best_box = results[0].boxes[0]
                coords = best_box.xywh[0].cpu().numpy()
                x_c, y_c, w, h = coords
                
                dist_meters = self.get_distance_gate(cv_depth, x_c, y_c, w, h) #Profundidad RealSense
                area_pixles = w * h #calculo del área normal
                area_limit = 120000

                if dist_meters < 0 and area_pixles > area_limit:
                    self.get_logger().warning("Realsense no detecta el gate completo, esta muy cerca")
                    dist_meters = 0.20

                if dist_meters > 0:
                    point_msg = Point()
                    point_msg.x = float(x_c)
                    point_msg.y = float(y_c)
                    point_msg.z = float(dist_meters)
                    
                    self.coord_publisher.publish(point_msg)
                    
                    log_msg = (f"Gate detectado: Centro=({x_c:.0f}, {y_c:.0f})"
                               f"Distancia={dist_meters:.2f}m"
                               f"Area={area_pixles:.0f}px")
                    self.get_logger().info(log_msg)

                    text = f"{dist_meters:.2f}m (Area:{int(area_pixles)})"
                    cv2.putText(annotated_frame, text, (int(x_c), int(y_c)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
                else:
                    self.get_logger().warning("Gate detectado sin profundidad")

            #Publicación de image procesada
            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            output_msg.header = color_msg.header
            self.img_publisher.publish(output_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

    def get_distance_gate(self, depth_img, x_c, y_c, w, h):

        """
        Se calcula la distancia del gate buscando los pixeles más cercanos
        de la bounding box
        """
        img_h, img_w = depth_img.shape

        # definición de limites
        x_min = int(max(0, x_c - w/2))
        x_max = int(min(img_w, x_c + w/2))
        y_min = int(max(0, y_c - h/2))
        y_max = int(min(img_h, y_c + h/2))

        # Limites del gate
        if x_max <= x_min or y_max <= y_min:
            return -1.0

        # recorte de matriz de profundidad
        roi = depth_img[y_min:y_max, x_min:x_max]

        # Filtro de valores invalidos
        valid_pixels = roi[roi > 0]

        if len(valid_pixels) == 0:
            return -1.0 # No encuntra una profundidad

        #Se ordenada por distamcia para la mejor probabilidad
        valid_pixels_sorted = np.sort(valid_pixels)

        #Se toma en cuenta solo el 10% de los pixeles
        num_samples = int(len(valid_pixels_sorted) * 0.10)
        num_samples = max(5, num_samples) 
        num_samples = min(num_samples, len(valid_pixels_sorted))
        closest_pixels = valid_pixels_sorted[:num_samples]
        avg_dist_mm = np.mean(closest_pixels) #promedio en mm
        return avg_dist_mm / 1000.0 #COnversión a m

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
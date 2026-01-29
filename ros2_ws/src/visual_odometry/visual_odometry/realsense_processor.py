import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RealSenseProcessor(Node):
    def __init__(self):
        super().__init__('realsense_processor_node')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.listener_callback, qos)

        self.publisher_=self.create_publisher(Image, '/camera/processed_image', 10)

        self.br = CvBridge()
        self.get_logger().info('Nodo iniciado')

    def listener_callback(self, data):

        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')

        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        new_msg = self.br.cv2_to_imgmsg(gray_frame, encoding='mono8')
        new_msg.header = data.header
        self.publisher_.publish(new_msg)

        display_frame = gray_frame.copy()
        cv2.putText(display_frame, "ROS2", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("Camara", display_frame)
        cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    node= RealSenseProcessor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

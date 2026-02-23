import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math 

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleStatus,
    DistanceSensor
)

class PX4Distance(Node):
    def __init__(self):
        super().__init__('px4_distance')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history= HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        #self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_cb, qos_profile)
        self.distance_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.distance_cb, qos_profile)


        self.timer = self.create_timer(0.5, self.timer_cb)

        self.current_z = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.current_d = 0.0
        
        
    def position_cb(self, msg):
        self.current_z = msg.z
        self.current_x = msg.x
        self.current_y = msg.y

    def distance_cb(self, msg):
        self.current_d = msg.current_distance

    def timer_cb(self):
        #now = self.get_clock().now().nanoseconds // 1000

        # offboard = OffboardControlMode()
        # offboard.timestamp = now
        # offboard.position = True

        # distance = DistanceSensor()
        # distance.timestamp = now 
        # distance.min_distance = 0.05
        # distance.max_distance = 4
        altura = -self.current_z
        self.get_logger().info(f"Distancia actual:{self.current_d:.2f} [m]")
        self.get_logger().info(f"Posición [x, y, z]: [{self.current_x:.2f}, {self.current_y:.2f}, {altura:.2f}] [m]")
        self.get_logger().info("-" * 40)
            
def main(args=None):
    rclpy.init(args=args)
    node = PX4Distance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



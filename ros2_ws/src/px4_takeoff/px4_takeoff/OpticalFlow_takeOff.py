import rclpy 
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleStatus
)

class PX4FlowPrecision(Node):

    def __init__(self):
        super().__init__('px4_flow_precision')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
        
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile)
        
        self.cmd_pub = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile)

        #Create Subscriptions
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile)

        self.attitude_sub = self.create_subscription(
             VehicleAttitude,
             '/fmu/out/vehicle_attitude', 
             self.attitude_cb,
             qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_cb) #10 Hz
        self.counter = 0

        self.current_z = 0.0
        self.current_yaw = 0.0
        self.locked_yaw = None
        self.target_z = -2.0 #System NED negative | -2.0 m = 2.0 
        self.hold_duration = 5.0 # 5s to 10Hz

        #Phases
        self.hold_counter = 0
        self.state = "INIT"

    def local_pos_cb(self, msg):
        self.current_z = msg.z

    def attitude_cb(self, msg):
        """
        #Quaternion (q) in angular Yaw
        q[0]=w, q[1]=x, q[2]=y, q[3]=z
        """
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 -2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True 
        offboard.velocity = True
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        # setpoint.yaw = 0.0 (North magnetic)
        
        if self.locked_yaw is None:
            self.locked_yaw = self.current_yaw
        setpoint.yaw = self.locked_yaw

        #Default Values | State Machine

        if self.state == "INIT":
            setpoint.position = [0.0,0.0,0.0]
            setpoint.velocity = [0.0,0.0,0.0]

            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = "ARMING"

        elif self.state == "ARMING":
            setpoint.position = [0.0,0.0,0.0]
            setpoint.velocity = [0.0,0.0,0.0]

            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info("ARMED")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, -0.8]
            distance_error = abs(self.current_z - self.target_z)

            if distance_error < 0.20:
                self.state = "HOLD"
                self.hold_counter = 0
                self.get_logger().info("Timer Inizialized")

        elif self.state == "HOLD":
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, 0.0]
            self.hold_counter += 1
            time_current = self.hold_counter * 0.1
            
            if time_current >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("LANDING")

        elif self.state == "LAND":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.3]
            if self.current_z > -0.20: #sensor Optical distance
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("LANDING COMPLETED")

        elif self.state == "LANDED":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.0]

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PX4FlowPrecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude
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

<<<<<<< HEAD
        #Create Subscriptions
=======
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile)

        # Subscriber
>>>>>>> bae0ee778fb642f5574264169781ac013bb73801
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile)

<<<<<<< HEAD
        self.attitude_sub = self.create_subscription(
             VehicleAttitude,
             '/fmu/out/vehicle_attitude', 
             self.attitude_cb,
             qos_profile
        )

        self.timer = self.create_timer(0.1, self.timer_cb) #10 Hz
        self.counter = 0
=======
        self.timer = self.create_timer(0.1, self.timer_cb)
>>>>>>> bae0ee778fb642f5574264169781ac013bb73801

        self.current_z = 0.0
<<<<<<< HEAD
        self.current_yaw = 0.0
        self.locked_yaw = None
        self.target_z = -2.0 #System NED negative | -2.0 m = 2.0 
        self.hold_duration = 40 # 4s to 10Hz

        #Phases
        self.state = "INIT"
=======
        self.target_z = -2.0
        self.hold_duration = 40
>>>>>>> bae0ee778fb642f5574264169781ac013bb73801
        self.hold_counter = 0
        self.counter = 0
        self.state = "INIT"

    def local_pos_cb(self, msg):
        self.current_z = msg.z

<<<<<<< HEAD
def attitude_cb(self, msg):
     """
     #Quaternion (q) in angular Yaw
     q[0]=w, q[1]=x, q[2]=y, q[3]=z
     """
     q = msg.q
     siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
     cosy_cosp = 1 -2 * (q[2] * q[2] + q[3] * q[3])
     yaw = math.atan2(siny_cosp, cosy_cosp)

     self.current_yaw = yaw

def timer_cb(self):
    now = self.get_clock().now().nanoseconds // 1000
=======
    def timer_cb(self):
>>>>>>> bae0ee778fb642f5574264169781ac013bb73801

        now = self.get_clock().now().nanoseconds // 1000

<<<<<<< HEAD
    setpoint = TrajectorySetpoint()
    setpoint.timestamp = now
    # setpoint.yaw = 0.0 (North magnetic)

    if self.state == "INIT" or self.state == "ARMING":
         self.locked_yaw = self.current_yaw
    
    setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0
    #Default Values
    setpoint.velocity = [0.0,0.0,float('nan')]
    setpoint.position = [float('nan'), float('nan'), float('nan')]
=======
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = True
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.yaw = 0.0
        setpoint.velocity = [0.0, 0.0, float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]
>>>>>>> bae0ee778fb642f5574264169781ac013bb73801

        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info("ARMED")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, -0.8]

<<<<<<< HEAD
        if abs(self.current_z - self.target_z) < 0.20: #0.15
            self.state = "HOLD"
            self.get_logger().info("HOLD POSITION")
=======
            if abs(self.current_z - self.target_z) < 0.15:
                self.state = "HOLD"
                self.get_logger().info("HOLD POSITION")
>>>>>>> bae0ee778fb642f5574264169781ac013bb73801

        elif self.state == "HOLD":
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, 0.0]
            self.hold_counter += 1

            if self.hold_counter >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("LANDING")

        elif self.state == "LAND":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.4]

            if self.current_z > -0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("LANDING COMPLETED")

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



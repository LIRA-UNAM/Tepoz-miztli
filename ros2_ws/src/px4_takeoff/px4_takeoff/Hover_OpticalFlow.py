import rclpy
import math
from rclpy.node import Node
from enum import Enum
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude
)

class State(Enum):
    INIT = 0
    ARMING = 1
    TAKEOFF = 2
    HOLD = 3
    LAND = 4
    LANDED = 5 


class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('px4_flow_precision')

        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        #Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode'
            # qos_profile
            )
        
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint'
            # qos_profile
            )
        
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command'
            # qos_profile
            )
        
        #Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb
            # qos_profile
            )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb
            # qos_profile
            )
        
        #Position current
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        #Position locked
        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        #Parameters
        self.target_z = -2.5
        self.hold_duration = 10.0
        #Timer y contadores
        # self.timer = self.create_timer(0.1, self.timer_cb) # 10 Hz estaba a 0.1
        self.counter = 0
        self.hold_counter = 0

        #Fases
        self.state = State.INIT

    def local_pos_cb(self, msg):
            self.current_x = msg.x
            self.current_y = msg.y
            self.current_z = msg.z
        
    def attitude_cb(self, msg):
            q = msg.q
            siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
            cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.current_yaw = yaw

    def send_cmd(self, command, param1=0.0, param2=0.0):
            msg = VehicleCommand()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000
            msg.command = command
            msg.param1 = float(param1)
            msg.param2 = float(param2)
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            self.cmd_pub.publish(msg)


        # def timer_cb(self):
        #     now = self.get_clock().now().nanoseconds
        #     offboard = OffboardControlMode()
        #     offboard.timestamp = now

        #     setpoint = TrajectorySetpoint()
        #     setpoint.timestrap = now

        #     setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else self.current_yaw
        #     setpoint.yawspeed = float('nan')

    def spin(self):
        self.get_logger().info("Initialized OpticalFlow Node")

        #Bucle princial de la maquina de estados
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds // 1000
            offboard = OffboardControlMode()
            offboard.timestamp = now

            setpoint = TrajectorySetpoint()
            setpoint.timestamp = now

            setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else self.current_yaw
            setpoint.yawspeed = float('nan')

            #Comienzo de estados
            if self.state == State.INIT:
                offboard.position = True
                offboard.velocity = False
                setpoint.position = [self.current_x, self.current_y, self.current_z]
                setpoint.velocity = [0.0, 0.0, 0.0]
                
                if self.counter > 20:
                    self.locked_yaw = self.current_yaw
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.state = State.ARMING

            elif self.state == State.ARMING:
                offboard.position = True
                offboard.velocity = False
                setpoint.position = [self.current_x, self.current_y, self.current_z]
                setpoint.velocity = [0.0, 0.0, 0.0]

                if self.counter > 40:
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.get_logger().info("ARMED")
                    self.state = State.TAKEOFF

            elif self.state == State.TAKEOFF:
                offboard.position = False
                offboard.velocity = True
                setpoint.position = [float('nan'), float('nan'), float('nan')]
                setpoint.velocity = [0.0,0.0,-0.8]

                if abs(self.current_z - self.target_z) < 0.15:
                    self.locked_x = self.current_x
                    self.locked_y = self.current_y
                    self.get_logger().info("HOLD")
                    self.state = State.HOLD

            elif self.state == State.HOLD:
                offboard.position = True
                offboard.velocity = False
                setpoint.position = [self.locked_x, self.locked_y, self.target_z]
                setpoint.velocity = [0.0,0.0,0.0]

                self.hold_counter +=1
                pass_time = self.hold_counter * 0.05 #Se puede cambiar a la 0.1

                if pass_time >= self.hold_duration:
                    self.get_logger().info("LANDING")
                    self.state = State.LAND

            elif self.state == State.LAND:
                offboard.position = True
                offboard.velocity = False
                setpoint.position = [self.locked_x, self.locked_y, float('nan')]
                setpoint.velocity = [0.0,0.0,0.4]

                if self.current_z > -0.20:
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                    self.get_logger().info("LANDED")
                    self.state = State.LANDED

            elif self.state == State.LANDED:
                offboard.position = True
                offboard.velocity = False
                setpoint.position = [self.locked_x, self.locked_y, self.current_z]
                setpoint.velocity = [0.0,0.0,0.0]

            self.offboard_pub.publish(offboard)
            self.trajectory_pub.publish(setpoint)
            self.counter +=1

            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.05))

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
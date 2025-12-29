import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Joy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import numpy as np 
from geometry_msgs.msg import Twist

class JoyOffboardControl(Node):
    def __init__(self):
        super().__init__('joy_offboard_control')

        self.AXIS_YAW = 0           #Left Stick L/R
        self.AXIS_THROTTLE = 1      #Left Stick U/D
        self.AXIS_ROLL = 2          #Right Stick L/R
        self.AXIS_PITCH = 3         #Right Stick U/D

        self.BTN_ARM = 0            
        self.BTN_DISARM = 1
        self.BTN_OFFBOARD = 2
        self.BTN_LAND = 3
            
        #Estado 
        # self.velocity = np.array([0.0, 0.0, 0.0]) 
        # self.yaw_rate = 0.0
        self.cmd_vel = Twist()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX


        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10 )

        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, 10)


        self.timer =self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Joy Control Started")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state

    def joy_callback(self, msg):
        # self.velocity[0] = msg.axes[self.AXIS_PITCH] * 5.0
        # self.velocity[1] = -msg.axes[self.AXIS_ROLL] * 5.0
        # self.velocity[2] = -msg.axes[self.AXIS_THROTTLE] * 2.0
        # self.yaw_rate = msg.axes[self.AXIS_YAW] * 1.5
        def apply_deadzone(value, limit=0.1):
            if abs(value) < limit:
                return 0.0
            return value

        roll_input = apply_deadzone(msg.axes[self.AXIS_ROLL])
        pitch_input = apply_deadzone(msg.axes[self.AXIS_PITCH])
        throttle_input = apply_deadzone(msg.axes[self.AXIS_THROTTLE])
        yaw_input = apply_deadzone(msg.axes[self.AXIS_YAW])


        self.cmd_vel.linear.x = roll_input * 5.0
        self.cmd_vel.linear.y = pitch_input * 5.0
        self.cmd_vel.linear.z = -throttle_input * 2.0
        self.cmd_vel.angular.z = -yaw_input * 1.5

        if msg.buttons[self.BTN_ARM] == 1:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Arm command sent")

        if msg.buttons[self.BTN_DISARM] == 1:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.get_logger().info("Disarm command sent")

        if msg.buttons[self.BTN_LAND] == 1:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Land command sent")

        if msg.buttons[self.BTN_OFFBOARD] == 1:
            self.publish_offboard_control_mode()
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Switching to Offboard Mode")

    def timer_callback(self):
        # PX4 needs this heartbeat constantly, even if not moving
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]

        #msg.velocity = [self.velocity[0], self.velocity[1], self.velocity[2]]
        msg.velocity = [
            self.cmd_vel.linear.x,
            self.cmd_vel.linear.y,
            self.cmd_vel.linear.z
        ]
        msg.yaw = float('nan')
        #msg.yawspeed = self.yaw_rate
        msg.yawspeed = self.cmd_vel.angular.z

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyOffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





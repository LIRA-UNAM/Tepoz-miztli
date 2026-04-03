#!/usr/bin/env python3

import rclpy
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor
)

from geometry_msgs.msg import Point


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

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            qos_profile)

        self.flow_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self.flow_cb,
            qos_profile)

        # Vision
        self.create_subscription(
            Point,
            '/m1/blue/coordinates',
            self.gate_cb,
            1
        )

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.counter = 0

        # Posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # Posición bloqueada
        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        # Visión
        self.gate = None

        # Parámetros
        self.target_z = -1.25
        self.hold_duration = 3.0
        self.hold_counter = 0

        self.start_time = None

        # Estados
        self.state = "INIT"

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

    def flow_cb(self, msg):
        self.get_logger().info(
            f"Flow quality: {msg.signal_quality} Dist: {msg.current_distance:.2f}")

    def gate_cb(self, msg):
        self.gate = msg

    def timer_cb(self):

        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        # Bloqueo inicial de posición
        if self.state in ["INIT", "ARMING"]:
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw else 0.0

        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        # MACHINE STATES

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
            error_z = abs(self.current_z - self.target_z)
            if error_z > 0.4:
                vz = -0.8
            elif error_z > 0.15:
                vz = -0.3
            else:
                vz = 0.0

            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [float('nan'), float('nan'), vz]

            if error_z < 0.15:
                self.state = "HOLD"
                self.get_logger().info("TAKEOFF COMPLETE")

        elif self.state == "HOLD":

            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [float('nan'), float('nan'), float('nan')]

            self.hold_counter += 1
            pass_time = self.hold_counter * 0.1

            if pass_time > self.hold_duration:
                self.state = "SEARCH"
                self.get_logger().info("SEARCHING GATE")

        elif self.state == "SEARCH":

            setpoint.position = [safe_x, float('nan'), self.target_z]
            setpoint.velocity = [0.0, 0.3, 0.0]

            if self.gate and self.gate.z < 3.0:
                self.state = "CENTER"
                self.get_logger().info("GATE DETECTED")

        elif self.state == "CENTER":

            if not self.gate:
                self.state = "SEARCH"
                return

            error_x = self.gate.x
            error_y = self.gate.y

            Kp = 0.002

            vy = -Kp * error_x
            vz = -Kp * error_y

            vy = max(min(vy, 0.5), -0.5)
            vz = max(min(vz, 0.4), -0.4)

            setpoint.velocity = [0.0, vy, vz]

            if abs(error_x) < 20 and abs(error_y) < 20:
                self.state = "CROSS_GATE"
                self.start_time = time.time()
                self.get_logger().info("CENTERED")

        elif self.state == "CROSS_GATE":

            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.8, 0.0, 0.0]

            if time.time() - self.start_time > 5.0:
                self.state = "LAND"

        elif self.state == "LAND":

            setpoint.position = [safe_x, safe_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]

            if self.current_z > -0.20:
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

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
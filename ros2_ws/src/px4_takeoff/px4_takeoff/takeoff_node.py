#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleCommand
)


class PX4AttitudeTakeoff(Node):

    def __init__(self):
        super().__init__('px4_attitude_takeoff')

        # -------------------------------
        # Publishers
        # -------------------------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Timer a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.counter = 0


    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # ------------------------------------------------
        # 1 OFFBOARD CONTROL MODE → ATTITUDE
        # ------------------------------------------------
        offboard = OffboardControlMode()
        offboard.timestamp = now

        offboard.position = False
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = True
        offboard.body_rate = False

        self.offboard_pub.publish(offboard)

        # ------------------------------------------------
        # 2 ATTITUDE SETPOINT
        # ------------------------------------------------
        att = VehicleAttitudeSetpoint()
        att.timestamp = now

        # Cuaternión identidad (sin inclinación)
        att.q_d = [1.0, 0.0, 0.0, 0.0]

        # Thrust (0.0 a 1.0)
        # ~0.6 normalmente ya levanta
        if self.counter < 60:
            att.thrust_body = [0.0, 0.0, -0.65]  # subir
        else:
            att.thrust_body = [0.0, 0.0, -0.45]  # sostener

        self.attitude_pub.publish(att)

        # ------------------------------------------------
        # 3 COMANDOS DE ESTADO
        # ------------------------------------------------
        if self.counter == 10:
            self.send_cmd(176, 1)  # OFFBOARD
            self.get_logger().info('OFFBOARD solicitado')

        if self.counter == 20:
            self.send_cmd(400, 1)  # ARM
            self.get_logger().info('ARM solicitado')

        self.counter += 1


    def send_cmd(self, command, param1=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.command = command
        msg.param1 = float(param1)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PX4AttitudeTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



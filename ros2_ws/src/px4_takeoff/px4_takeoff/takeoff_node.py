#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class PX4Takeoff(Node):

    def __init__(self):
        super().__init__('px4_takeoff_dds')

        # Publisher: modo offboard
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        # Publisher: setpoints (posición / velocidad)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        # Publisher: comandos (ARM, OFFBOARD, etc.)
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Timer a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.counter = 0


    def timer_cb(self):
        # Timestamp en microsegundos (PX4 lo requiere así)
        now = self.get_clock().now().nanoseconds // 1000

        # ------------------------------------------------
        # 1 OFFBOARD CONTROL MODE
        # ------------------------------------------------
        offboard = OffboardControlMode()
        offboard.timestamp = now

        # SOLO control por velocidad (sin GPS)
        offboard.position = False
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False

        self.offboard_pub.publish(offboard)

        # ------------------------------------------------
        # 2 TRAJECTORY SETPOINT (VELOCIDAD)
        # ------------------------------------------------
        sp = TrajectorySetpoint()
        sp.timestamp = now

        # PX4 usa marco NED:
        # z negativo = subir
        # z positivo = bajar
        if self.counter < 50:
            # Subir
            sp.velocity = [0.0, 0.0, -0.5]
        elif self.counter < 100:
            # Bajar
            sp.velocity = [0.0, 0.0, 0.3]
        else:
            # Detenerse
            sp.velocity = [0.0, 0.0, 0.0]

        self.traj_pub.publish(sp)

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


    def send_cmd(self, cmd, param1=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.command = cmd
        msg.param1 = float(param1)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PX4Takeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


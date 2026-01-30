#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class PX4OffboardNoGPS(Node):

    def __init__(self):
        super().__init__('px4_offboard_no_gps')

        # ===== Publishers hacia PX4 =====
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # ===== Timer (10 Hz) =====
        self.timer = self.create_timer(0.1, self.timer_cb)

        # Contador simple para la secuencia
        self.counter = 0


    def timer_cb(self):
        # Timestamp en microsegundos (PX4 lo exige así)
        now = self.get_clock().now().nanoseconds // 1000

        # ------------------------------------------------
        # 1) OFFBOARD CONTROL MODE
        # ------------------------------------------------
        offboard = OffboardControlMode()
        offboard.timestamp = now

        # Control por VELOCIDAD
        offboard.position = False
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False

        self.offboard_pub.publish(offboard)

        # ------------------------------------------------
        # 2) SETPOINT DE VELOCIDAD
        # ------------------------------------------------
        sp = TrajectorySetpoint()
        sp.timestamp = now

        # No moverse en X ni Y
        sp.vx = 0.0
        sp.vy = 0.0

        # Secuencia simple: subir -> bajar -> parar
        if self.counter < 50:
            sp.vz = -0.5   # subir
        elif self.counter < 100:
            sp.vz = 0.3    # bajar suave
        else:
            sp.vz = 0.0    # detenerse

        self.traj_pub.publish(sp)

        # ------------------------------------------------
        # 3) CAMBIOS DE ESTADO (OFFBOARD / ARM)
        # ------------------------------------------------
        if self.counter == 10:
            self.send_cmd(176, 1)  # MAV_CMD_DO_SET_MODE → OFFBOARD
            self.get_logger().info('OFFBOARD solicitado')

        if self.counter == 20:
            self.send_cmd(400, 1)  # MAV_CMD_COMPONENT_ARM_DISARM → ARM
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
    node = PX4OffboardNoGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



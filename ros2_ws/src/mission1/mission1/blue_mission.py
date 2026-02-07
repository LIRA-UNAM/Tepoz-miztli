#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleCommand
)
from geometry_msgs.msg import Point
import time
import math

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager_altitude')

        self.state = 'INIT'
        self.counter = 0
        self.start_time = None
        self.gate = None

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.att_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            10
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Vision
        self.create_subscription(
            Point,
            '/m1/blue/coordinates',
            self.gate_cb,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_cb)

    def gate_cb(self, msg):
        self.gate = msg

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # OFFBOARD heartbeat
        off = OffboardControlMode()
        off.timestamp = now
        off.attitude = True
        self.offboard_pub.publish(off)

        att = VehicleAttitudeSetpoint()
        att.timestamp = now

        # Default: plano, yaw fijo
        att.q_d = [1.0, 0.0, 0.0, 0.0]

        # Altitude control (PX4)
        att.thrust_body = [0.0, 0.0, -0.6]

        # ---------------- STATES ----------------

        if self.state == 'INIT':
            if self.counter == 20:
                self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
            if self.counter == 40:
                self.send_cmd(400, 1.0)       # ARM
                self.state = 'TAKEOFF'
                self.start_time = time.time()

        elif self.state == 'TAKEOFF':
            # Altitude controller sube solo
            att.thrust_body = [0.0, 0.0, -0.7]
            if time.time() - self.start_time > 3.0:
                self.state = 'HOLD'
                self.start_time = time.time()

        elif self.state == 'HOLD':
            att.thrust_body = [0.0, 0.0, -0.6]
            if time.time() - self.start_time > 2.0:
                self.state = 'MOVE_RIGHT'
                self.start_time = time.time()

        elif self.state == 'MOVE_RIGHT':
            # Roll pequeño a la derecha
            roll = 0.12  # rad
            att.q_d = self.euler_to_quaternion(roll, 0.0, 0.0)
            if time.time() - self.start_time > 1.5:
                self.state = 'SEARCH'

        elif self.state == 'SEARCH':
            if self.gate and self.gate.z < 3.0:
                self.state = 'MOVE_FORWARD'
                self.start_time = time.time()

        elif self.state == 'MOVE_FORWARD':
            pitch = -0.10  # adelante
            att.q_d = self.euler_to_quaternion(0.0, pitch, 0.0)
            if time.time() - self.start_time > 6.0:
                self.state = 'LAND'

        elif self.state == 'LAND':
            self.send_cmd(21)  # LAND

        self.att_pub.publish(att)
        self.counter += 1

    def euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = cmd
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()


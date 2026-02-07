#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)
from geometry_msgs.msg import Point
import time

class blue_mission(Node):

    def __init__(self):
        super().__init__('mission_manager')

        self.state = 'INIT'
        self.counter = 0
        self.gate_data = None
        self.start_time = None

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Subscriber (YOLO)
        self.create_subscription(
            Point,
            '/m1/blue/coordinates',
            self.gate_cb,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_cb)

    def gate_cb(self, msg):
        self.gate_data = msg

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # OFFBOARD heartbeat
        off = OffboardControlMode()
        off.timestamp = now
        off.velocity = True
        self.offboard_pub.publish(off)

        sp = TrajectorySetpoint()
        sp.timestamp = now

        # ---------------- STATES ----------------

        if self.state == 'INIT':
            if self.counter == 20:
                self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
            if self.counter == 40:
                self.send_cmd(400, 1.0)       # ARM
                self.state = 'TAKEOFF'
                self.start_time = time.time()

        elif self.state == 'TAKEOFF':
            sp.velocity = [0.0, 0.0, -0.4]   # subir
            if time.time() - self.start_time > 3.5:
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            sp.velocity = [0.0, 0.0, 0.0]
            self.start_time = time.time()
            self.state = 'MOVE_RIGHT'

        elif self.state == 'MOVE_RIGHT':
            sp.velocity = [0.3, 0.0, 0.0]   # derecha
            if time.time() - self.start_time > 1.7:
                self.state = 'SEARCH'

        elif self.state == 'SEARCH':
            sp.velocity = [0.0, 0.0, 0.0]
            if self.gate_data and self.gate_data.z < 3.0:
                self.state = 'FORWARD'
                self.start_time = time.time()

        elif self.state == 'FORWARD':
            sp.velocity = [0.0, 0.4, 0.0]
            if time.time() - self.start_time > 11.0:
                self.state = 'LAND'

        elif self.state == 'LAND':
            self.send_cmd(21)  # NAV_LAND

        self.setpoint_pub.publish(sp)
        self.counter += 1

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
    node = blue_mission()
    rclpy.spin(node)
    rclpy.shutdown()


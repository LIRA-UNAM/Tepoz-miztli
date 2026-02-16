#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)

from geometry_msgs.msg import Point
import time

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager_altitude')

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
            qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # Vision
        self.create_subscription(
            Point,
            '/m1/blue/coordinates',
            self.gate_cb,
            1
        )

        # Subscriber
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            qos_profile)

        self.timer = self.create_timer(0.1, self.timer_cb)
        
        self.state = 'INIT'
        self.counter = 0
        self.start_time = None
        self.gate = None
        self.current_z = 0.0
        self.target_z = -1.75
        self.hold_duration = 100
        self.hold_counter = 0


    def local_pos_cb(self, msg):
        self.current_z = msg.z

    def gate_cb(self, msg):
        self.gate = msg

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # OFFBOARD heartbeat
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

        # ---------------- STATES ----------------

        if self.state == 'INIT':
            if self.counter > 20:
                self.send_cmd(176, 1.0, 6.0)  # OFFBOARD
                self.state = "ARMING"

        elif self.state == "ARMING":
                if self.counter > 30:
                    self.send_cmd(400, param1=1.0) # ARM
                    self.get_logger().info("ARMED")
                    self.state = 'TAKEOFF'
                    self.start_time = time.time()

        elif self.state == 'TAKEOFF':
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, -0.8]

            if abs(self.current_z - self.target_z) < 0.15:
                self.state = 'HOLD'
                self.get_logger().info('TAKEOFF COMPLETE')

        elif self.state == 'HOLD':
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, 0.0]
            self.hold_counter += 1

            if self.hold_counter >= self.hold_duration:
                self.state = "SEARCH"
                self.get_logger().info("SEARCHING")

        elif self.state == 'SEARCH':
            setpoint.position = [float('nan'), float('nan'), self.target_z]
            setpoint.velocity = [0.0, 0.3, 0.0]
            if self.gate and self.gate.z < 3.0:
                self.state = 'CENTER'
                self.start_time = time.time()

        elif self.state == 'CENTER':

            if not self.gate:
                self.state = 'SEARCH'
                return
            
            error_x = self.gate.x
            error_y = self.gate.y

            Kp = 0.002

            vy = -Kp * error_x
            vz = -Kp * error_y

            vy = max(min(vy, 0.5), -0.5)
            vz = max(min(vz, 0.4), -0.4)

            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.0, vy, vz]

            if abs(error_x) < 20 and abs(error_y) < 20:
                self.state = 'CROSS_GATE'
                self.start_time = time.time()
                self.get_logger().info('Drone center')

        elif self.state == 'CROSS_GATE':
            setpoint.position = [float('nan'), float('nan'), self.target_z]
            setpoint.velocity = [0.8, 0.0, 0.0]

            if time.time() - self.start_time > 5.0:
                self.state = 'LAND'

        elif self.state == 'LAND':
            setpoint.position = [float('nan'), float('nan'), float('nan')]
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
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


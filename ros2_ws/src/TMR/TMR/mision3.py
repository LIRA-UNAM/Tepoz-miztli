"""
Mision 3: Circuito Gate Blue - Aterrizaje

Despegue, Cruce de Gate Blue, Esquivar columnas, Identiica zona de aterizaje, Landing.
"""

#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor
)

from geometry_msgs.msg import Point


TARGET_ALTITUDE = 1.3
TARGET_Z = -1.3
HOLD_DURATION = 3.0

SEARCH_SPEED = 0.1
CROSS_SPEED = 0.6


class BlueGateMission(Node):

    def __init__(self):
        super().__init__('blue_gate_mission')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PUBLISHERS

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            pub_qos)

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            pub_qos)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            pub_qos)

        # SUBSCRIBERS

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            sub_qos)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            sub_qos)

        self.flow_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self.flow_cb,
            sub_qos)

        self.aruco_sub = self.create_subscription(
            Point,
            '/m1/blue/coordinates',
            self.gate_cb,
            10
        )

        # VARIABLES 

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_distance = 0.0

        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        self.gate = None
        self.last_gate_time = 0.0

        self.state = "INIT"
        self.counter = 0

        self.stable_ticks = 0
        self.hold_start_time = None

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info("Blue Gate Mission iniciada")

    # CALLBACKS

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {self.current_distance:.4f} m"
            )

    def gate_cb(self, msg: Point):
        self.gate = msg
        self.last_gate_time = time.time()

        if self.counter % 10 == 0:
            self.get_logger().info(
                f"Gate detectada | x={msg.x:.1f} y={msg.y:.1f}"
            )

    # TIMER

    def timer_cb(self):

        now = self.get_clock().now().nanoseconds // 1000

        # Offboard mode
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = True
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # Trajectory setpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = float('nan')
        setpoint.yawspeed = float('nan')

        # Lock initial position
        if self.state in ("INIT", "ARMING"):
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # Maquina de Estados

        if self.state == 'INIT':

            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw

            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info("ARMED")
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':

            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            err_alt = abs(self.current_distance - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err_alt < 0.35 else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} "
                    f"err={err_alt:.2f} "
                    f"stable={self.stable_ticks}/10"
                )

            if self.stable_ticks >= 10:
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info("Altura estable")
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = setpoint_yaw
            elapsed = self._elapsed_s(self.hold_start_time, self.get_clock())
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"HOLD {elapsed:.1f}/"
                )
            if elapsed >= HOLD_DURATION:
                self.get_logger().info("SEARCHING GATE")
                self.state = 'SEARCH'

        elif self.state == 'SEARCH':

            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.0, SEARCH_SPEED, 0.0]

            if self.gate:
                self.state = "CROSS_GATE"
                self.start_time = time.time()
                self.get_logger().info("GATE DETECTED")

        elif self.state == 'CROSS_GATE':

            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.6, 0.0, 0.0]

            if time.time() - self.start_time > 4.0:
                self.state = "VEL_REDUC"
                self.start_time = time.time()
                self.get_logger().info("REDUCING VELOCITY")

        elif self.state == 'VEL_REDUC':

            elapsed = time.time() - self.start_time

            if elapsed < 2.0:
                setpoint.velocity = [0.4, 0.0, 0.0]

            elif elapsed < 4.0:
                setpoint.velocity = [0.2, 0.0, 0.0]

            elif elapsed < 6.0:
                setpoint.velocity = [0.1, 0.0, 0.0]

            else:
                self.get_logger().info("MISSION COMPLETED")
                self.state = 'TURN1'

        elif self.state == "TURN1":
            
            setpoint.position = [self.locked_x, self.locked_y, self.target_z]
            setpoint.yaw = 1.57 
            setpoint.yawspeed = 0.25

            #Condición para determinar el cambio de estado 
            if abs(self.current_yaw - 1.57) < 0.1:
                self.start_time = time.time()
                self.state = "ADVANCE"

        elif self.state == "ADVANCE":
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.2, 0.0, 0.0]

            if time.time() - self.start_time > 4.0:
                self.state = "VEL_REDUC"
                self.start_time = time.time()
                self.get_logger().info("TURN2")

        elif self.state == "TURN2":
            
            setpoint.position = [self.locked_x, self.locked_y, self.target_z]
            setpoint.yaw = 1.57 
            setpoint.yawspeed = 0.25

            #Condición para determinar el cambio de estado 
            if abs(self.current_yaw - 1.57) < 0.1:
                self.start_time = time.time()
                self.state = "ADVANCE"

        elif self.state == "ADVANCE2":
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.2, 0.0, 0.0]

            if time.time() - self.start_time > 4.0:
                self.state = "VEL_REDUC"
                self.start_time = time.time()
                self.get_logger().info("TURN2")

        # Publish setpoint
        self.trajectory_pub.publish(setpoint)

        self.counter += 1

    # ===================== COMMAND =====================

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
    node = BlueGateMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
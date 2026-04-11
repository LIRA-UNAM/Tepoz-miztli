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

        # Publishers
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

        # Subscribers
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

        # Vision
        self.create_subscription(
            Point,
            '/m1/blue/coordinates',
            self.gate_cb,
            1
        )

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.counter = 0

        # Posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        #Altura medida
        self.current_distance = 0.0

        # Posición bloqueada
        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        # Visión
        self.gate = None
        self.last_gate_time = 0

        # Parámetros
        self.target_altitude = 1.25
        self.target_z = -1.25
        self.hold_duration = 3.0

        # Control de estados
        self.state = "INIT"
        self.hold_start_time  = None
        self.stable_ticks     = 0
        self.stable_ticks_needed = 10
        
    #Callbacks
    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg):
        self.current_distance = msg.current_distance

        if self.counter % 10 == 0:
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {self.current_distance:.4f} m"
            )

    def gate_cb(self, msg):
        self.gate = msg
        self.last_gate_time = time.time()

    def timer_cb(self):

        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity =True
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        #Setpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        # Bloqueo inicial de posición
        if self.state in ["INIT", "ARMING"]:
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

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
            setpoint.position = [safe_x, safe_y, self.target_z]

            error_alt = abs(self.current_distance - self.target_altitude)

            if error_alt < 0.40:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} m "
                    f"target={self.target_altitude:.2f} m "
                    f"err={error_alt:.2f} m "
                    f"stable={self.stable_ticks}/{self.stable_ticks_needed}"
                )

            if self.stable_ticks >= self.stable_ticks_needed:
                self.state = "HOLD"
                self.get_logger().info(
                    f"HOLD POSITION - Estable en {self.current_distance:.2f} m "
                    f"(target={self.target_altitude:.2f} m, err={error_alt:.2f} m)"
                )

        elif self.state == "HOLD":

            setpoint.position = [safe_x, safe_y, self.target_z]
            
            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.hold_start_time
            ).nanoseconds / 1e9

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"HOLD {elapsed:.1f}s / {self.hold_duration}s | "
                    f"dist={self.current_distance:.2f} m"
                )

            if elapsed >= self.hold_duration:
                self.state = "SEARCH"
                self.get_logger().info("SEARCHING GATE")

        elif self.state == "SEARCH":

            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.0, 0.1, 0.0]

            if self.gate:
                self.state = "CROSS_GATE"
                self.start_time = time.time()
                self.get_logger().info("GATE DETECTED")

        elif self.state == "CENTER":

            error_x = self.gate.x
            error_y = self.gate.y

            Kp = 0.002

            vy = -Kp * error_x
            vz = -Kp * error_y

            vy = max(min(vy, 0.5), -0.5)
            vz = max(min(vz, 0.4), -0.4)

            setpoint.velocity = [0.0, vy, vz]

            if abs(error_x) < 50 and abs(error_y) < 50:
                self.state = "CROSS_GATE"
                self.start_time = time.time()
                self.get_logger().info("CENTERED")

        elif self.state == "CROSS_GATE":

            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [0.8, 0.0, 0.0]

            if time.time() - self.start_time > 6.0:
                self.state = "LAND"

        elif self.state == "LAND":

            setpoint.position = [safe_x, safe_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]

            if self.current_distance < 0.15:
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
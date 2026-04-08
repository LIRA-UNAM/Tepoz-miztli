import rclpy
import math
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


class PX4VelocityNode(Node):
    def __init__(self):
        super().__init__('px4_velocity')

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
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, sub_qos)
        self.distance_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self.distance_cb, sub_qos)

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.counter = 0

        # Posición y actitud actuales
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        # Yaw bloqueado al armar
        self.locked_yaw = None

        # ── Parámetros de la misión ──────────────────────────
        self.target_altitude  = 1.0    # metros desde el suelo
        self.ascent_vel       = 0.3    # m/s subida
        self.forward_vel      = 0.2    # m/s avance en X
        self.forward_duration = 10.0   # segundos → 0.2 * 10 = 2 m
        self.descent_vel      = 0.3    # m/s bajada

        # Tolerancias
        self.altitude_tol  = 0.08   # metros para considerar altura estable
        self.stable_needed = 20     # ticks a 20 Hz = 1 segundo estable

        # Control interno
        self.state         = "INIT"
        self.stable_ticks  = 0
        self.forward_ticks = 0

    # ======================== CALLBACKS ========================

    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def distance_cb(self, msg):
        self.current_distance = msg.current_distance
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"[SENSOR] dist={self.current_distance:.2f} m | "
                f"quality={msg.signal_quality}"
            )

    # ======================== LOOP PRINCIPAL ========================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # Bloquear yaw en tierra
        if self.state in ("INIT", "ARMING"):
            self.locked_yaw = self.current_yaw

        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ── OffboardControlMode ──────────────────────────────
        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = False
        offboard.velocity     = True
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        # ── Setpoint base (todo en NaN) ──────────────────────
        sp = TrajectorySetpoint()
        sp.timestamp    = now
        sp.position     = [float('nan')] * 3
        sp.velocity     = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk         = [float('nan')] * 3
        sp.yaw          = safe_yaw
        sp.yawspeed     = float('nan')

        # ======================== ESTADOS ========================

        if self.state == "INIT":
            sp.velocity = [0.0, 0.0, 0.0]
            if self.counter > 40:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = "ARMING"

        elif self.state == "ARMING":
            sp.velocity = [0.0, 0.0, 0.0]
            if self.counter > 60:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(
                    f"ARMED | Subiendo a {self.target_altitude:.1f} m "
                    f"a {self.ascent_vel:.1f} m/s"
                )
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            sp.velocity = [0.0, 0.0, -self.ascent_vel]

            error_alt = abs(self.current_distance - self.target_altitude)

            if error_alt < self.altitude_tol:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"[TAKEOFF] dist={self.current_distance:.2f} m | "
                    f"err={error_alt:.3f} m | "
                    f"stable={self.stable_ticks}/{self.stable_needed}"
                )

            if self.stable_ticks >= self.stable_needed:
                self.get_logger().info(
                    f"Altura {self.current_distance:.2f} m estable | "
                    f"Avanzando {self.forward_vel * self.forward_duration:.1f} m "
                    f"en {self.forward_duration:.0f} s "
                    f"a {self.forward_vel} m/s"
                )
                self.forward_ticks = 0
                self.state = "FORWARD"

        elif self.state == "FORWARD":
            sp.velocity = [self.forward_vel, 0.0, 0.0]

            self.forward_ticks += 1
            elapsed   = self.forward_ticks / 20.0
            distancia = self.forward_vel * elapsed

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"[FORWARD] t={elapsed:.1f} s | "
                    f"dist_estimada={distancia:.2f} m | "
                    f"alt={self.current_distance:.2f} m"
                )

            if self.forward_ticks >= int(self.forward_duration * 20):
                self.get_logger().info(
                    f"FORWARD completado: {elapsed:.1f} s | "
                    f"~{distancia:.2f} m | Aterrizando"
                )
                self.state = "LANDING"

        elif self.state == "LANDING":
            sp.velocity = [0.0, 0.0, self.descent_vel]

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"[LANDING] dist={self.current_distance:.2f} m"
                )

            if self.current_distance < 0.12:
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("LANDED - Motores desarmados")
                self.state = "LANDED"

        elif self.state == "LANDED":
            sp.velocity = [0.0, 0.0, 0.0]

        self.trajectory_pub.publish(sp)
        self.counter += 1

    # ======================== COMANDOS ========================

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PX4VelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
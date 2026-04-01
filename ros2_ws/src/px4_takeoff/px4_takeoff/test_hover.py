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

        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz
        self.counter = 0

        # Posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # Posición bloqueada al armar
        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        # Parámetros de vuelo
        self.target_z = -1.2        # 1.2 metros de altura (NED: negativo = arriba)
        self.hold_duration = 10.0   # Segundos en HOLD (ampliado para diagnóstico)

        # Contadores
        self.hold_counter = 0
        self.hold_stable_count = 0  # Ticks estables antes de entrar a HOLD

        # Estado inicial
        self.state = "INIT"

    # ─────────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────────

    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg):
        """Convierte cuaterniones a ángulo Euler Yaw para bloquear rotación."""
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg):
        self.get_logger().info(
            f"Flujo | Calidad: {msg.signal_quality} | Distancia: {msg.current_distance:.2f} m"
        )

    # ─────────────────────────────────────────────
    # TIMER PRINCIPAL
    # ─────────────────────────────────────────────

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # --- Offboard Control Mode ---
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # --- Setpoint base (todo en NaN por defecto) ---
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed     = float('nan')

        # --- Bloquear posición en INIT y ARMING ---
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        # Valores seguros por si acaso
        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0
        setpoint.yaw = safe_yaw

        # ─────────────────────────────────────────
        # MÁQUINA DE ESTADOS
        # ─────────────────────────────────────────

        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)  # Entrar a Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)  # Armar motores
                self.get_logger().info("ARMED — Iniciando despegue a 1.2 m")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            # Solo posición — sin inyección de velocidad para evitar overshoot
            setpoint.position = [safe_x, safe_y, self.target_z]

            error_z = abs(self.current_z - self.target_z)

            if error_z < 0.15:
                # Requiere 1 segundo estable (10 ticks a 10 Hz) antes de pasar a HOLD
                self.hold_stable_count += 1
                self.get_logger().info(
                    f"TAKEOFF estable: {self.hold_stable_count}/10 ticks "
                    f"| z_actual={self.current_z:.2f} z_meta={self.target_z:.2f}"
                )
                if self.hold_stable_count >= 10:
                    self.hold_counter = 0
                    self.state = "HOLD"
                    self.get_logger().info("HOLD — Manteniendo altura y posición")
            else:
                # Si sale del rango, reinicia el contador de estabilidad
                self.hold_stable_count = 0

        elif self.state == "HOLD":
            # Posición explícita + velocidad cero para mayor firmeza
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [0.0, 0.0, 0.0]

            self.hold_counter += 1
            elapsed = self.hold_counter * 0.1  # segundos

            # Log cada segundo
            if self.hold_counter % 10 == 0:
                self.get_logger().info(
                    f"HOLD: {elapsed:.1f}s / {self.hold_duration}s "
                    f"| z={self.current_z:.2f}"
                )

            if elapsed >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("LAND — Iniciando aterrizaje")

        elif self.state == "LAND":
            # Baja sobre las mismas coordenadas bloqueadas
            setpoint.position = [safe_x, safe_y, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.4]  # Descenso controlado

            if self.current_z > -0.20:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)  # Desarmar motores
                self.get_logger().info("LANDED — Motores desarmados")

        elif self.state == "LANDED":
            pass  # No hacer nada, esperar al operador

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    # ─────────────────────────────────────────────
    # UTILIDADES
    # ─────────────────────────────────────────────

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
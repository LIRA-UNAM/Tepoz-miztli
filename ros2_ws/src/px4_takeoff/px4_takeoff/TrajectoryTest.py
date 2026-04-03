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


class PX4TrajectoryNode(Node):
    def __init__(self):
        super().__init__('px4_trajectory')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_cb, qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, qos_profile)
        self.flow_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self.flow_cb, qos_profile)

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.counter = 0

        # Posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # Posición bloqueada al armar (origen del vuelo)
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        # Parámetros de vuelo
        self.target_z  = -1.5   # Altura objetivo en NED (negativo = arriba)
        self.point_x   = 2.0    # Metros a avanzar en X desde el origen

        # Target X calculado una sola vez al entrar a FORWARD
        self.target_x = None

        self.state = "INIT"

    # ===================== CALLBACKS =====================

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
        if self.counter % 20 == 0:  # 1 vez por segundo a 20Hz
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {msg.current_distance:.2f} m"
            )

    # ===================== LOOP PRINCIPAL =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # Modo Offboard (siempre publicar antes del arme)
        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False  # ← debe ser False en modo posición
        self.offboard_pub.publish(offboard)

        # Setpoint base con NaN (PX4 ignora lo no especificado)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed     = float('nan')

        # Bloquear posición de origen mientras el dron está en tierra
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ===================== MÁQUINA DE ESTADOS =====================

        if self.state == "INIT":
            # Publicar setpoints un tiempo antes de pedir OFFBOARD
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)  # Modo OFFBOARD
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)  # Armar motores
                self.get_logger().info(
                    f"ARMED | Ascendiendo a {abs(self.target_z):.1f} m"
                )
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]

            if abs(self.current_z - self.target_z) < 0.15:
                # Calcular destino X una sola vez
                self.target_x = safe_x + self.point_x
                self.get_logger().info(
                    f"Altura alcanzada: {self.current_z:.2f} m | "
                    f"Avanzando {self.point_x} m en X → target_x={self.target_x:.2f}"
                )
                self.state = "FORWARD"

        elif self.state == "FORWARD":
            # Mantener altura mientras avanza en X
            setpoint.position = [self.target_x, safe_y, self.target_z]

            if abs(self.current_x - self.target_x) < 0.15:
                self.get_logger().info(
                    f"Destino X alcanzado: {self.current_x:.2f} m | Aterrizando"
                )
                self.state = "LANDING"

        elif self.state == "LANDING":
            # Aterrizar en el punto X donde llegó
            setpoint.position = [self.target_x, safe_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]  # Descenso suave

            if self.current_z > -0.10:
                self.send_cmd(400, param1=0.0)  # Desarmar motores
                self.get_logger().info("LANDING COMPLETED - Motores desarmados")
                self.state = "LANDED"

        elif self.state == "LANDED":
            # Congelar posición final para no enviar NaN tras desarme
            setpoint.position = [self.current_x, self.current_y, self.current_z]

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    # ===================== COMANDOS =====================

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
    node = PX4TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
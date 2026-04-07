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
        self.offboard_pub   = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',  pub_qos)
        self.cmd_pub        = self.create_publisher(VehicleCommand,       '/fmu/in/vehicle_command',       pub_qos)

        # Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.attitude_sub  = self.create_subscription(VehicleAttitude,      '/fmu/out/vehicle_attitude',       self.attitude_cb,  sub_qos)
        self.flow_sub      = self.create_subscription(DistanceSensor,       '/fmu/out/distance_sensor',        self.flow_cb,      sub_qos)

        self.timer   = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.counter = 0

        # Posición actual
        self.current_x        = 0.0
        self.current_y        = 0.0
        self.current_z        = 0.0
        self.current_yaw      = 0.0
        self.current_distance = 0.0

        # Posición bloqueada al armar (origen)
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        # Posición bloqueada al llegar al destino (para HOLD y LANDING)
        self.landing_x = None
        self.landing_y = None

        # Parámetros de vuelo
        self.target_altitude = 1.5   # Altura objetivo en metros (desde el suelo)
        self.target_z        = -1.5  # Mismo valor en NED
        self.point_x         = 2.0   # Metros a avanzar en X
        self.hold_duration   = 3.0   # Segundos de hover en el destino antes de aterrizar

        # Parámetros calculados para avanzar por velocidad y tiempo
        self.forward_speed    = 0.5   # Velocidad de avance en m/s
        self.forward_duration = self.point_x / self.forward_speed  # Tiempo necesario para recorrer la distancia
        
        # Target X calculado al entrar a FORWARD (ya no se usa estrictamente, pero se deja por estructura)
        self.target_x = None

        # Control de estados
        self.state               = "INIT"
        self.stable_ticks        = 0
        self.stable_ticks_needed = 20   # 1s a 20 Hz para confirmar altura
        self.hold_origin_time    = None # Timestamp al entrar a HOLD_ORIGIN
        self.forward_start_time  = None # Timestamp al entrar a FORWARD
        self.hold_start_time     = None # Timestamp al entrar a HOLD (en destino)

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
        self.current_distance = msg.current_distance
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | Distancia: {self.current_distance:.2f} m"
            )

    # ===================== LOOP PRINCIPAL =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        offboard.attitude     = False
        offboard.body_rate    = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed     = float('nan')

        # Bloquear origen mientras está en tierra
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ===================== MÁQUINA DE ESTADOS =====================

        if self.state == "INIT":
            if self.counter > 40:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 60:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(f"ARMED | Ascendiendo a {self.target_altitude:.1f} m")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]

            error_alt = abs(self.current_distance - self.target_altitude)
            if error_alt < 0.40:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} m "
                    f"err={error_alt:.2f} m stable={self.stable_ticks}/{self.stable_ticks_needed}"
                )

            if self.stable_ticks >= self.stable_ticks_needed:
                self.hold_origin_time = self.get_clock().now()
                self.get_logger().info(
                    f"Altura estable en {self.current_distance:.2f} m | HOLD_ORIGIN 3s antes de avanzar"
                )
                self.state = "HOLD_ORIGIN"

        elif self.state == "HOLD_ORIGIN":
            setpoint.position = [safe_x, safe_y, self.target_z]

            elapsed = (self.get_clock().now() - self.hold_origin_time).nanoseconds / 1e9

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"HOLD_ORIGIN {elapsed:.1f}s / 3.0s | dist={self.current_distance:.2f} m"
                )

            if elapsed >= self.hold_duration:
                self.forward_start_time = self.get_clock().now()
                self.get_logger().info(
                    f"Avanzando {self.point_x} m a {self.forward_speed} m/s "
                    f"durante {self.forward_duration:.1f}s"
                )
                self.state = "FORWARD"

        elif self.state == "FORWARD":
            # Avanzar en X con velocidad controlada manteniendo altura
            # Usa velocidad en X en lugar de posición absoluta para no depender
            # de que VehicleLocalPosition esté disponible
            setpoint.velocity = [self.forward_speed, 0.0, float('nan')]
            setpoint.position = [float('nan'), float('nan'), self.target_z]

            elapsed_fwd = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"FORWARD | t={elapsed_fwd:.1f}s / {self.forward_duration:.1f}s | "
                    f"x={self.current_x:.2f} dist={self.current_distance:.2f} m"
                )

            if elapsed_fwd >= self.forward_duration:
                # Bloquear posición real al llegar
                self.landing_x = self.current_x
                self.landing_y = self.current_y
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info(
                    f"Destino alcanzado en t={elapsed_fwd:.1f}s | "
                    f"Bloqueando pos x={self.landing_x:.2f} | HOLD {self.hold_duration:.0f}s"
                )
                self.state = "HOLD"

        elif self.state == "HOLD":
            # Mantener posición bloqueada en destino
            setpoint.position = [self.landing_x, self.landing_y, self.target_z]

            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"HOLD {elapsed:.1f}s / {self.hold_duration:.0f}s | "
                    f"dist={self.current_distance:.2f} m"
                )

            if elapsed >= self.hold_duration:
                self.get_logger().info("LANDING - Bajando en línea recta")
                self.state = "LANDING"

        elif self.state == "LANDING":
            # Descender en vertical sobre la posición bloqueada
            setpoint.position = [self.landing_x, self.landing_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]  # Descenso suave

            if self.counter % 20 == 0:
                self.get_logger().info(
                    f"LANDING | dist={self.current_distance:.2f} m | "
                    f"x={self.current_x:.2f} y={self.current_y:.2f}"
                )

            if self.current_distance < 0.15:
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("LANDING COMPLETED - Motores desarmados")
                self.state = "LANDED"

        elif self.state == "LANDED":
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
        node.get_logger().info("Interrupción detectada. Cerrando nodo...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
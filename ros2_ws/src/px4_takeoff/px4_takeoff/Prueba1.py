import rclpy 
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor,
    VehicleStatus
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
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_cb,
            qos_profile
        )

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.counter = 0

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        self.locked_x = 0.0
        self.locked_y = 0.0
        self.locked_z = 0.0
        self.locked_yaw = 0.0
        
        self.target_z = -1.2 # Altura objetivo (NED, negativo es hacia arriba)

    # ===================== CALLBACKS =====================

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

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
        # Reducir frecuencia de logs para mantener limpia la terminal (1 vez por segundo)
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Calidad Flow: {msg.signal_quality} | Distancia Z: {msg.current_distance:.2f}"
            )

    # ===================== LOOP PRINCIPAL =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # 1. OFFBOARD MODE (SIEMPRE PUBLICAR)
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # 2. CONFIGURACIÓN BASE DEL SETPOINT
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        
        # Delegar velocidades a los límites internos de PX4
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        # 3. ACTUALIZAR POSICIÓN DE ORIGEN MIENTRAS ESTÉ DESARMADO
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_z = self.current_z
            self.locked_yaw = self.current_yaw

        # 4. ASIGNACIÓN DE OBJETIVOS SEGÚN EL ESTADO
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            # En Vuelo: Mantener X, Y, Yaw y subir a target_z
            setpoint.position = [self.locked_x, self.locked_y, self.target_z]
            setpoint.yaw = self.locked_yaw
        else:
            # En Tierra: Enviar posición actual para cumplir con la validación de PX4
            setpoint.position = [self.locked_x, self.locked_y, self.locked_z]
            setpoint.yaw = self.locked_yaw

        # 5. PUBLICAR SETPOINT
        self.trajectory_pub.publish(setpoint)

        # 6. MÁQUINA DE COMANDOS (a 20Hz: 20 ticks = 1 segundo)
        if self.counter == 20: 
            self.send_cmd(176, param1=1.0, param2=6.0)
            self.get_logger().info("Solicitando modo OFFBOARD...")

        if self.counter == 40: 
            self.send_cmd(400, param1=1.0)
            self.get_logger().info(f"ARMANDO MOTORES... Subiendo a {abs(self.target_z)}m y manteniendo Hover")

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
import rclpy 
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude
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
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos_profile)

        self.timer = self.create_timer(0.1, self.timer_cb) # 10 Hz
        self.counter = 0

        self.current_z = 0.0
        self.current_yaw = 0.0
        self.locked_yaw = None
        
        # Parámetros solicitados
        self.target_z = -2.5 # 2.5 metros de altura
        self.hold_duration = 5.0 # 8 segundos estable

        # Fases
        self.state = "INIT"
        self.hold_counter = 0

    def local_pos_cb(self, msg):
        self.current_z = msg.z

    def attitude_cb(self, msg):
        """
        Convertir Cuaterniones a Ángulo Euler Yaw para bloquear la rotación
        """
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # MODO OFFBOARD
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True 
        offboard.velocity = True
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # SETPOINT
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        # Bloqueamos el Yaw justo antes de despegar para que suba totalmente recto
        if self.state == "INIT" or self.state == "ARMING":
            self.locked_yaw = self.current_yaw

        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0
        
        # Valores por defecto para mantener XY bloqueado
        setpoint.velocity = [0.0, 0.0, float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]

        # MÁQUINA DE ESTADOS
        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) # Entrar a Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0) # Armar motores
                self.get_logger().info("ARMED - Ascendiendo a 2.5m")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            # Usar 0.0 absoluto en XY apoya al Optical Flow para subir totalmente recto
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, -0.8] # Inyección de potencia para subir

            if abs(self.current_z - self.target_z) < 0.15: 
                self.state = "HOLD"
                self.get_logger().info("HOLD POSITION - Manteniendo altura y posición por 8s")

        elif self.state == "HOLD":
            # Mantiene XY en 0, altura en -2.5, y quitamos la velocidad forzada para estabilizar
            setpoint.position = [0.0, 0.0, self.target_z]
            setpoint.velocity = [0.0, 0.0, float('nan')]
            
            self.hold_counter += 1
            pass_time = self.hold_counter * 0.1 # A 10Hz, 1 tick es 0.1s
            if pass_time >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("LANDING - Aterrizando suavemente")

        elif self.state == "LAND":
            setpoint.position = [0.0, 0.0, 0.0]
            setpoint.velocity = [0.0, 0.0, 0.4] # Velocidad de descenso controlada
            
            if self.current_z > -0.20:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0) # Desarmar motores
                self.get_logger().info("LANDING COMPLETED - Motores desarmados")

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
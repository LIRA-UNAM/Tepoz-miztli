import rclpy 
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

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
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos_profile)
        self.flow_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.flow_cb, qos_profile)

        self.timer = self.create_timer(0.1, self.timer_cb) # 10 Hz
        self.counter = 0

        # Variables de estado y posición
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        
        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None
        
        self.target_z = -1.2 # Altura objetivo (PX4 usa NED, -1.2 es hacia arriba)
        self.hold_duration = 3.0 
        self.hold_start_time = None # Para medir tiempo con ROS2 Clock

        self.state = "INIT"

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
         # Opcional: monitoreo de calidad del sensor
         self.get_logger().info(f"Calidad: {msg.signal_quality} | Distancia: {msg.current_distance:.2f}")
         if msg.signal_quality < 50:
             self.get_logger().warn(f"Baja calidad de Flow: {msg.signal_quality}")

    def timer_cb(self):
        now_nanos = self.get_clock().now().nanoseconds // 1000

        # Publicar Modo Offboard (Siempre activo para no perder el heartbeat)
        offboard = OffboardControlMode()
        offboard.timestamp = now_nanos
        offboard.position = True 
        offboard.velocity = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # Preparar Setpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now_nanos
        
        # Valores por defecto (NaN para que PX4 ignore lo que no usamos)
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = float('nan')

        # Bloqueo de coordenadas iniciales
        if self.state in ["INIT", "ARMING"]:
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        # Coordenadas de seguridad
        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # MAQUINA DE ESTADOS
        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) # OFFBOARD
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 40:
                self.send_cmd(400, param1=1.0) # ARM
                self.get_logger().info("ARMED - Iniciando Ascenso")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.yaw = safe_yaw
            
            # Si estamos cerca del objetivo, pasamos a HOLD y capturamos el tiempo
            if abs(self.current_z - self.target_z) < 0.15:
                self.state = "HOLD"
                self.hold_start_time = self.get_clock().now()
                self.get_logger().info("HOLD - Estabilizando posición")

        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.yaw = safe_yaw

            # Lógica de tiempo con ROS2 Clock
            if self.hold_start_time is not None:
                elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
                if elapsed >= self.hold_duration:
                    self.state = "LAND"
                    self.get_logger().info(f"HOLD Finalizado tras {elapsed:.1f}s. Aterrizando...")

        elif self.state == "LAND":
            setpoint.position = [safe_x, safe_y, 0.0]
            setpoint.yaw = safe_yaw
            setpoint.velocity = [0.0, 0.0, 0.4] # Descenso controlado
            
            # Detectar suelo (PX4 local z tiende a 0)
            if self.current_z > -0.15:
                self.send_cmd(400, param1=0.0) # DISARM
                self.state = "FINISHED"
                self.get_logger().info("LANDING COMPLETED - Motores apagados")

        elif self.state == "FINISHED":
            # Dejamos de enviar setpoints de posición para evitar conflictos
            return

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
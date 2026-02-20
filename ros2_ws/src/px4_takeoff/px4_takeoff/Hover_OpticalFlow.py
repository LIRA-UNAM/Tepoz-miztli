import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data  # <-- Importación crucial para solucionar el error de QoS
import time

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

class PrecisionFlightNode(Node):
    def __init__(self):
        super().__init__('precision_flight_node')

        # Publishers (La configuración por defecto de QoS 10 es suficiente para publicar hacia PX4)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers (Aquí aplicamos qos_profile_sensor_data para coincidir con BEST_EFFORT de PX4)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position', 
            self.position_callback, 
            qos_profile_sensor_data)
            
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.status_callback, 
            qos_profile_sensor_data)

        # Variables de estado
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.position_initialized = False

        # Referencias de despegue (para evitar drift y rotación)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        
        # Máquina de estados del vuelo
        self.flight_state = "INIT"
        self.target_altitude = -2.5  # NED (Negativo es hacia arriba)
        self.hover_duration = 8.0    # Segundos
        self.hover_start_time = 0.0

        # Timer a 50Hz (0.02 segundos)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Nodo iniciado. Esperando datos de posición local...")

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading
        
        # Guardar la posición inicial una sola vez cuando el sensor (Optical Flow) empiece a publicar
        if not self.position_initialized and msg.xy_valid and msg.z_valid:
            self.start_x = msg.x
            self.start_y = msg.y
            self.start_yaw = msg.heading
            self.position_initialized = True
            self.get_logger().info(f"Posición inicial bloqueada: X={self.start_x:.2f}, Y={self.start_y:.2f}, Yaw={self.start_yaw:.2f}")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        # No hacer nada hasta tener una lectura válida del Optical Flow
        if not self.position_initialized:
            return

        # Latido constante a 50Hz
        self.publish_offboard_control_mode()

        if self.flight_state == "INIT":
            # Enviar el setpoint actual antes de cambiar de modo (requisito de PX4)
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.current_z, self.start_yaw)
            
            # Cambiar a Offboard y Armar
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Motores Armados. Subiendo en línea recta...")
                self.flight_state = "CLIMBING"

        elif self.flight_state == "CLIMBING":
            # Subir manteniendo X, Y y Yaw fijos
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.target_altitude, self.start_yaw)
            
            # Tolerancia de 15 cm para considerar que llegó a la meta
            if abs(self.current_z - self.target_altitude) < 0.15:
                self.get_logger().info("Altura de 2.5m alcanzada. Manteniendo posición por 8 segundos...")
                self.hover_start_time = time.time()
                self.flight_state = "HOVERING"

        elif self.flight_state == "HOVERING":
            # Mantener posición estricta en el aire
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.target_altitude, self.start_yaw)
            
            if time.time() - self.hover_start_time >= self.hover_duration:
                self.get_logger().info("Tiempo completado. Iniciando aterrizaje suave...")
                self.flight_state = "LANDING"

        elif self.flight_state == "LANDING":
            # Comando automático de aterrizaje de PX4
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info("Aterrizaje exitoso y motores desarmados.")
                self.flight_state = "DONE"
                
        elif self.flight_state == "DONE":
            # Apagar el nodo limpiamente
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionFlightNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
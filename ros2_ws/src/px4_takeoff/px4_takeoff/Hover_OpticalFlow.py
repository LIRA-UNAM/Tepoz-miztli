import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

class PrecisionFlightNode(Node):
    def __init__(self):
        super().__init__('precision_flight_node')

        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

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

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.current_z = 0.0
        self.position_initialized = False

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        
        self.flight_state = "INIT"
        self.target_altitude = -2.5
        
        # Trabajaremos a 20 Hz (0.05s). PX4 lo prefiere y es más estable.
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.tick_counter = 0
        
        self.get_logger().info("Nodo iniciado. Esperando datos de posición local...")

    def position_callback(self, msg):
        self.current_z = msg.z
        
        if not self.position_initialized and msg.xy_valid and msg.z_valid:
            self.start_x = msg.x
            self.start_y = msg.y
            self.start_yaw = msg.heading
            self.position_initialized = True
            self.get_logger().info(f"Posición bloqueada: X={self.start_x:.2f}, Y={self.start_y:.2f}, Yaw={self.start_yaw:.2f}")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw, vz):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.velocity = [0.0, 0.0, float(vz)]
        
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yawspeed = math.nan
        
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        if not self.position_initialized:
            return

        self.publish_offboard_control_mode()
        self.tick_counter += 1

        if self.flight_state == "INIT":
            # Publicamos durante 2 segundos (40 ticks a 20Hz) para satisfacer la seguridad de PX4
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.current_z, self.start_yaw, vz=math.nan)
            
            if self.tick_counter > 40:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.flight_state = "ARMING"
                self.tick_counter = 0

        elif self.flight_state == "ARMING":
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.current_z, self.start_yaw, vz=math.nan)
            
            # Esperamos 1 segundo más (20 ticks) para asentar el modo Offboard antes de armar
            if self.tick_counter > 20:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                
                if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info("ARMED. Ascendiendo recto con potencia (-0.8 m/s)...")
                    self.flight_state = "CLIMBING"
                    self.tick_counter = 0

        elif self.flight_state == "CLIMBING":
            # Forzamos la velocidad de subida (-0.8) manteniendo el XY bloqueado
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.target_altitude, self.start_yaw, vz=-0.8)
            
            if abs(self.current_z - self.target_altitude) < 0.20:
                self.get_logger().info("HOLD POSITION. 2.5 metros alcanzados. Contando 8 segundos...")
                self.flight_state = "HOVERING"
                self.tick_counter = 0

        elif self.flight_state == "HOVERING":
            # Desactivamos el forzado de velocidad (NaN) para que solo pelee por mantener la posición
            self.publish_trajectory_setpoint(self.start_x, self.start_y, self.target_altitude, self.start_yaw, vz=math.nan)
            
            # 8 segundos a 20Hz son 160 ticks
            if self.tick_counter > 160:
                self.get_logger().info("LANDING. Iniciando descenso suave...")
                self.flight_state = "LANDING"

        elif self.flight_state == "LANDING":
            # Bajamos a Z=0.0 forzando una velocidad controlada de 0.4 m/s
            self.publish_trajectory_setpoint(self.start_x, self.start_y, 0.0, self.start_yaw, vz=0.4)
            
            # Si el dron ya reporta estar casi en el piso (-0.2m)
            if self.current_z > -0.20:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.get_logger().info("LANDING COMPLETED. Motores desarmados.")
                self.flight_state = "DONE"
                
        elif self.flight_state == "DONE":
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
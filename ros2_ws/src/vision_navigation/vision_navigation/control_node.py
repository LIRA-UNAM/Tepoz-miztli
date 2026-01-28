import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus

class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_offboard')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publicadores
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Timer (PX4 requiere comandos a > 2Hz, usaremos 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variables de control (Aquí inyectarás los datos de YOLO)
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0 # Cuidado con Z positivo = bajar
        self.target_yaw_speed = 0.0

    def timer_callback(self):
        # 1. Publicar Modo Offboard (Heartbeat)
        msg_mode = OffboardControlMode()
        msg_mode.position = False
        msg_mode.velocity = True      # Queremos controlar velocidad
        msg_mode.acceleration = False
        msg_mode.attitude = False
        msg_mode.body_rate = False
        msg_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg_mode)

        # 2. Publicar Setpoint de Velocidad
        msg_traj = TrajectorySetpoint()
        msg_traj.position = [float('nan'), float('nan'), float('nan')] # Ignorar posición
        msg_traj.velocity = [self.target_vx, self.target_vy, self.target_vz]
        msg_traj.yaw = float('nan') # Ignorar ángulo absoluto
        msg_traj.yawspeed = self.target_yaw_speed
        msg_traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.publisher_trajectory.publish(msg_traj)

        # LOGICA YOLO AQUÍ:
        # self.target_vx = (error_x_imagen) * ganancia_kp
        # self.target_vz = (error_y_imagen) * ganancia_kp

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
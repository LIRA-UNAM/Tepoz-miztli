import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_sensor_data
import time

class AutoNavigationNode(Node):
    def __init__(self):
        super().__init__('auto_navigation_node')

        #configuración inicial
        self.target_altitude = 2.0
        self.img_width = 640.0 
        self.target_x = self.img_width / 2.0
        self.target_area = 80000.0
        
        # Ganancias de PID
        self.kp_yaw = 0.002
        self.kp_alt = 0.005
        self.kp_fwd = 0.00005

        # Estado del drone
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.is_gate_detected = False
        self.last_detection_time = 0
        self.latest_yolo_msg = None

        # Estados usados
        # 0: Inicialización
        # 1: Despegando
        # 2: Buscando y siguiendo gate
        self.mission_state = 0 
        self.last_req = time.time()

        # Subscriptores
        self.coord_sub = self.create_subscription(Point, 'yolo/coordinates', self.coord_callback, 10)
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        # Posición del drone actual
        self.pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data)

        # Publicadores
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Clientes para el armado y desarmado / control de modos de vuelo
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio arming...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio set_mode...')

        # Loop principal (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.cmd_vel = Twist()

        self.get_logger().info("Nodo de Navegación Autónoma Iniciado")

    # Callbacks
    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def coord_callback(self, msg):
        self.last_detection_time = time.time()
        self.is_gate_detected = True
        self.latest_yolo_msg = msg

    def control_loop(self):
        #MAVROS necesita estar publicnado para que pueda mandar a OFFBOARD
        if self.mission_state == 0:
            self.handle_initialization()
        elif self.mission_state == 1:
            self.handle_takeoff()
        elif self.mission_state == 2:
            self.handle_tracking()

        # Velocidad calculada
        self.vel_pub.publish(self.cmd_vel)

    def handle_initialization(self):
        self.cmd_vel = Twist()
        
        if self.current_state.mode != "OFFBOARD" and (time.time() - self.last_req) > 5.0:
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)
            self.last_req = time.time()
            self.get_logger().info("Intentando modo OFFBOARD")

        elif self.current_state.mode == "OFFBOARD" and not self.current_state.armed and (time.time() - self.last_req) > 5.0:
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            self.last_req = time.time()
            self.get_logger().info("Armando motores")

        if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
            self.get_logger().info("Dron listo. Iniciando despegue")
            self.mission_state = 1

    def handle_takeoff(self):
        # Altura actual
        current_z = self.current_pose.pose.position.z

        if current_z < self.target_altitude:
            self.cmd_vel.linear.z = 0.5
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.get_logger().info(f"Subiendo... Altura: {current_z:.2f}")
        else:
            self.cmd_vel.linear.z = 0.0
            self.get_logger().info("Altura alcanzada. Buscando Gates.")
            self.mission_state = 2

    def handle_tracking(self):
        if time.time() - self.last_detection_time > 1.0:
            self.cmd_vel = Twist() # Se queda en modo hover
            self.get_logger().info("Buscando gate")
            return

        msg = self.latest_yolo_msg
        
        error_x = self.target_x - msg.x
        # Seguir al gate en altura también
        # error_y = (480/2) - msg.y 
        # self.cmd_vel.linear.z = error_y * self.kp_alt 

        # Mantenerse fijo a 2 metros y solo moverse en X y Y
        altitude_error = self.target_altitude - self.current_pose.pose.position.z
        self.cmd_vel.linear.z = altitude_error * 0.5 # Corrector simple de altura P
        
        area = msg.z
        error_area = self.target_area - area
        self.cmd_vel.angular.z = error_x * self.kp_yaw
        
        if abs(error_x) < 100:
            self.cmd_vel.linear.x = error_area * self.kp_fwd
            self.cmd_vel.linear.x = min(self.cmd_vel.linear.x, 0.8)
        else:
            self.cmd_vel.linear.x = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
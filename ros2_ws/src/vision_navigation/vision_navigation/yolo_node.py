import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time

class AutoNavigationNode(Node):
    def __init__(self):
        super().__init__('auto_navigation_node')

        # --- CONFIGURACIÓN DE OBJETIVO ---
        self.target_altitude = 2.0  # Metros a los que quieres subir
        self.img_width = 640.0 
        self.target_x = self.img_width / 2.0
        self.target_area = 80000.0
        
        # --- GANANCIAS PID ---
        self.kp_yaw = 0.002
        self.kp_alt = 0.005
        self.kp_fwd = 0.00005

        # --- VARIABLES DE ESTADO ---
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.is_gate_detected = False
        self.last_detection_time = 0
        
        # Estados de la misión
        # 0: Espera/Inicialización
        # 1: Despegando (Subiendo a 2m)
        # 2: Buscando/Siguiendo Gate (YOLO)
        self.mission_state = 0 
        self.last_req = time.time() # Para controlar los intentos de servicio

        # --- SUSCRIPTORES ---
        self.coord_sub = self.create_subscription(Point, 'yolo/coordinates', self.coord_callback, 10)
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        # Necesitamos la posición local para saber la altura
        self.pose_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_callback, 10)

        # --- PUBLICADOR ---
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # --- CLIENTES DE SERVICIO (Para Armar y poner Modo) ---
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # Esperar a que los servicios estén listos
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio arming...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio set_mode...')

        # Loop principal (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.cmd_vel = Twist()

        self.get_logger().info("Nodo de Navegación Autónoma Iniciado")

    # --- CALLBACKS ---
    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def coord_callback(self, msg):
        self.last_detection_time = time.time()
        self.is_gate_detected = True
        self.latest_yolo_msg = msg # Guardamos el dato para usarlo en el loop

    # --- LÓGICA PRINCIPAL ---
    def control_loop(self):
        # MAVROS necesita recibir setpoints continuamente antes de cambiar a OFFBOARD
        # Así que siempre publicamos algo, aunque sea ceros.
        
        if self.mission_state == 0:
            self.handle_initialization()
        elif self.mission_state == 1:
            self.handle_takeoff()
        elif self.mission_state == 2:
            self.handle_tracking()

        # Publicar la velocidad calculada
        self.vel_pub.publish(self.cmd_vel)

    def handle_initialization(self):
        # Enviamos velocidad 0 para "calentar" el link con MAVROS
        self.cmd_vel = Twist()
        
        # Intentar poner modo OFFBOARD cada 5 segundos si no está puesto
        if self.current_state.mode != "OFFBOARD" and (time.time() - self.last_req) > 5.0:
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)
            self.last_req = time.time()
            self.get_logger().info("Intentando modo OFFBOARD...")

        # Intentar ARMAR motores si ya es OFFBOARD y no está armado
        elif self.current_state.mode == "OFFBOARD" and not self.current_state.armed and (time.time() - self.last_req) > 5.0:
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            self.last_req = time.time()
            self.get_logger().info("Intentando ARMAR motores...")

        # Si ya estamos en OFFBOARD y ARMADOS, pasamos a despegar
        if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
            self.get_logger().info("Dron listo. Iniciando despegue...")
            self.mission_state = 1

    def handle_takeoff(self):
        # Obtener altura actual
        current_z = self.current_pose.pose.position.z

        if current_z < self.target_altitude:
            # Si estamos abajo de 2m, subimos
            self.cmd_vel.linear.z = 0.5  # Subir a 0.5 m/s
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            # Pequeño log para ver progreso
            # self.get_logger().info(f"Subiendo... Altura: {current_z:.2f}")
        else:
            # Ya llegamos a la altura
            self.cmd_vel.linear.z = 0.0
            self.get_logger().info("Altura alcanzada. Cambiando a modo TRACKING.")
            self.mission_state = 2

    def handle_tracking(self):
        # Lógica de YOLO (similar a la anterior)
        
        # Seguridad: Si YOLO deja de ver cosas por 1 seg, detenerse
        if time.time() - self.last_detection_time > 1.0:
            self.cmd_vel = Twist() # Hover en el lugar
            # self.get_logger().info("Buscando gate...")
            return

        # Recuperar datos guardados
        msg = self.latest_yolo_msg
        
        error_x = self.target_x - msg.x
        # Nota: Aquí ignoramos un poco la altura del gate y priorizamos mantener la altura de vuelo estable
        # O puedes mezclar ambos (mantener 2m pero corregir un poco si el gate está muy alto/bajo)
        
        # Opción A: Seguir al gate en altura también
        # error_y = (480/2) - msg.y 
        # self.cmd_vel.linear.z = error_y * self.kp_alt 

        # Opción B: Mantenerse fijo a 2 metros y solo moverse en X y Yaw (Más fácil para empezar)
        altitude_error = self.target_altitude - self.current_pose.pose.position.z
        self.cmd_vel.linear.z = altitude_error * 0.5 # Corrector simple de altura P
        
        area = msg.z
        error_area = self.target_area - area

        # Control
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
import math 
import time 
import rclpy 
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

#from geometry_msgs.msg import PoseArray, Point

class Mision4LazoAbierto(Node):
    def __init__(self):
        super().__init__('mision4_lazo_abierto')

        pub_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        sub_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        #Publicadores

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            pub_qos)

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            pub_qos)

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            pub_qos)


        #Subscriptores
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_cb,
            sub_qos)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            sub_qos)

        self.flow_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self.flow_cb,
            sub_qos)

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.counter = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.current_distance = 0.0

        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        self.target_altitude = 1.5
        self.target_z = -1.5
        self.hold_duration = 3.0

        # Control de estados
        self.state = "INIT"
        self.hold_start_time  = None
        self.stable_ticks     = 0
        self.stable_ticks_needed = 10

        self.hold_counter = 0


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

        if self.counter % 10 == 0:
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {self.current_distance:.4f} m"
            )     

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000
        
        # MODO OFFBOARD (siempre publicar)
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = True
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        if self.state in ["INIT", "ARMING"]:
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0        


        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = "ARMING"

        elif self.state == "ARMING":

            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info("ARMED")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            error_z = abs(self.current_z - self.target_z)

            if error_z > 0.4:
                vz = -0.8
            elif error_z > 0.1:
                vz = -0.3
            else:
                vz = 0.0
            
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [float('nan'), float('nan'), vz]
            
            if error_z < 0.15:
                self.state = "HOLD"
                self.hold_start_time = time.time()

        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.velocity = [float('nan'), float('nan'), float('nan')]
            
            elapsed = time.time() - self.hold_start_time 
            
            if elapsed > 5.0:
                self.state = "SEARCH"
                self.start_time = time.time()
                self.get_logger().info("SEARCHING GATE")
        
        elif self.state == "SEARCH":

            setpoint.position = [float('nan'), float('nan'), self.target_z]
            setpoint.velocity = [0.0, 0.1, float('nan')]

            elapsed = time.time() - self.start_time

            if elapsed > 5.0:
                self.state = "CROSS_GATE"
                self.start_time = time.time()

        elif self.state == "CROSS_GATE":

            setpoint.position = [float('nan'), float('nan'), self.target_z]
            setpoint.velocity = [0.2, 0.0, float('nan')]

            elapsed = time.time() - self.start_time

            if elapsed > 4.0:
                setpoint.velocity = [0.1, 0.0, float('nan')]
            
            if elapsed > 6.0:
                self.state = "TURN1"
                self.start_time = time.time()

        elif self.state == "TURN1":

            setpoint.position = [float('nan'), float('nan'), self.target_z]

            setpoint.yaw = 1.57
            setpoint.yawspeed = 0.25

            if abs(self.current_yaw - 1.57) < 0.1:
                self.start_time = time.time()
                self.state = "LAND"

        elif self.state == "LAND":
            setpoint.position = [float('nan'), float('nan'), 0.0]
            setpoint.velocity = [0.0, 0.0, float('nan')]

            setpoint.yaw = 1.57

            if self.current_distance < 0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("LANDING COMPLETED")
        
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
    node = Mision4LazoAbierto()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()




# import time
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# from px4_msgs.msg import (
#     OffboardControlMode,
#     TrajectorySetpoint,
#     VehicleCommand,
#     VehicleLocalPosition,
#     VehicleAttitude,
#     DistanceSensor
# )


# class Mision4LazoAbierto(Node):
#     def __init__(self):
#         super().__init__('mision4_lazo_abierto')

#         pub_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         sub_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Publicadores
#         self.offboard_pub = self.create_publisher(
#             OffboardControlMode,
#             '/fmu/in/offboard_control_mode',
#             pub_qos)

#         self.trajectory_pub = self.create_publisher(
#             TrajectorySetpoint,
#             '/fmu/in/trajectory_setpoint',
#             pub_qos)

#         self.cmd_pub = self.create_publisher(
#             VehicleCommand,
#             '/fmu/in/vehicle_command',
#             pub_qos)

#         # Subscriptores
#         self.local_pos_sub = self.create_subscription(
#             VehicleLocalPosition,
#             '/fmu/out/vehicle_local_position',
#             self.local_pos_cb,
#             sub_qos)

#         self.attitude_sub = self.create_subscription(
#             VehicleAttitude,
#             '/fmu/out/vehicle_attitude',
#             self.attitude_cb,
#             sub_qos)

#         self.flow_sub = self.create_subscription(
#             DistanceSensor,
#             '/fmu/out/distance_sensor',
#             self.flow_cb,
#             sub_qos)

#         self.timer = self.create_timer(0.05, self.timer_cb)

#         self.counter = 0

#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_z = 0.0
#         self.current_yaw = 0.0

#         self.current_distance = 0.0

#         self.locked_x = None
#         self.locked_y = None
#         self.locked_yaw = None

#         self.target_altitude = 1.5
#         self.target_z = -1.5
#         self.hold_duration = 3.0

#         # Control de estados
#         self.state = "INIT"
#         self.hold_start_time = None
#         self.stable_ticks = 0
#         self.stable_ticks_needed = 10

#     # ------------------------------------------------------------------
#     # Callbacks
#     # ------------------------------------------------------------------

#     def local_pos_cb(self, msg):
#         self.current_x = msg.x
#         self.current_y = msg.y
#         self.current_z = msg.z

#     def attitude_cb(self, msg):
#         q = msg.q
#         siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
#         cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

#     def flow_cb(self, msg):
#         self.current_distance = msg.current_distance

#         if self.counter % 10 == 0:
#             self.get_logger().info(
#                 f"Calidad: {msg.signal_quality} | "
#                 f"Distancia: {self.current_distance:.4f} m"
#             )

#     # ------------------------------------------------------------------
#     # Helper: publica el modo offboard activando solo lo necesario
#     # ------------------------------------------------------------------

#     def publish_offboard_mode(self, position=False, velocity=False):
#         offboard = OffboardControlMode()
#         offboard.timestamp    = self.get_clock().now().nanoseconds // 1000
#         offboard.position     = position
#         offboard.velocity     = velocity
#         offboard.acceleration = False
#         self.offboard_pub.publish(offboard)

#     # ------------------------------------------------------------------
#     # Timer principal
#     # ------------------------------------------------------------------

#     def timer_cb(self):
#         now = self.get_clock().now().nanoseconds // 1000

#         # Preparar setpoint vacío (NaN = ignorar ese campo)
#         setpoint = TrajectorySetpoint()
#         setpoint.timestamp    = now
#         setpoint.position     = [float('nan'), float('nan'), float('nan')]
#         setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
#         setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
#         setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
#         setpoint.yaw          = float('nan')
#         setpoint.yawspeed     = float('nan')

#         # Bloquear posición/yaw al inicio para no derivar
#         if self.state in ["INIT", "ARMING"]:
#             self.locked_x   = self.current_x
#             self.locked_y   = self.current_y
#             self.locked_yaw = self.current_yaw

#         safe_x   = self.locked_x   if self.locked_x   is not None else 0.0
#         safe_y   = self.locked_y   if self.locked_y   is not None else 0.0
#         safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

#         # ----------------------------------------------------------
#         # Máquina de estados
#         # ----------------------------------------------------------

#         if self.state == "INIT":
#             self.publish_offboard_mode(position=True)

#             if self.counter > 20:
#                 self.send_cmd(176, param1=1.0, param2=6.0)
#                 self.state = "ARMING"

#         elif self.state == "ARMING":
#             self.publish_offboard_mode(position=True)

#             if self.counter > 30:
#                 self.send_cmd(400, param1=1.0)
#                 self.get_logger().info("ARMED")
#                 self.state = "TAKEOFF"

#         elif self.state == "TAKEOFF":
#             self.publish_offboard_mode(position=True)
#             setpoint.velocity = []

#             setpoint.position = [safe_x, safe_y, self.target_z]
#             setpoint.yaw      = safe_yaw

#             error_alt = abs(self.current_distance - self.target_altitude)

#             if error_alt < 0.40:
#                 self.stable_ticks += 1
#             else:
#                 self.stable_ticks = 0

#             if self.counter % 10 == 0:
#                 self.get_logger().info(
#                     f"TAKEOFF | dist={self.current_distance:.2f} m "
#                     f"target={self.target_altitude:.2f} m "
#                     f"err={error_alt:.2f} m "
#                     f"stable={self.stable_ticks}/{self.stable_ticks_needed}"
#                 )

#             if self.stable_ticks >= self.stable_ticks_needed:
#                 self.state = "HOLD"
#                 self.get_logger().info(
#                     f"HOLD POSITION - Estable en {self.current_distance:.2f} m "
#                     f"(target={self.target_altitude:.2f} m, err={error_alt:.2f} m)"
#                 )

#         elif self.state == "HOLD":
#             self.publish_offboard_mode(position=True)

#             setpoint.position = [safe_x, safe_y, self.target_z]
#             setpoint.yaw      = safe_yaw

#             if self.hold_start_time is None:
#                 self.hold_start_time = self.get_clock().now()

#             elapsed = (
#                 self.get_clock().now() - self.hold_start_time
#             ).nanoseconds / 1e9

#             if self.counter % 10 == 0:
#                 self.get_logger().info(
#                     f"HOLD {elapsed:.1f}s / {self.hold_duration}s | "
#                     f"dist={self.current_distance:.2f} m"
#                 )

#             if elapsed >= self.hold_duration:
#                 self.state = "SEARCH"
#                 self.start_time = time.time()
#                 self.get_logger().info("SEARCHING GATE")

#         elif self.state == "SEARCH":
#             # Movimiento lateral (derecha en NED = +Y)
#             self.publish_offboard_mode(velocity=True)

#             setpoint.velocity = [0.0, 0.1, 0.0]
#             setpoint.yaw      = safe_yaw

#             elapsed = time.time() - self.start_time

#             if elapsed > 8.0:
#                 self.state = "CROSS_GATE"
#                 self.start_time = time.time()
#                 self.get_logger().info("CROSS GATE")

#         elif self.state == "CROSS_GATE":
#             # Avance frontal (+X en NED)
#             self.publish_offboard_mode(velocity=True)

#             if (time.time() - self.start_time) < 4.0:
#                 setpoint.velocity = [0.2, 0.0, 0.0]
#             else:
#                 setpoint.velocity = [0.1, 0.0, 0.0]

#             setpoint.yaw = safe_yaw

#             elapsed = time.time() - self.start_time

#             if elapsed > 6.0:
#                 self.state = "TURN1"
#                 self.start_time = time.time()
#                 self.get_logger().info("TURN1")

#         elif self.state == "TURN1":
#             # Giro a 90° (pi/2 rad)
#             self.publish_offboard_mode(position=True)

#             setpoint.position = [float('nan'), float('nan'), self.target_z]
#             setpoint.yaw      = 1.57
#             setpoint.yawspeed = 0.25

#             if abs(self.current_yaw - 1.57) < 0.1:
#                 self.state = "LAND"
#                 self.get_logger().info("LANDING")

#         elif self.state == "LAND":
#             # Descenso por velocidad
#             self.publish_offboard_mode(velocity=True)

#             setpoint.velocity = [0.0, 0.0, 0.2]   # +Z = bajar en NED
#             setpoint.yaw      = safe_yaw

#             if self.current_distance < 0.15:
#                 self.state = "LANDED"
#                 self.send_cmd(400, param1=0.0)
#                 self.get_logger().info("LANDING COMPLETED")

#         elif self.state == "LANDED":
#             # Seguir publicando para no perder offboard, pero sin moverse
#             self.publish_offboard_mode(position=True)

#         self.trajectory_pub.publish(setpoint)
#         self.counter += 1

#     # ------------------------------------------------------------------
#     # Helper: enviar VehicleCommand
#     # ------------------------------------------------------------------

#     def send_cmd(self, command, param1=0.0, param2=0.0):
#         msg = VehicleCommand()
#         msg.timestamp        = self.get_clock().now().nanoseconds // 1000
#         msg.command          = command
#         msg.param1           = float(param1)
#         msg.param2           = float(param2)
#         msg.target_system    = 1
#         msg.target_component = 1
#         msg.from_external    = True
#         self.cmd_pub.publish(msg)


# def main():
#     rclpy.init()
#     node = Mision4LazoAbierto()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

        






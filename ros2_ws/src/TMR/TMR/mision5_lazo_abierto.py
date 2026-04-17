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
    DistanceSensor,
)

TARGET_ALTITUDE  = 1.65       # altura objetivo medida por sensor [m]
TARGET_Z         = -1.65       # en NED (negativo = arriba)
HOLD_DURATION    = 3.0        # espera tras despegue [s]

MOVE_SPEED       = 0.2        # velocidad de desplazamiento [m/s]
MOVE_RIGHT_TIME  = 5.2        # ← CAMBIA ESTO para más/menos desplazamiento a la derecha [s]
MOVE_FWD_TIME    = 5.3        # ← CAMBIA ESTO para más/menos avance al frente [s]


class SimpleMission2(Node):
    def __init__(self):
        super().__init__('simple_mission_2')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # ── Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, sub_qos)
        self.flow_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.flow_cb, sub_qos)
    
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_distance = 0.0

        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        # ── Tiempos de inicio de cada movimiento
        self.move_right_start = None
        self.move_fwd_start = None

        # ── Contadores y estado
        self.state = 'INIT'
        self.counter = 0
        self.stable_ticks = 0
        self.hold_start = None

        self.timer = self.create_timer(0.05, self.timer_cb)   # 20 Hz

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = True
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position = [float('nan')] * 3
        setpoint.velocity = [float('nan')] * 3
        setpoint.acceleration = [float('nan')] * 3
        setpoint.jerk = [float('nan')] * 3
        setpoint.yaw = float('nan')
        setpoint.yawspeed = float('nan')

        if self.state in ('INIT', 'ARMING'):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x   = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y   = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        #MAQUINA DE ESTADOS

        if self.state == 'INIT':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = 'ARMING'
                self.get_logger().info('→ ARMING')

        elif self.state == 'ARMING':
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw

            if self.counter > 30:
                self.send_cmd(400, param1=1.0)
                self.get_logger().info(f'→ TAKEOFF | objetivo {TARGET_ALTITUDE:.1f} m')
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            # Sube hasta TARGET_ALTITUDE validado por sensor de distancia
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw

            err_alt = abs(self.current_distance - TARGET_ALTITUDE)

            self.stable_ticks = self.stable_ticks + 1 if err_alt < 0.35 else 0
            if self.stable_ticks >= 10:
                self.hold_start = self.get_clock().now()
                self.get_logger().info(f'Altura estable {self.current_distance:.2f} m → HOLD')
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            # Espera fija en el aire antes de empezar movimientos
            setpoint.position = [safe_x, safe_y, TARGET_Z]
            setpoint.yaw = safe_yaw

            elapsed = self._elapsed_s(self.hold_start, self.get_clock())

            if elapsed >= HOLD_DURATION:
                self.move_right_start = self.get_clock().now()
                self.get_logger().info(f'→ MOVE_RIGHT durante {MOVE_RIGHT_TIME:.1f} s')
                self.state = 'MOVE_RIGHT'

        elif self.state == 'MOVE_RIGHT':
            # ── Velocidad pura a la derecha (relativa al yaw bloqueado)
            # Derecha en body frame = (-sin(yaw), cos(yaw)) en NED
            yaw = safe_yaw
            setpoint.velocity[0] = -math.sin(yaw) * MOVE_SPEED
            setpoint.velocity[1] =  math.cos(yaw) * MOVE_SPEED
            setpoint.velocity[2] = 0.0
            setpoint.yaw = yaw

            elapsed = self._elapsed_s(self.move_right_start, self.get_clock())

            if elapsed >= MOVE_RIGHT_TIME:
                # Al terminar, congela la posición actual como nueva referencia
                self.locked_x = self.current_x
                self.locked_y = self.current_y
                self.move_fwd_start = self.get_clock().now()
                self.get_logger().info(f'→ MOVE_FORWARD durante {MOVE_FWD_TIME:.1f} s')
                self.state = 'MOVE_FORWARD'

        elif self.state == 'MOVE_FORWARD':
            # ── Velocidad pura al frente (relativa al yaw bloqueado)
            # Frente en body frame = (cos(yaw), sin(yaw)) en NED
            yaw = safe_yaw
            setpoint.velocity[0] = math.cos(yaw) * MOVE_SPEED
            setpoint.velocity[1] = math.sin(yaw) * MOVE_SPEED
            setpoint.velocity[2] = 0.0
            setpoint.yaw = yaw

            elapsed = self._elapsed_s(self.move_fwd_start, self.get_clock())

            if elapsed >= MOVE_FWD_TIME:
                self.locked_x = self.current_x
                self.locked_y = self.current_y
                self.get_logger().info('→ DONE | Misión completada, manteniendo posición')
                self.state = 'DONE'

        elif self.state == 'DONE':
            # Mantiene la última posición indefinidamente
            setpoint.position = [self.locked_x, self.locked_y, TARGET_Z]
            setpoint.yaw = safe_yaw

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    # ══════════════════════════════ HELPERS ══════════════════════════════════

    @staticmethod
    def _elapsed_s(start_time, clock) -> float:
        if start_time is None:
            return 0.0
        return (clock.now() - start_time).nanoseconds / 1e9

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main():
    rclpy.init()
    node = SimpleMission2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción detectada. Cerrando nodo')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
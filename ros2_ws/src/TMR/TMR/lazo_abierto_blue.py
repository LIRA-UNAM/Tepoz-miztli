#Hola
import math

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

# ─────────────────────────────────────────────
#  Parámetros editables
# ─────────────────────────────────────────────
TARGET_ALTITUDE = 1.3       # metros (sensor de distancia)
TARGET_Z        = -1.3      # metros NED  (negativo = arriba)
HOLD_DURATION   = 3.0       # segundos en cada hold

SLIDE_DIST      = 1.4       # metros a la DERECHA (cuerpo del dron)
FWD_DIST        = 4.6       # metros al FRENTE    (cuerpo del dron)

# Tolerancia de posición para confirmar llegada
POS_TOL         = 0.12      # metros
STABLE_TICKS    = 20        # ciclos a 20 Hz ≈ 1 s confirmando posición
# ─────────────────────────────────────────────


class SimpleMission(Node):

    def __init__(self):
        super().__init__('simple_mission')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.offboard_pub    = self.create_publisher(OffboardControlMode,  '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub  = self.create_publisher(TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',   pub_qos)
        self.cmd_pub         = self.create_publisher(VehicleCommand,        '/fmu/in/vehicle_command',       pub_qos)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.create_subscription(VehicleAttitude,       '/fmu/out/vehicle_attitude',       self.attitude_cb,  sub_qos)
        self.create_subscription(DistanceSensor,        '/fmu/out/distance_sensor',        self.distance_cb,  sub_qos)

        # ── Estado interno ───────────────────────────────────────────────
        self.x   = 0.0;  self.y   = 0.0;  self.z   = 0.0
        self.yaw = 0.0
        self.dist = 0.0          # distancia al suelo (sensor)

        # Posición/yaw bloqueados al armar (origen de la misión)
        self.origin_x   = None
        self.origin_y   = None
        self.origin_yaw = None

        # Waypoints calculados una vez al salir de HOLD
        self.wp_slide_x   = None   # target SLIDE_RIGHT
        self.wp_slide_y   = None
        self.wp_fwd_x     = None   # target FORWARD
        self.wp_fwd_y     = None

        # Contadores y timers
        self.counter       = 0
        self.stable_ticks  = 0
        self.pos_stable    = 0
        self.hold_start    = None

        self.state = 'INIT'

        self.timer = self.create_timer(0.05, self.loop)   # 20 Hz
        self.get_logger().info('SimpleMission iniciada')

    # ─────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.x = msg.x;  self.y = msg.y;  self.z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        self.yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
        )

    def distance_cb(self, msg: DistanceSensor):
        self.dist = msg.current_distance

    # ─────────────────────────────────────────────
    #  Loop principal
    # ─────────────────────────────────────────────

    def loop(self):
        now = self.get_clock().now().nanoseconds // 1000

        # ── OffboardControlMode (siempre) ─────────────────────────────
        ocm = OffboardControlMode()
        ocm.timestamp    = now
        ocm.position     = True
        ocm.velocity     = False
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        self.offboard_pub.publish(ocm)

        # ── Setpoint base (todo NaN = ignorar) ───────────────────────
        sp = TrajectorySetpoint()
        sp.timestamp    = now
        sp.position     = [float('nan')] * 3
        sp.velocity     = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk         = [float('nan')] * 3
        sp.yaw          = float('nan')
        sp.yawspeed     = float('nan')

        # ── Bloquear origen mientras está en tierra ───────────────────
        if self.state in ('INIT', 'ARMING'):
            self.origin_x   = self.x
            self.origin_y   = self.y
            self.origin_yaw = self.yaw

        ox  = self.origin_x   or 0.0
        oy  = self.origin_y   or 0.0
        oyw = self.origin_yaw or 0.0

        # ══════════════════════════════════════════
        #  Máquina de estados
        # ══════════════════════════════════════════

        if self.state == 'INIT':
            # Mínimo 20 ciclos publicando antes de pedir Offboard
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oyw
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)   # MAV_CMD_DO_SET_MODE → Offboard
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oyw
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)               # MAV_CMD_COMPONENT_ARM_DISARM
                self.get_logger().info(f'ARMED — subiendo a {TARGET_ALTITUDE:.1f} m')
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oyw
            err = abs(self.dist - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err < 0.25 else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'TAKEOFF | dist={self.dist:.2f} m  err={err:.2f} m  stable={self.stable_ticks}/10'
                )
            if self.stable_ticks >= 10:
                self.hold_start = self.get_clock().now()
                self.get_logger().info(f'Altitud estable → HOLD')
                self.state = 'HOLD'

        elif self.state == 'HOLD':
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oyw
            elapsed = self._elapsed(self.hold_start)
            if self.counter % 10 == 0:
                self.get_logger().info(f'HOLD {elapsed:.1f}/{HOLD_DURATION:.0f}s | dist={self.dist:.2f} m')
            if elapsed >= HOLD_DURATION:
                # ── Calcular los dos waypoints en NED ─────────────────
                yaw = self.origin_yaw

                # Derecha en cuerpo → NED:  right_NED = [-sin(yaw), cos(yaw)]  ← ver nota(*)
                self.wp_slide_x = ox + (-math.sin(yaw)) * SLIDE_DIST
                self.wp_slide_y = oy + ( math.cos(yaw)) * SLIDE_DIST

                # Adelante en cuerpo → NED: fwd_NED = [cos(yaw), sin(yaw)]
                self.wp_fwd_x = self.wp_slide_x + math.cos(yaw) * FWD_DIST
                self.wp_fwd_y = self.wp_slide_y + math.sin(yaw) * FWD_DIST

                self.pos_stable = 0
                self.get_logger().info(
                    f'SLIDE_RIGHT — target NED ({self.wp_slide_x:.2f}, {self.wp_slide_y:.2f}) | '
                    f'yaw={math.degrees(yaw):.1f}°'
                )
                self.state = 'SLIDE_RIGHT'

        elif self.state == 'SLIDE_RIGHT':
            sp.position = [self.wp_slide_x, self.wp_slide_y, TARGET_Z]
            sp.yaw      = self.origin_yaw
            err_xy = math.hypot(self.x - self.wp_slide_x, self.y - self.wp_slide_y)
            self.pos_stable = self.pos_stable + 1 if err_xy < POS_TOL else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'SLIDE_RIGHT | err={err_xy:.3f} m  stable={self.pos_stable}/{STABLE_TICKS}'
                )
            if self.pos_stable >= STABLE_TICKS:
                self.hold_start = self.get_clock().now()
                self.get_logger().info(f'Posición derecha alcanzada → HOLD_SLIDE')
                self.state = 'HOLD_SLIDE'

        elif self.state == 'HOLD_SLIDE':
            sp.position = [self.wp_slide_x, self.wp_slide_y, TARGET_Z]
            sp.yaw      = self.origin_yaw
            elapsed = self._elapsed(self.hold_start)
            if self.counter % 10 == 0:
                self.get_logger().info(f'HOLD_SLIDE {elapsed:.1f}/{HOLD_DURATION:.0f}s')
            if elapsed >= HOLD_DURATION:
                self.pos_stable = 0
                self.get_logger().info(
                    f'FORWARD — target NED ({self.wp_fwd_x:.2f}, {self.wp_fwd_y:.2f})'
                )
                self.state = 'FORWARD'

        elif self.state == 'FORWARD':
            sp.position = [self.wp_fwd_x, self.wp_fwd_y, TARGET_Z]
            sp.yaw      = self.origin_yaw
            err_xy = math.hypot(self.x - self.wp_fwd_x, self.y - self.wp_fwd_y)
            self.pos_stable = self.pos_stable + 1 if err_xy < POS_TOL else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'FORWARD | err={err_xy:.3f} m  stable={self.pos_stable}/{STABLE_TICKS}'
                )
            if self.pos_stable >= STABLE_TICKS:
                self.get_logger().info('Posición final alcanzada → HOLD_FINAL')
                self.state = 'HOLD_FINAL'

        elif self.state == 'HOLD_FINAL':
            sp.position = [self.wp_fwd_x, self.wp_fwd_y, TARGET_Z]
            sp.yaw      = self.origin_yaw
            if self.counter % 40 == 0:
                self.get_logger().info('✓ MISIÓN COMPLETADA — manteniendo posición')

        self.trajectory_pub.publish(sp)
        self.counter += 1

    # ─────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg              = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)

    def _elapsed(self, start) -> float:
        if start is None:
            return 0.0
        return (self.get_clock().now() - start).nanoseconds / 1e9


# ─────────────────────────────────────────────
#  Nota (*) — Conversión cuerpo → NED
#
#  En PX4/NED con yaw medido desde Norte (CW positivo):
#    Frente (body +X) → NED: [ cos(yaw),  sin(yaw), 0 ]
#    Derecha (body +Y) → NED: [ sin(yaw), -cos(yaw), 0 ]
#
#  Verificación rápida:
#    yaw = 0° (mirando Norte) → derecha = [0, 1] = Este  ✓
#    yaw = 90° (mirando Este) → derecha = [1, 0] = Norte... 
#      espera, si miras al Este tu derecha es el Sur (-Norte).
#  
#  Corrección con signo correcto:
#    right_x = -sin(yaw),  right_y =  cos(yaw)   (NO sin/-cos)
#    yaw=0°  → right = [0,  1] = Este  ✓
#    yaw=90° → right = [-1, 0] = Sur   ✓  (mirando Este, derecha = Sur)
# ─────────────────────────────────────────────


def main():
    rclpy.init()
    node = SimpleMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción detectada — cerrando nodo')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
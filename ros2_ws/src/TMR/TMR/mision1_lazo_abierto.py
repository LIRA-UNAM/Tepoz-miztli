"""
Misión 1 — Lazo Abierto (Open Loop)
Sin dependencia de cámara ni sensores de visión.
Todo el vuelo se basa en velocidades fijas y tiempos.

Secuencia:
  1. TAKEOFF      — sube a TARGET_ALTITUDE y espera estabilización
  2. SLIDE_RIGHT  — se desplaza hacia la derecha (body +Y)  durante SLIDE_RIGHT_TIME
  3. FWD_1        — avanza al frente (body +X)               durante FWD_1_TIME
  4. TURN_1       — gira 90° a la derecha sobre su eje
  5. FWD_2        — avanza al frente (nuevo heading)         durante FWD_2_TIME
  6. TURN_2       — gira 90° a la derecha sobre su eje
  7. FWD_3        — avanza al frente (nuevo heading)         durante FWD_3_TIME
  8. LAND         — desciende hasta tocar tierra y desarma
"""

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

# ─────────────────────────────────────────────
#  PARÁMETROS EDITABLES
# ─────────────────────────────────────────────

TARGET_ALTITUDE  = 1.0    # [m]  Altura de vuelo (distancia al suelo)
TARGET_Z         = -1.0   # [m]  Equivalente NED (negativo = arriba)
TAKEOFF_STABLE   = 15     # ticks seguidos dentro del umbral para confirmar altura

# 1. Deslizamiento lateral derecha
SLIDE_RIGHT_VY   = 0.25   # [m/s]  velocidad lateral
SLIDE_RIGHT_TIME = 4.0    # [s]    duración

# 2. Primer avance recto
FWD_1_VX         = 0.40   # [m/s]
FWD_1_TIME       = 5.0    # [s]

# 3. Segundo avance recto (tras TURN_1)
FWD_2_VX         = 0.40   # [m/s]
FWD_2_TIME       = 5.0    # [s]

# 4. Tercer avance recto (tras TURN_2)
FWD_3_VX         = 0.40   # [m/s]
FWD_3_TIME       = 5.0    # [s]

# Giros (cada uno 90° a la derecha)
TURN_YAW_RATE    = 0.50   # [rad/s]   velocidad de giro (yaw rate)
TURN_TOL         = 0.04   # [rad]     tolerancia de error angular
TURN_STABLE      = 25     # ticks seguidos dentro de tolerancia para confirmar giro

# Aterrizaje
LAND_VZ          = 0.20   # [m/s]  velocidad de descenso (positivo = bajar en NED)
LAND_ALT_THRESH  = 0.12   # [m]    distancia al suelo para desarmar

# ─────────────────────────────────────────────
#  NODO PRINCIPAL
# ─────────────────────────────────────────────

class Mision1OpenLoop(Node):
    def __init__(self):
        super().__init__('mision1_open_loop')

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

        # ── Publishers ──────────────────────────────
        self.offboard_pub    = self.create_publisher(OffboardControlMode,  '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub  = self.create_publisher(TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',   pub_qos)
        self.cmd_pub         = self.create_publisher(VehicleCommand,       '/fmu/in/vehicle_command',        pub_qos)

        # ── Subscribers ─────────────────────────────
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.create_subscription(VehicleAttitude,      '/fmu/out/vehicle_attitude',        self.attitude_cb,  sub_qos)
        self.create_subscription(DistanceSensor,       '/fmu/out/distance_sensor',         self.distance_cb,  sub_qos)

        # ── Estado del vehículo ──────────────────────
        self.current_x    = 0.0
        self.current_y    = 0.0
        self.current_z    = 0.0
        self.current_yaw  = 0.0
        self.current_dist = 0.0   # distancia al suelo (sensor)

        # ── Origen bloqueado al armar ────────────────
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        # ── Variables de control de fases ───────────
        self.state          = 'INIT'
        self.counter        = 0
        self.stable_ticks   = 0
        self.phase_start    = None   # timestamp float (time.time())

        # Para los giros
        self.target_yaw     = None
        self.turn_stable    = 0

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.get_logger().info('=== Misión 1 Lazo Abierto — iniciando ===')

    # ─────────────────────────────────────────────
    #  CALLBACKS
    # ─────────────────────────────────────────────

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy = 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        self.current_yaw = math.atan2(siny, cosy)

    def distance_cb(self, msg: DistanceSensor):
        self.current_dist = msg.current_distance

    # ─────────────────────────────────────────────
    #  TIMER PRINCIPAL (20 Hz)
    # ─────────────────────────────────────────────

    def timer_cb(self):
        now_us = self.get_clock().now().nanoseconds // 1000

        # ── OffboardControlMode ──────────────────────
        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position  = True
        ocm.velocity  = True
        self.offboard_pub.publish(ocm)

        # ── Setpoint base (todo NaN → lo rellenamos por estado) ──
        sp = TrajectorySetpoint()
        sp.timestamp    = now_us
        sp.position     = [float('nan')] * 3
        sp.velocity     = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.jerk         = [float('nan')] * 3
        sp.yaw          = float('nan')
        sp.yawspeed     = float('nan')

        # ── Bloqueo de origen ────────────────────────
        if self.state in ('INIT', 'ARMING'):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        ox  = self.locked_x   if self.locked_x   is not None else 0.0
        oy  = self.locked_y   if self.locked_y   is not None else 0.0
        oya = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ─────────────────────────────────────────────
        #  MÁQUINA DE ESTADOS
        # ─────────────────────────────────────────────

        if self.state == 'INIT':
            # Publica setpoints durante ~1 s antes de pedir Offboard
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oya
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)  # SET_MODE Offboard
                self.state = 'ARMING'

        elif self.state == 'ARMING':
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oya
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)              # ARM
                self.get_logger().info(f'ARMED — subiendo a {TARGET_ALTITUDE:.1f} m')
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            sp.position = [ox, oy, TARGET_Z]
            sp.yaw      = oya
            err = abs(self.current_dist - TARGET_ALTITUDE)
            self.stable_ticks = self.stable_ticks + 1 if err < 0.30 else 0
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f'TAKEOFF | alt={self.current_dist:.2f} m  err={err:.2f}  stable={self.stable_ticks}/{TAKEOFF_STABLE}'
                )
            if self.stable_ticks >= TAKEOFF_STABLE:
                self.get_logger().info('Altura estable — SLIDE_RIGHT')
                self._start_phase('SLIDE_RIGHT')

        elif self.state == 'SLIDE_RIGHT':
            # Deslizamiento lateral derecha en body-frame → NED
            yaw  = oya
            ned_vx = -math.sin(yaw) * SLIDE_RIGHT_VY  # body +Y → NED
            ned_vy =  math.cos(yaw) * SLIDE_RIGHT_VY
            sp.velocity = [ned_vx, ned_vy, 0.0]
            sp.yaw      = yaw
            elapsed = self._elapsed()
            if self.counter % 10 == 0:
                self.get_logger().info(f'SLIDE_RIGHT | {elapsed:.1f}/{SLIDE_RIGHT_TIME:.0f}s')
            if elapsed >= SLIDE_RIGHT_TIME:
                self.get_logger().info('SLIDE_RIGHT completo — FWD_1')
                self._start_phase('FWD_1')

        elif self.state == 'FWD_1':
            sp, elapsed = self._advance(sp, oya, FWD_1_VX, FWD_1_TIME, 'FWD_1')
            if elapsed >= FWD_1_TIME:
                self.get_logger().info('FWD_1 completo — TURN_1')
                self._start_turn()
                self.state = 'TURN_1'

        elif self.state == 'TURN_1':
            done = self._do_turn(sp, oya)
            if done:
                oya = self.target_yaw   # actualizar heading bloqueado
                self.locked_yaw = self.target_yaw
                self.get_logger().info('TURN_1 completo — FWD_2')
                self._start_phase('FWD_2')

        elif self.state == 'FWD_2':
            sp, elapsed = self._advance(sp, self.locked_yaw, FWD_2_VX, FWD_2_TIME, 'FWD_2')
            if elapsed >= FWD_2_TIME:
                self.get_logger().info('FWD_2 completo — TURN_2')
                self._start_turn()
                self.state = 'TURN_2'

        elif self.state == 'TURN_2':
            done = self._do_turn(sp, self.locked_yaw)
            if done:
                self.locked_yaw = self.target_yaw
                self.get_logger().info('TURN_2 completo — FWD_3')
                self._start_phase('FWD_3')

        elif self.state == 'FWD_3':
            sp, elapsed = self._advance(sp, self.locked_yaw, FWD_3_VX, FWD_3_TIME, 'FWD_3')
            if elapsed >= FWD_3_TIME:
                self.get_logger().info('FWD_3 completo — LAND')
                self._start_phase('LAND')

        elif self.state == 'LAND':
            sp.velocity = [0.0, 0.0, LAND_VZ]   # descender (NED +Z = abajo)
            sp.yaw      = self.locked_yaw
            if self.counter % 10 == 0:
                self.get_logger().info(f'LAND | dist_suelo={self.current_dist:.2f} m')
            if self.current_dist < LAND_ALT_THRESH:
                self.send_cmd(400, param1=0.0)   # DISARM
                self.get_logger().info('=== ATERRIZAJE COMPLETADO — MISIÓN FINALIZADA ===')
                self.state = 'DONE'

        elif self.state == 'DONE':
            if self.counter % 40 == 0:
                self.get_logger().info('MISIÓN FINALIZADA')

        self.trajectory_pub.publish(sp)
        self.counter += 1

    # ─────────────────────────────────────────────
    #  HELPERS
    # ─────────────────────────────────────────────

    def _start_phase(self, next_state: str):
        """Resetea el temporizador de fase y cambia de estado."""
        self.phase_start = time.time()
        self.state       = next_state

    def _elapsed(self) -> float:
        """Segundos transcurridos desde el inicio de la fase actual."""
        return time.time() - self.phase_start if self.phase_start else 0.0

    def _advance(self, sp: TrajectorySetpoint, yaw: float, vx: float, duration: float, name: str):
        """
        Avanza en línea recta en body +X durante `duration` segundos.
        Devuelve el setpoint modificado y el tiempo transcurrido.
        """
        ned_vx = math.cos(yaw) * vx
        ned_vy = math.sin(yaw) * vx
        sp.velocity = [ned_vx, ned_vy, 0.0]
        sp.yaw      = yaw
        elapsed = self._elapsed()
        if self.counter % 10 == 0:
            self.get_logger().info(f'{name} | {elapsed:.1f}/{duration:.0f}s')
        return sp, elapsed

    def _start_turn(self):
        """Prepara un giro de 90° a la derecha desde el yaw actual."""
        raw = self.current_yaw - math.pi / 2.0   # -90° = derecha en NED/ENU
        self.target_yaw  = math.atan2(math.sin(raw), math.cos(raw))
        self.turn_stable = 0
        self.get_logger().info(
            f'Iniciando giro 90° derecha: '
            f'{math.degrees(self.current_yaw):.1f}° → {math.degrees(self.target_yaw):.1f}°'
        )

    def _do_turn(self, sp: TrajectorySetpoint, yaw_hold: float) -> bool:
        """
        Ejecuta el giro hacia self.target_yaw.
        Envía yawspeed constante mientras no se confirme la llegada.
        Devuelve True cuando el giro queda estable.
        """
        # Mantener posición horizontal mientras gira
        sp.velocity = [0.0, 0.0, 0.0]
        sp.position[2] = TARGET_Z

        yaw_err = math.atan2(
            math.sin(self.target_yaw - self.current_yaw),
            math.cos(self.target_yaw - self.current_yaw),
        )

        # Usar yawspeed proporcional al error (con mínimo para no pararse)
        yaw_rate = max(0.10, min(TURN_YAW_RATE, abs(yaw_err) * 1.5))
        sp.yawspeed = -yaw_rate   # negativo = derecha (NED)

        self.turn_stable = self.turn_stable + 1 if abs(yaw_err) < TURN_TOL else 0

        if self.counter % 10 == 0:
            self.get_logger().info(
                f'TURN | yaw={math.degrees(self.current_yaw):.1f}°  '
                f'meta={math.degrees(self.target_yaw):.1f}°  '
                f'err={math.degrees(yaw_err):.1f}°  stable={self.turn_stable}/{TURN_STABLE}'
            )

        if self.turn_stable >= TURN_STABLE:
            sp.yawspeed = 0.0
            return True
        return False

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

def main():
    rclpy.init()
    node = Mision1OpenLoop()
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
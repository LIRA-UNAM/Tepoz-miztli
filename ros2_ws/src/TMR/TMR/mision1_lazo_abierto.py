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
    DistanceSensor
)

from geometry_msgs.msg import PoseArray, Point

#Parametros editables para la misión
TARGET_ALTITUDE = 1.0 # metros de altura'(en modelo normal)
TARGET_Z = -1.0 #(Modelo NED)
HOLD_DURATION = 3.0

#Busqueda de Gates azules
GATE_SEARCH_VY  = 0.10          # desetpointlazamiento lateral buscando gate
CROSS_GATE_VX   = 0.60          # velocidad de cruce
CROSS_GATE_TIME = 4.0           # tiempo cruzando la gate
VEL_REDUC_STEPS = [(2.0, 0.4), (4.0, 0.2), (6.0, 0.1)]  # (t_acum_s, vel_m/s)

#Esquivar columna
DODGE_VX          = 0.20        # avance suave durante la esquiva
DODGE_VY_RIGHT    = 0.25        # velocidad lateral DERECHA (Se puede cambiar a la izquierda)
DODGE_CLEAR_TICKS = 15          # ticks sin columna
DODGE_TIMEOUT     = 6.0         # si la columna nunca aparece -> saltar (Segundos)
COLUMN_TIMEOUT    = 0.4         # No hay columna

#Aruco
APPROACH_DIST = 0.80        #distancia entre el aruco y el drone.
APPROACH_KP_FWD = 0.2       # ganancia de profundidad
APPROACH_KP_LAT  = 0.5      # ganancia lateral
APPROACH_KP_VERT = 0.3      # ganancia vertical
APPROACH_MAX_V   = 0.3      # velocidad máx [m/s]
APPROACH_TOL_XY  = 0.10     # tolerancia lateral
APPROACH_TOL_Z   = 0.15     # tolerancia de profundidad
APPROACH_STABLE  = 20       # ticks para confirmar la llegada al aruco

#Velocidad de busqueda girando y deslizado de busqueda
SEARCH_YAWsetpointEED = 0.25 #rad/s
SLIDE_setpointEED = 0.3 # m/s
SLIDE_DURATION = 4.0 # segundos 

#rotación para finalización de tarea
ROTATE_DIR = -1 #-1 = 90 CW
ROTATE_TOL = 0.04 #en radianas
ROTATE_STABLE = 40 #confirmación de rotación con ticks

ARUCO_TIMEOUT = 0.8

#Modo Landing
LAND_SEARCH_VX = 0.10
LAND_DESCENT_VZ = 0.20
LAND_ALT_THRESH = 0.15

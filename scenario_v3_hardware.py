"""
scenario_v3_hardware.py
═══════════════════════
Hardware realisation of the blind-intersection evasion scenario (scenario_v3.py).

Platform
  Jetson Orin Nano 8 GB  ─ USB-CAN adapter  ─ PIX Moving Hooke chassis
                         ─ Ethernet          ─ Ouster OS1-64 3D LiDAR
                         ─ USB               ─ YDLidar X2

Architecture (4-process, no CAN write here)
  ydlidar_forwarder.py        →  :5005  →  jetson_ouster_arbiter.py
  jetson_ouster_arbiter.py    →  :5006  →  can_publisher.py
  scenario_v3_hardware.py     →  :5007  →  can_publisher.py  (sole CAN writer)
  can_publisher.py            →  CAN 0x130/0x131/0x132/0x133

Evasion priority (near-range sensor, front threat while EVADING)
  1. PUSH       — sprint if gap timing clears and pursuer is NOT head-on
  2. LATERAL-DODGE — bounded swerve if lane wide enough (> DODGE_LIMIT_M + 0.1 m)
  3. CURB-CLIMB — last-resort if some room but not enough for full swerve
  4. NR-REVERSE — reverse along prior waypoints until OBB gap > NR_CLEAR_GAP

Set-up checklist
  1. Bring up CAN:      sudo ip link set can0 up type can bitrate 500000
  2. Start cron jobs:   ydlidar_forwarder.py, jetson_ouster_arbiter.py, can_publisher.py
  3. Record waypoints with wp_logger_odo.py; paste into EGO_WAYPOINTS_RAW.
  4. Set JUNCTION_XY to junction entry in odometry world frame.
  5. Set ROAD_CURB_Y, DODGE_LAT_SIGN for your road geometry (see comments below).
  6. python3 scenario_v3_hardware.py

Coordinate conventions (same as odometry_eval.py / wp_logger_odo.py)
  World frame   x = east,  y = north  (right-hand, CCW positive)
  Heading       theta = CCW from east, radians  (90° = north at start)
  Sensor frame  x = forward, y = left
"""

import can
import copy
import math
import time
import threading
import socket
import json
import argparse
import sys
import os
import subprocess
import urllib.request
import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

from shapely.geometry import Polygon, Point
from shapely.affinity import scale as poly_scale
from shapely.validation import make_valid

try:
    from ouster.sdk import open_source
    from ouster.sdk.core import XYZLut, ChanField
    OUSTER_AVAILABLE = True
except ImportError:
    OUSTER_AVAILABLE = False
    print("[WARN] ouster-sdk not found — LiDAR perception disabled")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odometry_eval import (
    OdometryIntegrator,
    decode_drive_fb, decode_steer_fb, decode_wheel_rpm_fb,
    ID_DRIVE_FB, ID_STEER_FB, ID_WHEEL_RPM_FB,
    WHEELBASE_M,
)

# ═══════════════════════════════════════════════════════════════════════════════
#  USER CONFIGURATION  — edit before running
# ═══════════════════════════════════════════════════════════════════════════════

# ── Hardware ──────────────────────────────────────────────────────────────────
CAN_CHANNEL     = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
OUSTER_IP       = '169.254.135.171'
CONTROL_HZ      = 20
SCENARIO_PORT   = 5007
BROADCAST_ADDR  = "255.255.255.255"

# ── Ouster OS1 mount ──────────────────────────────────────────────────────────
OUSTER_Z_MIN    = -1.65
OUSTER_Z_MAX    =  0.25
OUSTER_INVERTED = True
OUSTER_MIN_PTS  = 5

# ── Ego waypoints ─────────────────────────────────────────────────────────────
# EGO_WAYPOINTS_RAW: list = [
#     {"xw": 0.00, "yw": 0.00},
#     {"xw": 1.00, "yw": 0.00},
#     {"xw": 2.00, "yw": 0.00},
#     {"xw": 3.00, "yw": 0.00},
#     {"xw": 4.00, "yw": 0.00},
#     {"xw": 5.00, "yw": 0.00},
# ]

EGO_WAYPOINTS_RAW: list = [
    {"x":   +0.0000, "y":   +0.0000, "theta":   +90.00, "dist":   0.0000, "t":    0.00},
    {"x":   +0.5618, "y":   +0.0046, "theta":   +89.25, "dist":   0.5618, "t":    3.51},
    {"x":   +1.1176, "y":   +0.0359, "theta":   +87.26, "dist":   1.1183, "t":    4.31},
    {"x":   +1.6530, "y":   +0.0934, "theta":   +84.80, "dist":   1.6563, "t":    5.11},
    {"x":   +2.2136, "y":   +0.0947, "theta":   +84.75, "dist":   2.2170, "t":    6.01},
    {"x":   +2.7881, "y":   +0.0949, "theta":   +84.75, "dist":   2.7915, "t":    6.71},
    {"x":   +3.3658, "y":   +0.0981, "theta":   +84.69, "dist":   3.3694, "t":    7.32},
    {"x":   +3.8948, "y":   +0.1013, "theta":   +84.64, "dist":   3.8985, "t":    8.02},
    {"x":   +4.4390, "y":   +0.1043, "theta":   +84.60, "dist":   4.4428, "t":    8.82},
    {"x":   +4.9734, "y":   +0.1130, "theta":   +84.49, "dist":   4.9773, "t":    9.42},
    {"x":   +5.4948, "y":   +0.1360, "theta":   +84.24, "dist":   5.4993, "t":    9.92},
    {"x":   +6.0604, "y":   +0.1693, "theta":   +83.91, "dist":   6.0658, "t":   10.42},
    {"x":   +6.6500, "y":   +0.2094, "theta":   +83.55, "dist":   6.6566, "t":   10.92},
    {"x":   +7.2224, "y":   +0.2416, "theta":   +83.29, "dist":   7.2300, "t":   11.42},
    {"x":   +7.7665, "y":   +0.2745, "theta":   +83.03, "dist":   7.7752, "t":   11.92},
    {"x":   +8.3102, "y":   +0.3128, "theta":   +82.76, "dist":   8.3203, "t":   12.43},
    {"x":   +8.8909, "y":   +0.3704, "theta":   +82.38, "dist":   8.9033, "t":   12.93},
    {"x":   +9.4984, "y":   +0.4563, "theta":   +81.84, "dist":   9.5146, "t":   13.43},
    {"x":  +10.0926, "y":   +0.6012, "theta":   +81.00, "dist":  10.1166, "t":   13.93},
    {"x":  +10.5843, "y":   +0.7982, "theta":   +79.91, "dist":  10.6216, "t":   14.33},
    {"x":  +11.1652, "y":   +1.2036, "theta":   +77.78, "dist":  11.2397, "t":   14.83},
    {"x":  +11.6897, "y":   +1.5517, "theta":   +76.03, "dist":  11.8062, "t":   15.33},
    {"x":  +12.2216, "y":   +1.7614, "theta":   +75.02, "dist":  12.3673, "t":   15.83},
    {"x":  +12.8079, "y":   +1.8604, "theta":   +74.57, "dist":  12.9679, "t":   16.33},
    {"x":  +13.3344, "y":   +1.9880, "theta":   +74.01, "dist":  13.5131, "t":   16.73},
    {"x":  +13.8237, "y":   +2.2359, "theta":   +72.96, "dist":  14.0409, "t":   17.14},
    {"x":  +14.3741, "y":   +2.5166, "theta":   +71.82, "dist":  14.6386, "t":   17.64},
    {"x":  +14.8727, "y":   +2.8319, "theta":   +70.59, "dist":  15.1948, "t":   18.14},
    {"x":  +15.3990, "y":   +3.2466, "theta":   +69.02, "dist":  15.8042, "t":   18.64},
    {"x":  +15.7955, "y":   +3.8067, "theta":   +66.97, "dist":  16.3272, "t":   19.04},
    {"x":  +16.1692, "y":   +4.6930, "theta":   +63.79, "dist":  16.9364, "t":   19.54},
    {"x":  +16.4485, "y":   +5.4825, "theta":   +61.02, "dist":  17.4617, "t":   20.04},
    {"x":  +16.6095, "y":   +6.4874, "theta":   +57.54, "dist":  17.9863, "t":   20.54},
    {"x":  +16.6804, "y":   +7.6405, "theta":   +53.57, "dist":  18.5463, "t":   21.04},
    {"x":  +16.5039, "y":   +9.1416, "theta":   +48.39, "dist":  19.1280, "t":   21.54},
    {"x":  +16.2256, "y":  +10.5853, "theta":   +43.34, "dist":  19.7195, "t":   22.04},
    {"x":  +15.8497, "y":  +11.8545, "theta":   +38.81, "dist":  20.2312, "t":   22.55},
    {"x":  +15.2484, "y":  +13.3161, "theta":   +33.43, "dist":  20.8125, "t":   23.15},
    {"x":  +14.5753, "y":  +14.6188, "theta":   +28.43, "dist":  21.3592, "t":   23.65},
    {"x":  +13.6402, "y":  +16.0122, "theta":   +22.77, "dist":  21.9358, "t":   24.15},
    {"x":  +13.1293, "y":  +16.8571, "theta":   +19.15, "dist":  22.4638, "t":   24.75},
    {"x":  +12.4733, "y":  +17.7249, "theta":   +15.27, "dist":  22.9788, "t":   25.65},
    {"x":  +11.8943, "y":  +18.4616, "theta":   +11.81, "dist":  23.4943, "t":   26.45},
    {"x":  +11.3754, "y":  +19.1193, "theta":    +8.57, "dist":  24.0379, "t":   27.46},
    {"x":  +11.5543, "y":  +19.3168, "theta":    +7.57, "dist":  24.5521, "t":   28.16},
    {"x":  +12.1182, "y":  +19.3146, "theta":    +7.58, "dist":  25.1126, "t":   28.86},
    {"x":  +12.6352, "y":  +19.3142, "theta":    +7.58, "dist":  25.6288, "t":   30.26},
    {"x":  +13.1388, "y":  +19.3140, "theta":    +7.58, "dist":  26.1322, "t":   32.26},
    {"x":  +13.6427, "y":  +19.3142, "theta":    +7.58, "dist":  26.6363, "t":   34.17},
    {"x":  +14.1437, "y":  +19.3139, "theta":    +7.58, "dist":  27.1369, "t":   35.77},
    {"x":  +14.6462, "y":  +19.3141, "theta":    +7.58, "dist":  27.6396, "t":   37.88},
    {"x":  +15.1973, "y":  +19.3136, "theta":    +7.58, "dist":  28.1902, "t":   39.28},
    {"x":  +15.7341, "y":  +19.3142, "theta":    +7.58, "dist":  28.7277, "t":   40.28},
    {"x":  +16.2890, "y":  +19.3142, "theta":    +7.58, "dist":  29.2826, "t":   41.28},
    {"x":  +16.7938, "y":  +19.3142, "theta":    +7.58, "dist":  29.7874, "t":   42.09},
    {"x":  +17.3091, "y":  +19.3135, "theta":    +7.58, "dist":  30.3019, "t":   42.89},
    {"x":  +17.8293, "y":  +19.3137, "theta":    +7.58, "dist":  30.8223, "t":   43.79},
    {"x":  +18.3730, "y":  +19.3142, "theta":    +7.58, "dist":  31.3666, "t":   44.69},
]

CROSS_ROAD_POINTS_RAW: list = []
JUNCTION_XY: Tuple[float, float] = (17.0, 7.0)

# ── Planner parameters ────────────────────────────────────────────────────────
MAX_SPEED_MPS        = 0.833   # 3 km/h
MAX_STEER_DEG        = 22.73  # 500° actuator / ratio 22
PLANNER_MAX_DECEL    = -3.0   # m/s²
PLANNER_MAX_ACCEL    =  1.0   # m/s²
PURSUER_MAX_SPEED    =  2.5   # m/s
LIDAR_RANGE          = 25.0   # m
MIN_PURSUER_SPEED    =  0.80  # m/s — filter static obstacles (smoothed EMA velocity)

# ── Lateral dodge / NR parameters ─────────────────────────────────────────────
#
#  ROAD_CURB_Y   : y-coordinate (odometry frame) of the curb toward which the
#                  ego dodges.  Clearance = e.y - ROAD_CURB_Y must be > 0 when
#                  there is room to dodge.  Example: if the south curb is at
#                  y = -2.0 m and the ego travels east at y ≈ 0, set -2.0.
#                  For a north curb with y = +2.0 and ego at y ≈ 0, set +2.0
#                  and set DODGE_LAT_SIGN = +1.
#
#  DODGE_LAT_SIGN: direction of the dodge relative to ego heading.
#                  +1 = dodge toward ego's LEFT  (increases y if heading east)
#                  -1 = dodge toward ego's RIGHT (decreases y if heading east)
#
ROAD_CURB_Y      =  23.35    # L-turn, negligible dodge room
ROAD_CURB_X      =  23.25    # T-junction, curb starts at x ≈ 23 m
DODGE_LIMIT_M    =  0.8   # m — max lateral displacement before return phase
DODGE_LAT_SIGN   =  +1     # ± 1 — see above

NR_CLEAR_GAP     =  5.0    # m — OBB gap to declare NR-reverse done
REV_WPS_BACK     =  4      # max waypoints to step back through in NR-reverse
REV_SPEED_MPS    =  0.5    # m/s — reverse speed (curb-recovery + NR-reverse)
# Chassis geometry: 2.6 m long × 1.7 m wide; Ouster at center (1.30 m from front bumper); YDLidar 0.30 m from front bumper.
# Guard distances are measured from each sensor. "chassis surface" = sensor + distance_to_bumper.
OUSTER_BODY_IGNORE = 1.30  # m — Ouster to front bumper; ignore returns closer than this (body artefacts)
# Stopping budget at max speed: v²/(2a) + v·t_lat = 1.39²/6.0 + 1.39·0.10 = 0.461 m
# BRAKE fires far enough that vehicle stops before bumper reaches the obstacle (static-obstacle guarantee).
# BRAKE threshold = sensor-to-bumper + 0.30 m (30 cm margin) + 0.461 m (stop budget) rounded up.
# E-STOP threshold = sensor-to-bumper + 0.20 m — damage-limitation if object appears inside BRAKE zone.
OUSTER_ESTOP_M     = 1.50  # m — Ouster: 20 cm from front bumper → always active (damage limitation)
OUSTER_BRAKE_M     = 2.10  # m — Ouster: 30 cm margin + stop budget → guaranteed stop before contact
OUSTER_BODY_IGNORE_SIDE = 0.85  # m — chassis half-width; ignore Ouster returns closer than this on flanks
OUSTER_ESTOP_SIDE_M     = 1.00  # m — Ouster flanks: imminent → estop (15 cm from chassis edge)
OUSTER_BRAKE_SIDE_M     = 1.20  # m — Ouster flanks: near → brake (35 cm from chassis edge)
YDL_ESTOP_M        = 0.50  # m — YDLidar: 20 cm from front bumper → always active (damage limitation)
YDL_BRAKE_M        = 1.10  # m — YDLidar: 30 cm margin + stop budget → guaranteed stop before contact
YDL_ESTOP_SIDE_M   = 0.90  # m — YDLidar flanks: imminent → estop (body mask already 0.85 m in forwarder)
YDL_BRAKE_SIDE_M   = 1.20  # m — YDLidar flanks: near → brake (matches THRESH_SIDE in ydlidar_forwarder)

# ── Vehicle bounding-box half-dims (OBB gap sensor) ───────────────────────────
EGO_HL, EGO_HW   = 1.30, 0.85   # Hooke 2.6 × 1.7 m chassis (half-length, half-width)
PUR_HL, PUR_HW   = 2.35, 0.95   # pursuer vehicle (approximate)

# ── Safety limits ─────────────────────────────────────────────────────────────
ODO_STALE_S      =  0.5
LIDAR_STALE_S    =  0.15   # 1.5× Ouster scan period; limits polygon staleness to ~0.2 m at max speed
MAX_RUN_S        = 300.0
STUCK_WINDOW_S   =  5.0
STOP_DRAIN_S     =  2.0

# ═══════════════════════════════════════════════════════════════════════════════
#  Waypoint loader
# ═══════════════════════════════════════════════════════════════════════════════

def load_waypoints(raw: list) -> List[Tuple[float, float]]:
    wps = []
    for wp in raw:
        if 'xw' in wp and 'yw' in wp:
            wps.append((float(wp['xw']), float(wp['yw'])))
        elif 'x' in wp and 'y' in wp and 'theta' in wp:
            t  = math.radians(float(wp['theta']))
            xw = wp['x'] * math.cos(t) - wp['y'] * math.sin(t)
            yw = wp['x'] * math.sin(t) + wp['y'] * math.cos(t)
            wps.append((float(xw), float(yw)))
        else:
            raise ValueError(f"Bad waypoint entry: {wp}")
    return wps

# ═══════════════════════════════════════════════════════════════════════════════
#  UDP command sender  (scenario → can_publisher on :5007)
# ═══════════════════════════════════════════════════════════════════════════════

def _tx_sock() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    return s

def _front_guard_dist(hw_lidar) -> float:
    """Nearest hit in the forward ±30° cone beyond the chassis body. Returns inf if clear."""
    nearest = math.inf
    for hx, hy, hz in hw_lidar._snapshot():
        if hx > OUSTER_BODY_IGNORE and abs(hy) < hx * 0.577:   # 1.30 m = Ouster-to-front-bumper (chassis half-length)
            d = math.hypot(hx, hy)
            if d < nearest:
                nearest = d
    return nearest


def _side_guard_dists(hw_lidar) -> tuple:
    """Nearest hit in the left and right flanks (45°–135° each side) beyond chassis half-width.
    Returns (nearest_left_m, nearest_right_m); inf means clear.
    Sensor frame: x = forward, y = left. Flank condition: |hy| > |hx| (beyond ±45°).
    """
    nearest_left  = math.inf
    nearest_right = math.inf
    for hx, hy, hz in hw_lidar._snapshot():
        if abs(hy) > abs(hx) and abs(hy) > OUSTER_BODY_IGNORE_SIDE:
            d = math.hypot(hx, hy)
            if hy > 0:
                nearest_left  = min(nearest_left,  d)
            else:
                nearest_right = min(nearest_right, d)
    return nearest_left, nearest_right


class YDLidarGuard:
    """Subscribes to ydlidar_forwarder JSON on :5005.
    Packets: {"safe": bool, "front": float, "left": float, "right": float, "trigger": str}
    Body-artefact masking (THRESH_FRONT_MIN=0.45 m front, THRESH_SIDE_MIN=0.85 m sides)
    is applied in ydlidar_forwarder.py before broadcast; values here are real obstacles.
    """
    _PORT = 5005

    def __init__(self):
        self._front = -1.0   # -1.0 = no valid hit
        self._left  = -1.0
        self._right = -1.0
        self._lock  = threading.Lock()

    def listen(self, stop_event: threading.Event):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', self._PORT))
        sock.settimeout(0.1)
        print(f"[YDL ] Guard listener on :{self._PORT}", flush=True)
        while not stop_event.is_set():
            try:
                data, _ = sock.recvfrom(4096)
                pkt = json.loads(data.decode())
                with self._lock:
                    self._front = float(pkt.get('front', -1.0))
                    self._left  = float(pkt.get('left',  -1.0))
                    self._right = float(pkt.get('right', -1.0))
            except socket.timeout:
                pass
            except Exception:
                pass
        sock.close()

    def front_dist(self) -> float:
        with self._lock:
            return self._front

    def side_dists(self) -> tuple:
        """Returns (left_m, right_m); -1.0 means no valid hit."""
        with self._lock:
            return self._left, self._right


def send_command(sock: socket.socket, action: str, speed_mps: float,
                 steer_deg: float, fsm: str):
    """Broadcast a motion command for can_publisher to execute.
    action: "drive" | "brake" | "stop" | "reverse"
    """
    pkt = {
        "action"   : action,
        "speed_mps": round(speed_mps, 3),
        "steer_deg": round(steer_deg, 2),
        "fsm"      : fsm,
    }
    sock.sendto(json.dumps(pkt).encode(), (BROADCAST_ADDR, SCENARIO_PORT))

# ═══════════════════════════════════════════════════════════════════════════════
#  OBB helpers  (ported from scenario_v3.py)
# ═══════════════════════════════════════════════════════════════════════════════

def _box_proj(hl: float, hw: float, yaw: float, ax: float, ay: float) -> float:
    """Project OBB half-extent onto unit axis (ax, ay)."""
    lx, ly = math.cos(yaw), math.sin(yaw)
    return hl * abs(lx * ax + ly * ay) + hw * abs(-ly * ax + lx * ay)

def obb_gap(ca, yaw_a, hl_a, hw_a, cb, yaw_b, hl_b, hw_b) -> float:
    """SAT minimum separating distance between two 2-D OBBs.
    Positive = separated (gap), ≤ 0 = overlapping."""
    dx, dy = cb[0] - ca[0], cb[1] - ca[1]
    max_gap = float('-inf')
    for yaw in (yaw_a, yaw_b):
        lx, ly = math.cos(yaw), math.sin(yaw)
        for ax, ay in ((lx, ly), (-ly, lx)):
            proj_d   = abs(dx * ax + dy * ay)
            proj_sum = _box_proj(hl_a, hw_a, yaw_a, ax, ay) + _box_proj(hl_b, hw_b, yaw_b, ax, ay)
            max_gap  = max(max_gap, proj_d - proj_sum)
    return max_gap

def _aligned_wp(e, wps: list, center_idx: int, radius: int = 2) -> int:
    """Waypoint index best aligned with ego heading near center_idx,
    advanced one step (so pure-pursuit has a target to track)."""
    lo = max(0, center_idx - radius)
    hi = min(len(wps) - 1, center_idx + radius + 2)
    def _score(i):
        dx, dy = wps[i][0] - e.x, wps[i][1] - e.y
        return abs((math.atan2(dy, dx) - e.yaw + math.pi) % (2 * math.pi) - math.pi)
    return min(min(range(lo, hi), key=_score) + 1, len(wps) - 1)

# ═══════════════════════════════════════════════════════════════════════════════
#  Thread-safe odometry
# ═══════════════════════════════════════════════════════════════════════════════

class LockedIntegrator(OdometryIntegrator):
    def __init__(self):
        self._lock = threading.Lock()
        self._last_update: float = 0.0
        super().__init__()

    def reset(self):
        with self._lock:
            super().reset()

    def update_steer(self, f, r, mode=None, ts=None):
        with self._lock:
            super().update_steer(f, r, mode=mode, ts=ts)
            self._last_update = time.time()

    def update_speed(self, v, a, ts):
        with self._lock:
            super().update_speed(v, a, ts)
            self._last_update = time.time()

    def update_rpm(self, lf, rf, lr, rr, ts):
        with self._lock:
            super().update_rpm(lf, rf, lr, rr, ts)
            self._last_update = time.time()

    def pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.x, self.y, self.theta_rad

    def current_speed(self) -> float:
        with self._lock:
            return self.speed_samples[-1] if self.speed_samples else 0.0

    def odo_age(self) -> float:
        with self._lock:
            return float('inf') if self._last_update == 0.0 else time.time() - self._last_update


def can_reader(odo: LockedIntegrator, stop_event: threading.Event, channel: str):
    try:
        bus = can.interface.Bus(interface='socketcan', channel=channel)
        print(f"[ODO ] Listening on {channel}")
    except Exception as exc:
        print(f"[ODO ] Failed to open {channel}: {exc}")
        stop_event.set()
        return
    sub = {ID_DRIVE_FB, ID_STEER_FB, ID_WHEEL_RPM_FB}
    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.arbitration_id not in sub:
                continue
            data = bytes(msg.data)
            now  = time.time()
            if msg.arbitration_id == ID_STEER_FB and len(data) >= 6:
                d = decode_steer_fb(data)
                odo.update_steer(d['steer_angle_front'], d['steer_angle_rear'],
                                 mode=d['steer_mode'], ts=now)
            elif msg.arbitration_id == ID_DRIVE_FB and len(data) >= 7:
                d = decode_drive_fb(data)
                odo.update_speed(d['speed_mps'], d['acceleration_mps2'], now)
            elif msg.arbitration_id == ID_WHEEL_RPM_FB and len(data) >= 8:
                d = decode_wheel_rpm_fb(data)
                odo.update_rpm(d['rpm_lf'], d['rpm_rf'], d['rpm_lr'], d['rpm_rr'], now)
        except Exception:
            pass
    bus.shutdown()

# ═══════════════════════════════════════════════════════════════════════════════
#  Hardware LiDAR  (Ouster OS1)
# ═══════════════════════════════════════════════════════════════════════════════

class HWLidar:
    def __init__(self, lidar_range: float):
        self.r             = lidar_range
        self._lock         = threading.Lock()
        self._hits         : List[Tuple[float, float, float]] = []
        self._bins         : List[float] = [lidar_range] * 360
        self._dirty        : bool = False
        self._last_scan_ts : float = 0.0

    def update_from_scan(self, scan, xyzlut):
        xyz = xyzlut(scan.field(ChanField.RANGE))
        if OUSTER_INVERTED:
            sx, sy = -xyz[..., 0], -xyz[..., 1]
        else:
            sx, sy =  xyz[..., 0],  xyz[..., 1]
        sz = xyz[..., 2]
        try:
            r_field    = scan.field(ChanField.RANGE)
            valid_mask = r_field > 0
        except Exception:
            valid_mask = np.ones(sx.shape, dtype=bool)
        horiz = np.sqrt(sx ** 2 + sy ** 2)
        mask  = (valid_mask & (sz > OUSTER_Z_MIN) & (sz < OUSTER_Z_MAX)
                 & (horiz > 0.3) & (horiz < self.r))
        xs = sx[mask].flatten().tolist()
        ys = sy[mask].flatten().tolist()
        zs = sz[mask].flatten().tolist()
        with self._lock:
            self._hits         = list(zip(xs, ys, zs))
            self._dirty        = True
            self._last_scan_ts = time.time()

    def scan_age(self) -> float:
        with self._lock:
            return float('inf') if self._last_scan_ts == 0.0 else time.time() - self._last_scan_ts

    def _snapshot(self) -> List[Tuple[float, float, float]]:
        with self._lock:
            return list(self._hits)

    def _rebuild(self, hits: list):
        bd = [self.r] * 360
        for hx, hy, hz in hits:
            d = math.hypot(hx, hy)
            i = int(math.degrees(math.atan2(hy, hx)) % 360)
            bd[i] = min(bd[i], d)
        return bd

    def is_visible(self, target_world_xy, ego_world_xy, ego_yaw, threshold=4.0) -> bool:
        hits = self._snapshot()
        if len(hits) < 3:
            return False
        ex, ey = ego_world_xy
        tx, ty = target_world_xy
        cy, sy = math.cos(ego_yaw), math.sin(ego_yaw)
        for hx, hy, hz in hits:
            wx = ex + hx * cy - hy * sy
            wy = ey + hx * sy + hy * cy
            if math.hypot(wx - tx, wy - ty) < threshold:
                return True
        return False

    def get_shadow_polygon(self, el, road_poly, ego_yaw=0.0,
                           _hits=None) -> Polygon:
        hits = _hits if _hits is not None else self._snapshot()
        if len(hits) < 3:
            return Polygon()
        bd = self._rebuild(hits)
        ex, ey = el
        pts = []
        for i in range(360):
            if bd[i] < self.r - 0.5:
                world_angle = math.radians(i) + ego_yaw
                pts.append((ex + self.r * math.cos(world_angle),
                             ey + self.r * math.sin(world_angle)))
        if len(pts) < 3:
            return Polygon()
        sh = _safe_geom(Polygon(pts).convex_hull)
        rp = _safe_geom(road_poly)
        return _safe_geom(sh.intersection(rp)) if not (sh.is_empty or rp.is_empty) else Polygon()

    def get_clear_polygon(self, el, ego_yaw=0.0, _hits=None) -> Polygon:
        hits = _hits if _hits is not None else self._snapshot()
        if len(hits) < 3:
            return Polygon()
        bd = self._rebuild(hits)
        ex, ey = el
        pts = [(ex, ey)]
        for i in range(360):
            if bd[i] >= self.r - 0.5:
                world_angle = math.radians(i) + ego_yaw
                pts.append((ex + self.r * math.cos(world_angle),
                             ey + self.r * math.sin(world_angle)))
        return _safe_geom(Polygon(pts).convex_hull) if len(pts) >= 4 else Polygon()

    def snapshot_and_bins(self) -> Tuple[list, List[float]]:
        """Single atomic snapshot + bin build for consistent shadow/clear pair."""
        hits = self._snapshot()
        return hits, self._rebuild(hits)


def ouster_thread(hw_lidar: HWLidar, tracker, stop_event: threading.Event, ouster_ip: str):
    if not OUSTER_AVAILABLE:
        print("[OS1 ] ouster-sdk not available — perception disabled"); return

    from ouster.sdk.core import SensorInfo
    url = f"http://{ouster_ip}/api/v1/sensor/metadata"

    # Fetch metadata with retry (sensor may not be ready at startup)
    while not stop_event.is_set():
        try:
            with urllib.request.urlopen(url, timeout=10) as r:
                meta = SensorInfo(r.read().decode())
            break
        except Exception as exc:
            print(f"[OS1 ] Metadata not ready ({exc}) — retrying in 5 s …", flush=True)
            stop_event.wait(5)
    if stop_event.is_set():
        return

    xyzlut = XYZLut(meta)
    print(f"[OS1 ] Metadata OK — connecting {ouster_ip}", flush=True)

    while not stop_event.is_set():
        try:
            source = open_source(ouster_ip)
            print(f"[OS1 ] Connected  {ouster_ip}", flush=True)
            for scan_set in source:
                if stop_event.is_set():
                    break
                scan = scan_set[0]   # LidarScanSet — not a list, index directly
                if scan is None:
                    continue
                hw_lidar.update_from_scan(scan, xyzlut)
                tracker.update(hw_lidar)
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            print(f"\n[OS1 ] Stream lost ({exc}) — reconnecting in 5 s …", flush=True)
            stop_event.wait(5)
        finally:
            try:
                source.close()
            except Exception:
                pass

# ═══════════════════════════════════════════════════════════════════════════════
#  Obstacle tracker  (nearest moving vehicle-sized cluster → pur_obs)
# ═══════════════════════════════════════════════════════════════════════════════

class ObstacleTracker:
    MIN_WIDTH    = 0.40
    MAX_WIDTH    = 4.00   # reject walls / multi-object scatter
    TRACK_RANGE  = 30.0
    CORRIDOR_R   = 16.0   # m — only consider points near the junction conflict zone
    SELF_IGNORE  = 1.3    # m — exclude ego body / sensor mount
    CLUSTER_R    = 2.0    # m — radius around nearest foreground point to cluster

    VEL_ALPHA = 0.3   # EMA smoothing — lower = more lag, higher = noisier

    def __init__(self, odo: LockedIntegrator):
        self._odo    = odo
        self._lock   = threading.Lock()
        self._obs    : Optional[Tuple[float, float, float, float]] = None
        self._prev_c : Optional[Tuple[float, float, float]]        = None
        self._vel    : Tuple[float, float]                         = (0.0, 0.0)

    def update(self, hw_lidar: HWLidar):
        hits = hw_lidar._snapshot()
        if len(hits) < OUSTER_MIN_PTS:
            with self._lock: self._obs = None; return

        ex, ey, eyaw = self._odo.pose()
        cy, sy = math.cos(eyaw), math.sin(eyaw)
        jx, jy = JUNCTION_XY
        wpts = []

        CLUSTER_Z_MIN = -0.7   # m — ignore ground returns (bushes, kerb)
        CLUSTER_Z_MAX =  0.1   # m — ignore high returns (tree branches)
        for hx, hy, hz in hits:
            if hz < CLUSTER_Z_MIN or hz > CLUSTER_Z_MAX:
                continue    # Reject points too close to ego (body/sensor artefacts) or too far (beyond tracking range)
            dist = math.hypot(hx, hy)
            if dist < self.TRACK_RANGE:
                wx = ex + hx * cy - hy * sy
                wy = ey + hx * sy + hy * cy
                if math.hypot(wx - jx, wy - jy) < self.CORRIDOR_R:
                    wpts.append((wx, wy))

        if len(wpts) < OUSTER_MIN_PTS:
            with self._lock: self._obs = None; return

        # Find nearest foreground point to ego (exclude self / sensor housing)
        nearest = None
        min_d   = float('inf')
        for wx, wy in wpts:
            d = math.hypot(wx - ex, wy - ey)
            if d > self.SELF_IGNORE and d < min_d:
                min_d, nearest = d, (wx, wy)

        if nearest is None:
            with self._lock: self._obs = None; return

        # Cluster: all corridor points within CLUSTER_R of nearest foreground point
        nx, ny = nearest
        cluster = [(wx, wy) for wx, wy in wpts
                   if math.hypot(wx - nx, wy - ny) < self.CLUSTER_R]
        if len(cluster) < OUSTER_MIN_PTS:
            with self._lock: self._obs = None; return

        xs  = [p[0] for p in cluster]
        ys  = [p[1] for p in cluster]
        cx  = sum(xs) / len(xs)
        cy_ = sum(ys) / len(ys)
        w   = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if w < self.MIN_WIDTH or w > self.MAX_WIDTH:
            with self._lock: self._obs = None; return

        now  = time.time()
        pvx, pvy = self._vel
        if self._prev_c is not None:
            ox, oy, ts_old = self._prev_c
            dt = now - ts_old
            if 0 < dt < 0.5:
                raw_vx = (cx - ox) / dt
                raw_vy = (cy_ - oy) / dt
                _spd = math.hypot(raw_vx, raw_vy)
                if _spd > 5.0:
                    raw_vx, raw_vy = raw_vx * 5.0 / _spd, raw_vy * 5.0 / _spd
                a = self.VEL_ALPHA
                pvx = a * raw_vx + (1 - a) * pvx
                pvy = a * raw_vy + (1 - a) * pvy
        self._vel    = (pvx, pvy)
        self._prev_c = (cx, cy_, now)
        with self._lock:
            self._obs = (cx, cy_, pvx, pvy)

    def get(self) -> Optional[Tuple[float, float, float, float]]:
        with self._lock:
            if self._obs is None:
                return None
            _, _, pvx, pvy = self._obs
            if math.hypot(pvx, pvy) < MIN_PURSUER_SPEED:
                return None
            return self._obs

# ═══════════════════════════════════════════════════════════════════════════════
#  Geometry helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _safe_geom(g: Polygon) -> Polygon:
    if g is None or g.is_empty:
        return Polygon()
    if not g.is_valid:
        try:    g = make_valid(g)
        except: g = g.buffer(0)
    if g.geom_type == 'GeometryCollection':
        ps = [x for x in g.geoms if x.geom_type in ('Polygon', 'MultiPolygon')]
        g = max(ps, key=lambda x: x.area) if ps else Polygon()
    return g if not g.is_empty else Polygon()

def _perp(wps, i):
    n = len(wps); j = min(i + 1, n - 1); k = max(i, 0) if j == i else i
    dx, dy = wps[j][0] - wps[k][0], wps[j][1] - wps[k][1]
    d = math.hypot(dx, dy)
    return (-dy / d, dx / d) if d > 0.01 else (0.0, 1.0)

def build_road_polygon(wps: list, hw: float = 5.0) -> Polygon:
    L, R = [], []
    for i in range(len(wps)):
        px, py = _perp(wps, i); x, y = wps[i]
        L.append((x + hw * px, y + hw * py))
        R.append((x - hw * px, y - hw * py))
    R.reverse()
    return _safe_geom(Polygon(L + R))

def braking_distance(v: float, a: float) -> float:
    return v ** 2 / (2.0 * abs(a))

def compute_danger_zone(e, p) -> Polygon:
    d  = braking_distance(e.speed, p.max_decel) + p.safe_stop_margin
    c, s, hw = math.cos(e.yaw), math.sin(e.yaw), 3.0
    def f(dd, ll): return (e.x + dd * c - ll * s, e.y + dd * s + ll * c)
    return Polygon([f(0, -hw), f(d, -hw), f(d, hw), f(0, hw)])

def theorem1_check(h, d, min_overlap_area: float = 2.0) -> bool:
    if not h or h.is_empty: return True
    h, d = _safe_geom(h), _safe_geom(d)
    if h.is_empty or d.is_empty: return True
    return h.intersection(d).area < min_overlap_area

# ═══════════════════════════════════════════════════════════════════════════════
#  Dataclasses
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class EgoState:
    x: float = 0.0; y: float = 0.0; yaw: float = 0.0
    speed: float = 0.0; ax: float = 0.0

@dataclass
class PlannerParams:
    dt:               float = 1.0 / CONTROL_HZ
    max_decel:        float = PLANNER_MAX_DECEL
    max_accel:        float = PLANNER_MAX_ACCEL
    max_speed:        float = MAX_SPEED_MPS
    pursuer_max_speed:float = PURSUER_MAX_SPEED
    sensor_noise_m:   float = 0.40
    safe_stop_margin: float = 1.50
    horizon_steps:    int   = 60
    lidar_range:      float = LIDAR_RANGE
    wheelbase:        float = WHEELBASE_M

@dataclass
class HWCtrl:
    speed_mps: float = 0.0
    steer_deg: float = 0.0
    brake:     bool  = False
    reverse:   bool  = False

# ═══════════════════════════════════════════════════════════════════════════════
#  Hidden-set tracker
# ═══════════════════════════════════════════════════════════════════════════════

class HiddenSetTracker:
    def __init__(s, rp, p):
        s.rp, s.p, s._h, s.elapsed = rp, p, None, 0.0

    def incorporate_shadow(s, shadow_on_road):
        sh = _safe_geom(shadow_on_road)
        if sh.is_empty: return
        inflated   = _safe_geom(sh.buffer(s.p.sensor_noise_m))
        new_region = _safe_geom(inflated.intersection(_safe_geom(s.rp)))
        if new_region.is_empty: return
        if s._h is None or s._h.is_empty:
            s._h = new_region; s.elapsed = 0.0
        else:
            s._h = _safe_geom(s._h.union(new_region))

    def propagate(s, dt):
        if not s._h or s._h.is_empty: return
        s._h = _safe_geom(
            _safe_geom(s._h).buffer(s.p.pursuer_max_speed * dt)
            .intersection(_safe_geom(s.rp)))
        s.elapsed += dt

    def prune_observed(s, cl):
        if not s._h or s._h.is_empty: return
        s._h = _safe_geom(_safe_geom(s._h).difference(_safe_geom(cl)))

    def prune_by_velocity_bound(s, L):
        if s.elapsed < 0.5 or not s._h or s._h.is_empty: return
        sh = min(1.0, (L / s.elapsed) / s.p.pursuer_max_speed)
        if sh < 1:
            s._h = _safe_geom(
                poly_scale(s._h, sh, sh, origin=s._h.centroid)
                .intersection(_safe_geom(s.rp)))

    @property
    def polygon(s): return s._h
    def is_empty(s): return not s._h or s._h.is_empty

# ═══════════════════════════════════════════════════════════════════════════════
#  Evasive policy  (hidden-set primitive selector, used when no pursuer observed)
# ═══════════════════════════════════════════════════════════════════════════════

class EvasivePolicy:
    def __init__(s, p): s.p = p

    def _sim(s, e, n, af):
        t, st = [], EgoState(e.x, e.y, e.yaw, e.speed)
        for _ in range(n):
            ns = max(0.0, min(s.p.max_speed, st.speed + af(st) * s.p.dt))
            st = EgoState(st.x + ns * math.cos(st.yaw) * s.p.dt,
                          st.y + ns * math.sin(st.yaw) * s.p.dt, st.yaw, ns)
            t.append(st)
            if ns == 0 and af(st) <= 0: break
        return t

    def select_primitive(s, e, ht, rp) -> str:
        for nm, fn in [('BRAKE', lambda _: s.p.max_decel),
                       ('PUSH',  lambda _: s.p.max_accel)]:
            ok, hs = True, copy.deepcopy(ht)
            for f in s._sim(e, s.p.horizon_steps, fn):
                hs.propagate(s.p.dt)
                if not theorem1_check(hs.polygon, compute_danger_zone(f, s.p)):
                    ok = False; break
            if ok: return nm
        return 'BRAKE'

# ═══════════════════════════════════════════════════════════════════════════════
#  Pure pursuit — forward and reverse, returns road-wheel degrees
# ═══════════════════════════════════════════════════════════════════════════════

def pp_steer_deg(e: EgoState, tgt, wb: float) -> float:
    dx, dy = tgt[0] - e.x, tgt[1] - e.y
    ld = math.hypot(dx, dy)
    if ld < 0.1:
        return 0.0
    alpha     = (math.atan2(dy, dx) - e.yaw + math.pi) % (2 * math.pi) - math.pi
    steer_rad = math.atan2(2.0 * wb * math.sin(alpha), ld)
    return max(-MAX_STEER_DEG, min(MAX_STEER_DEG, math.degrees(steer_rad)))

def pp_steer_rev_deg(e: EgoState, tgt, wb: float) -> float:
    """Pure-pursuit for reversing: treats yaw+π as the reference heading."""
    dx, dy = tgt[0] - e.x, tgt[1] - e.y
    ld = math.hypot(dx, dy)
    if ld < 0.1:
        return 0.0
    rev_yaw   = e.yaw + math.pi
    alpha     = (math.atan2(dy, dx) - rev_yaw + math.pi) % (2 * math.pi) - math.pi
    steer_rad = math.atan2(2.0 * wb * math.sin(alpha), ld)
    return -max(-MAX_STEER_DEG, min(MAX_STEER_DEG, math.degrees(steer_rad)))

# ═══════════════════════════════════════════════════════════════════════════════
#  FSM state labels
# ═══════════════════════════════════════════════════════════════════════════════

class ST:
    APPROACHING = "APPROACHING"
    PEEKING     = "PEEKING"
    YIELDING    = "YIELDING"
    PROCEEDING  = "PROCEEDING"
    EVADING     = "EVADING"

# ═══════════════════════════════════════════════════════════════════════════════
#  Planner  (adapted from scenario_v3.py — returns HWCtrl)
# ═══════════════════════════════════════════════════════════════════════════════

class Planner:
    def __init__(s, p: PlannerParams, rp: Polygon,
                 entry: Tuple[float, float], hw_lidar: HWLidar):
        s.p, s.rp, s.entry = p, rp, entry
        s.ht    = HiddenSetTracker(rp, p)
        s.ev    = EvasivePolicy(p)
        s.lidar = hw_lidar
        s.state = ST.APPROACHING
        s._dbg_count    = 0
        s._evade_ticks  = 0
        s._resume_ticks = 0
        s._mode         = 'OPEN'
        s._pur_converging = False
        # Lateral dodge
        s._dodge_active  = False
        s._dodge_start_x = 0.0
        s._dodge_start_y = 0.0
        s._dodge_lat_x   = 0.0   # toward-curb unit vector, saved at dodge start
        s._dodge_lat_y   = 0.0
        # Curb-recovery (stuck in PROCEEDING)
        s._recovering       = False
        s._recover_ticks    = 0
        s._recover_attempts = 0
        s._post_recover     = False
        # NR last-resort reverse
        s._nr_reversing = False
        s._rev_wi       = 0
        s._rev_wi_min   = 0
        s._nr_alerted   = False
        # Vehicle OBB dims (can be updated from actual bounding box if available)
        s.ego_hl, s.ego_hw = EGO_HL, EGO_HW
        s.pur_hl, s.pur_hw = PUR_HL, PUR_HW

    # ── Threat helpers ─────────────────────────────────────────────────────────

    def _observed_threat(s, e: EgoState, pur_obs) -> bool:
        if pur_obs is None: return False
        px, py, pvx, pvy = pur_obs
        dx, dy = e.x - px, e.y - py
        dist = math.hypot(dx, dy)
        if dist > s.p.lidar_range: return False
        if dist < 2.0: return True   # within combined footprint — always a threat
        if not s.lidar.is_visible((px, py), (e.x, e.y), e.yaw, threshold=3.0):
            return False
        dz = _safe_geom(compute_danger_zone(e, s.p))
        if dz.is_empty: return False
        if dz.contains(Point(px, py)): return True
        # Include ego's own velocity toward the pursuer so TTC is accurate
        # when both are moving toward each other (head-on or same-road approach).
        ego_vx = e.speed * math.cos(e.yaw)
        ego_vy = e.speed * math.sin(e.yaw)
        closing = ((pvx - ego_vx) * dx + (pvy - ego_vy) * dy) / dist
        if closing > 1.0 and dist / closing < 4.0: return True
        return False

    def _push_clears_conflict(s, e: EgoState, pur_obs) -> bool:
        """True only if sprinting forward is demonstrably safe.

        Safety gates (returns False immediately if any fails):
          1. Pursuer head-on and closing fast → sprinting increases closing rate, never safe.
          2. TTC check: ego must clear the full crossing width before pursuer arrives.
        Falls back to hidden-set theorem-1 when no pursuer is observed.
        """
        if pur_obs is not None:
            px, py, pvx, pvy = pur_obs
            dx, dy = e.x - px, e.y - py
            dist = math.hypot(dx, dy)
            if dist < 0.1: return False
            closing = (pvx * dx + pvy * dy) / dist
            if closing <= 0.5: return True   # pursuer moving away / lateral — safe to go
            # Gate 1: head-on check — pursuer in front and closing toward ego
            ego_fx, ego_fy = math.cos(e.yaw), math.sin(e.yaw)
            in_front = ego_fx * (px - e.x) + ego_fy * (py - e.y)
            pur_spd = math.hypot(pvx, pvy)
            if in_front > 0 and pur_spd > 0.5:
                toward_ego = dx * pvx + dy * pvy
                if toward_ego > 0:
                    return False   # head-on: never sprint
            # Gate 2: TTC — ego must clear crossing before pursuer arrives
            ttc = dist / closing
            CROSSING_WIDTH = 8.0
            jx, jy = s.entry
            dist_to_entry = math.hypot(e.x - jx, e.y - jy)
            if s.state == ST.PEEKING:
                d_clear = dist_to_entry + CROSSING_WIDTH + s.p.safe_stop_margin
            else:
                d_clear = max(CROSSING_WIDTH - dist_to_entry, 0.0) + s.p.safe_stop_margin
            return d_clear / s.p.max_speed + 0.5 < ttc
        return s.ev.select_primitive(e, s.ht, s.rp) == 'PUSH'

    # ── Main step ──────────────────────────────────────────────────────────────

    def step(s, e: EgoState, wps: list, wi: int,
             pur_obs=None) -> Tuple[HWCtrl, str, int]:
        p = s.p

        # ── 1. Curb-recovery: reverse briefly, then resume ────────────────────
        if s._recovering:
            s._recover_ticks += 1
            if s._recover_ticks < 40:   # 2 s at 20 Hz
                return HWCtrl(speed_mps=REV_SPEED_MPS, steer_deg=0.0, reverse=True), s.state, wi
            s._recovering  = False
            s._recover_ticks = 0
            s._resume_ticks  = 0
            s._post_recover  = True
            print("[RECOVER-done] resuming forward drive (post-recover slow mode)")

        # ── 2. NR last-resort reverse: back up until OBB gap clears ──────────
        if s._nr_reversing:
            if pur_obs is not None:
                _pur_yaw = (math.atan2(pur_obs[3], pur_obs[2])
                            if math.hypot(pur_obs[2], pur_obs[3]) > 0.1 else 0.0)
                _rgap = obb_gap((e.x, e.y), e.yaw, s.ego_hl, s.ego_hw,
                                (pur_obs[0], pur_obs[1]), _pur_yaw,
                                s.pur_hl, s.pur_hw)
                if _rgap > NR_CLEAR_GAP:
                    s._nr_reversing = False
                    s._nr_alerted   = False
                    print(f"[NR-REV-done] gap={_rgap:.1f}m — pursuer clear, "
                          f"resuming EVADING hold")
            if s._nr_reversing:
                rev_tgt = wps[s._rev_wi]
                if (math.hypot(e.x - rev_tgt[0], e.y - rev_tgt[1]) < 2.0
                        and s._rev_wi > s._rev_wi_min):
                    s._rev_wi -= 1
                    rev_tgt = wps[s._rev_wi]
                steer = pp_steer_rev_deg(e, rev_tgt, p.wheelbase)
                return HWCtrl(speed_mps=REV_SPEED_MPS, steer_deg=steer, reverse=True), s.state, wi

        # ── 3. Shadow / clear / hidden-set update ─────────────────────────────
        # Single snapshot so shadow and clear are computed from the same scan frame.
        _hits, _ = s.lidar.snapshot_and_bins()
        shadow = s.lidar.get_shadow_polygon((e.x, e.y), s.rp, ego_yaw=e.yaw, _hits=_hits)
        clear  = s.lidar.get_clear_polygon((e.x, e.y), ego_yaw=e.yaw, _hits=_hits)

        if not shadow.is_empty and s._dbg_count < 5:
            s._dbg_count += 1
            print(f"  [DBG] shadow={shadow.area:.1f}m²  "
                  f"clear={clear.area if not clear.is_empty else 0:.1f}m²  "
                  f"yaw={math.degrees(e.yaw):.1f}°  hits={len(s.lidar._snapshot())}")

        if s.state in (ST.APPROACHING, ST.PEEKING):
            s.ht.incorporate_shadow(shadow)
        s.ht.propagate(p.dt)
        s.ht.prune_observed(clear)
        s.ht.prune_by_velocity_bound(15)

        dz        = compute_danger_zone(e, p)
        safe      = theorem1_check(s.ht.polygon, dz)
        de        = math.hypot(e.x - s.entry[0], e.y - s.entry[1])
        ri        = wi
        obs_threat = s._observed_threat(e, pur_obs)

        # Mode logging
        if obs_threat and s._mode == 'OPEN':
            s._mode = 'CLOSED'
            if pur_obs:
                px, py, pvx, pvy = pur_obs
                d  = math.hypot(e.x - px, e.y - py)
                cl = (pvx * (e.x - px) + pvy * (e.y - py)) / d if d > 0 else 0
                print(f"  [MODE→ClosedLoop] pursuer ({px:.1f},{py:.1f}) "
                      f"dist={d:.1f}m closing={cl:.1f}m/s")
        elif not obs_threat and s._mode == 'CLOSED' and s.state == ST.PROCEEDING:
            s._mode = 'OPEN'
            print(f"  [MODE→OpenLoop]   pursuer cleared")

        # ── 4. FSM transitions ────────────────────────────────────────────────
        if s.state == ST.APPROACHING:
            if de < 25: s.state = ST.PEEKING

        elif s.state == ST.PEEKING:
            # Hold PEEKING if pursuer is known and approaching the junction
            pur_converging = False
            if pur_obs is not None:
                px, py, pvx, pvy = pur_obs
                _pur_dist = math.hypot(e.x - px, e.y - py)
                _pur_spd  = math.hypot(pvx, pvy)
                # Velocity component toward junction entry
                jx, jy = s.entry
                djx, djy = jx - px, jy - py
                dist_to_junc = math.hypot(djx, djy)
                if dist_to_junc > 0.1:
                    vdot_junc = (pvx * djx + pvy * djy) / dist_to_junc
                    if _pur_spd > 0.3 and _pur_dist < 60.0 and vdot_junc > 0.5:
                        pur_converging = True
                        if s.ht.is_empty() and not obs_threat and not s._pur_converging:
                            print(f"  [PEEKING-HOLD] pursuer converging toward junction "
                                  f"pur=({px:.1f},{py:.1f}) dist={_pur_dist:.1f}m "
                                  f"spd={_pur_spd:.1f}m/s")
            s._pur_converging = pur_converging

            PEEK_HOLD_DIST  = 4.0   # m — switch from H̃=∅ to theorem-1 when this close
            PEEK_EVADE_DIST = 8.0   # m — hidden-set EVADING only within this distance
            _proceed_ok = (s.ht.is_empty() if de >= PEEK_HOLD_DIST else safe)
            if _proceed_ok and not obs_threat and not pur_converging:
                s.state = ST.PROCEEDING
            elif obs_threat or (not safe and de < PEEK_EVADE_DIST):
                _obs = pur_obs if obs_threat else None
                if s._push_clears_conflict(e, _obs):
                    print("  [PUSH-THROUGH] timing clears conflict — proceeding")
                    s.state = ST.PROCEEDING
                else:
                    s.state = ST.EVADING; s._evade_ticks = 0

        elif s.state == ST.PROCEEDING:
            if not safe or obs_threat:
                _obs = pur_obs if obs_threat else None
                if not s._push_clears_conflict(e, _obs):
                    s.state = ST.EVADING; s._evade_ticks = 0
                elif s._mode == 'CLOSED' and pur_obs is not None:
                    if not s._push_clears_conflict(e, pur_obs):
                        s.state = ST.EVADING; s._evade_ticks = 0

        elif s.state == ST.YIELDING:
            if s.ht.is_empty() and safe: s.state = ST.PROCEEDING

        elif s.state == ST.EVADING:
            s._evade_ticks += 1
            if (safe and e.speed < 0.5 and s._evade_ticks >= 20 and not obs_threat):
                ds = [math.hypot(e.x - w[0], e.y - w[1]) for w in wps]
                ri = _aligned_wp(e, wps, int(np.argmin(ds)))
                s._evade_ticks    = 0
                s._resume_ticks   = 0
                s._dodge_active   = False
                s._post_recover   = False
                s._nr_reversing   = False
                s.state = ST.PROCEEDING

        # ── 5. Near-range OBB sensor ──────────────────────────────────────────
        #
        #  Covers the LiDAR near-range blind-spot.  Computes the SAT gap between
        #  ego and pursuer footprints; fires before bodies overlap.
        #  Priority: PUSH → LATERAL-DODGE → CURB-CLIMB → NR-REVERSE.
        NR_EDGE      = 2.0   # m — proximity alert threshold (logging only, no action)
        NR_DODGE_EDGE = 4.0  # m — front lateral-dodge advance trigger
        if pur_obs is not None:
            _px, _py, _pvx, _pvy = pur_obs
            _pur_spd  = math.hypot(_pvx, _pvy)
            _pur_yaw  = math.atan2(_pvy, _pvx) if _pur_spd > 0.1 else 0.0
            _nr_gap   = obb_gap((e.x, e.y), e.yaw, s.ego_hl, s.ego_hw,
                                (_px, _py), _pur_yaw, s.pur_hl, s.pur_hw)
            _fwdx, _fwdy = math.cos(e.yaw), math.sin(e.yaw)
            _in_front = _fwdx * (_px - e.x) + _fwdy * (_py - e.y)

            if _nr_gap < NR_EDGE:
                _nr_side = "FRONT" if _in_front > 0 else "REAR"
                if not s._nr_alerted:
                    print(f"  [NR-SENSOR !!] gap={_nr_gap:.2f}m {_nr_side}  "
                          f"pur=({_px:.1f},{_py:.1f}) ego=({e.x:.1f},{e.y:.1f})")
                    s._nr_alerted = True
                # Pursuer in rear = they have passed behind the stopped ego.
                # In the three-actor model this is the successful EVADING outcome.
                # The normal RESUME condition (safe + evade_ticks≥20 + not obs_threat)
                # will fire once the pursuer is clear — no sprint needed.

            # Front near-range: pursuer closing while EVADING — 4-way priority
            # Lateral-dodge trigger:
            #   Primary — obs_threat: pursuer seen and in front → evaluate priority tree.
            #   Secondary — NR gap: pursuer within NR_DODGE_EDGE (blind-spot cover).
            _dodge_trigger = (obs_threat and _in_front > 0) or (0 < _nr_gap < NR_DODGE_EDGE and _in_front > 0)
            if _dodge_trigger and s.state == ST.EVADING:
                if not s._dodge_active:
                    _push_safe = s._push_clears_conflict(e, pur_obs)
                    if _push_safe:
                        # 1. PUSH: timing clears conflict and pursuer is NOT head-on
                        print(f"  [NR-PUSH] gap={_nr_gap:.2f}m — timing clears, sprinting")
                        ds = [math.hypot(e.x - w[0], e.y - w[1]) for w in wps]
                        ri = _aligned_wp(e, wps, int(np.argmin(ds)))
                        s.state         = ST.PROCEEDING
                        s._evade_ticks  = 0; s._resume_ticks = 0
                        s._dodge_active = False; s._post_recover = False
                    else:
                        _lftx, _lfty = -_fwdy, _fwdx  # left perpendicular to ego heading
                        _lat_x = _lftx * DODGE_LAT_SIGN
                        _lat_y = _lfty * DODGE_LAT_SIGN
                        _clearance = (ROAD_CURB_Y - e.y) * _lat_y
                        # _clearance = (ROAD_CURB_X - e.x) * _lat_x

                        if _clearance > DODGE_LIMIT_M + 0.1:
                            # 2. LATERAL-DODGE: full bounded swerve
                            print(f"  [LATERAL-DODGE] gap={_nr_gap:.2f}m  "
                                  f"clearance={_clearance:.2f}m — bounded swerve")
                            s._dodge_active  = True
                            s._dodge_start_x = e.x
                            s._dodge_start_y = e.y
                            s._dodge_lat_x   = _lat_x
                            s._dodge_lat_y   = _lat_y
                        elif _clearance > 0:
                            # 3. CURB-CLIMB: last-resort partial swerve
                            print(f"  [LATERAL-DODGE-CURB] gap={_nr_gap:.2f}m  "
                                  f"clearance={_clearance:.2f}m — curb-climb last-resort")
                            s._dodge_active  = True
                            s._dodge_start_x = e.x
                            s._dodge_start_y = e.y
                            s._dodge_lat_x   = _lat_x
                            s._dodge_lat_y   = _lat_y
                        elif not s._nr_reversing:
                            # 4. NR-REVERSE: no room — back up along waypoints
                            print(f"  [NR-REVERSE] gap={_nr_gap:.2f}m  "
                                  f"clearance={_clearance:.2f}m — reversing to safe waypoint")
                            s._nr_reversing = True
                            s._rev_wi       = max(0, wi - 1)
                            s._rev_wi_min   = max(0, wi - REV_WPS_BACK)
                            s._nr_alerted   = True

            # ── Mirror-follower abort ─────────────────────────────────────────
            # If the full DODGE_LIMIT_M swerve is complete and the pursuer is
            # still closing from the front, they mirrored the lateral move.
            # No further swerve can help (clearance exhausted; they're faster).
            # Safest response: hard stop. A stationary contact at 2 m/s is lower
            # energy than a moving collision at 3+ m/s combined speed.
            if s._dodge_active and obs_threat and _in_front > 0:
                _lat_disp = ((e.x - s._dodge_start_x) * s._dodge_lat_x +
                             (e.y - s._dodge_start_y) * s._dodge_lat_y)
                if _lat_disp >= DODGE_LIMIT_M:
                    print(f"  [DODGE-ABORT] full {DODGE_LIMIT_M:.1f}m swerve done, "
                          f"pursuer still closing — hard stop")
                    s._dodge_active = False   # EVADING + no dodge → apply_brake=True

            elif _nr_gap >= NR_DODGE_EDGE and not obs_threat:
                s._nr_alerted   = False
                s._dodge_active = False   # pursuer clear on both sensors; end dodge

        # ── 6. Lookahead and speed ────────────────────────────────────────────
        RESUME_PHASE  = 40
        JUNCTION_ZONE = 10.0   # m from entry — keep PEEKING speed while crossing

        if s.state == ST.PROCEEDING and s._resume_ticks < RESUME_PHASE:
            s._resume_ticks += 1

        def _curv(i):
            if i < 1 or i >= len(wps) - 1: return 0.0
            ax, ay = wps[i][0] - wps[i-1][0], wps[i][1] - wps[i-1][1]
            bx, by = wps[i+1][0] - wps[i][0],  wps[i+1][1] - wps[i][1]
            cross  = abs(ax * by - ay * bx)
            return cross / max(math.hypot(ax, ay) * math.hypot(bx, by), 0.01)

        curv = _curv(wi)
        if s._resume_ticks > 0 and s._resume_ticks < RESUME_PHASE:
            look_ahead = 1
        else:
            look_ahead = 2 if curv > 0.15 else (3 if curv > 0.05 else 5)
        lk = wps[min(wi + look_ahead, len(wps) - 1)]

        # Lateral-dodge lookahead override
        if s._dodge_active:
            fwdx, fwdy = math.cos(e.yaw), math.sin(e.yaw)
            lat_disp = ((e.x - s._dodge_start_x) * s._dodge_lat_x +
                        (e.y - s._dodge_start_y) * s._dodge_lat_y)
            if lat_disp > DODGE_LIMIT_M:
                # Phase 2: return toward lane (1 m back)
                lk = (e.x + fwdx * 2.0 - s._dodge_lat_x,
                      e.y + fwdy * 2.0 - s._dodge_lat_y)
            else:
                # Phase 1: swerve toward curb (1 m lateral)
                lk = (e.x + fwdx * 2.0 + s._dodge_lat_x,
                      e.y + fwdy * 2.0 + s._dodge_lat_y)
        elif s._post_recover:
            # Force close forward target for maximum heading-correction authority
            fwdx, fwdy = math.cos(e.yaw), math.sin(e.yaw)
            lk = (e.x + fwdx * 2.0, e.y + fwdy * 2.0)

        # Target speed per state
        if s.state == ST.APPROACHING:
            target_v = p.max_speed * 0.60
        elif s.state == ST.PEEKING:
            # Hold only on a confirmed moving pursuer converging to junction.
            # H̃-based holds (ht.is_empty / theorem-1) are not used here because:
            #  - ht.is_empty() false-triggers on any static shadow (trees, shoulder walls)
            #  - theorem-1 false-triggers mid-turn (forward danger zone sweeps into turn-wall shadow)
            # At PEEKING speed (30% = 0.42 m/s), braking distance is ~3 cm; the car
            # can stop essentially instantly, so creeping is as safe as holding.
            # The PEEKING→EVADING transition at de < PEEK_EVADE_DIST handles genuine threats.
            if s._pur_converging:
                target_v = 0.0   # hold: tracked pursuer confirmed heading for junction
            else:
                target_v = p.max_speed * 0.30
        elif s.state in (ST.YIELDING, ST.EVADING):
            if s._dodge_active:
                target_v = p.max_speed   # full speed during dodge — need lateral displacement before contact
            else:
                target_v = 0.0
                if s.state == ST.EVADING:
                    reason = "obs" if obs_threat else "hidden"
                    print(f"  [EVADING] BRAKE  v={e.speed:.2f}m/s  reason={reason}")
        elif s.state == ST.PROCEEDING:
            if s._post_recover:
                target_v = 0.5   # crawl until heading re-aligns
                # Clear post_recover after RESUME_PHASE ticks
                if s._resume_ticks >= RESUME_PHASE:
                    s._post_recover = False
            elif de < JUNCTION_ZONE:
                target_v = p.max_speed * 0.30
            elif s._resume_ticks > 0 and s._resume_ticks < RESUME_PHASE:
                target_v = p.max_speed * 0.30
            else:
                target_v = p.max_speed
        else:
            target_v = 0.0

        # Curvature speed cap
        if curv > 0.05:
            cap = 0.60 if curv > 0.15 else 0.833
            target_v = min(target_v, cap)

        # Final-approach slow-down
        goal_dist = math.hypot(e.x - wps[-1][0], e.y - wps[-1][1])
        if goal_dist < 2.0:
            target_v = min(target_v, p.max_speed * 0.25)

        # Brake when explicitly stopping (EVADING/YIELDING) or when holding at zero speed
        # in PEEKING — prevents coasting on slopes.
        apply_brake = ((s.state in (ST.EVADING, ST.YIELDING)) and not s._dodge_active) \
                      or (target_v == 0.0 and s.state == ST.PEEKING and not s._dodge_active)
        steer_deg   = pp_steer_deg(e, lk, p.wheelbase)


        # Smoothen out the velocity changes
        if apply_brake:
            commanded_v = 0.0
        elif target_v > e.speed:
            commanded_v = min(target_v, e.speed + p.max_accel * p.dt)
        elif target_v < e.speed:
            commanded_v = max(target_v, e.speed + p.max_decel * p.dt)
        else:
            commanded_v = target_v

        ctrl = HWCtrl(speed_mps=commanded_v, steer_deg=steer_deg, brake=apply_brake)
        return ctrl, s.state, ri

# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='scenario_v3 hardware — PIX Hooke')
    parser.add_argument('--channel',   default=CAN_CHANNEL)
    parser.add_argument('--ouster-ip', default=OUSTER_IP)
    args = parser.parse_args()

    wps  = load_waypoints(EGO_WAYPOINTS_RAW)
    xwps = load_waypoints(CROSS_ROAD_POINTS_RAW)
    if len(wps) < 2:
        print("ERROR: need ≥2 waypoints in EGO_WAYPOINTS_RAW"); return
    print(f"[WP  ] {len(wps)} waypoints  start={wps[0]}  goal={wps[-1]}")

    rpoly = build_road_polygon(wps + xwps, hw=6.0)
    p     = PlannerParams()

    stop_event = threading.Event()
    odo        = LockedIntegrator()
    hw_lidar   = HWLidar(p.lidar_range)
    tracker    = ObstacleTracker(odo)

    _script_dir   = os.path.dirname(os.path.abspath(__file__))
    _arbiter_path = os.path.join(_script_dir, 'jetson_ouster_arbiter.py')

    # Kill arbiter before starting ouster_thread so ports 7502/7503 are free
    subprocess.run(['pkill', '-f', 'jetson_ouster_arbiter.py'], check=False)
    time.sleep(1.0)
    print("[OS1 ] Arbiter stopped — Ouster UDP ports released", flush=True)

    ydl_guard = YDLidarGuard()
    threading.Thread(target=can_reader,
                     args=(odo, stop_event, args.channel), daemon=True).start()
    threading.Thread(target=ouster_thread,
                     args=(hw_lidar, tracker, stop_event, args.ouster_ip), daemon=True).start()
    threading.Thread(target=ydl_guard.listen,
                     args=(stop_event,), daemon=True).start()

    tx      = _tx_sock()
    planner = Planner(p, rpoly, JUNCTION_XY, hw_lidar)
    step_s  = p.dt
    wi      = 0
    last_state       = None
    stuck_window     = int(STUCK_WINDOW_S * CONTROL_HZ)
    pos_history      = deque(maxlen=stuck_window)
    _guard_hold_ticks = 0          # consecutive ticks any guard is blocking PROCEEDING
    GUARD_EVADE_TICKS = int(3.0 * CONTROL_HZ)  # 3 s of guard-hold → force EVADING

    print("=" * 65)
    print("  SCENARIO V3  HARDWARE  —  PIX Hooke + Ouster OS1 + YDLidar")
    print(f"  Waypoints   : {len(wps)}")
    print(f"  Junction    : {JUNCTION_XY}")
    print(f"  Max speed   : {p.max_speed * 3.6:.1f} km/h")
    print(f"  CAN channel : {args.channel}  (odometry RX only)")
    print(f"  Ouster IP   : {args.ouster_ip}")
    print(f"  Command UDP : :{SCENARIO_PORT}  (→ can_publisher)")
    print(f"  Dodge curb  : ROAD_CURB_Y={ROAD_CURB_Y}  DODGE_LAT_SIGN={DODGE_LAT_SIGN:+d}")
    print("  Ensure can_publisher.py is running.")
    print("  Set remote to AUTONOMOUS (rod 6 ↓) then press ENTER.")
    print("=" * 65)
    input()

    t_start = time.time()

    try:
        while True:
            t0      = time.time()
            elapsed = t0 - t_start

            # ── Run time cap ──────────────────────────────────────────────────
            if elapsed > MAX_RUN_S:
                print(f"[TIMEOUT] {MAX_RUN_S:.0f} s cap reached — stopping.")
                break

            # ── Stale odometry ────────────────────────────────────────────────
            odo_age = odo.odo_age()
            if odo_age > ODO_STALE_S:
                send_command(tx, 'brake', 0.0, 0.0, 'STALE_ODO')
                print(f"\r  [SAFE] Odometry stale ({odo_age:.2f}s) — brake   ",
                      end='', flush=True)
                time.sleep(max(0.0, step_s - (time.time() - t0)))
                continue

            # ── Stale LiDAR ───────────────────────────────────────────────────
            if OUSTER_AVAILABLE:
                lidar_age = hw_lidar.scan_age()
                if lidar_age > LIDAR_STALE_S:
                    send_command(tx, 'brake', 0.0, 0.0, 'STALE_LIDAR')
                    print(f"\r  [SAFE] LiDAR stale ({lidar_age:.2f}s) — brake   ",
                          end='', flush=True)
                    time.sleep(max(0.0, step_s - (time.time() - t0)))
                    continue

            # ── Background thread abort ───────────────────────────────────────
            if stop_event.is_set():
                print("\n[ABORT] Background thread signalled stop.")
                break

            # ── Ego state ─────────────────────────────────────────────────────
            ex, ey, eyaw = odo.pose()
            spd = odo.current_speed()
            e   = EgoState(x=ex, y=ey, yaw=eyaw, speed=spd)

            pur_obs = tracker.get()

            # ── Waypoint advance ──────────────────────────────────────────────
            if wi < len(wps) - 1:
                if math.hypot(ex - wps[wi][0], ey - wps[wi][1]) < 0.8:
                    wi += 1

            # ── Goal check ────────────────────────────────────────────────────
            if math.hypot(ex - wps[-1][0], ey - wps[-1][1]) < 0.5:
                print(f"[DONE] Goal reached  ({ex:.2f},{ey:.2f})")
                break

            # ── Planner ───────────────────────────────────────────────────────
            try:
                ctrl, fsm, ri = planner.step(e, wps, wi, pur_obs)
            except Exception as exc:
                print(f"\n  [SAFE] Planner exception: {exc!r} — braking")
                send_command(tx, 'brake', 0.0, 0.0, 'PLANNER_ERR')
                time.sleep(max(0.0, step_s - (time.time() - t0)))
                continue

            if ri != wi:
                print(f"  [RESUME] wp {wi}→{ri}  [{planner._mode}]")
                wi = ri

            # ── Chassis-geometry guard thresholds ────────────────────────────
            # Fixed margins from chassis surface (sensor + distance to bumper).
            # E-STOP (20 cm from bumper): always active, overrides dodge.
            # BRAKE  (30 cm from bumper): exempted during active dodge so lateral
            #   displacement can proceed once the evasive manoeuvre is committed.

            # ── Raw front guard (Ouster) ──────────────────────────────────────
            _fg = _front_guard_dist(hw_lidar)
            _guard_firing = False
            if not ctrl.reverse:
                if _fg < OUSTER_ESTOP_M:                           # E-STOP: always fires
                    if not ctrl.brake:
                        print(f"  [FRONT-GUARD E-STOP] {_fg:.2f}m (thresh={OUSTER_ESTOP_M:.2f}m)", flush=True)
                    ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                    _guard_firing = True
                elif _fg < OUSTER_BRAKE_M and not planner._dodge_active:   # BRAKE: dodge exempt
                    if not ctrl.brake:
                        print(f"  [FRONT-GUARD BRAKE] {_fg:.2f}m (thresh={OUSTER_BRAKE_M:.2f}m)", flush=True)
                    ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                    _guard_firing = True

            # ── Ouster side guard ─────────────────────────────────────────────
            _sl, _sr = _side_guard_dists(hw_lidar)
            for _sd, _label in ((_sl, 'L'), (_sr, 'R')):
                if _sd < OUSTER_ESTOP_SIDE_M:
                    if not ctrl.brake:
                        print(f"  [SIDE-GUARD E-STOP {_label}] {_sd:.2f}m (thresh={OUSTER_ESTOP_SIDE_M:.2f}m)", flush=True)
                    ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                    _guard_firing = True
                elif _sd < OUSTER_BRAKE_SIDE_M and not planner._dodge_active:
                    if not ctrl.brake:
                        print(f"  [SIDE-GUARD BRAKE {_label}] {_sd:.2f}m (thresh={OUSTER_BRAKE_SIDE_M:.2f}m)", flush=True)
                    ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                    _guard_firing = True

            # ── YDLidar front guard ───────────────────────────────────────────
            fd = ydl_guard.front_dist()
            if fd > 0 and not ctrl.reverse:
                if fd < YDL_ESTOP_M:                               # E-STOP: always fires
                    if not ctrl.brake:
                        print(f"  [YDL-GUARD E-STOP] front={fd:.2f}m (thresh={YDL_ESTOP_M:.2f}m)", flush=True)
                    ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                    _guard_firing = True
                elif fd < YDL_BRAKE_M and not planner._dodge_active:       # BRAKE: dodge exempt
                    if not ctrl.brake:
                        print(f"  [YDL-GUARD BRAKE] front={fd:.2f}m (thresh={YDL_BRAKE_M:.2f}m)", flush=True)
                    ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                    _guard_firing = True

            # ── YDLidar side guard ────────────────────────────────────────────
            _yl, _yr = ydl_guard.side_dists()
            for _yd, _label in ((_yl, 'L'), (_yr, 'R')):
                if _yd > 0:
                    if _yd < YDL_ESTOP_SIDE_M:
                        if not ctrl.brake:
                            print(f"  [YDL-SIDE E-STOP {_label}] {_yd:.2f}m (thresh={YDL_ESTOP_SIDE_M:.2f}m)", flush=True)
                        ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                        _guard_firing = True
                    elif _yd < YDL_BRAKE_SIDE_M and not planner._dodge_active:
                        if not ctrl.brake:
                            print(f"  [YDL-SIDE BRAKE {_label}] {_yd:.2f}m (thresh={YDL_BRAKE_SIDE_M:.2f}m)", flush=True)
                        ctrl = HWCtrl(speed_mps=0.0, steer_deg=ctrl.steer_deg, brake=True)
                        _guard_firing = True

            # ── Guard-hold escalation ─────────────────────────────────────────
            # If either guard blocks PROCEEDING for > 3 s straight, the planner
            # has no visibility of the hold (it thinks it's moving freely).
            # Transition to EVADING so the NR sensor / lateral-dodge can respond.
            if _guard_firing and fsm == ST.PROCEEDING:
                _guard_hold_ticks += 1
                if _guard_hold_ticks == GUARD_EVADE_TICKS:
                    print(f"  [GUARD-HOLD] {_guard_hold_ticks} ticks — forcing EVADING "
                          f"so planner can lateral-dodge or NR-reverse")
                    planner.state      = ST.EVADING
                    planner._evade_ticks = 0
            else:
                _guard_hold_ticks = 0

            # ── Send UDP command ──────────────────────────────────────────────
            if ctrl.reverse:
                action = 'reverse'
            elif ctrl.brake:
                action = 'brake'
            else:
                action = 'drive'
            send_command(tx, action, ctrl.speed_mps, ctrl.steer_deg, fsm)

            # ── Console status ────────────────────────────────────────────────
            changed    = fsm != last_state
            last_state = fsm
            tick       = int(elapsed / step_s)
            if changed or tick % 20 == 0:
                ha = planner.ht.polygon.area if not planner.ht.is_empty() else 0.0
                sf = theorem1_check(planner.ht.polygon, compute_danger_zone(e, p))
                ps = ''
                if pur_obs:
                    px, py, pvx, pvy = pur_obs
                    ps = (f' | pur=({px:+.1f},{py:+.1f})'
                          f' v={math.hypot(pvx, pvy):.1f}m/s')
                flags = ''
                if planner._dodge_active:    flags += ' [DODGE]'
                if planner._recovering:      flags += ' [RECOVER]'
                if planner._nr_reversing:    flags += ' [NR-REV]'
                if _guard_hold_ticks > 0:    flags += f' [GUARD×{_guard_hold_ticks}]'
                print(f"t={elapsed:6.1f}s | {fsm:12s} | [{planner._mode:6s}] | "
                      f"v={spd:.2f}m/s | ({ex:+.2f},{ey:+.2f}) | "
                      f"|H|={ha:.1f} | {'OK' if sf else 'UNSAFE'} | "
                      f"wp={wi}/{len(wps)-1}{ps}{flags}{'  ***' if changed else ''}")

            # ── Stuck detection ───────────────────────────────────────────────
            # Clear window during intentional holds and recovery phases.
            # EVADING holds at zero speed by design — not stuck.
            # NR-reverse is also intentional backward motion.
            # Guard-firing: vehicle is stopped by sensor, not by being physically stuck.
            if (fsm in (ST.PEEKING, ST.EVADING) or planner._recovering
                    or planner._nr_reversing or _guard_firing):
                pos_history.clear()
            else:
                pos_history.append((ex, ey))
                if len(pos_history) == pos_history.maxlen:
                    ox, oy = pos_history[0]
                    if math.hypot(ex - ox, ey - oy) < 0.30:
                        if fsm == ST.PROCEEDING and planner._recover_attempts < 3:
                            planner._recover_attempts += 1
                            print(f"[Stuck-RECOVER] attempt {planner._recover_attempts}/3 "
                                  f"— reversing off obstacle")
                            planner._recovering     = True
                            planner._recover_ticks  = 0
                            pos_history.clear()
                        else:
                            print(f"[Stuck] No displacement in {STUCK_WINDOW_S:.0f} s — stopping.")
                            break

            # ── Loop overrun warning ──────────────────────────────────────────
            loop_dt = time.time() - t0
            if loop_dt > step_s * 2.0:
                print(f"\n  [WARN] Loop overrun {loop_dt*1000:.0f} ms "
                      f"(expected {step_s*1000:.0f} ms)")

            time.sleep(max(0.0, step_s - loop_dt))

    except KeyboardInterrupt:
        print("\n[Ctrl+C] Stopping.")

    finally:
        print(f"[STOP] Sending stop commands for {STOP_DRAIN_S:.0f} s...")
        t_drain = time.time() + STOP_DRAIN_S
        while time.time() < t_drain:
            send_command(tx, 'stop', 0.0, 0.0, 'STOPPED')
            time.sleep(step_s)
        stop_event.set()
        tx.close()

        # Restart the arbiter so safety monitoring resumes after autonomous run
        try:
            log = open('/tmp/arbiter.log', 'a')
            subprocess.Popen(['python3', _arbiter_path], stdout=log, stderr=log)
            print(f"[OS1 ] Arbiter restarted → /tmp/arbiter.log", flush=True)
        except Exception as exc:
            print(f"[OS1 ] Could not restart arbiter: {exc}", flush=True)

        print("[Done]")


if __name__ == '__main__':
    main()

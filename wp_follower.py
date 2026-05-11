"""
wp_follower.py
══════════════
Waypoint follower for PIX Moving Hooke chassis.

Position source : CAN odometry (OdometryIntegrator, same as pose_disp.py)
Steering        : pure pursuit  (same geometry as scenario_v3.py pp_steer)
Speed           : fixed cruise / approach / stop stages
Obstacle stop   : UDP alerts from lidar_guard.py running on the Raspberry Pi

Paste your WAYPOINTS array below — direct output from wp_logger_odo.py works.
The script auto-converts body-frame (x fwd, y left, theta) → world frame.

CAN command IDs (PIX Hooke VCU — confirmed from test_04_odometry_square.py):
  0x130  drive   0x131  brake   0x132  steer   0x133  vehicle (headlamp)
Feedback IDs (from odometry_eval.py):
  0x530  drive   0x532  steer   0x539  wheel RPM

Usage:
  python3 wp_follower.py --file waypoints_20260422_185549.py
  python3 wp_follower.py --file waypoints_20260422_185549.py --reverse
  python3 wp_follower.py [--channel can0] [--lidar-host 192.168.x.y]
"""

import can
import math
import time
import threading
import struct
import socket
import json
import argparse
import importlib.util
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odometry_eval import (
    OdometryIntegrator,
    decode_drive_fb,
    decode_steer_fb,
    decode_wheel_rpm_fb,
    ID_DRIVE_FB, ID_STEER_FB, ID_WHEEL_RPM_FB,
    STEER_MODE_NAMES, WHEELBASE_M, STEER_DECODE_DIVISOR,
)

# =============================================================================
#  WAYPOINTS  — fallback when --file is not given
#
#  Prefer passing a file on the command line:
#    python3 wp_follower.py --file waypoints_20260422_185549.py
#
#  Body-frame format (direct .py output from wp_logger_odo.py):
#    x     = forward from start  (+ahead)
#    y     = left from start     (+left)
#    theta = heading at that point, deg CCW from east (90° = north at start)
#  Auto-converted to world frame on load.
#
#  World-frame format (from the .csv _xw / _yw columns):
#    Use {"xw":..,"yw":..} dict keys — load_waypoints handles both.
# =============================================================================

WAYPOINTS = [
    {"x": +0.0000, "y": +0.0000, "theta": +90.00, "dist":  0.0000, "t":   0.00},
    {"x": +2.0000, "y": +0.0000, "theta": +90.00, "dist":  2.0000, "t":   5.00},
    {"x": +4.0000, "y": +0.0000, "theta": +90.00, "dist":  4.0000, "t":  10.00},
    # ── paste more rows here ──
]

# =============================================================================
#  CONFIGURATION
# =============================================================================

CAN_CHANNEL       = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
CONTROL_HZ        = 50          # 50 Hz = 20 ms — matches VCU watchdog requirement

CRUISE_SPEED_KPH  = 1.5         # normal following speed
APPROACH_SPEED_KPH= 0.8         # slow zone near final waypoint
APPROACH_ZONE_M   = 2.0         # switch to approach speed within this dist of goal
GOAL_RADIUS_M     = 0.35        # stop and declare "arrived" within this dist

LOOKAHEAD_M       = 1.2         # pure pursuit lookahead distance (m)
                                 # increase for smoother path, decrease for tighter

MAX_STEER_DEG     = 55.0        # road-wheel max angle (Hooke physical limit)

HANDSHAKE_S       = 3.0         # keepalive brake frames before motion starts

# LiDAR guard (lidar_guard.py on Raspberry Pi)
LIDAR_HOST        = '0.0.0.0'   # ← set to RPi IP, e.g. '192.168.1.42'
                                 #   '0.0.0.0' = accept from any host (broadcast-friendly)
LIDAR_PORT        = 5005
LIDAR_TIMEOUT_S   = 0.5         # treat as unsafe if no packet received for this long

# Brake constants (60% pressure — from test_04_odometry_square.py)
BRAKE_B1          = 0x58
BRAKE_B2          = 0x02

# CAN byte 0 values
DRIVE_BYTE0       = 0x11        # drive enable, speed mode, gear D
DRIVE_BYTE0_REV   = 0x31        # drive enable, speed mode, gear R (confirmed from CAN log)
STEER_BYTE0       = 0x01        # steer enable, front Ackermann

# =============================================================================
#  WAYPOINT LOADER
# =============================================================================

def load_waypoints_from_file(path):
    """Import a wp_logger_odo .py file and return its WAYPOINTS list."""
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Waypoint file not found: {path}")
    spec = importlib.util.spec_from_file_location('_wp_file', path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, 'WAYPOINTS'):
        raise ValueError(f"No WAYPOINTS list found in {path}")
    return mod.WAYPOINTS


def load_waypoints(raw):
    """
    Convert input waypoints to a flat list of (x_world, y_world) tuples.

    Body-frame input  {"x", "y", "theta"}:
      x_world = x_fwd · cos(θ) − y_left · sin(θ)
      y_world = x_fwd · sin(θ) + y_left · cos(θ)

    World-frame input  {"xw", "yw"}:
      passed through unchanged.
    """
    wps = []
    for wp in raw:
        if 'xw' in wp and 'yw' in wp:
            wps.append((float(wp['xw']), float(wp['yw'])))
        elif 'x' in wp and 'y' in wp and 'theta' in wp:
            t = math.radians(wp['theta'])
            xw = wp['x'] * math.cos(t) - wp['y'] * math.sin(t)
            yw = wp['x'] * math.sin(t) + wp['y'] * math.cos(t)
            wps.append((xw, yw))
        else:
            raise ValueError(f"Waypoint {wp} has neither (x,y,theta) nor (xw,yw)")
    return wps

# =============================================================================
#  CAN COMMAND HELPERS  (encoding from test_04_odometry_square.py)
# =============================================================================

def _xor_checksum(payload7):
    cs = 0
    for b in payload7:
        cs ^= b
    return cs

def _build(can_id, payload7):
    cs = _xor_checksum(payload7)
    return can.Message(
        arbitration_id=can_id,
        data=bytearray(payload7 + [cs]),
        is_extended_id=False,
    )

def _speed_bytes(speed_mps):
    raw = int(round(max(0.0, speed_mps) / 0.01))   # factor 0.01 m/s, unsigned
    raw = min(raw, 0x7FFF)
    return raw & 0xFF, (raw >> 8) & 0xFF

def _steer_bytes(steer_deg):
    """Encode steer angle for 0x132: raw = degrees × STEER_DECODE_DIVISOR (mirrors feedback)."""
    clamped = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))
    raw = int(round(clamped * STEER_DECODE_DIVISOR))
    raw16 = raw & 0xFFFF        # two's complement for negatives
    return raw16 & 0xFF, (raw16 >> 8) & 0xFF

def _send_headlamp(bus):
    # 0x133 vehicle control: byte0=0x02 → headlamp on (200 ms period per spec)
    bus.send(_build(0x133, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

def send_drive_steer(bus, cycle, speed_mps, steer_deg, drive_byte0=DRIVE_BYTE0):
    sb1, sb2 = _speed_bytes(speed_mps)
    tb1, tb2 = _steer_bytes(steer_deg)
    c = cycle & 0x0F
    bus.send(_build(0x130, [drive_byte0, sb1, sb2, 0x00, 0x01, 0x00, c]))
    bus.send(_build(0x131, [0x00, 0x00,  0x00, 0x00, 0x00, 0x00, c]))   # brake OFF
    bus.send(_build(0x132, [STEER_BYTE0, tb1, tb2, 0x00, 0x00, 0x00, c]))
    _send_headlamp(bus)

def send_brake(bus, cycle, steer_deg=0.0):
    tb1, tb2 = _steer_bytes(steer_deg)
    c = cycle & 0x0F
    bus.send(_build(0x130, [DRIVE_BYTE0, 0x00,    0x00,    0x00, 0x01, 0x00, c]))
    bus.send(_build(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, c]))
    bus.send(_build(0x132, [STEER_BYTE0, tb1, tb2, 0x00, 0x00, 0x00, c]))
    _send_headlamp(bus)

def send_stop(bus, cycle):
    c = cycle & 0x0F
    bus.send(_build(0x130, [0x00, 0x00, 0x00, 0x00, 0x01, 0x00, c]))
    bus.send(_build(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, c]))
    bus.send(_build(0x132, [STEER_BYTE0, 0x00, 0x00, 0x00, 0x00, 0x00, c]))
    _send_headlamp(bus)

# =============================================================================
#  THREAD-SAFE ODOMETRY
# =============================================================================

class LockedIntegrator(OdometryIntegrator):
    def __init__(self):
        self._lock = threading.Lock()
        super().__init__()

    def reset(self):
        with self._lock:
            super().reset()

    def update_steer(self, f, r, mode=None, ts=None):
        with self._lock:
            super().update_steer(f, r, mode, ts)

    def update_speed(self, v, a, ts):
        with self._lock:
            super().update_speed(v, a, ts)

    def update_rpm(self, lf, rf, lr, rr, ts):
        with self._lock:
            super().update_rpm(lf, rf, lr, rr, ts)

    def pose(self):
        """Return (x_world, y_world, theta_rad) thread-safely."""
        with self._lock:
            return self.x, self.y, self.theta_rad


def can_reader(odo, stop_event, channel):
    try:
        bus = can.interface.Bus(interface='socketcan', channel=channel)
        print(f"[ODO ] Listening on {channel}")
    except Exception as e:
        print(f"[ODO ] Failed: {e}")
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
                odo.update_rpm(d['rpm_lf'], d['rpm_rf'],
                               d['rpm_lr'], d['rpm_rr'], now)
        except Exception:
            pass
    bus.shutdown()

# =============================================================================
#  LIDAR GUARD UDP LISTENER
# =============================================================================

class LidarState:
    """Shared state updated by the UDP listener thread."""
    def __init__(self):
        self._lock     = threading.Lock()
        self.safe      = True
        self.front_m   = float('inf')
        self.left_m    = float('inf')
        self.right_m   = float('inf')
        self.trigger   = ''
        self.last_ts   = 0.0     # epoch of last received packet

    def update(self, pkt: dict):
        with self._lock:
            self.safe    = bool(pkt.get('safe', True))
            self.front_m = float(pkt.get('front', float('inf')))
            self.left_m  = float(pkt.get('left',  float('inf')))
            self.right_m = float(pkt.get('right', float('inf')))
            self.trigger = str(pkt.get('trigger', ''))
            self.last_ts = time.time()

    def is_safe(self):
        with self._lock:
            # If no packet in LIDAR_TIMEOUT_S, treat as safe (guard is offline)
            if time.time() - self.last_ts > LIDAR_TIMEOUT_S:
                return True
            return self.safe

    def summary(self):
        with self._lock:
            age = time.time() - self.last_ts
            if age > LIDAR_TIMEOUT_S:
                return 'LIDAR:offline'
            status = 'OK' if self.safe else f'STOP({self.trigger})'
            return (f'LIDAR:{status} '
                    f'F={self.front_m:.2f}m '
                    f'L={self.left_m:.2f}m '
                    f'R={self.right_m:.2f}m')


def lidar_listener(lidar_state: LidarState, stop_event: threading.Event,
                   host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(0.2)
    print(f"[LIDAR] UDP listening on {host}:{port}")
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(256)
            pkt = json.loads(data.decode())
            lidar_state.update(pkt)
        except socket.timeout:
            pass
        except Exception:
            pass
    sock.close()

# =============================================================================
#  PURE PURSUIT
# =============================================================================

def pure_pursuit_steer_deg(car_x, car_y, car_yaw_rad,
                            tgt_x, tgt_y, wheelbase):
    """
    Pure pursuit steering angle in road-wheel degrees.

    Geometry (same as scenario_v3.py pp_steer, un-normalised):
      α     = angle from car heading to lookahead point
      δ     = atan2(2L·sin(α), ld)     — bicycle model steer angle
    """
    dx = tgt_x - car_x
    dy = tgt_y - car_y
    ld = math.hypot(dx, dy)
    if ld < 0.05:
        return 0.0
    alpha = (math.atan2(dy, dx) - car_yaw_rad + math.pi) % (2 * math.pi) - math.pi
    steer_rad = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
    steer_deg = math.degrees(steer_rad)
    return max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))


def find_lookahead(car_x, car_y, car_yaw_rad, wps, wp_idx, lookahead_m):
    """
    Return (target_x, target_y, updated_wp_idx).

    Step 1 — snap to nearest remaining waypoint:
      Among wps[wp_idx:], find the one closest to the vehicle.  Use that
      as the new wp_idx.  This handles both normal progress (close waypoint
      is the next one) and corner overshoots / post-intervention recovery
      (close waypoint may be several indices ahead).

    Step 2 — lookahead:
      Scan forward from the snapped index for the first waypoint at least
      lookahead_m away and return it as the steering target.
    """
    n = len(wps)

    # Snap to nearest remaining waypoint
    best_idx  = wp_idx
    best_dist = math.hypot(wps[wp_idx][0] - car_x, wps[wp_idx][1] - car_y)
    for i in range(wp_idx + 1, n):
        d = math.hypot(wps[i][0] - car_x, wps[i][1] - car_y)
        if d < best_dist:
            best_dist = d
            best_idx  = i
    wp_idx = best_idx

    # Find the lookahead point
    for i in range(wp_idx, n):
        dx = wps[i][0] - car_x
        dy = wps[i][1] - car_y
        if math.hypot(dx, dy) >= lookahead_m:
            return wps[i][0], wps[i][1], wp_idx

    # All remaining waypoints are closer than lookahead — aim at last
    return wps[-1][0], wps[-1][1], wp_idx

# =============================================================================
#  MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='Waypoint follower — PIX Hooke')
    parser.add_argument('--file',       default=None,
                        help='Path to a wp_logger_odo .py waypoint file')
    parser.add_argument('--reverse',    action='store_true',
                        help='Reverse the waypoint list (return trip to origin)')
    parser.add_argument('--channel',    default=CAN_CHANNEL)
    parser.add_argument('--lidar-host', default=LIDAR_HOST)
    parser.add_argument('--lidar-port', type=int, default=LIDAR_PORT)
    args = parser.parse_args()

    # ── Load and validate waypoints ──────────────────────────────────────────
    if args.file:
        try:
            raw = load_waypoints_from_file(args.file)
        except (FileNotFoundError, ValueError) as e:
            print(f"ERROR: {e}")
            return
    else:
        raw = WAYPOINTS

    wps = load_waypoints(raw)
    if args.reverse:
        wps = list(reversed(wps))
        # The reversed waypoints are in the forward run's world frame, where the
        # forward start = (0,0).  The reverse run's odometry also starts at (0,0),
        # but physically the car is now at what was the forward END point = wps[0].
        # Translate every waypoint so that wps[0] becomes (0,0), aligning the two
        # frames.  The goal (original forward start) ends up at a negative offset.
        ox, oy = wps[0]
        wps = [(x - ox, y - oy) for x, y in wps]

    if len(wps) < 2:
        print("ERROR: Need at least 2 waypoints.")
        return

    direction = ' [REVERSE]' if args.reverse else ''
    src = os.path.basename(args.file) if args.file else 'inline WAYPOINTS'
    print(f"[WP  ] Loaded {len(wps)} waypoints from {src}{direction}")
    print(f"[WP  ] start={wps[0]}  goal={wps[-1]}")

    # ── Start threads ────────────────────────────────────────────────────────
    stop_event  = threading.Event()
    odo         = LockedIntegrator()

    # In reverse the car starts at the forward run's END point, which has a
    # different heading from the integrator's default 90° (north).  Seed the
    # integrator with the last forward waypoint's theta so that position
    # integration uses the correct heading from the first step.
    if args.reverse and raw and 'theta' in raw[-1]:
        _init_theta = math.radians(raw[-1]['theta'])
        odo.theta_rad     = _init_theta
        odo.theta_rpm_rad = _init_theta

    lidar_state = LidarState()

    t_odo = threading.Thread(
        target=can_reader, args=(odo, stop_event, args.channel), daemon=True)
    t_odo.start()

    t_lidar = threading.Thread(
        target=lidar_listener,
        args=(lidar_state, stop_event, args.lidar_host, args.lidar_port),
        daemon=True)
    t_lidar.start()

    # ── Open command bus ─────────────────────────────────────────────────────
    try:
        cmd_bus = can.interface.Bus(interface='socketcan', channel=args.channel)
    except Exception as e:
        print(f"[CAN ] Command bus failed: {e}")
        stop_event.set()
        return

    cruise_mps   = CRUISE_SPEED_KPH   / 3.6
    approach_mps = APPROACH_SPEED_KPH / 3.6
    step_s       = 1.0 / CONTROL_HZ
    drive_b0     = DRIVE_BYTE0_REV if args.reverse else DRIVE_BYTE0

    print("=" * 62)
    print("  WAYPOINT FOLLOWER  —  PIX Hooke CAN")
    print(f"  Waypoints    : {len(wps)}")
    print(f"  Direction    : {'REVERSE (gear R)' if args.reverse else 'FORWARD (gear D)'}")
    if args.reverse:
        print(f"  Init heading : {math.degrees(odo.theta_rad):.1f}°  (from last wp)")
    print(f"  Cruise speed : {CRUISE_SPEED_KPH} km/h")
    print(f"  Lookahead    : {LOOKAHEAD_M} m")
    print(f"  Channel      : {args.channel}")
    print(f"  LiDAR guard  : {args.lidar_host}:{args.lidar_port}")
    print("  Switch remote to autonomous (rod 6 down) then press ENTER.")
    print("=" * 62)
    input()

    cycle  = 0
    wp_idx = 0

    # ── Phase 1: Handshake keepalive ─────────────────────────────────────────
    print(f"[INIT] Handshake {HANDSHAKE_S}s keepalive...")
    t_end = time.time() + HANDSHAKE_S
    while time.time() < t_end:
        send_brake(cmd_bus, cycle)
        cycle = (cycle + 1) & 0x0F
        time.sleep(step_s)
    print("[INIT] Handshake done. Starting.")

    t_start = time.time()
    lidar_was_safe = True

    try:
        while True:
            t0 = time.time()

            car_x, car_y, car_yaw = odo.pose()
            elapsed = t0 - t_start

            # ── Goal check ───────────────────────────────────────────────────
            goal_dist = math.hypot(car_x - wps[-1][0], car_y - wps[-1][1])
            if goal_dist < GOAL_RADIUS_M:
                print(f"\n[DONE] Goal reached. dist={goal_dist:.3f}m")
                break

            # ── LiDAR safety check ───────────────────────────────────────────
            safe = lidar_state.is_safe()
            if not safe and lidar_was_safe:
                t_name = lidar_state.trigger
                print(f"\n[LIDAR STOP] zone={t_name}  {lidar_state.summary()}")
            if safe and not lidar_was_safe:
                print(f"\n[LIDAR CLEAR] resuming")
            lidar_was_safe = safe

            if not safe:
                send_brake(cmd_bus, cycle)
                cycle = (cycle + 1) & 0x0F
                # Print status and sleep remainder of step
                print(f"  BRAKE  x={car_x:+7.3f}  y={car_y:+7.3f}  "
                      f"θ={math.degrees(car_yaw):5.1f}°  "
                      f"{lidar_state.summary()}",
                      end='\r')
                dt = time.time() - t0
                time.sleep(max(0.0, step_s - dt))
                continue

            # ── Lookahead and pure pursuit ────────────────────────────────────
            tgt_x, tgt_y, wp_idx = find_lookahead(
                car_x, car_y, car_yaw, wps, wp_idx, LOOKAHEAD_M)

            steer_deg = pure_pursuit_steer_deg(
                car_x, car_y, car_yaw, tgt_x, tgt_y, WHEELBASE_M)

            # ── Speed selection ───────────────────────────────────────────────
            speed_mps = approach_mps if goal_dist < APPROACH_ZONE_M else cruise_mps

            # ── Send commands ─────────────────────────────────────────────────
            send_drive_steer(cmd_bus, cycle, speed_mps, steer_deg, drive_b0)
            cycle = (cycle + 1) & 0x0F

            # ── Console status ────────────────────────────────────────────────
            mode_name = STEER_MODE_NAMES.get(odo.steer_mode, '?')
            print(
                f"  t={elapsed:6.1f}s  "
                f"x={car_x:+7.3f}  y={car_y:+7.3f}  "
                f"θ={math.degrees(car_yaw):5.1f}°  "
                f"wp={wp_idx}/{len(wps)-1}  "
                f"tgt=({tgt_x:+.2f},{tgt_y:+.2f})  "
                f"steer={steer_deg:+5.1f}°  "
                f"spd={speed_mps*3.6:.1f}km/h  "
                f"goal={goal_dist:.2f}m",
                end='\r'
            )

            dt = time.time() - t0
            time.sleep(max(0.0, step_s - dt))

    except KeyboardInterrupt:
        print("\n[Ctrl+C] Stopping.")

    finally:
        print("\n[STOP] Applying brake...")
        for _ in range(50):     # 1 s of brake frames
            send_stop(cmd_bus, cycle)
            cycle = (cycle + 1) & 0x0F
            time.sleep(step_s)
        stop_event.set()
        cmd_bus.shutdown()
        print("[Done]")


if __name__ == '__main__':
    main()

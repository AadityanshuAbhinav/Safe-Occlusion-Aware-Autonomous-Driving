"""
wp_logger_odo.py
════════════════
Waypoint logger for PIX Moving Hooke chassis using CAN odometry.

Drive the vehicle manually with the wireless remote.  The script records a
waypoint every RECORD_INTERVAL metres of travel (Euclidean distance in the
odometry world frame).  Press Ctrl+C to stop and save.

Position source: OdometryIntegrator from odometry_eval.py — same integrator
that powers pose_disp.py.  No CARLA connection required.

Coordinate convention (CARLA / body frame — same as pose_disp.py):
  x     : forward displacement from start  (+ahead)
  y     : left    displacement from start  (+left)
  theta : heading in degrees CCW from world-east  (90° = north at start)

Saved files (both written on exit):
  waypoints_<timestamp>.py   — importable Python list, compatible with
                                the CARLA scenario scripts
  waypoints_<timestamp>.csv  — full log with timestamps and distances

Usage:
  python3 wp_logger_odo.py [--interval 0.5] [--channel can0] [--out .]
"""

import can
import math
import time
import threading
import csv
import os
import sys
import argparse
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odometry_eval import (
    OdometryIntegrator,
    decode_drive_fb,
    decode_steer_fb,
    decode_wheel_rpm_fb,
    ID_DRIVE_FB,
    ID_STEER_FB,
    ID_WHEEL_RPM_FB,
    STEER_MODE_NAMES,
    REAR_TRACK_M,
    WHEELBASE_M,
    WHEEL_RADIUS_M,
)

# =============================================================================
#  CONFIGURATION (overridable via CLI)
# =============================================================================

RECORD_INTERVAL = 0.5       # metres between waypoints
MIN_SPEED_MPS   = 0.02      # below this speed, suppress recording
OUTPUT_DIR      = '.'
CAN_CHANNEL     = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
PRINT_HZ        = 10        # console refresh rate

# =============================================================================
#  Thread-safe odometry wrapper (minimal — no GUI trail needed)
# =============================================================================

class LockedIntegrator(OdometryIntegrator):
    def __init__(self):
        self._lock = threading.Lock()
        super().__init__()

    def reset(self):
        with self._lock:
            super().reset()

    def update_steer(self, front_deg, rear_deg, mode=None):
        with self._lock:
            super().update_steer(front_deg, rear_deg, mode)

    def update_speed(self, speed_mps, accel_mps2, ts):
        with self._lock:
            super().update_speed(speed_mps, accel_mps2, ts)

    def update_rpm(self, lf, rf, lr, rr, ts):
        with self._lock:
            super().update_rpm(lf, rf, lr, rr, ts)

    def snapshot(self):
        """Return a consistent pose snapshot for the logger main loop."""
        with self._lock:
            theta_r = self.theta_rad
            ct = math.cos(theta_r)
            st = math.sin(theta_r)
            return {
                'x_w'    : self.x,                              # world east
                'y_w'    : self.y,                              # world north
                'x_fwd'  :  self.x * ct + self.y * st,         # body forward
                'y_left' : -self.x * st + self.y * ct,         # body left
                'theta'  : self.theta_deg % 360,
                'dist'   : self.distance_m,
                'speed'  : self.speed_samples[-1] if self.speed_samples else 0.0,
                'sf'     : self.steer_front_deg,
                'sr'     : self.steer_rear_deg,
                'mode'   : self.steer_mode,
            }


# =============================================================================
#  CAN reader thread
# =============================================================================

def can_reader(odo: LockedIntegrator, stop_event: threading.Event,
               channel: str):
    try:
        bus = can.interface.Bus(interface='socketcan', channel=channel)
        print(f"[CAN] Listening on {channel}")
    except Exception as e:
        print(f"[CAN] Failed to open {channel}: {e}")
        stop_event.set()
        return

    subscribed = {ID_DRIVE_FB, ID_STEER_FB, ID_WHEEL_RPM_FB}

    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.arbitration_id not in subscribed:
                continue
            data = bytes(msg.data)
            now  = time.time()

            if msg.arbitration_id == ID_STEER_FB and len(data) >= 6:
                d = decode_steer_fb(data)
                odo.update_steer(d['steer_angle_front'], d['steer_angle_rear'],
                                 mode=d['steer_mode'])

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
    print("[CAN] Bus closed.")


# =============================================================================
#  Waypoint record helpers
# =============================================================================

def make_waypoint(snap, elapsed):
    return {
        'x'    : round(snap['x_fwd'],  4),   # body forward  (m)
        'y'    : round(snap['y_left'], 4),   # body left     (m)
        'theta': round(snap['theta'],  2),   # heading       (deg)
        'dist' : round(snap['dist'],   4),   # cumulative odometry distance (m)
        't'    : round(elapsed,        2),   # elapsed time  (s)
        # world frame kept for reference / debugging
        '_xw'  : round(snap['x_w'],    4),
        '_yw'  : round(snap['y_w'],    4),
    }


def save_python(waypoints, args, path):
    with open(path, 'w') as f:
        f.write('"""\n')
        f.write(f'Recorded waypoints -- real vehicle odometry\n')
        f.write(f'Interval : ~{args.interval} m\n')
        f.write(f'Points   : {len(waypoints)}\n')
        f.write(f'Wheelbase: {WHEELBASE_M} m  '
                f'Track: {REAR_TRACK_M} m  '
                f'Wheel r: {WHEEL_RADIUS_M} m\n')
        f.write(f'Conv     : x=forward, y=left, theta=CCW-from-east (deg)\n')
        f.write('"""\n\n')
        f.write('WAYPOINTS = [\n')
        for wp in waypoints:
            f.write(
                f'    {{"x": {wp["x"]:+9.4f}, '
                f'"y": {wp["y"]:+9.4f}, '
                f'"theta": {wp["theta"]:+8.2f}, '
                f'"dist": {wp["dist"]:8.4f}, '
                f'"t": {wp["t"]:7.2f}}},\n'
            )
        f.write(']\n')


def save_csv(waypoints, path):
    fields = ['idx', 'x', 'y', 'theta', 'dist', 't', '_xw', '_yw']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for i, wp in enumerate(waypoints):
            w.writerow({'idx': i, **wp})


def save_all(waypoints, args, output_dir):
    if not waypoints:
        print("[Save] No waypoints recorded.")
        return
    ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
    py_path  = os.path.join(output_dir, f'waypoints_{ts}.py')
    csv_path = os.path.join(output_dir, f'waypoints_{ts}.csv')
    save_python(waypoints, args, py_path)
    save_csv(waypoints, csv_path)
    print(f"\n[Save] {len(waypoints)} waypoints")
    print(f"       {py_path}")
    print(f"       {csv_path}")


# =============================================================================
#  Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='Odometry-based waypoint logger')
    parser.add_argument('--interval', type=float, default=RECORD_INTERVAL,
                        help='Distance between waypoints (m)')
    parser.add_argument('--channel', default=CAN_CHANNEL,
                        help='CAN channel (default: can0)')
    parser.add_argument('--out', default=OUTPUT_DIR,
                        help='Output directory')
    args = parser.parse_args()

    odo        = LockedIntegrator()
    stop_event = threading.Event()

    reader = threading.Thread(
        target=can_reader, args=(odo, stop_event, args.channel), daemon=True
    )
    reader.start()

    print("=" * 62)
    print("  WAYPOINT LOGGER  —  CAN odometry")
    print(f"  Interval : {args.interval} m")
    print(f"  Channel  : {args.channel}")
    print(f"  Output   : {args.out}/")
    print(f"  Conv     : x=forward  y=left  theta=CCW-from-east")
    print("  Drive the vehicle.  Ctrl+C to stop and save.")
    print("=" * 62)

    waypoints      = []
    last_world_pos = None   # (x_w, y_w) of last recorded waypoint
    t_start        = time.time()
    t_last_print   = t_start

    # Record start position
    time.sleep(0.5)         # let CAN frames arrive before first record
    snap = odo.snapshot()
    elapsed = 0.0
    waypoints.append(make_waypoint(snap, elapsed))
    last_world_pos = (snap['x_w'], snap['y_w'])
    print(f"  [  0] START  x={snap['x_fwd']:+8.4f}m  y={snap['y_left']:+8.4f}m  "
          f"θ={snap['theta']:6.1f}°")

    try:
        while True:
            time.sleep(1.0 / PRINT_HZ)

            if stop_event.is_set():
                print("\n[CAN] Bus lost — stopping.")
                break

            snap    = odo.snapshot()
            elapsed = time.time() - t_start

            # ── Distance-triggered waypoint recording ────────────────────────
            if last_world_pos is not None:
                d = math.hypot(snap['x_w'] - last_world_pos[0],
                               snap['y_w'] - last_world_pos[1])
                if d >= args.interval and abs(snap['speed']) > MIN_SPEED_MPS:
                    wp = make_waypoint(snap, elapsed)
                    waypoints.append(wp)
                    last_world_pos = (snap['x_w'], snap['y_w'])
                    print(
                        f"\r  [{len(waypoints)-1:3d}]  "
                        f"x={wp['x']:+8.4f}m  y={wp['y']:+8.4f}m  "
                        f"θ={wp['theta']:6.1f}°  "
                        f"dist={wp['dist']:7.3f}m  t={wp['t']:6.1f}s"
                    )

            # ── Console status line ──────────────────────────────────────────
            if (time.time() - t_last_print) >= (1.0 / PRINT_HZ):
                t_last_print = time.time()
                mode_name = STEER_MODE_NAMES.get(snap['mode'], '?')
                last_wp   = waypoints[-1] if waypoints else None
                since_wp  = (math.hypot(snap['x_w'] - last_world_pos[0],
                                        snap['y_w'] - last_world_pos[1])
                             if last_world_pos else 0.0)
                print(
                    f"  t={elapsed:6.1f}s  "
                    f"x={snap['x_fwd']:+7.3f}m  y={snap['y_left']:+7.3f}m  "
                    f"θ={snap['theta']:6.1f}°  "
                    f"dist={snap['dist']:7.3f}m  "
                    f"spd={snap['speed']*3.6:5.2f}km/h  "
                    f"wpts={len(waypoints)}  "
                    f"Δ={since_wp:.2f}/{args.interval:.2f}m  "
                    f"[{mode_name}]",
                    end='\r'
                )

    except KeyboardInterrupt:
        print("\n\n[Ctrl+C] Stopping.")

    finally:
        stop_event.set()
        save_all(waypoints, args, args.out)
        print("[Done]")


if __name__ == '__main__':
    main()

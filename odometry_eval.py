"""
odometry_eval.py
────────────────
CAN odometry logger and integrator for PIX Moving Hooke chassis.

Steering modes reported by VCU_ChassisSteerModeFb (0x532 byte0 bits4-7):
  0 — Front Ackerman   (only front wheels steer, rear fixed)
  1 — Crab             (front and rear steer same direction = pure lateral)
  2 — Front diff back  (front and rear steer opposite directions = tight turn)

Kinematic model used per mode:
  ┌──────────────────────┬─────────────────────────────────────────────────────┐
  │ Mode                 │ Heading rate dθ/dt                                  │
  ├──────────────────────┼─────────────────────────────────────────────────────┤
  │ 0  Front Ackerman    │ v * tan(front) / L                                  │
  │ 1  Crab              │ 0  (pure lateral translation, no heading change)    │
  │ 2  Front diff back   │ v * (tan(front) - tan(rear)) / L  (tightest turns)  │
  └──────────────────────┴─────────────────────────────────────────────────────┘

  Lateral velocity (crab component):
    Non-zero only in mode 1 (crab).
    lat_v = v * sin(avg(front, rear))

  Forward velocity:
    fwd_v = v * cos(avg(front, rear)) for crab,
    fwd_v = v for Ackerman and front-diff-back

Usage:
  Set KNOWN_DISTANCE_M at the top, run the script, drive between markings,
  press Ctrl+C. Summary prints at the end.

This module is also imported by pose_display.py — do not rename.
"""

import can
import time
import csv
import math
import struct
import os
from collections import deque
from datetime import datetime

# =============================================================================
#  CONFIGURATION
# =============================================================================

CAN_CHANNEL      = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
LOG_DIR          = '.'
PRINT_HZ         = 10

KNOWN_DISTANCE_M = None     # ← set before running, e.g. 5.0

WHEELBASE_M      = 1.91     # Hooke wheelbase — axle-to-axle (measured: 191 cm)
WHEEL_RADIUS_M   = 0.32     # ← set to actual tyre radius for RPM cross-check
REAR_TRACK_M     = 1.47     # rear axle track — wheel-centre to wheel-centre (measured: 147 cm)
FRONT_TRACK_M    = 1.47     # front axle track — wheel-centre to wheel-centre (measured: 147 cm)
WHEEL_WIDTH_M    = 0.22     # tyre section width (measured: 22 cm) — for reference only

# Steer angle decode: angle_deg = (raw + STEER_ZERO_RAW) / STEER_DECODE_DIVISOR
# Calibration measurements (0x532 B1–B2 s16_le vs physical wheel angle):
#   raw =    0  →   0° (dead straight, verified)
#   raw = -275  →  ~-12.5°  (measured as -10°, ~2.5° physical measurement under-read)
#   raw = +385  →  ~+17.5°  (measured as +20°, ~2.5° physical measurement under-read)
#   raw = +500  →  ~+22.7°  (measured as +25°, ~2.3° physical measurement under-read)
#   Best-fit: divisor = 22,  zero offset = 0 (encoder reads 0 when straight)
# NOTE: STEER_DECODE_DIVISOR can be further tuned to the most accurate value by performing a sweep over a range of divisor values and checking which gives the best final heading accuracy at the end of the run. Out of the scope of my project timeline but a good future improvement.

STEER_DECODE_DIVISOR = 22.0  # raw-int → degrees  (measured best-fit: 22.0)
STEER_ZERO_RAW       = 0     # raw value when wheels are physically straight (verified: 0)

# =============================================================================
#  CAN frame IDs
# =============================================================================

ID_DRIVE_FB      = 0x530
ID_BRAKE_FB      = 0x531
ID_STEER_FB      = 0x532
ID_WORK_FB       = 0x534
ID_POWER_FB      = 0x535
ID_WHEEL_RPM_FB  = 0x539

SUBSCRIBED_IDS   = {
    ID_DRIVE_FB, ID_BRAKE_FB, ID_STEER_FB,
    ID_WORK_FB, ID_POWER_FB, ID_WHEEL_RPM_FB,
}

# Steer mode constants (VCU_ChassisSteerModeFb)
STEER_MODE_FRONT_ACKERMAN  = 0
STEER_MODE_CRAB            = 1   # same front and back (lateral translation)
STEER_MODE_FRONT_DIFF_BACK = 2   # front opposite to rear (tight turn)

STEER_MODE_NAMES = {
    0: 'Front Ackerman',
    1: 'Crab',
    2: 'Front-diff-back',
}

# =============================================================================
#  Decode helpers
# =============================================================================

def u16_le(data, byte_offset):
    return struct.unpack_from('<H', data, byte_offset)[0]

def s16_le(data, byte_offset):
    return struct.unpack_from('<h', data, byte_offset)[0]

def u8(data, byte_offset):
    return data[byte_offset]

def bits(byte_val, start_bit, length):
    return (byte_val >> start_bit) & ((1 << length) - 1)

def u10_le(data, byte_offset):
    return u16_le(data, byte_offset) & 0x3FF


def decode_drive_fb(data):
    b0 = u8(data, 0)
    return {
        'drive_enable'      : bits(b0, 0, 1),
        'drive_slopover'    : bits(b0, 1, 1),
        'drive_mode'        : bits(b0, 2, 2),
        'gear'              : bits(b0, 4, 2),
        'speed_mps'         : s16_le(data, 1) * 0.01,
        'throttle_pct'      : u10_le(data, 3) * 0.1,
        'acceleration_mps2' : s16_le(data, 5) * 0.01,
        'drive_life'        : bits(u8(data, 7), 0, 4),
    }


def decode_brake_fb(data):
    b0 = u8(data, 0)
    return {
        'brake_enable'      : bits(b0, 0, 1),
        'brake_lamp'        : bits(b0, 2, 1),
        'epb_state'         : bits(b0, 4, 2),
        'brake_pedal_pct'   : u10_le(data, 1) * 0.1,
        'brake_pressure_bar': u8(data, 3),
        'brake_life'        : bits(u8(data, 6), 0, 4),
    }


def decode_steer_fb(data):
    b0 = u8(data, 0)
    return {
        'steer_enable'      : bits(b0, 0, 1),
        'steer_slopover'    : bits(b0, 1, 1),
        'steer_mode'        : bits(b0, 4, 4),
        'steer_angle_front' : (s16_le(data, 1) + STEER_ZERO_RAW) / STEER_DECODE_DIVISOR,
        'steer_angle_rear'  : (s16_le(data, 3) + STEER_ZERO_RAW) / STEER_DECODE_DIVISOR,
        'steer_angle_speed' : u8(data, 5) * 2,   # deg/s
        'steer_life'        : bits(u8(data, 6), 0, 4),
    }


def decode_work_fb(data):
    b0 = u8(data, 0)
    b1 = u8(data, 1)
    b5 = u8(data, 5)
    return {
        'driving_mode'      : bits(b0, 0, 2),
        'power_state'       : bits(b0, 2, 2),
        'dc_state'          : bits(b0, 4, 2),
        'speed_limit_mode'  : bits(b1, 0, 1),
        'speed_limit_mps'   : u16_le(data, 2) * 0.1,
        'low_volt_V'        : u8(data, 4) * 0.1,
        'estop_state'       : bits(b5, 0, 4),
        'crash_front'       : bits(b5, 4, 1),
        'crash_rear'        : bits(b5, 5, 1),
        'vcu_life'          : bits(u8(data, 6), 0, 4),
    }


def decode_power_fb(data):
    return {
        'charge_state'      : bits(u8(data, 0), 4, 2),
        'soc_pct'           : u8(data, 1),
        'battery_volt_V'    : u16_le(data, 2) * 0.1,
        'battery_curr_A'    : (u16_le(data, 4) * 0.1) - 1000,
        'bms_max_temp_C'    : u8(data, 6) - 40,
    }


def decode_wheel_rpm_fb(data):
    return {
        'rpm_lf' : s16_le(data, 0),
        'rpm_rf' : s16_le(data, 2),
        'rpm_lr' : s16_le(data, 4),
        'rpm_rr' : s16_le(data, 6),
    }


DECODERS = {
    ID_DRIVE_FB     : decode_drive_fb,
    ID_BRAKE_FB     : decode_brake_fb,
    ID_STEER_FB     : decode_steer_fb,
    ID_WORK_FB      : decode_work_fb,
    ID_POWER_FB     : decode_power_fb,
    ID_WHEEL_RPM_FB : decode_wheel_rpm_fb,
}

# =============================================================================
#  Kinematic model
# =============================================================================

def compute_motion(speed_mps, steer_front_deg, steer_rear_deg, steer_mode, dt):
    """
    Given chassis speed, steer angles, steer mode and time step,
    return (ds_forward, ds_lateral, dtheta_rad).

    ds_forward  : forward displacement along body X axis (m)
    ds_lateral  : lateral displacement along body Y axis (m, +left)
    dtheta_rad  : heading change (rad, +CCW = turning left)

    Sign conventions:
      steer_front_deg > 0  → front wheels turned left
      steer_rear_deg  > 0  → rear wheels turned left
      speed_mps       > 0  → forward motion
    """
    L  = WHEELBASE_M
    sf = math.radians(steer_front_deg)
    sr = math.radians(steer_rear_deg)
    v  = speed_mps

    if steer_mode == STEER_MODE_FRONT_ACKERMAN:
        # Only front steers. Pure Ackerman.
        # Vehicle moves along body axis, heading rotates by front steer.
        ds_fwd  = v * dt
        ds_lat  = 0.0
        if abs(sf) > 1e-6:
            dtheta = v * math.tan(sf) / L * dt
        else:
            dtheta = 0.0

    elif steer_mode == STEER_MODE_CRAB:
        # Front and rear steer the same direction and same magnitude.
        # Result: pure lateral translation, no heading change.
        # The vehicle slides sideways at angle avg_steer relative to body.
        avg = (sf + sr) / 2.0
        ds_fwd  = v * math.cos(avg) * dt
        ds_lat  = v * math.sin(avg) * dt
        dtheta  = 0.0

    elif steer_mode == STEER_MODE_FRONT_DIFF_BACK:
        # Front and rear steer opposite directions.
        # Tightest turning — both axles contribute to rotation.
        # No lateral slip assumed.
        ds_fwd  = v * dt
        ds_lat  = 0.0
        dtheta  = v * (math.tan(sf) - math.tan(sr)) / L * dt

    else:
        # Unknown mode — fall back to front-Ackermann model
        ds_fwd  = v * dt
        ds_lat  = 0.0
        dtheta  = v * math.tan(sf) / L * dt if abs(sf) > 1e-6 else 0.0

    return ds_fwd, ds_lat, dtheta


# =============================================================================
#  Odometry integrator
# =============================================================================

class OdometryIntegrator:
    """
    Dead-reckoning integrator. Maintains (x, y, theta) from the starting point.

    Imported by pose_display.py — keep the interface stable.

    RPM-differential yaw cross-check
    ─────────────────────────────────
    In Front-Ackermann mode (mode 0) the rear axle does not steer, so the
    rear-left and rear-right wheel speeds differ only because of vehicle yaw.
    This gives an independent heading-rate estimate immune to steer-frame lag:

        ω_rpm = (v_rr − v_rl) / d_track          [rad/s]
        v_rr  = rpm_rr * 2π/60 * wheel_radius
        v_rl  = rpm_lr * 2π/60 * wheel_radius

    NOT valid in Crab (mode 1) or Front-diff-back (mode 2) — the rear axle
    steers in both, making the formula structurally wrong.

    Results stored in:
      theta_rpm_rad       — accumulated heading from RPM differential
      yaw_rate_rpm_rads   — instantaneous RPM yaw rate (nan if rear steers)
      heading_err_samples — per-frame (theta_rpm − theta_steer) in degrees
    """

    def __init__(self, wheelbase_m=WHEELBASE_M, wheel_radius_m=WHEEL_RADIUS_M,
                 rear_track_m=REAR_TRACK_M):
        self.wheelbase_m    = wheelbase_m
        self.wheel_radius_m = wheel_radius_m
        self.rear_track_m   = rear_track_m
        self.reset()

    def reset(self):
        # Pose
        self.x              = 0.0
        self.y              = 0.0
        self.theta_rad      = math.radians(90.0)   # start heading north

        # Accumulated scalars
        self.distance_m     = 0.0   # total forward path length
        self.distance_rpm_m = 0.0   # forward path from wheel RPM
        self.lateral_m      = 0.0   # net lateral displacement

        # RPM-differential heading cross-check (Front-Ackermann only)
        self.theta_rpm_rad       = math.radians(90.0)  # mirrors theta_rad at reset
        self.yaw_rate_rpm_rads   = 0.0
        self.yaw_rate_steer_rads = 0.0
        self.heading_err_samples = []   # (theta_rpm - theta_steer) in degrees

        # Speed/accel history
        self.speed_samples  = []
        self.accel_samples  = []
        self.lateral_samples= []

        # Extremes
        self.max_speed_mps  = 0.0
        self.min_speed_mps  = float('inf')

        # Latest steer state (updated from 0x532) — used for display only
        self.steer_front_deg = 0.0
        self.steer_rear_deg  = 0.0
        self.steer_mode      = STEER_MODE_FRONT_ACKERMAN

        # Timestamped steer history for lag-compensated integration
        # Each entry: (timestamp, front_deg, rear_deg, mode)
        self._steer_history = deque(maxlen=40)

        # Timestamps
        self._last_speed_ts = None
        self._last_rpm_ts   = None

    def update_steer(self, front_deg, rear_deg, mode=None, ts=None):
        """
        Call on every 0x532 frame.  ts should be time.time() at frame receipt.
        Stores a timestamped entry so update_speed can interpolate the steer
        angle that was true at the exact moment of each speed frame, eliminating
        the CAN timing lag between 0x532 and 0x530.
        """
        if mode is not None:
            self.steer_mode = mode
        self.steer_front_deg = front_deg
        self.steer_rear_deg  = rear_deg
        if ts is not None:
            self._steer_history.append((ts, front_deg, rear_deg, self.steer_mode))

    def _steer_at(self, ts):
        """
        Return (front_deg, rear_deg, mode) interpolated to timestamp ts.
        Uses linear interpolation between the two surrounding history entries.
        Falls back to the latest known value if history is too short.
        """
        h = self._steer_history
        if not h:
            return self.steer_front_deg, self.steer_rear_deg, self.steer_mode
        if ts <= h[0][0]:
            return h[0][1], h[0][2], h[0][3]
        if ts >= h[-1][0]:
            return h[-1][1], h[-1][2], h[-1][3]
        # Linear scan for the bracketing pair (history is short, O(n) is fine)
        for i in range(len(h) - 1):
            t0, f0, r0, m0 = h[i]
            t1, f1, r1, m1 = h[i + 1]
            if t0 <= ts <= t1:
                alpha = (ts - t0) / (t1 - t0)
                return (f0 + alpha * (f1 - f0),
                        r0 + alpha * (r1 - r0),
                        m0 if (ts - t0) < (t1 - ts) else m1)
        return h[-1][1], h[-1][2], h[-1][3]

    def update_speed(self, speed_mps, accel_mps2, ts):
        """
        Call on every 0x530 frame.
        Integrates pose using the steer angle interpolated to ts, not the most
        recently received steer frame — this eliminates CAN timing lag at corners.
        """
        if self._last_speed_ts is not None:
            dt = ts - self._last_speed_ts
            if 0 < dt <= 0.5:
                sf, sr, smode = self._steer_at(ts)
                ds_fwd, ds_lat, dtheta = compute_motion(
                    speed_mps,
                    sf,
                    sr,
                    smode,
                    dt,
                )

                # Update heading first, then project displacement into world frame
                # using mid-point heading for better accuracy
                theta_mid = self.theta_rad + dtheta / 2.0

                self.x         += ds_fwd * math.cos(theta_mid) \
                                 - ds_lat * math.sin(theta_mid)
                self.y         += ds_fwd * math.sin(theta_mid) \
                                 + ds_lat * math.cos(theta_mid)
                self.theta_rad += dtheta

                self.distance_m += abs(ds_fwd)
                self.lateral_m  += ds_lat
                self.lateral_samples.append(ds_lat / dt if dt > 0 else 0)

                self.yaw_rate_steer_rads = dtheta / dt if dt > 0 else 0.0

        self._last_speed_ts = ts
        self.max_speed_mps  = max(self.max_speed_mps, abs(speed_mps))
        if abs(speed_mps) > 0.001:
            self.min_speed_mps = min(self.min_speed_mps, abs(speed_mps))
        self.speed_samples.append(speed_mps)
        self.accel_samples.append(accel_mps2)

    def update_rpm(self, rpm_lf, rpm_rf, rpm_lr, rpm_rr, ts):
        """
        RPM-based forward distance cross-check, plus yaw-rate cross-check.

        Yaw from rear-wheel differential:
          Valid ONLY in Front-Ackermann mode (steer_mode == 0).  In that mode
          the rear axle does not steer, so:
              ω = (v_rr − v_rl) / d_track
          where v = rpm * 2π/60 * wheel_radius (positive = forward rotation).

          In all other modes the rear axle is steered, the formula is wrong,
          and only the scalar distance accumulation is performed.
        """
        rpm_to_mps = 2.0 * math.pi / 60.0 * self.wheel_radius_m

        v_rl = rpm_lr * rpm_to_mps   # rear-left  wheel surface speed
        v_rr = rpm_rr * rpm_to_mps   # rear-right wheel surface speed

        avg_rpm = (abs(rpm_lf) + abs(rpm_rf) + abs(rpm_lr) + abs(rpm_rr)) / 4.0
        v_rpm   = avg_rpm * rpm_to_mps

        if self._last_rpm_ts is not None:
            dt = ts - self._last_rpm_ts
            if 0 < dt <= 0.5:
                self.distance_rpm_m += v_rpm * dt

                # Yaw cross-check — Front-Ackermann only
                if self.steer_mode == STEER_MODE_FRONT_ACKERMAN:
                    omega_rpm = (v_rr - v_rl) / self.rear_track_m
                    self.yaw_rate_rpm_rads = omega_rpm
                    self.theta_rpm_rad    += omega_rpm * dt

                    # Heading divergence: RPM heading minus steer-model heading
                    err = self.theta_rpm_rad - self.theta_rad
                    self.heading_err_samples.append(math.degrees(err))
                else:
                    # Rear axle steers — differential formula not valid
                    self.yaw_rate_rpm_rads = float('nan')

        self._last_rpm_ts = ts

    @property
    def theta_deg(self):
        return math.degrees(self.theta_rad)

    def summary(self):
        avg_speed = (sum(self.speed_samples) / len(self.speed_samples)
                     if self.speed_samples else 0.0)
        avg_accel = (sum(self.accel_samples) / len(self.accel_samples)
                     if self.accel_samples else 0.0)
        avg_lat   = (sum(self.lateral_samples) / len(self.lateral_samples)
                     if self.lateral_samples else 0.0)
        max_lat   = max((abs(v) for v in self.lateral_samples), default=0.0)

        herr = self.heading_err_samples
        return {
            'distance_speed_m'      : round(self.distance_m, 4),
            'distance_rpm_m'        : round(self.distance_rpm_m, 4),
            'lateral_total_m'       : round(self.lateral_m, 4),
            'lateral_max_mps'       : round(max_lat, 4),
            'lateral_avg_mps'       : round(avg_lat, 4),
            'final_x_m'             : round(self.x, 4),
            'final_y_m'             : round(self.y, 4),
            'final_theta_deg'       : round(self.theta_deg % 360, 4),
            # RPM-differential heading (Front-Ackermann only)
            'final_theta_rpm_deg'   : round(math.degrees(self.theta_rpm_rad) % 360, 4),
            'heading_err_rpm_deg'   : round(herr[-1], 4) if herr else float('nan'),
            'heading_err_max_deg'   : round(max(abs(e) for e in herr), 4) if herr else float('nan'),
            'max_speed_mps'         : round(self.max_speed_mps, 4),
            'max_speed_kph'         : round(self.max_speed_mps * 3.6, 4),
            'min_speed_mps'         : round(self.min_speed_mps, 4)
                                      if self.min_speed_mps < float('inf') else 0.0,
            'avg_speed_mps'         : round(avg_speed, 4),
            'avg_speed_kph'         : round(avg_speed * 3.6, 4),
            'avg_accel_mps2'        : round(avg_accel, 4),
            'samples'               : len(self.speed_samples),
        }


# =============================================================================
#  CSV logger
# =============================================================================

CSV_FIELDS = [
    'timestamp_s', 'elapsed_s', 'can_id',
    # drive
    'drive_enable', 'drive_mode', 'gear', 'speed_mps', 'speed_kph',
    'throttle_pct', 'acceleration_mps2', 'drive_slopover', 'drive_life',
    # brake
    'brake_enable', 'brake_lamp', 'epb_state',
    'brake_pedal_pct', 'brake_pressure_bar', 'brake_life',
    # steer
    'steer_enable', 'steer_mode', 'steer_mode_name', 'steer_slopover',
    'steer_angle_front', 'steer_angle_rear', 'steer_angle_speed', 'steer_life',
    # work
    'driving_mode', 'power_state', 'dc_state',
    'speed_limit_mode', 'speed_limit_mps',
    'low_volt_V', 'estop_state', 'crash_front', 'crash_rear', 'vcu_life',
    # power
    'charge_state', 'soc_pct', 'battery_volt_V', 'battery_curr_A', 'bms_max_temp_C',
    # wheel rpm
    'rpm_lf', 'rpm_rf', 'rpm_lr', 'rpm_rr',
    # integrated pose
    'odo_x_m', 'odo_y_m', 'odo_theta_deg',
    'odo_theta_rpm_deg', 'odo_heading_err_deg',
    'odo_distance_speed_m', 'odo_distance_rpm_m',
    'odo_lateral_m', 'odo_lateral_vel_mps',
]


# =============================================================================
#  Main
# =============================================================================

def main():
    if KNOWN_DISTANCE_M is None:
        print("ERROR: Set KNOWN_DISTANCE_M at the top of the script before running.")
        return

    ts_str   = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(LOG_DIR, f'odometry_{ts_str}.csv')

    print("=" * 70)
    print("  ODOMETRY EVALUATION LOGGER")
    print(f"  Known distance  : {KNOWN_DISTANCE_M} m")
    print(f"  Wheelbase       : {WHEELBASE_M} m")
    print(f"  Rear track      : {REAR_TRACK_M} m")
    print(f"  Log file        : {log_path}")
    print("=" * 70)
    print("  Drive the chassis between the markings using the remote.")
    print("  Press Ctrl+C when the journey is complete.")
    print("=" * 70)
    input("\n  Press ENTER to start logging...\n")

    bus   = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
    odo   = OdometryIntegrator()
    state = {f: '' for f in CSV_FIELDS}

    t_start      = time.time()
    t_last_print = t_start
    row_count    = 0

    with open(log_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=CSV_FIELDS, extrasaction='ignore')
        writer.writeheader()

        try:
            while True:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if msg.arbitration_id not in SUBSCRIBED_IDS:
                    continue

                now     = time.time()
                elapsed = now - t_start
                decoder = DECODERS.get(msg.arbitration_id)
                if decoder is None:
                    continue

                decoded = decoder(bytes(msg.data))

                # ── Odometry update ───────────────────────────────────────
                if msg.arbitration_id == ID_STEER_FB:
                    odo.update_steer(
                        decoded['steer_angle_front'],
                        decoded['steer_angle_rear'],
                        mode=decoded['steer_mode'],
                        ts=now,
                    )
                    decoded['steer_mode_name'] = STEER_MODE_NAMES.get(
                        decoded['steer_mode'], '?')

                elif msg.arbitration_id == ID_DRIVE_FB:
                    odo.update_speed(
                        decoded['speed_mps'], decoded['acceleration_mps2'], now)
                    decoded['speed_kph'] = round(decoded['speed_mps'] * 3.6, 4)

                elif msg.arbitration_id == ID_WHEEL_RPM_FB:
                    odo.update_rpm(
                        decoded['rpm_lf'], decoded['rpm_rf'],
                        decoded['rpm_lr'], decoded['rpm_rr'], now,
                    )

                # ── Merge into rolling state ──────────────────────────────
                herr = (odo.heading_err_samples[-1]
                        if odo.heading_err_samples else float('nan'))
                state.update(decoded)
                state['timestamp_s']         = round(now, 4)
                state['elapsed_s']           = round(elapsed, 4)
                state['can_id']              = hex(msg.arbitration_id)
                state['odo_x_m']             = round(odo.x, 4)
                state['odo_y_m']             = round(odo.y, 4)
                state['odo_theta_deg']       = round(odo.theta_deg % 360, 4)
                state['odo_theta_rpm_deg']   = round(
                    math.degrees(odo.theta_rpm_rad) % 360, 4)
                state['odo_heading_err_deg'] = round(herr, 4) if not math.isnan(herr) else ''
                state['odo_distance_speed_m']= round(odo.distance_m, 4)
                state['odo_distance_rpm_m']  = round(odo.distance_rpm_m, 4)
                state['odo_lateral_m']       = round(odo.lateral_m, 4)
                state['odo_lateral_vel_mps'] = round(
                    odo.lateral_samples[-1] if odo.lateral_samples else 0, 4)

                writer.writerow(state)
                row_count += 1

                # ── Console refresh ───────────────────────────────────────
                if (now - t_last_print) >= (1.0 / PRINT_HZ):
                    t_last_print = now
                    spd = state.get('speed_mps', 0) or 0
                    kph = float(spd) * 3.6 if spd != '' else 0
                    mode_name   = STEER_MODE_NAMES.get(odo.steer_mode, '?')
                    theta_steer = odo.theta_deg % 360
                    theta_rpm   = math.degrees(odo.theta_rpm_rad) % 360
                    herr_disp   = (odo.heading_err_samples[-1]
                                   if odo.heading_err_samples else float('nan'))
                    print(
                        f"  t={elapsed:6.1f}s  "
                        f"spd={kph:5.2f}kph  "
                        f"x={odo.x:+6.3f}m y={odo.y:+6.3f}m  "
                        f"θsteer={theta_steer:6.1f}°  "
                        f"θrpm={theta_rpm:6.1f}°  "
                        f"Δθ={herr_disp:+5.1f}°  "
                        f"steer={odo.steer_front_deg:+5.1f}°F "
                        f"{odo.steer_rear_deg:+5.1f}°R [{mode_name}]",
                        end='\r'
                    )

        except KeyboardInterrupt:
            print("\n\nLogging stopped.")
        finally:
            bus.shutdown()

    summary  = odo.summary()
    duration = time.time() - t_start

    print("\n" + "=" * 70)
    print("  ODOMETRY SUMMARY")
    print("=" * 70)
    print(f"  Known distance (measured)  : {KNOWN_DISTANCE_M:.4f} m")
    print(f"  Estimated distance (speed) : {summary['distance_speed_m']:.4f} m")
    print(f"  Estimated distance (RPM)   : {summary['distance_rpm_m']:.4f} m")
    print(f"  Error (speed)              : "
          f"{summary['distance_speed_m'] - KNOWN_DISTANCE_M:+.4f} m  "
          f"({(summary['distance_speed_m'] - KNOWN_DISTANCE_M) / KNOWN_DISTANCE_M * 100:+.2f}%)")
    print(f"  Error (RPM)                : "
          f"{summary['distance_rpm_m'] - KNOWN_DISTANCE_M:+.4f} m  "
          f"({(summary['distance_rpm_m'] - KNOWN_DISTANCE_M) / KNOWN_DISTANCE_M * 100:+.2f}%)")
    print(f"  ── Final pose ───────────────────────────────────────────────")
    print(f"  Final X                    : {summary['final_x_m']:+.4f} m")
    print(f"  Final Y                    : {summary['final_y_m']:+.4f} m")
    print(f"  Final θ (steer model)      : {summary['final_theta_deg']:.4f}°")
    print(f"  Final θ (RPM differential) : {summary['final_theta_rpm_deg']:.4f}°"
          f"  (Front-Ack only; nan if rear steers)")
    print(f"  Heading error at end       : {summary['heading_err_rpm_deg']:+.4f}°"
          f"  (RPM − steer model)")
    print(f"  Peak heading error         : {summary['heading_err_max_deg']:.4f}°")
    print(f"  ── Lateral ─────────────────────────────────────────────────")
    print(f"  Net lateral displacement   : {summary['lateral_total_m']:+.4f} m  "
          f"({'left' if summary['lateral_total_m'] >= 0 else 'right'})")
    print(f"  Peak lateral velocity      : {summary['lateral_max_mps']:.4f} m/s")
    print(f"  ── Speed ───────────────────────────────────────────────────")
    print(f"  Max speed                  : {summary['max_speed_kph']:.4f} km/h")
    print(f"  Avg speed                  : {summary['avg_speed_kph']:.4f} km/h")
    print(f"  Avg acceleration           : {summary['avg_accel_mps2']:.4f} m/s²")
    print(f"  Duration                   : {duration:.2f} s  |  "
          f"Samples: {summary['samples']}  |  Rows: {row_count}")
    print(f"  Log                        : {log_path}")
    print("=" * 70)


if __name__ == '__main__':
    main()

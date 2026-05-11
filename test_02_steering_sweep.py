"""
test_02_steering_sweep.py
─────────────────────────
Steering sweep test for PIX Moving Hooke chassis.

Sequence:
  1. Handshake (keepalive at zero speed, brake held) for HANDSHAKE_S seconds
  2. Operator confirmation
  3. Sweep: 0° → MAX_STEER_DEG (full left)
  4. Sweep: full left → -MAX_STEER_DEG (full right)
  5. Sweep: full right → 0° (centre)
  6. Graceful stop — brake applied, steer centred

The chassis is stationary throughout. This tests the steering actuator in
isolation before any drive test.

Steer frame (0x132):
  byte[0]   = 0x01 (steer enable) | mode bits
  byte[1:2] = angle as signed 16-bit little-endian, factor=1 deg/count
  byte[6]   = cycle counter (0–15)
  byte[7]   = XOR checksum of bytes 0–6
"""

import can
import os
import time
import struct

# =============================================================================
#  CONFIGURABLE PARAMETERS — edit here
# =============================================================================

CAN_CHANNEL     = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'

MAX_STEER_DEG   = 55        # degrees — road-wheel angle limit (±55°)
SWEEP_RATE_DPS  = 10.0      # degrees per second — sweep speed (lower = gentler)
STEP_INTERVAL_S = 0.02      # seconds between CAN frames (= 20 ms / 50 Hz)
HANDSHAKE_S     = 5.0       # seconds of zero-speed keepalive before sweep begins
DWELL_S         = 1.0       # seconds to dwell at each extreme before reversing

# Brake during sweep (chassis is stationary — hold brake throughout)
# 60% → raw=600 → B1=0x58, B2=0x02
BRAKE_B1        = 0x58
BRAKE_B2        = 0x02

# Drive byte0: enable=1, speed mode, gear=D
DRIVE_BYTE0     = 0x11

# Steer byte0: enable=1, front Ackerman mode (bits 4–7 = 0)
STEER_BYTE0     = 0x01

# =============================================================================
#  Derived — do not edit
# =============================================================================

DEG_PER_STEP    = SWEEP_RATE_DPS * STEP_INTERVAL_S   # degrees advanced per frame


# =============================================================================
#  CAN helpers
# =============================================================================

def build_frame(can_id, payload):
    """7-byte payload → append XOR checksum → return can.Message."""
    checksum = 0
    for b in payload[:7]:
        checksum ^= b
    payload.append(checksum)
    return can.Message(
        arbitration_id=can_id,
        data=bytearray(payload),
        is_extended_id=False,
    )


def steer_frame(angle_deg, cycle):
    """
    Build 0x132 steer frame for a given road-wheel angle.
    angle_deg: signed float, positive = left, negative = right
    Clamped to ±MAX_STEER_DEG before encoding.
    """
    angle_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, angle_deg))
    raw       = int(round(angle_deg))          # factor=1 deg/count
    lo        = raw & 0xFF
    hi        = (raw >> 8) & 0xFF
    return build_frame(0x132, [STEER_BYTE0, lo, hi, 0x00, 0x00, 0x00, cycle & 0x0F])


def keepalive(bus, cycle, angle_deg=0.0):
    """Send zero-speed drive + brake + steer keepalive."""
    raw = int(round(max(-MAX_STEER_DEG, min(MAX_STEER_DEG, angle_deg))))
    lo  = raw & 0xFF
    hi  = (raw >> 8) & 0xFF
    bus.send(build_frame(0x130, [DRIVE_BYTE0, 0x00, 0x00, 0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x132, [STEER_BYTE0, lo, hi, 0x00, 0x00, 0x00, cycle & 0x0F]))


def full_stop(bus, cycle):
    """Drive disable + brake + steer centred."""
    bus.send(build_frame(0x130, [0x00, 0x00,    0x00, 0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x132, [STEER_BYTE0, 0x00, 0x00, 0x00, 0x00, 0x00, cycle & 0x0F]))
    print("\n[STOP] Steer centred. Brake applied.")


# =============================================================================
#  Sweep helper
# =============================================================================

def sweep(bus, cycle_ref, from_deg, to_deg, label):
    """
    Sweep steer from from_deg to to_deg at SWEEP_RATE_DPS.
    Returns final cycle counter.
    Sends keepalive drive+brake frames alongside steer frame every tick.
    """
    cycle   = cycle_ref
    current = float(from_deg)
    step    = DEG_PER_STEP if to_deg > from_deg else -DEG_PER_STEP
    total   = abs(to_deg - from_deg)
    done    = 0.0

    print(f"\n  [{label}]  {from_deg:+.1f}° → {to_deg:+.1f}°  "
          f"({total:.0f}° at {SWEEP_RATE_DPS} °/s = {total/SWEEP_RATE_DPS:.1f}s)")

    while True:
        remaining = to_deg - current
        if abs(remaining) < abs(step):
            current = to_deg
        else:
            current += step

        # Drive keepalive (zero speed) + steer command
        raw = int(round(current))
        lo  = raw & 0xFF
        hi  = (raw >> 8) & 0xFF
        bus.send(build_frame(0x130, [DRIVE_BYTE0, 0x00, 0x00, 0x00, 0x01, 0x00, cycle & 0x0F]))
        bus.send(build_frame(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
        bus.send(build_frame(0x132, [STEER_BYTE0, lo, hi, 0x00, 0x00, 0x00, cycle & 0x0F]))
        cycle = (cycle + 1) & 0x0F

        done = abs(current - from_deg)
        print(f"    steer = {current:+6.1f}°  ({done:.1f}° / {total:.0f}°)", end='\r')

        time.sleep(STEP_INTERVAL_S)

        if current == to_deg:
            print(f"    steer = {current:+6.1f}°  [REACHED]              ")
            break

    return cycle


def dwell(bus, cycle_ref, angle_deg, seconds, label):
    """Hold a fixed steer angle for a dwell period, continuing keepalive."""
    cycle   = cycle_ref
    end     = time.time() + seconds
    raw     = int(round(angle_deg))
    lo      = raw & 0xFF
    hi      = (raw >> 8) & 0xFF
    print(f"  [DWELL {label}]  holding {angle_deg:+.1f}° for {seconds}s...")
    while time.time() < end:
        bus.send(build_frame(0x130, [DRIVE_BYTE0, 0x00, 0x00, 0x00, 0x01, 0x00, cycle & 0x0F]))
        bus.send(build_frame(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
        bus.send(build_frame(0x132, [STEER_BYTE0, lo, hi, 0x00, 0x00, 0x00, cycle & 0x0F]))
        cycle = (cycle + 1) & 0x0F
        time.sleep(STEP_INTERVAL_S)
    return cycle


# =============================================================================
#  Main
# =============================================================================

def main():
    print("=" * 70)
    print("  TEST 02 — STEERING SWEEP")
    print("=" * 70)
    print(f"  Range      : ±{MAX_STEER_DEG}°")
    print(f"  Sweep rate : {SWEEP_RATE_DPS} °/s")
    print(f"  Dwell      : {DWELL_S}s at each extreme")
    print(f"  Handshake  : {HANDSHAKE_S}s")
    print(f"  Sequence   : 0° → +{MAX_STEER_DEG}° → -{MAX_STEER_DEG}° → 0°")
    print("=" * 70)
    input("\n  Ensure chassis is stationary and area around wheels is clear.")
    input("  Press ENTER to begin handshake...\n")

    bus   = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
    cycle = 0

    try:
        # ── Phase 1: Handshake ─────────────────────────────────────────────
        print(f"Phase 1: Handshake ({HANDSHAKE_S}s, steer=0°, brake held)...")
        end = time.time() + HANDSHAKE_S
        while time.time() < end:
            keepalive(bus, cycle, angle_deg=0.0)
            cycle = (cycle + 1) & 0x0F
            print(f"  {end - time.time():.1f}s remaining...", end='\r')
            time.sleep(STEP_INTERVAL_S)
        print("  Handshake complete.                    ")

        # ── Operator gate ──────────────────────────────────────────────────
        print("\n" + "=" * 70)
        print(f"  GO — about to sweep ±{MAX_STEER_DEG}° at {SWEEP_RATE_DPS} °/s")
        print("=" * 70)
        answer = input("\n  Authorize sweep? (y/n): ").strip().lower()
        if answer != 'y':
            print("  Aborted by operator.")
            return

        t_start = time.time()

        # ── Phase 2: Sweep ─────────────────────────────────────────────────
        print("\nPhase 2: Sweep starting...")

        # Leg 1: centre → full left
        cycle = sweep(bus, cycle, 0.0, +MAX_STEER_DEG, 'CENTRE → LEFT')
        cycle = dwell(bus, cycle, +MAX_STEER_DEG, DWELL_S, 'LEFT')

        # Leg 2: full left → full right
        cycle = sweep(bus, cycle, +MAX_STEER_DEG, -MAX_STEER_DEG, 'LEFT → RIGHT')
        cycle = dwell(bus, cycle, -MAX_STEER_DEG, DWELL_S, 'RIGHT')

        # Leg 3: full right → centre
        cycle = sweep(bus, cycle, -MAX_STEER_DEG, 0.0, 'RIGHT → CENTRE')
        cycle = dwell(bus, cycle, 0.0, DWELL_S, 'CENTRE')

        elapsed = time.time() - t_start
        print(f"\nSweep complete in {elapsed:.1f}s.")

    except KeyboardInterrupt:
        print("\nUser abort (Ctrl+C).")

    finally:
        full_stop(bus, cycle)
        bus.shutdown()
        print("Steering sweep test finished.")


if __name__ == '__main__':
    main()

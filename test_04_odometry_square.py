"""
test_04_odometry_square.py
──────────────────────────
Odometry closure test for PIX Moving Hooke chassis.

Drives a square of configurable side length at low speed, making four 90°
turns. After the run, the operator measures physical start-to-end displacement
to assess odometry quality.

Sequence per side:
  1. Drive straight for SIDE_DURATION_S seconds (timed from speed assumption)
  2. Stop, apply brake
  3. Turn in-place by commanding a fixed steer angle + short drive arc for
     TURN_DURATION_S seconds (approximates 90°)
  4. Centre steer, brief settle

Physical constants (Hooke):
  Wheelbase   : 1900 mm = 1.9 m
  Track width : 1465 mm = 1.465 m

CAN IDs used:
  0x130 — drive control
  0x131 — brake control
  0x132 — steer control

Frame rate: 20 ms (50 Hz) throughout to satisfy VCU watchdog.
"""

import can
import os
import time
import math

# =============================================================================
#  CONFIGURABLE PARAMETERS — edit here
# =============================================================================

CAN_CHANNEL         = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'

# Square geometry
SIDE_LENGTH_M       = 2.0       # metres per side
DRIVE_SPEED_KPH     = 1.0       # km/h — straight-line speed

# Turn parameters
# At DRIVE_SPEED_KPH with MAX_STEER_DEG, the chassis traces a circle.
# TURN_DURATION_S is computed automatically for a 90° arc, but can be
# overridden manually by setting TURN_DURATION_OVERRIDE_S to a float.
MAX_STEER_DEG       = 55.0      # degrees — road-wheel angle for turn
TURN_DIRECTION      = 'left'    # 'left' or 'right' — which way to turn at each corner
TURN_DURATION_OVERRIDE_S = None # set to e.g. 3.5 to override auto-calculation

# Timing
HANDSHAKE_S         = 5.0       # seconds keepalive before motion
BRAKE_SETTLE_S      = 1.0       # seconds brake hold between straight and turn
STEER_SETTLE_S      = 0.5       # seconds to hold centred steer after each turn

# Brake
# 60% → raw=600 → B1=0x58, B2=0x02
BRAKE_B1            = 0x58
BRAKE_B2            = 0x02

# =============================================================================
#  Derived — do not edit below unless you know what you're doing
# =============================================================================

WHEELBASE_M         = 1.9       # Hooke wheelbase (fixed physical constant)
STEP_S              = 0.02      # 20 ms CAN frame interval

# Speed in m/s and raw CAN count
DRIVE_SPEED_MPS     = DRIVE_SPEED_KPH * 1000.0 / 3600.0
SPEED_RAW           = int(round(DRIVE_SPEED_MPS / 0.01))   # factor=0.01 m/s
SPEED_B1            = SPEED_RAW & 0xFF
SPEED_B2            = (SPEED_RAW >> 8) & 0xFF

# Straight-line duration from distance and speed
SIDE_DURATION_S     = SIDE_LENGTH_M / DRIVE_SPEED_MPS

# Turn radius from bicycle model: R = L / tan(steer)
_steer_rad          = math.radians(MAX_STEER_DEG)
TURN_RADIUS_M       = WHEELBASE_M / math.tan(_steer_rad)

# Arc length for 90°: s = R × π/2
_arc_length_m       = TURN_RADIUS_M * math.pi / 2.0
TURN_DURATION_S     = TURN_DURATION_OVERRIDE_S or (_arc_length_m / DRIVE_SPEED_MPS)

# Steer sign
_steer_sign         = +1 if TURN_DIRECTION == 'left' else -1
TURN_ANGLE_DEG      = _steer_sign * MAX_STEER_DEG
_turn_raw           = int(round(TURN_ANGLE_DEG))
TURN_STEER_B1       = _turn_raw & 0xFF
TURN_STEER_B2       = (_turn_raw >> 8) & 0xFF

DRIVE_BYTE0         = 0x11   # enable=1, speed mode, gear=D
STEER_BYTE0         = 0x01   # steer enable, front Ackerman


# =============================================================================
#  CAN helpers
# =============================================================================

def build_frame(can_id, payload):
    checksum = 0
    for b in payload[:7]:
        checksum ^= b
    payload.append(checksum)
    return can.Message(
        arbitration_id=can_id,
        data=bytearray(payload),
        is_extended_id=False,
    )


def send_straight(bus, cycle):
    """Drive straight, steer centred, brake released."""
    bus.send(build_frame(0x130, [DRIVE_BYTE0, SPEED_B1, SPEED_B2, 0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x131, [0x01, 0x00,    0x00,    0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x132, [STEER_BYTE0, 0x00, 0x00, 0x00, 0x00, 0x00, cycle & 0x0F]))


def send_turn(bus, cycle):
    """Drive at speed with turn steer angle applied — traces arc."""
    bus.send(build_frame(0x130, [DRIVE_BYTE0, SPEED_B1,      SPEED_B2,      0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x131, [0x01,        0x00,          0x00,          0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x132, [STEER_BYTE0, TURN_STEER_B1, TURN_STEER_B2, 0x00, 0x00, 0x00, cycle & 0x0F]))


def send_brake(bus, cycle, steer_b1=0x00, steer_b2=0x00):
    """Zero speed + brake, optional steer hold."""
    bus.send(build_frame(0x130, [DRIVE_BYTE0, 0x00,    0x00,    0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x131, [0x01,        BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x132, [STEER_BYTE0, steer_b1, steer_b2, 0x00, 0x00, 0x00, cycle & 0x0F]))


def send_stop(bus, cycle):
    """Full disable + brake + steer centred."""
    bus.send(build_frame(0x130, [0x00,        0x00,    0x00,    0x00, 0x01, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x131, [0x01,        BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, cycle & 0x0F]))
    bus.send(build_frame(0x132, [STEER_BYTE0, 0x00,    0x00,    0x00, 0x00, 0x00, cycle & 0x0F]))
    print("\n[STOP] All channels disabled. Brake applied.")


def timed_phase(bus, cycle_ref, duration_s, send_fn, label):
    """
    Run send_fn(bus, cycle) every STEP_S for duration_s seconds.
    Returns (final_cycle, actual_elapsed).
    """
    cycle   = cycle_ref
    start   = time.time()
    end     = start + duration_s
    while time.time() < end:
        send_fn(bus, cycle)
        cycle = (cycle + 1) & 0x0F
        elapsed = time.time() - start
        print(f"  [{label}]  {elapsed:.2f}s / {duration_s:.2f}s", end='\r')
        time.sleep(STEP_S)
    actual = time.time() - start
    print(f"  [{label}]  {actual:.2f}s  DONE                    ")
    return cycle, actual


# =============================================================================
#  Main
# =============================================================================

def main():
    print("=" * 70)
    print("  TEST 04 — ODOMETRY CLOSURE (SQUARE)")
    print("=" * 70)
    print(f"  Side length    : {SIDE_LENGTH_M} m")
    print(f"  Speed          : {DRIVE_SPEED_KPH} km/h  ({DRIVE_SPEED_MPS:.3f} m/s)")
    print(f"  Side duration  : {SIDE_DURATION_S:.2f} s")
    print(f"  Turn direction : {TURN_DIRECTION.upper()} at each corner")
    print(f"  Turn angle     : {TURN_ANGLE_DEG:+.1f}° (road wheel)")
    print(f"  Turn radius    : {TURN_RADIUS_M:.3f} m  (bicycle model)")
    print(f"  Turn duration  : {TURN_DURATION_S:.2f} s  "
          f"({'auto' if TURN_DURATION_OVERRIDE_S is None else 'OVERRIDE'})")
    print(f"  Handshake      : {HANDSHAKE_S}s")
    print(f"  Brake settle   : {BRAKE_SETTLE_S}s  |  Steer settle: {STEER_SETTLE_S}s")
    print("=" * 70)
    print("\n  SETUP INSTRUCTIONS:")
    print("  1. Mark the START position of the chassis (tape/chalk).")
    print("  2. Ensure a clear square area of at least "
          f"{SIDE_LENGTH_M + 1:.0f} x {SIDE_LENGTH_M + 1:.0f} m.")
    print("  3. Switch remote to autonomous mode (push rod 6 down).")
    input("\n  Press ENTER to begin handshake...\n")

    bus    = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
    cycle  = 0
    aborted = False

    # Timing log per side and turn
    side_times = []
    turn_times = []

    try:
        # ── Phase 1: Handshake ─────────────────────────────────────────────
        print(f"Phase 1: Handshake ({HANDSHAKE_S}s)...")
        end = time.time() + HANDSHAKE_S
        while time.time() < end:
            send_brake(bus, cycle)
            cycle = (cycle + 1) & 0x0F
            print(f"  {end - time.time():.1f}s remaining...", end='\r')
            time.sleep(STEP_S)
        print("  Handshake complete.              ")

        # ── Operator gate ──────────────────────────────────────────────────
        print("\n" + "=" * 70)
        print(f"  GO — {SIDE_LENGTH_M}m square, {DRIVE_SPEED_KPH} km/h, turning {TURN_DIRECTION.upper()}")
        print("=" * 70)
        answer = input("\n  Authorize square run? (y/n): ").strip().lower()
        if answer != 'y':
            print("  Aborted by operator.")
            aborted = True
            return

        t_total_start = time.time()

        # ── Phase 2: Four sides ────────────────────────────────────────────
        for side in range(1, 5):
            print(f"\n{'─'*70}")
            print(f"  SIDE {side} of 4  —  driving {SIDE_LENGTH_M} m straight")
            print(f"{'─'*70}")

            cycle, st = timed_phase(
                bus, cycle, SIDE_DURATION_S,
                send_straight, f'SIDE {side}'
            )
            side_times.append(st)

            # Brake settle
            cycle, _ = timed_phase(
                bus, cycle, BRAKE_SETTLE_S,
                send_brake, f'BRAKE {side}'
            )

            if side < 4:
                print(f"\n  TURN {side} — {TURN_DIRECTION.upper()} "
                      f"({TURN_ANGLE_DEG:+.1f}° for {TURN_DURATION_S:.2f}s)")

                cycle, tt = timed_phase(
                    bus, cycle, TURN_DURATION_S,
                    send_turn, f'TURN {side}'
                )
                turn_times.append(tt)

                # Steer settle (centred)
                cycle, _ = timed_phase(
                    bus, cycle, STEER_SETTLE_S,
                    send_brake, f'SETTLE {side}'
                )

        total_elapsed = time.time() - t_total_start

        # ── Summary ────────────────────────────────────────────────────────
        print(f"\n{'='*70}")
        print("  SQUARE RUN COMPLETE")
        print(f"{'='*70}")
        print(f"  Total elapsed : {total_elapsed:.2f} s")
        print(f"  Side times    : " +
              "  ".join(f"S{i+1}={t:.2f}s" for i, t in enumerate(side_times)))
        print(f"  Turn times    : " +
              "  ".join(f"T{i+1}={t:.2f}s" for i, t in enumerate(turn_times)))
        print(f"  Expected perimeter : {4 * SIDE_LENGTH_M:.2f} m")
        print(f"  Expected distance  : ~0.00 m closure (ideal square)")
        print(f"{'='*70}")
        print("\n  ACTION REQUIRED:")
        print("  Measure the physical displacement between the START mark")
        print("  and the current chassis position.")
        print("  Record this as the CLOSURE ERROR for this run.")

    except KeyboardInterrupt:
        print("\nUser abort (Ctrl+C).")
        aborted = True

    finally:
        send_stop(bus, cycle)
        bus.shutdown()
        print(f"\nTest {'aborted' if aborted else 'completed'}.")


if __name__ == '__main__':
    main()

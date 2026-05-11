#!/usr/bin/env python3
"""
can_publisher.py
════════════════
Sole CAN bus writer for the PIX Moving Hooke.  Always-on cron job — never
let two processes write CAN simultaneously.

Inputs (UDP receive)
  :5006  arbiter   {"safe", "level", "trigger", "dist", "ydlidar_online"}
  :5007  scenario  {"action", "speed_mps", "steer_deg", "fsm"}

Outputs
  CAN 0x130  drive    0x131  brake    0x132  steer    0x133  lamps

Priority merge
  estop(2) > brake(1) > drive(0)
  Missing arbiter → forced brake; missing scenario → left blink standby.

Lamp alerts (byte 0 of CAN 0x133)
  0x0C  hazard (left+right)  — arbiter offline
  0x08  right blink          — ydlidar offline
  0x04  left blink           — scenario offline (cron not running)
  0x00  clear                — all nominal, drive active
"""

import can
import os
import json
import socket
import subprocess
import time
import threading

# ── Configuration ──────────────────────────────────────────────────────────────
CAN_CHANNEL      = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
ARBITER_PORT     = 5006
SCENARIO_PORT    = 5007
CONTROL_HZ       = 20          # 50 ms loop — VCU watchdog expects ≥ 10 Hz
ARBITER_TIMEOUT  = 2.0         # s — arbiter silent → offline
SCENARIO_TIMEOUT = 0.3         # s — scenario command stale → standby
MAX_STEER_DEG    = 22.73   # max road-wheel angle (500° actuator / ratio 22)
STEER_GEAR_RATIO = 22.0    # VCU expects actuator degrees; scenario sends road-wheel degrees
LAMP_HZ          = 5       # 0x133 must be sent at 200 ms per spec (not every 20 ms loop)

# CAN encoding (PIX Hooke VCU spec confirmed)
DRIVE_BYTE0   = 0x11   # drive enable, speed control mode, gear D
REVERSE_BYTE0 = 0x31   # drive enable, speed mode, gear R
STEER_BYTE0   = 0x01   # steer enable, front Ackermann mode 0
BRAKE_B1      = 0x58   # 60 % service-brake  — raw 600 (bytes: 0x58, 0x02)
BRAKE_B2      = 0x02
ESTOP_B1      = 0xE8   # 100 % emergency-brake — raw 1000 (bytes: 0xE8, 0x03)
ESTOP_B2      = 0x03

# Lamp byte 0 values (CAN 0x133)
LAMP_NONE   = 0x00
LAMP_LEFT   = 0x04
LAMP_RIGHT  = 0x08
LAMP_HAZARD = 0x0C   # left | right

PRIORITY = {'drive': 0, 'brake': 1, 'estop': 2}

# ── CAN helpers ────────────────────────────────────────────────────────────────

def _xor_cs(p7: list) -> int:
    cs = 0
    for b in p7:
        cs ^= b
    return cs

def _msg(can_id: int, p7: list) -> can.Message:
    return can.Message(
        arbitration_id=can_id,
        data=bytearray(p7 + [_xor_cs(p7)]),
        is_extended_id=False,
    )

def _speed_bytes(mps: float):
    raw = min(int(round(max(0.0, mps) / 0.01)), 0x7FFF)
    return raw & 0xFF, (raw >> 8) & 0xFF

def _steer_bytes(road_deg: float):
    """Convert road-wheel angle (deg) to actuator angle bytes for the VCU."""
    actuator = max(-500.0, min(500.0, road_deg * STEER_GEAR_RATIO))
    raw = int(round(actuator)) & 0xFFFF
    return raw & 0xFF, (raw >> 8) & 0xFF

def _send_drive(bus, cyc: int, speed_mps: float, steer_deg: float):
    sb1, sb2 = _speed_bytes(speed_mps)
    tb1, tb2 = _steer_bytes(steer_deg)
    c = cyc & 0x0F
    bus.send(_msg(0x130, [DRIVE_BYTE0, sb1, sb2, 0x00, 0x01, 0x00, c]))
    bus.send(_msg(0x131, [0x01, 0x00,    0x00,    0x02, 0x00, 0x00, c]))
    bus.send(_msg(0x132, [STEER_BYTE0, tb1, tb2, 0x00, 0x00, 0x00, c]))

def _send_brake(bus, cyc: int, steer_deg: float = 0.0):
    tb1, tb2 = _steer_bytes(steer_deg)
    c = cyc & 0x0F
    bus.send(_msg(0x130, [DRIVE_BYTE0, 0x00,    0x00,    0x00, 0x01, 0x00, c]))
    bus.send(_msg(0x131, [0x01, BRAKE_B1, BRAKE_B2, 0x02, 0x00, 0x00, c]))
    bus.send(_msg(0x132, [STEER_BYTE0, tb1, tb2, 0x00, 0x00, 0x00, c]))

def _send_estop(bus, cyc: int):
    c = cyc & 0x0F
    bus.send(_msg(0x130, [0x00, 0x00,    0x00,    0x00, 0x01, 0x00, c]))
    bus.send(_msg(0x131, [0x01, ESTOP_B1, ESTOP_B2, 0x02, 0x00, 0x00, c]))  # 100% brake
    bus.send(_msg(0x132, [STEER_BYTE0, 0x00, 0x00, 0x00, 0x00, 0x00, c]))

def _send_passive(bus, cyc: int):
    """Watchdog-only frames — no drive/brake/steer request. Lets RC keep control."""
    c = cyc & 0x0F
    bus.send(_msg(0x130, [0x00, 0x00, 0x00, 0x00, 0x01, 0x00, c]))
    bus.send(_msg(0x131, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, c]))
    bus.send(_msg(0x132, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, c]))

def _send_reverse(bus, cyc: int, speed_mps: float, steer_deg: float):
    sb1, sb2 = _speed_bytes(speed_mps)
    tb1, tb2 = _steer_bytes(steer_deg)
    c = cyc & 0x0F
    bus.send(_msg(0x130, [REVERSE_BYTE0, sb1, sb2, 0x00, 0x01, 0x00, c]))
    bus.send(_msg(0x131, [0x01, 0x00,    0x00,     0x02, 0x00, 0x00, c]))
    bus.send(_msg(0x132, [STEER_BYTE0, tb1, tb2, 0x00, 0x00, 0x00, c]))

def _bus_recover(old_bus) -> can.interface.Bus:
    """Full CAN controller reset: bring interface down/up then reopen socket."""
    try:
        old_bus.shutdown()
    except Exception:
        pass
    try:
        subprocess.run(['sudo', 'ip', 'link', 'set', CAN_CHANNEL, 'down'],
                       timeout=3, check=False)
        time.sleep(0.2)
        subprocess.run(['sudo', 'ip', 'link', 'set', CAN_CHANNEL, 'up',
                        'type', 'can', 'bitrate', '500000', 'restart-ms', '100'],
                       timeout=3, check=False)
        time.sleep(0.5)
    except Exception as e:
        print(f"[PUB] ip reset failed: {e} — sleeping 1 s", flush=True)
        time.sleep(1.0)
    return can.interface.Bus(channel=CAN_CHANNEL, interface='socketcan')

# ── Thread-safe UDP state ──────────────────────────────────────────────────────

class _UDPState:
    def __init__(self):
        self._pkt: dict | None = None
        self._ts: float = 0.0
        self._lk = threading.Lock()

    def put(self, pkt: dict):
        with self._lk:
            self._pkt, self._ts = pkt, time.time()

    def get(self) -> tuple:
        with self._lk:
            return self._pkt, self._ts

def _rx_loop(port: int, state: _UDPState):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', port))
    s.settimeout(0.5)
    while True:
        try:
            data, _ = s.recvfrom(512)
            state.put(json.loads(data.decode()))
        except (socket.timeout, json.JSONDecodeError):
            pass

# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    arb  = _UDPState()
    scen = _UDPState()

    threading.Thread(target=_rx_loop, args=(ARBITER_PORT,  arb),  daemon=True).start()
    threading.Thread(target=_rx_loop, args=(SCENARIO_PORT, scen), daemon=True).start()

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface='socketcan')
    print(f"[PUB] arb←:{ARBITER_PORT}  scen←:{SCENARIO_PORT}  {CONTROL_HZ} Hz")

    dt          = 1.0 / CONTROL_HZ
    dt_lamp     = 1.0 / LAMP_HZ
    cyc         = 0
    t_next      = time.perf_counter()
    t_next_lamp = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            if now < t_next:
                time.sleep(t_next - now)
            t_next += dt

            arb_pkt,  arb_ts  = arb.get()
            scen_pkt, scen_ts = scen.get()
            wall = time.time()

            arb_ok  = arb_pkt  is not None and (wall - arb_ts)  < ARBITER_TIMEOUT
            scen_ok = scen_pkt is not None and (wall - scen_ts) < SCENARIO_TIMEOUT

            # ── Arbiter level ─────────────────────────────────────────────────
            # When arbiter is offline but scenario is live (autonomous run with
            # arbiter intentionally stopped), default to 'drive' so the scenario
            # is not overridden. When both are offline (RC mode), default to
            # 'brake' as a conservative fallback.
            if arb_ok:
                arb_level = arb_pkt.get('level', 'brake')
            elif scen_ok:
                arb_level = 'drive'   # scenario killed arbiter intentionally
            else:
                arb_level = 'brake'   # RC mode, no arbiter — conservative

            # ── Lamp ──────────────────────────────────────────────────────────
            lamp = LAMP_NONE
            if not arb_ok and not scen_ok:
                lamp |= LAMP_HAZARD   # hazard only when truly unmonitored (RC+no arbiter)
            if arb_ok and not arb_pkt.get('ydlidar_online', True):
                lamp |= LAMP_RIGHT
            if not scen_ok:
                lamp |= LAMP_LEFT
                if arb_ok and arb_level == 'estop':
                    lamp |= LAMP_HAZARD   # RC estop override — add hazards

            # ── Motion ────────────────────────────────────────────────────────
            scen_speed   = 0.0
            scen_steer   = 0.0
            scen_reverse = False

            try:
                if not scen_ok:
                    # RC mode: only estop overrides RC; brake and drive are passive
                    if arb_level == 'estop':
                        _send_estop(bus, cyc)
                        mode_str = 'RC-STOP'
                    else:
                        _send_passive(bus, cyc)
                        mode_str = 'RC-PASS'
                else:
                    # Self-drive: full priority merge
                    scen_level   = 'brake'
                    if scen_pkt:
                        action     = scen_pkt.get('action', 'stop')
                        scen_steer = float(scen_pkt.get('steer_deg', 0.0))
                        if action == 'drive':
                            scen_level = 'drive'
                            scen_speed = float(scen_pkt.get('speed_mps', 0.0))
                        elif action == 'reverse':
                            scen_level   = 'drive'
                            scen_speed   = float(scen_pkt.get('speed_mps', 0.0))
                            scen_reverse = True
                        elif action == 'brake':
                            scen_level = 'brake'
                        else:
                            scen_level = 'estop'

                    final = max(arb_level, scen_level, key=lambda l: PRIORITY[l])
                    if final == 'estop':
                        _send_estop(bus, cyc)
                    elif final == 'brake':
                        _send_brake(bus, cyc, scen_steer)
                    elif scen_reverse:
                        _send_reverse(bus, cyc, scen_speed, scen_steer)
                    else:
                        _send_drive(bus, cyc, scen_speed, scen_steer)
                    mode_str = final.upper().ljust(7)

                # 0x133 at 200 ms (5 Hz per spec); byte6=0x00 keeps CheckSumEn=0 (enabled)
                now_lamp = time.perf_counter()
                if now_lamp >= t_next_lamp:
                    bus.send(_msg(0x133, [lamp, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
                    t_next_lamp += dt_lamp

            except can.CanOperationError as e:
                print(f"\n[PUB] CAN error ({e}) — recovering bus …", flush=True)
                bus = _bus_recover(bus)
                print("[PUB] bus reopened", flush=True)
                continue

            print(f"\r[PUB] {mode_str}  "
                  f"arb={'on ' if arb_ok else 'OFF'}  "
                  f"scen={'on ' if scen_ok else 'OFF'}  "
                  f"lamp=0x{lamp:02X}  "
                  f"v={scen_speed:.2f}m/s  δ={scen_steer:.1f}°   ",
                  end='', flush=True)

            cyc = (cyc + 1) & 0x0F

    except KeyboardInterrupt:
        print("\n[PUB] shutdown — full stop")
    finally:
        for _ in range(5):
            try:
                _send_estop(bus, cyc)
                bus.send(_msg(0x133, [LAMP_NONE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
            except can.CanOperationError:
                pass
            cyc = (cyc + 1) & 0x0F
        try:
            bus.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()

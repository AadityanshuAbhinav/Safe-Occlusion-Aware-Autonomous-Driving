#!/usr/bin/env python3
"""
ydlidar_forwarder.py
════════════════════
Reads YDLidar X2 (USB) and broadcasts safety JSON on :5005.
Runs on the Jetson alongside jetson_ouster_arbiter.py.

Serial port: /dev/ttyUSB0 by default. If the USB-CAN adapter also
appears as a ttyUSB device (some slcan adapters do), increment to
/dev/ttyUSB1 and verify with: ls -l /dev/ttyUSB*
"""
import json
import math
import socket
import ydlidar

BROADCAST_ADDR = "255.255.255.255"
GUARD_PORT     = 5005

# Sensor at chassis FRONT CENTER, 30 cm inside front bumper, mounted facing REARWARD.
# SDK 0° points to vehicle rear; ±180° points to vehicle front.
# CW-positive angles (X2 SDK): +90° from rear = vehicle LEFT; −90° from rear = vehicle RIGHT.
# Chassis: 2.6 m long × 1.7 m wide.
# Front zone (|veh_angle| ≤ 45°): bumper returns at 0.30 m (head-on) to 0.30/cos(45°)=0.424 m (corner).
#   THRESH_FRONT_MIN = 0.45 m masks all bumper artefacts with 2.6 cm margin.
# Side zones (45°–90° each side): chassis half-width = 0.85 m from sensor centre-line.
#   THRESH_SIDE_MIN = 0.85 m masks body returns; matches arbiter DIST_IGNORE_SIDE.
# Outer thresholds are set generously — the scenario guards enforce the precise stop margins.
THRESH_FRONT     = 2.00   # m — outer unsafe threshold, vehicle-front zone
THRESH_FRONT_MIN = 0.45   # m — inner filter: masks bumper body artefacts (max return = 0.424 m)
THRESH_SIDE      = 1.20   # m — outer unsafe threshold, side zones
THRESH_SIDE_MIN  = 0.85   # m — inner filter; chassis half-width from sensor centre

_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def send_packet(safe: bool, front: float, left: float, right: float, trigger: str):
    pkt = {"safe": safe, "front": round(front, 3),
           "left": round(left, 3), "right": round(right, 3), "trigger": trigger}
    _udp_sock.sendto(json.dumps(pkt).encode(), (BROADCAST_ADDR, GUARD_PORT))

def main():
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort,     "/dev/ttyUSB0")
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType,      ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType,     ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel,  True)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy,      False)
    laser.setlidaropt(ydlidar.LidarPropAutoReconnect,  True)
    laser.setlidaropt(ydlidar.LidarPropMinRange,       0.05)
    laser.setlidaropt(ydlidar.LidarPropMaxRange,       8.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle,      -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle,       180.0)

    if not laser.initialize():
        print("[ERROR] YDLidar failed to initialise")
        return
    if not laser.turnOn():
        print("[ERROR] YDLidar failed to start scanning")
        return

    scan = ydlidar.LaserScan()
    try:
        while ydlidar.os_isOk():
            ret = laser.doProcessSimple(scan)   # fills scan, returns bool
            if not ret:
                send_packet(True, -1.0, -1.0, -1.0, 'none')   # sensor heartbeat
                continue

            min_front = math.inf
            min_left  = math.inf
            min_right = math.inf

            for p in scan.points:
                a = math.degrees(p.angle)   # SDK gives radians → degrees
                d = p.range
                if d < 0.05 or d > 8.0:
                    continue
                # Sensor 0° faces vehicle rear; ±180° is vehicle front.
                # CW-positive angles (X2 convention): +90° from rear = vehicle LEFT; −90° = vehicle RIGHT.
                if a >= 135.0 or a <= -135.0:
                    if d >= THRESH_FRONT_MIN:
                        min_front = min(min_front, d)
                elif 45.0 < a < 135.0:       # +90° zone = vehicle LEFT
                    if d >= THRESH_SIDE_MIN:
                        min_left  = min(min_left, d)
                elif -135.0 < a < -45.0:     # −90° zone = vehicle RIGHT
                    if d >= THRESH_SIDE_MIN:
                        min_right = min(min_right, d)

            # Pick the closest unsafe zone as trigger
            unsafe = []
            if min_front < THRESH_FRONT:
                unsafe.append(('front', min_front))
            if min_left < THRESH_SIDE:
                unsafe.append(('left',  min_left))
            if min_right < THRESH_SIDE:
                unsafe.append(('right', min_right))

            safe    = len(unsafe) == 0
            trigger = min(unsafe, key=lambda x: x[1])[0] if unsafe else 'none'

            f = min_front if math.isfinite(min_front) else -1.0
            l = min_left  if math.isfinite(min_left)  else -1.0
            r = min_right if math.isfinite(min_right) else -1.0
            send_packet(safe, f, l, r, trigger)
            print(f"\r[YDL] {'SAFE  ' if safe else 'UNSAFE'}  "
                  f"front={f:5.2f}m  left={l:5.2f}m  right={r:5.2f}m  "
                  f"trig={trigger:5s}",
                  end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        laser.turnOff()
        laser.disconnecting()

if __name__ == "__main__":
    main()


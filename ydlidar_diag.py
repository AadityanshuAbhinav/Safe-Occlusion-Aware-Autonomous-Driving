#!/usr/bin/env python3
"""
ydlidar_diag.py
═══════════════
Listens on :5005 for JSON packets from ydlidar_forwarder.py and prints
directional distances live.  Run ydlidar_forwarder.py first.

Sign convention inside ydlidar_forwarder.py (sensor at vehicle FRONT, facing REARWARD):
  SDK 0°    → vehicle REAR
  SDK ±180° → vehicle FRONT
  SDK uses CW-positive angles (X2): +90° from rear = vehicle LEFT; −90° = vehicle RIGHT
  "front"  : |sensor_angle| ≥ 135°    (vehicle-front zone)
  "left"   : 45° < sensor_angle < 135°  (+90° = LEFT)
  "right"  : −135° < sensor_angle < −45° (−90° = RIGHT)

Distances are in metres; -1.0 means no valid hit in that zone.
"""
import socket, json, time

PORT = 5005

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', PORT))
    sock.settimeout(1.0)
    print(f"Listening on :{PORT} for ydlidar_forwarder packets …  (Ctrl-C to stop)\n")
    last_rx = time.time()
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            pkt  = json.loads(data.decode())
            safe = pkt.get('safe',  True)
            f    = pkt.get('front', -1.0)
            l    = pkt.get('left',  -1.0)
            r    = pkt.get('right', -1.0)
            trig = pkt.get('trigger', 'none')

            fs = f"{f:5.2f}" if f >= 0 else "  -- "
            ls = f"{l:5.2f}" if l >= 0 else "  -- "
            rs = f"{r:5.2f}" if r >= 0 else "  -- "

            print(f"\r  {'SAFE  ' if safe else 'UNSAFE'}  "
                  f"FRONT={fs}m  LEFT={ls}m  RIGHT={rs}m  "
                  f"trigger={trig:5s}",
                  end='', flush=True)
            last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > 3.0:
                print("\r  [YDL ] No packets for 3 s — is ydlidar_forwarder.py running?   ",
                      end='', flush=True)
        except KeyboardInterrupt:
            break
    sock.close()
    print()

if __name__ == '__main__':
    main()

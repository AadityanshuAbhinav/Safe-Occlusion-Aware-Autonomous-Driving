#!/usr/bin/env python3
"""
ouster_diag.py
══════════════
Live directional distance readout from the Ouster OS1.
Applies the same OUSTER_INVERTED and Z filter as scenario_v3_hardware.py
so you can verify the sign convention against physical reality.

After OUSTER_INVERTED=True is applied:
  hx = forward  (positive = in front of vehicle)
  hy = left     (positive = to vehicle's left)

Sectors (±45° quadrants):
  FRONT  hx > 0, |hx| > |hy|   (0° ± 45°)
  REAR   hx < 0, |hx| > |hy|   (180° ± 45°)
  LEFT   hy > 0, |hy| > |hx|   (90° ± 45°)
  RIGHT  hy < 0, |hy| > |hx|   (270° ± 45°)
  ALL    overall minimum across all hits

Physical check: walk toward the vehicle front → FRONT distance should decrease.
"""
import math, sys, urllib.request

OUSTER_IP       = '169.254.135.171'
OUSTER_INVERTED = True
OUSTER_Z_MIN    = -1.75
OUSTER_Z_MAX    =  0.25
LIDAR_RANGE     = 25.0

try:
    from ouster.sdk import open_source
    from ouster.sdk.core import XYZLut, ChanField, SensorInfo
    import numpy as np
except ImportError:
    print("ouster-sdk not found — install it or run on the Jetson"); sys.exit(1)


def main():
    url = f"http://{OUSTER_IP}/api/v1/sensor/metadata"
    print(f"Fetching metadata from {url} …", flush=True)
    with urllib.request.urlopen(url, timeout=10) as resp:
        meta = SensorInfo(resp.read().decode())
    xyzlut = XYZLut(meta)
    print("Connected. Walk around the vehicle and verify directions. Ctrl-C to stop.\n")

    source = open_source(OUSTER_IP)
    try:
        for scan_set in source:
            scan = scan_set[0]
            if scan is None:
                continue

            xyz = xyzlut(scan.field(ChanField.RANGE))
            if OUSTER_INVERTED:
                sx, sy = -xyz[..., 0], -xyz[..., 1]
            else:
                sx, sy =  xyz[..., 0],  xyz[..., 1]
            sz = xyz[..., 2]

            r_field    = scan.field(ChanField.RANGE)
            valid_mask = r_field > 0
            horiz      = np.sqrt(sx**2 + sy**2)
            mask       = (valid_mask
                          & (sz    > OUSTER_Z_MIN)
                          & (sz    < OUSTER_Z_MAX)
                          & (horiz > 0.3)
                          & (horiz < LIDAR_RANGE))

            hx = sx[mask].flatten()
            hy = sy[mask].flatten()
            n  = len(hx)

            if n == 0:
                print("\r  [OS1] No valid hits after filtering", end='', flush=True)
                continue

            d = np.sqrt(hx**2 + hy**2)
            ax, ay = np.abs(hx), np.abs(hy)

            front_m = (hx > 0) & (ax > ay)
            rear_m  = (hx < 0) & (ax > ay)
            left_m  = (hy > 0) & (ay > ax)
            right_m = (hy < 0) & (ay > ax)

            f  = float(d[front_m].min()) if front_m.any() else float('inf')
            re = float(d[rear_m ].min()) if rear_m.any()  else float('inf')
            l  = float(d[left_m ].min()) if left_m.any()  else float('inf')
            ri = float(d[right_m].min()) if right_m.any() else float('inf')
            a  = float(d.min())

            def fmt(v): return f"{v:5.2f}" if math.isfinite(v) else "  inf"

            print(f"\r  FRONT={fmt(f)}m  REAR={fmt(re)}m  "
                  f"LEFT={fmt(l)}m  RIGHT={fmt(ri)}m  "
                  f"ALL={fmt(a)}m  pts={n:5d}",
                  end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            source.close()
        except Exception:
            pass
    print()


if __name__ == '__main__':
    main()

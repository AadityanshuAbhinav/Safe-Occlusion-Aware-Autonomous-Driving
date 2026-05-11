"""
mapper.py
─────────────────────────────────────────────────────────────────────────────
2-D LiDAR occupancy-grid mapper for PIX Moving Hooke chassis.

Sensors
  • PIX Hooke CAN bus  — speed + steer angles → dead-reckoning odometry
                         (reuses OdometryIntegrator from odometry_eval.py)
  • Ouster OS1 LiDAR   — 3-D point cloud flattened to 2-D occupancy grid

No IMU — pose is pure dead-reckoning from CAN data.

Outputs (written to LOG_DIR/<run folder> on shutdown):
  pose_log.csv          — timestamp, elapsed, x, y, theta_deg per drive frame
  scan_NNNNNN.npy       — raw 3-D xyz float32 (H×W×3) per LiDAR scan
  occupancy_grid.npy    — final log-odds grid  (float32, N×N)
  map.png               — rendered map image

Live display:
  matplotlib window — occupancy grid viewport following vehicle + trail + arrow

Controls:
  Ctrl-C or close the window → stop & save

Dependencies:
  pip install python-can ouster-sdk numpy matplotlib

Usage:
  python mapper.py
"""

import sys
import os
import math
import time
import csv
import threading
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.figure
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_agg import FigureCanvasAgg
import can

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
)


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION — edit before running
# ══════════════════════════════════════════════════════════════════════════════

CAN_CHANNEL     = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
LIDAR_HOST      = '169.254.135.171'  # Ouster OS1 IP address
                                     # (open_source auto-discovers UDP ports)
LIDAR_REAR_MOUNT = True              # True = sensor faces rear (x,y inverted)

# LiDAR point filter (sensor frame, z = up)
LIDAR_Z_MIN     = -1.0     # m — include points this far below the sensor
LIDAR_Z_MAX     =  2.0     # m — exclude points above this height
LIDAR_R_MIN     =  0.3     # m — discard near-field noise
LIDAR_R_MAX     = 50.0     # m — discard very distant / noisy returns

# LiDAR mounting offset from vehicle odometry reference point
# Positive = sensor is in front of / to the left of the reference point
LIDAR_OFFSET_X  =  0.0     # m forward
LIDAR_OFFSET_Y  =  0.0     # m left
LIDAR_YAW_DEG   =  0.0     # deg CCW — yaw of sensor relative to vehicle forward

# Occupancy grid
MAP_RESOLUTION  =  0.10    # m per cell
MAP_SIZE_M      = 200.0    # total map side length in metres (±100 m from origin)

# Display
DISPLAY_HZ      =  4       # map window refresh rate
VIEW_RADIUS_M   = 30.0     # half-side of viewport (metres around vehicle)

LOG_DIR         = '.'      # parent folder for run logs

# Log-odds update values
L_OCC  =  0.85
L_FREE = -0.20
L_MIN  = -3.0
L_MAX  =  3.5


# ══════════════════════════════════════════════════════════════════════════════
#  Thread-safe odometry integrator with position trail
# ══════════════════════════════════════════════════════════════════════════════

class TrackedIntegrator(OdometryIntegrator):
    """Thin thread-safe wrapper around OdometryIntegrator with a position trail."""

    MAX_TRAIL = 2000

    def __init__(self):
        self._lock  = threading.Lock()
        self._trail = []
        super().__init__()

    def reset(self):
        with self._lock:
            super().reset()
            self._trail.clear()

    def update_steer(self, front_deg, rear_deg, mode=None):
        with self._lock:
            super().update_steer(front_deg, rear_deg, mode)

    def update_speed(self, speed_mps, accel_mps2, ts):
        with self._lock:
            super().update_speed(speed_mps, accel_mps2, ts)
            self._trail.append((self.x, self.y))
            if len(self._trail) > self.MAX_TRAIL:
                self._trail.pop(0)

    def update_rpm(self, lf, rf, lr, rr, ts):
        with self._lock:
            super().update_rpm(lf, rf, lr, rr, ts)

    def get_pose(self):
        """Return (x, y, theta_rad, trail_copy) — thread-safe snapshot."""
        with self._lock:
            return (
                self.x,
                self.y,
                self.theta_rad,
                list(self._trail),
            )

    def get_display_state(self):
        with self._lock:
            speed = self.speed_samples[-1] if self.speed_samples else 0.0
            return (
                self.x,
                self.y,
                self.theta_rad,
                speed,
                self.steer_front_deg,
                self.steer_rear_deg,
                self.steer_mode,
                self.distance_m,
                list(self._trail),
            )


# ══════════════════════════════════════════════════════════════════════════════
#  CAN reader thread
# ══════════════════════════════════════════════════════════════════════════════

def can_reader(odo: TrackedIntegrator, stop_event: threading.Event, pose_rows: list):
    try:
        bus = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
        print(f"[CAN] Listening on {CAN_CHANNEL}")
    except Exception as exc:
        print(f"[CAN] Failed to open {CAN_CHANNEL}: {exc}")
        stop_event.set()
        return

    subscribed = {ID_DRIVE_FB, ID_STEER_FB, ID_WHEEL_RPM_FB}
    t0 = time.time()

    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.arbitration_id not in subscribed:
                continue
            data = bytes(msg.data)
            now  = time.time()

            if msg.arbitration_id == ID_STEER_FB and len(data) >= 6:
                d = decode_steer_fb(data)
                odo.update_steer(
                    d['steer_angle_front'],
                    d['steer_angle_rear'],
                    mode=d['steer_mode'],
                )

            elif msg.arbitration_id == ID_DRIVE_FB and len(data) >= 7:
                d = decode_drive_fb(data)
                odo.update_speed(d['speed_mps'], d['acceleration_mps2'], now)
                x, y, theta_rad, _ = odo.get_pose()
                pose_rows.append((
                    round(now, 4),
                    round(now - t0, 4),
                    round(x, 4),
                    round(y, 4),
                    round(math.degrees(theta_rad) % 360, 4),
                ))

            elif msg.arbitration_id == ID_WHEEL_RPM_FB and len(data) >= 8:
                d = decode_wheel_rpm_fb(data)
                odo.update_rpm(
                    d['rpm_lf'], d['rpm_rf'],
                    d['rpm_lr'], d['rpm_rr'],
                    now,
                )

        except Exception:
            pass

    bus.shutdown()
    print("[CAN] Bus closed.")


# ══════════════════════════════════════════════════════════════════════════════
#  Occupancy grid
# ══════════════════════════════════════════════════════════════════════════════

class OccupancyGrid:
    """
    Log-odds 2-D occupancy grid.

    World frame matches odometry_eval.py convention:
      x = east  (+right on map)
      y = north (+up on map)
    Origin (0,0) at grid centre (cell [half, half]).
    """

    def __init__(self, size_m: float = MAP_SIZE_M, res: float = MAP_RESOLUTION):
        self.res  = float(res)
        N = int(round(size_m / res))
        if N % 2 != 0:
            N += 1
        self.N    = N
        self.half = N // 2
        self.log_odds = np.zeros((N, N), dtype=np.float32)
        self._lock = threading.Lock()

    # ── coordinate helpers ────────────────────────────────────────────────────

    def _w2c(self, wx: float, wy: float):
        """World metres → integer cell (col, row). No bounds check."""
        col = int(round(wx / self.res)) + self.half
        row = int(round(wy / self.res)) + self.half
        return col, row

    # ── update from one LiDAR scan ────────────────────────────────────────────

    def update(self, robot_x: float, robot_y: float, theta_rad: float,
               pts_sensor: np.ndarray):
        """
        pts_sensor: (N,2) float32 — LiDAR hits in sensor frame (x fwd, y left).
        Runs in the LiDAR thread.
        """
        if pts_sensor.shape[0] == 0:
            return

        N = self.N

        # ── Apply LiDAR mounting offset ───────────────────────────────────────
        yaw_off = math.radians(LIDAR_YAW_DEG)
        cos_y   = math.cos(yaw_off)
        sin_y   = math.sin(yaw_off)
        # Rotate points from sensor frame to vehicle frame
        px_v =  cos_y * pts_sensor[:, 0] - sin_y * pts_sensor[:, 1] + LIDAR_OFFSET_X
        py_v =  sin_y * pts_sensor[:, 0] + cos_y * pts_sensor[:, 1] + LIDAR_OFFSET_Y

        # ── Rotate vehicle frame → world frame ────────────────────────────────
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)
        wx = robot_x + cos_t * px_v - sin_t * py_v
        wy = robot_y + sin_t * px_v + cos_t * py_v

        hx = np.round(wx / self.res).astype(np.int32) + self.half
        hy = np.round(wy / self.res).astype(np.int32) + self.half

        in_bounds = (hx >= 0) & (hx < N) & (hy >= 0) & (hy < N)
        hx = hx[in_bounds]
        hy = hy[in_bounds]
        if hx.size == 0:
            return

        rx, ry = self._w2c(robot_x, robot_y)

        with self._lock:
            # Mark occupied hits (vectorised)
            np.add.at(self.log_odds, (hy, hx), L_OCC)

            # Mark free space along each beam using 3 fractional samples
            for frac in (0.25, 0.50, 0.75):
                fx = np.clip(
                    np.round(rx + frac * (hx - rx)).astype(np.int32), 0, N - 1)
                fy = np.clip(
                    np.round(ry + frac * (hy - ry)).astype(np.int32), 0, N - 1)
                np.add.at(self.log_odds, (fy, fx), L_FREE)

            np.clip(self.log_odds, L_MIN, L_MAX, out=self.log_odds)

    # ── view extraction ───────────────────────────────────────────────────────

    def get_view(self, cx_m: float, cy_m: float, radius_m: float):
        """
        Return a fixed-size patch (always 2r × 2r cells) centred on (cx_m, cy_m).
        Out-of-grid regions are filled with 0 (unknown).
        Fixed size avoids matplotlib resampler crashes when shape and extent
        change simultaneously on different frames.
        """
        cx, cy = self._w2c(cx_m, cy_m)
        r    = int(round(radius_m / self.res))
        size = 2 * r

        # Source corners (may reach outside grid)
        src_c0 = cx - r;  src_c1 = cx + r
        src_r0 = cy - r;  src_r1 = cy + r

        # Destination offsets inside the fixed patch
        dst_c0 = max(0, -src_c0)
        dst_c1 = size - max(0, src_c1 - self.N)
        dst_r0 = max(0, -src_r0)
        dst_r1 = size - max(0, src_r1 - self.N)

        # Clamp source to valid grid range
        act_c0 = max(0, src_c0);  act_c1 = min(self.N, src_c1)
        act_r0 = max(0, src_r0);  act_r1 = min(self.N, src_r1)

        patch = np.zeros((size, size), dtype=np.float32)
        if act_c1 > act_c0 and act_r1 > act_r0:
            with self._lock:
                patch[dst_r0:dst_r1, dst_c0:dst_c1] = (
                    self.log_odds[act_r0:act_r1, act_c0:act_c1]
                )

        # World extents — always centred on vehicle, fixed size
        wx0 = cx_m - radius_m;  wx1 = cx_m + radius_m
        wy0 = cy_m - radius_m;  wy1 = cy_m + radius_m
        return patch, wx0, wx1, wy0, wy1

    def full_extent(self):
        half_m = self.N * self.res / 2
        return -half_m, half_m


# ══════════════════════════════════════════════════════════════════════════════
#  LiDAR reader thread  (Ouster OS1 via ouster-sdk)
# ══════════════════════════════════════════════════════════════════════════════

def lidar_reader(odo: TrackedIntegrator, grid: OccupancyGrid,
                 stop_event: threading.Event, log_dir: str):
    # ouster-sdk 0.16.x API:
    #   open_source(hostname)  — connects to live sensor
    #   XYZLut from ouster.sdk.core
    #   iterator yields List[LidarScan] — always index [0]
    try:
        from ouster.sdk import open_source
        from ouster.sdk.core import XYZLut, ChanField
    except ImportError as exc:
        print(f"[LiDAR] WARNING: ouster-sdk not importable ({exc}) — odometry-only.")
        print("[LiDAR] Thread exiting. CAN odometry and display continue.")
        return   # do NOT set stop_event

    print(f"[LiDAR] Connecting to {LIDAR_HOST} …")
    try:
        source   = open_source(LIDAR_HOST)
        metadata = source.sensor_info[0]   # SDK 0.16: sensor_info[0], not .metadata
        xyzlut   = XYZLut(metadata)
        print(f"[LiDAR] Connected — {metadata.prod_line}  "
              f"mode: {metadata.config.lidar_mode}")
    except Exception as exc:
        print(f"[LiDAR] Connection failed: {exc}")
        print("[LiDAR] Thread exiting. CAN odometry and display continue.")
        return   # do NOT set stop_event

    scan_idx = 0
    try:
        for scan_set in source:              # SDK 0.16: yields List[LidarScan]
            if stop_event.is_set():
                break

            scan = scan_set[0]              # single sensor → always index [0]

            # ── Full 3-D point cloud (H × W × 3, sensor frame) ───────────────
            xyz_full = xyzlut(scan)         # raw: x=fwd, y=left, z=up  [metres]

            # Rear-facing mount: invert x and y to get vehicle-forward frame
            if LIDAR_REAR_MOUNT:
                xyz_full = xyz_full.copy()
                xyz_full[..., 0] *= -1   # x: sensor-back → vehicle-front
                xyz_full[..., 1] *= -1   # y: keep right-hand rule

            # Save raw scan for post-processing / offline replay
            np.save(
                os.path.join(log_dir, f'scan_{scan_idx:06d}.npy'),
                xyz_full.astype(np.float32),
            )
            scan_idx += 1

            # ── 2-D projection: height + range filter ─────────────────────────
            pts = xyz_full.reshape(-1, 3)
            horiz_r = np.hypot(pts[:, 0], pts[:, 1])
            mask = (
                (~np.all(pts == 0, axis=1))  &   # discard zero-range pixels
                (pts[:, 2] >= LIDAR_Z_MIN)   &
                (pts[:, 2] <= LIDAR_Z_MAX)   &
                (horiz_r   >= LIDAR_R_MIN)   &
                (horiz_r   <= LIDAR_R_MAX)
            )
            pts_2d = pts[mask, :2].astype(np.float32)   # (N, 2) x fwd, y left

            # ── Fuse into occupancy grid ──────────────────────────────────────
            if pts_2d.shape[0] > 0:
                x, y, theta_rad, _ = odo.get_pose()
                grid.update(x, y, theta_rad, pts_2d)

    except Exception as exc:
        if not stop_event.is_set():
            print(f"[LiDAR] Error: {exc}")
    finally:
        try:
            source.close()
        except Exception:
            pass
        print(f"[LiDAR] Closed. {scan_idx} scans saved to {log_dir}")


# ══════════════════════════════════════════════════════════════════════════════
#  Live map display  (matplotlib FuncAnimation)
# ══════════════════════════════════════════════════════════════════════════════

class MapDisplay:
    BG    = '#090d12'
    TRAIL = '#00cfff'
    DOT   = '#ff6b2b'
    TEXT  = '#c0d8e8'
    DIM   = '#405060'

    def __init__(self, odo: TrackedIntegrator, grid: OccupancyGrid,
                 stop_event: threading.Event):
        self.odo        = odo
        self.grid       = grid
        self.stop_event = stop_event

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.fig.patch.set_facecolor(self.BG)
        self.ax.set_facecolor(self.BG)
        self.ax.set_title('2-D LiDAR Map  [CAN odometry, no IMU]',
                          color=self.TEXT, fontsize=11, fontfamily='monospace')
        self.ax.set_xlabel('X  (m, east →)',  color=self.DIM, fontsize=9)
        self.ax.set_ylabel('Y  (m, north ↑)', color=self.DIM, fontsize=9)
        self.ax.tick_params(colors=self.DIM)
        for spine in self.ax.spines.values():
            spine.set_edgecolor('#1a2a38')

        # Initial placeholder — same shape as every subsequent patch so
        # matplotlib's resampler never sees a shape change mid-animation.
        _r = int(round(VIEW_RADIUS_M / MAP_RESOLUTION))
        dummy = np.zeros((2 * _r, 2 * _r), dtype=np.float32)
        self.im = self.ax.imshow(
            dummy, origin='lower',
            cmap='RdYlGn',
            vmin=L_MIN, vmax=L_MAX,
            interpolation='nearest',
            extent=[-VIEW_RADIUS_M, VIEW_RADIUS_M,
                    -VIEW_RADIUS_M, VIEW_RADIUS_M],
        )
        cbar = self.fig.colorbar(self.im, ax=self.ax, fraction=0.03, pad=0.02)
        cbar.set_label('log-odds  (green=free, red=occupied)',
                       color=self.DIM, fontsize=7)
        cbar.ax.yaxis.set_tick_params(colors=self.DIM)

        self.trail_line, = self.ax.plot(
            [], [], '-', color=self.TRAIL, lw=1.2, alpha=0.7, zorder=3)
        self.robot_dot, = self.ax.plot(
            [], [], 'o', color=self.DOT, ms=9, zorder=5)
        self._arrow = None

        self.info = self.ax.text(
            0.01, 0.99, '', transform=self.ax.transAxes,
            va='top', ha='left', color=self.TEXT,
            fontsize=8, fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#0a0e14', alpha=0.75),
            zorder=6,
        )

        self.fig.canvas.mpl_connect('close_event', self._on_close)

        self._anim = FuncAnimation(
            self.fig, self._animate,
            interval=int(1000 / DISPLAY_HZ),
            blit=False,
            cache_frame_data=False,
        )

    def _on_close(self, _event):
        self.stop_event.set()

    def _animate(self, _frame):
        x, y, theta_rad, speed, sf, sr, smode, dist, trail = \
            self.odo.get_display_state()
        theta_deg = math.degrees(theta_rad) % 360

        patch, wx0, wx1, wy0, wy1 = self.grid.get_view(x, y, VIEW_RADIUS_M)
        self.im.set_data(patch)
        self.im.set_extent([wx0, wx1, wy0, wy1])
        self.ax.set_xlim(wx0, wx1)
        self.ax.set_ylim(wy0, wy1)

        # Trail
        if len(trail) >= 2:
            self.trail_line.set_data(
                [p[0] for p in trail],
                [p[1] for p in trail],
            )
        else:
            self.trail_line.set_data([], [])

        # Vehicle dot
        self.robot_dot.set_data([x], [y])

        # Heading arrow
        if self._arrow is not None:
            self._arrow.remove()
        arrow_len = VIEW_RADIUS_M * 0.06
        self._arrow = self.ax.annotate(
            '',
            xy=(x + arrow_len * math.cos(theta_rad),
                y + arrow_len * math.sin(theta_rad)),
            xytext=(x, y),
            arrowprops=dict(arrowstyle='->', color=self.DOT, lw=2.5),
            zorder=6,
        )

        self.info.set_text(
            f"x={x:+7.2f} m     y={y:+7.2f} m     θ={theta_deg:6.1f}°\n"
            f"spd={speed*3.6:+6.2f} km/h   "
            f"steer F={sf:+5.1f}°  R={sr:+5.1f}°\n"
            f"mode: {STEER_MODE_NAMES.get(smode,'?'):<18s}  "
            f"dist={dist:.1f} m"
        )
        return self.im, self.trail_line, self.robot_dot, self.info

    def show(self):
        plt.tight_layout()
        plt.show()


# ══════════════════════════════════════════════════════════════════════════════
#  Shutdown: save all logs
# ══════════════════════════════════════════════════════════════════════════════

def save_outputs(run_dir: str, grid: OccupancyGrid, pose_rows: list):
    print("\n[MAPPER] Saving outputs …")

    # Occupancy grid (numpy)
    grid_npy = os.path.join(run_dir, 'occupancy_grid.npy')
    with grid._lock:
        np.save(grid_npy, grid.log_odds)
    print(f"  Grid (npy)  → {grid_npy}")

    # Map image — use non-interactive Agg canvas, avoids backend conflicts
    try:
        half_m = grid.N * grid.res / 2
        extent = [-half_m, half_m, -half_m, half_m]
        fig2 = matplotlib.figure.Figure(figsize=(14, 14))
        ax2  = fig2.add_subplot(111)
        with grid._lock:
            data = grid.log_odds.copy()
        ax2.imshow(
            data, origin='lower', cmap='RdYlGn',
            vmin=L_MIN, vmax=L_MAX,
            extent=extent, interpolation='nearest',
        )
        ax2.set_title('2-D Occupancy Map')
        ax2.set_xlabel('X (m, east)')
        ax2.set_ylabel('Y (m, north)')
        fig2.colorbar(ax2.images[0], ax=ax2, fraction=0.03, pad=0.02,
                      label='log-odds')
        png_path = os.path.join(run_dir, 'map.png')
        FigureCanvasAgg(fig2).print_figure(png_path, dpi=150,
                                           bbox_inches='tight')
        print(f"  Map (PNG)   → {png_path}")
    except Exception as exc:
        print(f"  Map PNG failed: {exc}")

    # Pose CSV
    pose_csv = os.path.join(run_dir, 'pose_log.csv')
    with open(pose_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['timestamp_s', 'elapsed_s', 'x_m', 'y_m', 'theta_deg'])
        w.writerows(pose_rows)
    print(f"  Pose (csv)  → {pose_csv}  ({len(pose_rows)} rows)")
    print(f"[MAPPER] All outputs in: {run_dir}")


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ts_str  = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = os.path.join(LOG_DIR, f'mapper_{ts_str}')
    os.makedirs(run_dir, exist_ok=True)

    print("=" * 70)
    print("  2-D LiDAR MAPPER")
    print(f"  CAN channel  : {CAN_CHANNEL}")
    print(f"  LiDAR host   : {LIDAR_HOST}")
    print(f"  Map size     : {MAP_SIZE_M} m  |  resolution: {MAP_RESOLUTION} m/cell")
    print(f"  Log dir      : {run_dir}")
    print("=" * 70)
    print("  Drive freely. Ctrl-C or close the window to stop & save.")
    print("=" * 70)
    input("\n  Press ENTER to start …\n")

    odo        = TrackedIntegrator()
    grid       = OccupancyGrid()
    stop_event = threading.Event()
    pose_rows  = []

    can_thread = threading.Thread(
        target=can_reader,
        args=(odo, stop_event, pose_rows),
        daemon=True,
    )
    lidar_thread = threading.Thread(
        target=lidar_reader,
        args=(odo, grid, stop_event, run_dir),
        daemon=True,
    )

    can_thread.start()
    lidar_thread.start()

    display = MapDisplay(odo, grid, stop_event)
    try:
        display.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        # Give threads a moment to flush their last log entries
        can_thread.join(timeout=2.0)
        lidar_thread.join(timeout=2.0)
        save_outputs(run_dir, grid, pose_rows)


if __name__ == '__main__':
    main()

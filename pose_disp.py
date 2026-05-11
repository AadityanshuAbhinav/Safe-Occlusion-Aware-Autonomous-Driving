"""
pose_display.py
───────────────
Live X / Y / θ display for PIX Moving Hooke chassis.

Imports OdometryIntegrator from odometry_eval.py (must be in same directory).

Coordinate convention (CARLA / body frame):
  X : forward  (+ahead of the vehicle)
  Y : left     (+to the left of the vehicle)
  θ : heading in degrees CCW from world-east (90° = facing north at start)

  Displayed X and Y are the displacement from the start point projected onto
  the vehicle's current body axes — i.e. how far ahead and how far to the
  left the origin (start point) is from the vehicle's current perspective.

Map display:
  Heading-up: the vehicle is always centred and faces screen-up.
  The world grid rotates as the vehicle turns, exactly like Google Maps
  navigation mode.  The compass rose in the top-right corner shows where
  world cardinal directions (N/E/S/W) are relative to the current heading.

Controls:
  R   — reset pose to (0, 0, 90°)
  Esc — quit
"""

import can
import math
import time
import threading
import tkinter as tk
import sys
import os

# ── Import integrator and decoders from odometry_eval ────────────────────────
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

# =============================================================================
#  CONFIGURATION
# =============================================================================

CAN_CHANNEL  = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
DISPLAY_HZ   = 20          # GUI refresh rate
TRAIL_LEN    = 800         # number of past positions drawn on map

# =============================================================================
#  CAN reader thread
# =============================================================================

def can_reader(odo: OdometryIntegrator, stop_event: threading.Event):
    try:
        bus = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
        print(f"[CAN] Listening on {CAN_CHANNEL}")
    except Exception as e:
        print(f"[CAN] Failed to open {CAN_CHANNEL}: {e}")
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
                odo.update_steer(
                    d['steer_angle_front'],
                    d['steer_angle_rear'],
                    mode=d['steer_mode'],
                    ts=now,
                )

            elif msg.arbitration_id == ID_DRIVE_FB and len(data) >= 7:
                d = decode_drive_fb(data)
                odo.update_speed(d['speed_mps'], d['acceleration_mps2'], now)

            elif msg.arbitration_id == ID_WHEEL_RPM_FB and len(data) >= 8:
                d = decode_wheel_rpm_fb(data)
                odo.update_rpm(
                    d['rpm_lf'], d['rpm_rf'],
                    d['rpm_lr'], d['rpm_rr'], now,
                )

        except Exception:
            pass

    bus.shutdown()
    print("[CAN] Bus closed.")


# =============================================================================
#  TrackedIntegrator — adds trail and thread-safe access
# =============================================================================

class TrackedIntegrator(OdometryIntegrator):
    """
    Thin wrapper that adds a position trail and a threading lock
    so pose_display can read state safely from the GUI thread.
    """
    def __init__(self):
        self._lock  = threading.Lock()
        self._trail = []
        super().__init__()

    def reset(self):
        with self._lock:
            super().reset()
            self._trail = []

    def update_steer(self, front_deg, rear_deg, mode=None, ts=None):
        with self._lock:
            super().update_steer(front_deg, rear_deg, mode, ts)

    def update_speed(self, speed_mps, accel_mps2, ts):
        with self._lock:
            super().update_speed(speed_mps, accel_mps2, ts)
            self._trail.append((self.x, self.y))
            if len(self._trail) > TRAIL_LEN:
                self._trail.pop(0)

    def update_rpm(self, lf, rf, lr, rr, ts):
        with self._lock:
            super().update_rpm(lf, rf, lr, rr, ts)

    def get_display_state(self):
        """
        Returns a snapshot safe to read from the GUI thread.

        Body-frame coordinates (x_fwd, y_left):
          x_fwd  = displacement from origin projected onto current forward axis
          y_left = displacement from origin projected onto current left axis
        These match the CARLA / navigation convention: x forward, y left.
        """
        with self._lock:
            spd     = self.speed_samples[-1] if self.speed_samples else 0.0
            theta_r = self.theta_rad
            ct = math.cos(theta_r)
            st = math.sin(theta_r)
            # Project world-frame (east/north) displacement onto body axes
            x_fwd  =  self.x * ct + self.y * st   # dot with forward direction
            y_left = -self.x * st + self.y * ct   # dot with left direction
            return (
                self.x,           # world east  (for map rendering)
                self.y,           # world north (for map rendering)
                x_fwd,            # body X (forward)  — displayed as X
                y_left,           # body Y (left)      — displayed as Y
                self.theta_deg % 360,
                spd,
                self.steer_front_deg,
                self.steer_rear_deg,
                self.steer_mode,
                self.distance_m,
                list(self._trail),
            )


# =============================================================================
#  Display
# =============================================================================

class PoseDisplay:
    BG         = '#090d12'
    ACCENT     = '#00cfff'
    ORANGE     = '#ff6b2b'
    GREEN      = '#39ff14'
    GRID       = '#161f28'
    GRID2      = '#1e2e3c'
    TEXT_DIM   = '#3a5060'
    TEXT_MID   = '#8ab0c0'
    TEXT_MAIN  = '#deeef8'
    TRAIL_HUE  = '#00cfff'
    ORIGIN_COL = '#ffffff'
    NORTH_COL  = '#ff4444'

    def __init__(self, root, odo: TrackedIntegrator, stop_event):
        self.root       = root
        self.odo        = odo
        self.stop_event = stop_event

        root.title("POSE MONITOR")
        root.configure(bg=self.BG)
        root.attributes('-fullscreen', True)
        root.bind('<Escape>', lambda _e: self._quit())
        root.bind('r',        lambda _e: odo.reset())
        root.bind('R',        lambda _e: odo.reset())

        W = root.winfo_screenwidth()
        H = root.winfo_screenheight()
        self.W, self.H = W, H

        SPLIT       = int(W * 0.40)
        self.map_w  = W - SPLIT
        self.map_h  = H

        left = tk.Frame(root, bg=self.BG, width=SPLIT, height=H)
        left.place(x=0, y=0)
        self._build_left(left, SPLIT, H)

        self.canvas = tk.Canvas(
            root, width=self.map_w, height=self.map_h,
            bg=self.BG, highlightthickness=0,
        )
        self.canvas.place(x=SPLIT, y=0)

        self.scale        = 60.0
        self.view_cx      = 0.0
        self.view_cy      = 0.0
        self._heading_rad = math.radians(90.0)  # initial heading north

        self._update()

    # ── left panel ────────────────────────────────────────────────────────────

    def _build_left(self, parent, W, _H):
        pad      = 36
        VAL_FONT = ('Courier New', 60, 'bold')
        LBL_FONT = ('Courier New', 14)
        SUB_FONT = ('Courier New', 10)
        UNT_FONT = ('Courier New', 14)
        ROW_H    = 100
        y0       = 30

        def hsep(y):
            tk.Frame(parent, bg='#1a2a38', height=2, width=W - pad*2
                     ).place(x=pad, y=y)

        tk.Label(parent, text='POSE  MONITOR',
                 font=('Courier New', 14, 'bold'),
                 fg=self.TEXT_DIM, bg=self.BG).place(x=pad, y=8)
        hsep(y0 - 4)

        # (key, axis-label, sub-label, unit, color, y)
        coords = [
            ('X', 'X', 'fwd',  'm',   self.ACCENT,  y0 + 8),
            ('Y', 'Y', 'left', 'm',   self.ACCENT,  y0 + 8 + ROW_H),
            ('θ', 'θ', '',     'deg', self.ORANGE,  y0 + 8 + ROW_H * 2),
        ]
        self.val_lbl = {}
        for key, name, sub, unit, color, y in coords:
            tk.Label(parent, text=name,
                     font=LBL_FONT, fg=self.TEXT_DIM, bg=self.BG,
                     anchor='w').place(x=pad, y=y)
            if sub:
                tk.Label(parent, text=sub,
                         font=SUB_FONT, fg=self.TEXT_DIM, bg=self.BG,
                         anchor='w').place(x=pad + 24, y=y + 4)
            v = tk.Label(parent, text='+0.000',
                         font=VAL_FONT, fg=color, bg=self.BG, anchor='w')
            v.place(x=pad, y=y + 24)
            tk.Label(parent, text=unit,
                     font=UNT_FONT, fg=self.TEXT_DIM, bg=self.BG,
                     ).place(x=W - pad - 60, y=y + 90)
            self.val_lbl[key] = v

        y1 = y0 + 8 + ROW_H * 3 + 10
        hsep(y1)

        SEC_LBL = ('Courier New', 15, 'bold')
        SEC_VAL = ('Courier New', 40, 'bold')
        sec_rows = [
            ('SPD',         'km/h', self.GREEN,    y1 + 16),
            ('STEER F / R', '°',    self.TEXT_MAIN, y1 + 110),
            ('MODE',        '',     self.TEXT_DIM,  y1 + 204),
            ('DIST (0,0)',  'm',    self.TEXT_MAIN, y1 + 280),
        ]
        self.sec_lbl = {}
        for name, unit, color, y in sec_rows:
            label_text = f'{name}  [{unit}]' if unit else name
            tk.Label(parent, text=label_text,
                     font=SEC_LBL, fg=self.TEXT_DIM, bg=self.BG,
                     anchor='w').place(x=pad, y=y)
            v = tk.Label(parent, text='—',
                         font=SEC_VAL, fg=color, bg=self.BG, anchor='w')
            v.place(x=pad, y=y + 22)
            self.sec_lbl[name] = v

    # ── coordinate transform ──────────────────────────────────────────────────

    def _w2s(self, wx, wy):
        """
        World (east/north) → screen pixels.

        Heading-up transform: the vehicle's forward direction always maps to
        screen-up.  The world rotates around the centred vehicle icon.

        For world displacement (dx, dy) relative to vehicle:
          screen-right = dx·sin(θ) − dy·cos(θ)   (car right = screen right)
          screen-up    = dx·cos(θ) + dy·sin(θ)   (car fwd   = screen up)
        """
        dx = wx - self.view_cx
        dy = wy - self.view_cy
        ct = math.cos(self._heading_rad)
        st = math.sin(self._heading_rad)
        x_disp =  dx * st - dy * ct   # screen-right component
        y_disp =  dx * ct + dy * st   # screen-up   component
        sx = self.map_w / 2 + x_disp * self.scale
        sy = self.map_h / 2 - y_disp * self.scale
        return sx, sy

    # ── blend helper ──────────────────────────────────────────────────────────

    @staticmethod
    def _blend(hex_col, t):
        r = int(hex_col[1:3], 16)
        g = int(hex_col[3:5], 16)
        b = int(hex_col[5:7], 16)
        return f'#{int(r*t):02x}{int(g*t):02x}{int(b*t):02x}'

    # ── map ───────────────────────────────────────────────────────────────────

    def _draw_map(self, x, y, theta_deg, trail):
        """
        Heading-up map:
          • Vehicle always centred, always faces screen-up.
          • World grid lines (1 m spacing) rotate as heading changes.
          • Compass rose shows cardinal directions relative to current heading.
        """
        self._heading_rad = math.radians(theta_deg)

        # Vehicle is always the viewport centre
        self.view_cx = x
        self.view_cy = y

        # Auto-zoom to fit trail
        if len(trail) > 1:
            xs   = [p[0] for p in trail]
            ys   = [p[1] for p in trail]
            span = max(max(xs) - min(xs), max(ys) - min(ys), 3.0)
            target     = min(self.map_w, self.map_h) * 0.68 / span
            self.scale += (target - self.scale) * 0.04
        else:
            self.scale = 60.0

        c = self.canvas
        c.delete('all')

        # ── World-aligned grid ──────────────────────────────────────────────
        # Each grid line is a world-axis-aligned line rendered through _w2s,
        # so it rotates on screen as the heading changes.
        gs = 1.0
        # Radius in world-units that covers the full screen diagonal
        R  = math.sqrt(self.map_w ** 2 + self.map_h ** 2) / (2.0 * self.scale) + gs * 2

        # Lines of constant world-X (run along world-Y direction)
        gx = math.floor((x - R) / gs) * gs
        while gx <= x + R:
            major = abs(round(gx) - gx) < 0.01 and int(round(gx)) % 5 == 0
            sx1, sy1 = self._w2s(gx, y - R)
            sx2, sy2 = self._w2s(gx, y + R)
            c.create_line(sx1, sy1, sx2, sy2,
                          fill=self.GRID2 if major else self.GRID,
                          width=2 if major else 1)
            if major and abs(round(gx)) > 0:
                lx, ly = self._w2s(gx, y - R * 0.88)
                c.create_text(lx, ly, text=f'{int(round(gx))}',
                              fill=self.TEXT_DIM, font=('Courier New', 9))
            gx += gs

        # Lines of constant world-Y (run along world-X direction)
        gy = math.floor((y - R) / gs) * gs
        while gy <= y + R:
            major = abs(round(gy) - gy) < 0.01 and int(round(gy)) % 5 == 0
            sx1, sy1 = self._w2s(x - R, gy)
            sx2, sy2 = self._w2s(x + R, gy)
            c.create_line(sx1, sy1, sx2, sy2,
                          fill=self.GRID2 if major else self.GRID,
                          width=2 if major else 1)
            if major and abs(round(gy)) > 0:
                lx, ly = self._w2s(x - R * 0.88, gy)
                c.create_text(lx, ly, text=f'{int(round(gy))}',
                              fill=self.TEXT_DIM, font=('Courier New', 9))
            gy += gs

        # ── World origin axes and marker ────────────────────────────────────
        ox, oy = self._w2s(0, 0)
        if -100 <= ox <= self.map_w + 100 and -100 <= oy <= self.map_h + 100:
            ax1, ay1 = self._w2s(-R, 0)
            ax2, ay2 = self._w2s( R, 0)
            c.create_line(ax1, ay1, ax2, ay2, fill='#1e3545', width=2)
            bx1, by1 = self._w2s(0, -R)
            bx2, by2 = self._w2s(0,  R)
            c.create_line(bx1, by1, bx2, by2, fill='#1e3545', width=2)
            r = 7
            c.create_oval(ox-r, oy-r, ox+r, oy+r,
                          outline=self.ORIGIN_COL, width=2)
            c.create_text(ox + 12, oy - 12, text='(0,0)',
                          fill=self.ORIGIN_COL, font=('Courier New', 11, 'bold'))

        # ── Trail ───────────────────────────────────────────────────────────
        n = len(trail)
        if n >= 2:
            for i in range(1, n):
                t_frac   = i / n
                col      = self._blend(self.TRAIL_HUE, 0.15 + 0.85 * t_frac)
                w        = 1 + int(t_frac * 2)
                sx0, sy0 = self._w2s(trail[i-1][0], trail[i-1][1])
                sx1, sy1 = self._w2s(trail[i][0],   trail[i][1])
                c.create_line(sx0, sy0, sx1, sy1, fill=col, width=w)

        # ── Vehicle icon (always centred, always faces screen-up) ───────────
        vx = self.map_w / 2
        vy = self.map_h / 2
        VR = 16
        c.create_oval(vx-VR, vy-VR, vx+VR, vy+VR,
                      fill=self.ACCENT, outline=self.BG, width=3)
        AL = 38
        # Arrow always points straight up — car forward = screen up
        c.create_line(vx, vy, vx, vy - AL,
                      fill=self.ORANGE, width=5,
                      arrow=tk.LAST, arrowshape=(14, 18, 6))

        # ── Compass rose (NESW rotate with heading) ─────────────────────────
        # Cardinal at world angle φ appears at screen offset:
        #   x_off = sin(θ − φ),  y_off = cos(θ − φ)
        cx0, cy0, cr = self.map_w - 60, 60, 38
        c.create_oval(cx0-cr, cy0-cr, cx0+cr, cy0+cr,
                      outline=self.GRID2, width=1)
        for lbl, ang in [('N', 90), ('E', 0), ('S', 270), ('W', 180)]:
            phi   = math.radians(ang)
            x_off = math.sin(self._heading_rad - phi)
            y_off = math.cos(self._heading_rad - phi)
            col   = self.NORTH_COL if lbl == 'N' else self.TEXT_MID
            c.create_text(cx0 + (cr + 14) * x_off,
                          cy0 - (cr + 14) * y_off,
                          text=lbl, fill=col,
                          font=('Courier New', 11, 'bold'))
        # Red dot on compass circle marks world-north
        phi_n = math.pi / 2
        n_x = cx0 + cr * math.sin(self._heading_rad - phi_n)
        n_y = cy0 - cr * math.cos(self._heading_rad - phi_n)
        c.create_oval(n_x-4, n_y-4, n_x+4, n_y+4,
                      fill=self.NORTH_COL, outline='')

        # ── "FWD ▲" label ───────────────────────────────────────────────────
        c.create_text(self.map_w / 2, 14, text='▲  FWD',
                      fill=self.TEXT_DIM, font=('Courier New', 9, 'bold'))

        # ── Scale bar ───────────────────────────────────────────────────────
        bar_m  = 2
        bar_px = bar_m * self.scale
        bx, by = 20, self.map_h - 30
        c.create_line(bx, by, bx + bar_px, by, fill=self.TEXT_MID, width=3)
        c.create_line(bx, by-5, bx, by+5, fill=self.TEXT_MID, width=2)
        c.create_line(bx+bar_px, by-5, bx+bar_px, by+5, fill=self.TEXT_MID, width=2)
        c.create_text(bx + bar_px/2, by - 12, text=f'{bar_m} m',
                      fill=self.TEXT_MID, font=('Courier New', 10))

    # ── refresh ───────────────────────────────────────────────────────────────

    def _update(self):
        x, y, x_fwd, y_left, theta, speed, sf, sr, smode, _dist, trail = \
            self.odo.get_display_state()

        # Body-frame coordinates displayed (x forward, y left)
        self.val_lbl['X'].config(text=f'{x_fwd:+.3f}')
        self.val_lbl['Y'].config(text=f'{y_left:+.3f}')
        self.val_lbl['θ'].config(text=f'{theta:.1f}')

        self.sec_lbl['SPD'].config(text=f'{speed*3.6:+.2f}')
        self.sec_lbl['STEER F / R'].config(text=f'{sf:+.1f}  /  {sr:+.1f}')
        self.sec_lbl['MODE'].config(
            text=STEER_MODE_NAMES.get(smode, f'mode {smode}'))
        # Euclidean distance from start uses world-frame x, y
        self.sec_lbl['DIST (0,0)'].config(
            text=f'{math.sqrt(x**2 + y**2):.3f}')

        self._draw_map(x, y, theta, trail)
        self.root.after(int(1000 / DISPLAY_HZ), self._update)

    def _quit(self):
        self.stop_event.set()
        self.root.destroy()


# =============================================================================
#  Entry point
# =============================================================================

def main():
    odo        = TrackedIntegrator()
    stop_event = threading.Event()

    reader = threading.Thread(
        target=can_reader, args=(odo, stop_event), daemon=True
    )
    reader.start()

    root = tk.Tk()
    PoseDisplay(root, odo, stop_event)

    print("=" * 60)
    print("  POSE MONITOR  —  heading-up, body-frame coords")
    print("  Origin: (0,0)  |  Initial heading: 90° (north)")
    print("  X = forward (body), Y = left (body)")
    print("  Map rotates — vehicle always faces screen-up")
    print("  R   — reset pose")
    print("  Esc — quit")
    print("=" * 60)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        print("Exiting.")


if __name__ == '__main__':
    main()

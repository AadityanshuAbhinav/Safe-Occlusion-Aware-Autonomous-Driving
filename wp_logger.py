"""
waypoint_logger.py
══════════════════
Drive manually in Town03 using keyboard, record waypoints.

Requires: pygame  (pip install pygame)

Controls (in the PYGAME window — must be focused):
  W / ↑     — throttle
  S / ↓     — brake / reverse
  A / ←     — steer left
  D / →     — steer right
  SPACE     — hand brake
  R         — reset to start position
  P         — print all waypoints so far and save to file
  Q / Esc   — quit and save

The script logs ego position every RECORD_INTERVAL metres of travel.
On exit it saves the waypoint list as a Python file you can import directly.

Usage:
  python3 waypoint_logger.py [--start-x 50] [--start-y 200] [--start-yaw 180]
"""

import carla
import math
import time
import argparse
import sys
from datetime import datetime

try:
    import pygame
    from pygame.locals import (
        K_w, K_s, K_a, K_d, K_UP, K_DOWN, K_LEFT, K_RIGHT,
        K_SPACE, K_r, K_p, K_q, K_ESCAPE, QUIT, KEYDOWN,
    )
except ImportError:
    print("ERROR: pygame is required.  pip install pygame")
    sys.exit(1)


RECORD_INTERVAL = 2.0   # metres between recorded waypoints
OUTPUT_DIR = '.'
WINDOW_W, WINDOW_H = 800, 600


def get_keyboard_control():
    """Read current key state and return a carla.VehicleControl."""
    ctrl = carla.VehicleControl()
    keys = pygame.key.get_pressed()

    if keys[K_w] or keys[K_UP]:
        ctrl.throttle = 0.7
    if keys[K_s] or keys[K_DOWN]:
        ctrl.brake = 0.8
        ctrl.throttle = 0.0

    steer = 0.0
    if keys[K_a] or keys[K_LEFT]:
        steer = -0.5
    if keys[K_d] or keys[K_RIGHT]:
        steer = 0.5
    ctrl.steer = steer
    ctrl.hand_brake = keys[K_SPACE]

    return ctrl


def main():
    parser = argparse.ArgumentParser(description='Manual waypoint logger')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--start-x', type=float, default=50.0)
    parser.add_argument('--start-y', type=float, default=200.0)
    parser.add_argument('--start-yaw', type=float, default=180.0)
    parser.add_argument('--vehicle', default='vehicle.pixloop.hooke')
    parser.add_argument('--interval', type=float, default=RECORD_INTERVAL)
    args = parser.parse_args()

    # ── Pygame init ──
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption('Waypoint Logger — WASD to drive, Q to quit')
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('courier', 18)

    print("=" * 60)
    print("  WAYPOINT LOGGER")
    print(f"  Start: ({args.start_x}, {args.start_y}) yaw={args.start_yaw}")
    print(f"  Record interval: {args.interval} m")
    print("  >>> Focus the PYGAME window to drive <<<")
    print("=" * 60)

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world('Town03')

    settings = world.get_settings()
    settings.synchronous_mode = False
    world.apply_settings(settings)

    bpl = world.get_blueprint_library()
    carla_map = world.get_map()

    # Snap to get z height, spawn at exact requested position
    start_loc = carla.Location(x=args.start_x, y=args.start_y, z=0.3)
    start_wp = carla_map.get_waypoint(start_loc, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
    snapped = start_wp.transform.location
    print(f"[Snap] ({args.start_x}, {args.start_y}) -> "
          f"({snapped.x:.1f}, {snapped.y:.1f}) "
          f"road={start_wp.road_id} lane={start_wp.lane_id}")

    spawn_t = carla.Transform(
        carla.Location(x=args.start_x, y=args.start_y, z=snapped.z + 0.5),
        carla.Rotation(yaw=args.start_yaw),
    )

    candidates = bpl.filter(args.vehicle)
    if not candidates:
        candidates = bpl.filter('vehicle.audi.tt')
    if not candidates:
        candidates = bpl.filter('vehicle.*')
    bp = candidates[0]
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', 'hero')
    print(f"[Vehicle] {bp.id}")

    vehicle = world.spawn_actor(bp, spawn_t)
    print(f"[Spawned] ({args.start_x:.1f}, {args.start_y:.1f}) "
          f"yaw={args.start_yaw:.1f}")

    spectator = world.get_spectator()

    # ── Recording state ──
    waypoints = []
    last_record_pos = None
    interval = args.interval
    start_time = time.time()

    def record_point(loc, rot, label=""):
        wp = {
            'x': round(loc.x, 2),
            'y': round(loc.y, 2),
            'z': round(loc.z, 2),
            'yaw': round(rot.yaw, 1),
            't': round(time.time() - start_time, 2),
        }
        waypoints.append(wp)
        tag = f"  ({label})" if label else ""
        print(f"  [{len(waypoints):3d}] x={wp['x']:+8.2f}  y={wp['y']:+8.2f}  "
              f"z={wp['z']:+5.2f}  yaw={wp['yaw']:+7.1f}  "
              f"t={wp['t']:6.1f}s{tag}")

    def save_waypoints():
        if not waypoints:
            print("[Save] No waypoints recorded.")
            return None
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f'{OUTPUT_DIR}/waypoints_{ts}.py'
        with open(fname, 'w') as f:
            f.write('"""\n')
            f.write(f'Recorded waypoints -- Town03\n')
            f.write(f'Start: ({args.start_x}, {args.start_y}) '
                    f'yaw={args.start_yaw}\n')
            f.write(f'Points: {len(waypoints)}\n')
            f.write(f'Interval: ~{interval} m\n')
            f.write('"""\n\n')
            f.write('WAYPOINTS = [\n')
            for wp in waypoints:
                f.write(f'    {{"x": {wp["x"]:+9.2f}, '
                        f'"y": {wp["y"]:+9.2f}, '
                        f'"z": {wp["z"]:+5.2f}, '
                        f'"yaw": {wp["yaw"]:+7.1f}}},\n')
            f.write(']\n')
        print(f"\n[Save] {len(waypoints)} waypoints -> {fname}")
        return fname

    def reset_vehicle():
        vehicle.set_transform(spawn_t)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
        vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        waypoints.clear()
        nonlocal last_record_pos
        last_record_pos = None
        time.sleep(0.2)
        t = vehicle.get_transform()
        record_point(t.location, t.rotation, "START")
        last_record_pos = (t.location.x, t.location.y)
        print("[Reset] Vehicle and waypoints reset.")

    # Record start
    time.sleep(0.3)  # let physics settle
    t = vehicle.get_transform()
    record_point(t.location, t.rotation, "START")
    last_record_pos = (t.location.x, t.location.y)

    print("\n  >>> Focus the PYGAME window and drive with WASD <<<\n")

    running = True
    try:
        while running:
            clock.tick(30)

            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                if event.type == KEYDOWN:
                    if event.key in (K_q, K_ESCAPE):
                        running = False
                    elif event.key == K_r:
                        reset_vehicle()
                    elif event.key == K_p:
                        save_waypoints()

            if not running:
                break
            if not vehicle.is_alive:
                print("[!] Vehicle destroyed.")
                break

            # ── Apply keyboard control ──
            ctrl = get_keyboard_control()
            vehicle.apply_control(ctrl)

            # ── Read state ──
            t = vehicle.get_transform()
            v = vehicle.get_velocity()
            speed = math.hypot(v.x, v.y)
            loc = t.location

            # ── Spectator chase cam ──
            yaw_r = math.radians(t.rotation.yaw)
            spectator.set_transform(carla.Transform(
                carla.Location(
                    x=loc.x - 6 * math.cos(yaw_r),
                    y=loc.y - 6 * math.sin(yaw_r),
                    z=loc.z + 8.0,
                ),
                carla.Rotation(pitch=-40, yaw=t.rotation.yaw),
            ))

            # ── Record if moved enough ──
            if last_record_pos is not None:
                dist = math.hypot(loc.x - last_record_pos[0],
                                  loc.y - last_record_pos[1])
                if dist >= interval:
                    record_point(loc, t.rotation)
                    last_record_pos = (loc.x, loc.y)

            # ── Road info ──
            wp_snap = carla_map.get_waypoint(
                loc, project_to_road=True,
                lane_type=carla.LaneType.Driving)

            # ── HUD ──
            screen.fill((30, 30, 30))

            lines = [
                "WAYPOINT LOGGER  --  Town03",
                "",
                f"pos:   ({loc.x:+7.1f}, {loc.y:+7.1f}, {loc.z:+5.1f})",
                f"yaw:   {t.rotation.yaw:+7.1f}",
                f"speed: {speed:5.1f} m/s  ({speed*3.6:5.1f} km/h)",
                f"road:  {wp_snap.road_id}  lane: {wp_snap.lane_id}",
                "",
                f"waypoints recorded: {len(waypoints)}",
                (f"last: ({waypoints[-1]['x']:+.1f}, {waypoints[-1]['y']:+.1f})"
                 if waypoints else "last: --"),
                "",
                f"throttle: {ctrl.throttle:.1f}  brake: {ctrl.brake:.1f}  "
                f"steer: {ctrl.steer:+.1f}",
                "",
                "[WASD] drive   [SPACE] handbrake",
                "[R] reset   [P] save   [Q/Esc] quit+save",
            ]

            for i, line in enumerate(lines):
                color = (0, 200, 255) if i == 0 else (200, 200, 200)
                surf = font.render(line, True, color)
                screen.blit(surf, (20, 20 + i * 24))

            # Mini trail map
            if len(waypoints) > 1:
                xs = [w['x'] for w in waypoints]
                ys = [w['y'] for w in waypoints]
                cx = (min(xs) + max(xs)) / 2
                cy = (min(ys) + max(ys)) / 2
                span = max(max(xs) - min(xs), max(ys) - min(ys), 20)
                scale = min(WINDOW_W, WINDOW_H) * 0.3 / span
                ox = WINDOW_W - 200
                oy = WINDOW_H - 150

                for j in range(1, len(waypoints)):
                    x0 = ox + (waypoints[j-1]['x'] - cx) * scale
                    y0 = oy - (waypoints[j-1]['y'] - cy) * scale
                    x1 = ox + (waypoints[j]['x'] - cx) * scale
                    y1 = oy - (waypoints[j]['y'] - cy) * scale
                    pygame.draw.line(screen, (0, 200, 100),
                                     (int(x0), int(y0)),
                                     (int(x1), int(y1)), 2)

                px = ox + (loc.x - cx) * scale
                py = oy - (loc.y - cy) * scale
                pygame.draw.circle(screen, (255, 100, 50),
                                   (int(px), int(py)), 5)

            pygame.display.flip()

    except KeyboardInterrupt:
        print("\n[Ctrl+C]")

    finally:
        save_waypoints()
        try:
            if vehicle and vehicle.is_alive:
                vehicle.destroy()
        except Exception:
            pass
        pygame.quit()
        print("[Done]")


if __name__ == '__main__':
    main()
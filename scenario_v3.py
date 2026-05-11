"""
blind_intersection_town03.py
════════════════════════════
Blind-intersection scenario — Town03, junction near (147, -74).
Manually recorded waypoints, RHD.

Ego:     southbound x≈154, turns right (west) at junction
Pursuer: eastbound y≈-74, overtakes parked truck in left lane,
         turns right (south) at junction — into ego's post-turn lane
Occluder: static truck at (136.5, -72) facing east (yaw=0)

Target: CARLA 0.9.16  |  Ego: vehicle.pixloop.hooke
"""

import carla, math, numpy as np, argparse, copy
from dataclasses import dataclass
from typing import List, Optional, Tuple
from shapely.geometry import Polygon, Point
from shapely.affinity import scale as poly_scale
from shapely.validation import make_valid

# ═══════════════════════════════════════════════════════════════════════════════
#  Recorded waypoints (x, y)
# ═══════════════════════════════════════════════════════════════════════════════

# Ego: southbound on N-S road, right turn west at junction


    # (154.49, -17.01), (153.76, -19.26), (153.32, -21.40), (153.40, -23.50),
    # (153.56, -25.61), (153.70, -27.77), (153.82, -29.95), (153.81, -32.12),
    # (153.63, -34.16), (153.43, -36.32), (153.39, -38.45), (153.24, -40.55),
    # (152.96, -42.63), (152.90, -44.70), (152.89, -46.90), (152.92, -49.00),
    # (152.96, -51.11), (153.01, -53.22), (153.05, -55.33), (153.09, -57.46),
    # (153.08, -59.47), (153.07, -61.60), (153.05, -63.79), (152.62, -65.88),
EGO_WAYPOINTS = [
    (151.46, -67.74), (150.20, -69.43), (148.75, -70.94), (146.91, -71.95),
    (145.06, -72.79), (143.14, -73.62), (141.19, -74.31), (139.08, -74.88),
    (137.05, -75.18), (134.96, -75.21), (132.73, -75.10), (130.44, -75.07),
    (128.15, -75.16), (125.89, -75.31), (123.66, -75.46), (121.57, -75.60),
    (119.40, -75.73), (117.22, -75.83), (114.99, -75.92), (112.74, -76.01),
    (110.74, -76.09), (108.66, -76.16), (106.54, -76.23), (104.30, -76.31),
    (102.30, -76.38), (100.25, -76.45), ( 98.13, -76.52), ( 95.96, -76.59),
    ( 93.74, -76.66), ( 91.66, -76.73), ( 89.34, -76.81), ( 86.98, -76.89),
    ( 84.75, -76.96), ( 82.52, -77.03), ( 80.45, -77.10),
]

# Pursuer: eastbound, overtakes truck in left lane, turns right south at junction
PURSUER_WAYPOINTS = [
    (106.50, -74.00), (108.54, -74.00), (110.61, -74.01), (112.77, -74.24),
    (114.82, -74.50), (116.90, -74.97), (118.93, -75.38), (121.06, -75.44),
    (123.08, -75.40), (125.17, -75.37), (127.19, -75.37), (129.38, -75.37),
    (131.56, -75.36), (133.65, -75.36), (135.66, -75.30), (137.79, -75.18),
    (140.03, -75.02), (142.33, -75.01), (144.30, -75.43), (146.18, -76.59),
    (147.80, -78.19), (149.33, -79.82), (150.46, -81.54), (151.49, -83.45),
    (152.41, -85.45), (153.08, -87.48), (153.51, -89.51), (153.85, -91.62),
    (154.10, -93.78), (154.24, -95.92), (154.27, -97.97), (154.26, -100.11),
    (154.22, -102.25), (154.19, -104.49), (154.25, -106.81), (154.35, -108.87),
    (154.47, -110.98), (154.59, -113.14), (154.71, -115.36), (154.83, -117.43),
    (154.95, -119.74), (155.08, -122.09), (155.22, -124.49), (155.33, -126.51),
    (155.44, -128.57), (155.57, -130.94), (155.70, -133.21), (155.81, -135.31),
]


# Constants
JUNCTION_X, JUNCTION_Y = 147.0, -74.0
OCCLUDER_X, OCCLUDER_Y, OCCLUDER_YAW = 138.5, -72.0, 0.0
EGO_SPAWN_YAW = -90.0
PURSUER_SPAWN_YAW = 0.0
END_X = 100.0  # ego finishes when x < this (heading west)
ROAD_SOUTH_Y = -77.5      # approximate south-curb y (observed: -77.3; waypoints min: -77.10)
DODGE_SOUTH_LIMIT = 0.35  # m — max southward displacement before bounded-swerve return phase
NR_CLEAR_GAP  = 5.0       # m — OBB gap threshold to end NR-reverse (hysteresis)
REV_WPS_BACK  = 4         # max waypoints to reverse through in NR-reverse maneuver

# ═══════════════════════════════════════════════════════════════════════════════
#  Geometry
# ═══════════════════════════════════════════════════════════════════════════════

def safe_geom(g):
    if g is None or g.is_empty: return Polygon()
    if not g.is_valid:
        try: g = make_valid(g)
        except: g = g.buffer(0)
    if g.geom_type == 'GeometryCollection':
        ps = [x for x in g.geoms if x.geom_type in ('Polygon','MultiPolygon')]
        g = max(ps, key=lambda x: x.area) if ps else Polygon()
    return g if not g.is_empty else Polygon()

@dataclass
class EgoState:
    x:float=0; y:float=0; yaw:float=0; speed:float=0; ax:float=0

@dataclass
class PlannerParams:
    dt:float=0.05; max_decel:float=-8.0; max_accel:float=2.5
    max_speed:float=5.56; pursuer_max_speed:float=4.5
    sensor_noise_m:float=0.4; safe_stop_margin:float=2.0
    horizon_steps:int=60; lidar_range:float=30.0
    wheelbase:float=1.900; track_width:float=1.465

def braking_distance(v, a): return v**2/(2*abs(a))

def _aligned_wp(e, wps, center_idx, radius=2):
    """Return the waypoint index best aligned with ego heading near center_idx.
    Searches [center_idx-radius, center_idx+radius] and returns the index one
    step ahead of the best match (so pure-pursuit has a target to chase)."""
    lo = max(0, center_idx - radius)
    hi = min(len(wps) - 1, center_idx + radius + 2)
    def _score(i):
        dx, dy = wps[i][0] - e.x, wps[i][1] - e.y
        return abs((math.atan2(dy, dx) - e.yaw + math.pi) % (2*math.pi) - math.pi)
    return min(min(range(lo, hi), key=_score) + 1, len(wps) - 1)

# Vehicle half-dims (length, width) in metres
EGO_HL, EGO_HW   = 2.30, 1.00   # Hooke ~4.6 × 2.0 m
PUR_HL, PUR_HW   = 2.35, 0.95   # Tesla Model3 ~4.7 × 1.9 m

def _box_proj(hl, hw, yaw, ax, ay):
    """Project OBB half-extent onto unit axis (ax,ay)."""
    lx, ly = math.cos(yaw), math.sin(yaw)
    return hl*abs(lx*ax + ly*ay) + hw*abs(-ly*ax + lx*ay)

def obb_gap(ca, yaw_a, hl_a, hw_a, cb, yaw_b, hl_b, hw_b):
    """SAT minimum separating distance between two 2-D OBBs.
    Returns positive value (gap) when separated, ≤ 0 when overlapping."""
    dx, dy = cb[0]-ca[0], cb[1]-ca[1]
    max_gap = float('-inf')
    for yaw in (yaw_a, yaw_b):
        lx,ly = math.cos(yaw), math.sin(yaw)
        for ax,ay in ((lx,ly), (-ly,lx)):           # longitudinal then lateral axis
            proj_d   = abs(dx*ax + dy*ay)
            proj_sum = _box_proj(hl_a,hw_a,yaw_a,ax,ay)+_box_proj(hl_b,hw_b,yaw_b,ax,ay)
            max_gap  = max(max_gap, proj_d - proj_sum)
    return max_gap

def compute_danger_zone(e, p):
    d = braking_distance(e.speed, p.max_decel) + p.safe_stop_margin
    c, s, hw = math.cos(e.yaw), math.sin(e.yaw), 3.0
    def f(dd,ll): return (e.x+dd*c-ll*s, e.y+dd*s+ll*c)
    return Polygon([f(0,-hw),f(d,-hw),f(d,hw),f(0,hw)])

# ═══════════════════════════════════════════════════════════════════════════════
#  Hidden Set, Theorem 1, Evasive Policy
# ═══════════════════════════════════════════════════════════════════════════════

class HiddenSetTracker:
    """
    Paper Def 2: H̃(t) = forward reach-avoid set of pursuer from H⁰,
    avoiding ego's FOV and obstacle constraints, clipped to road.

    Implementation: each tick we
      1. incorporate_shadow: union newly occluded road area into H̃
         (captures new occlusions as ego moves and new occluders enter range)
      2. propagate: grow H̃ by pursuer_max_speed × dt (reachability)
      3. prune_observed: subtract FOV (clear polygon) from H̃
      4. prune_by_velocity_bound: shrink by Neel & Saripalli [19] bound
    """
    def __init__(s, rp, p): s.rp,s.p,s._h,s.elapsed = rp,p,None,0.0

    def incorporate_shadow(s, shadow_on_road):
        """Union newly occluded driveable area into H̃. Paper Eq. 4 + 9."""
        sh = safe_geom(shadow_on_road)
        if sh.is_empty: return
        inflated = safe_geom(sh.buffer(s.p.sensor_noise_m))
        new_region = safe_geom(inflated.intersection(safe_geom(s.rp)))
        if new_region.is_empty: return
        if s._h is None or s._h.is_empty:
            s._h = new_region
            s.elapsed = 0.0
        else:
            s._h = safe_geom(s._h.union(new_region))

    def propagate(s, dt):
        if not s._h or s._h.is_empty: return
        s._h = safe_geom(safe_geom(s._h).buffer(s.p.pursuer_max_speed*dt).intersection(safe_geom(s.rp)))
        s.elapsed += dt

    def prune_observed(s, cl):
        if not s._h or s._h.is_empty: return
        s._h = safe_geom(safe_geom(s._h).difference(safe_geom(cl)))

    def prune_by_velocity_bound(s, L):
        if s.elapsed<0.5 or not s._h or s._h.is_empty: return
        sh = min(1.0, (L/s.elapsed)/s.p.pursuer_max_speed)
        if sh<1: s._h = safe_geom(poly_scale(s._h,sh,sh,origin=s._h.centroid).intersection(safe_geom(s.rp)))

    @property
    def polygon(s): return s._h
    def is_empty(s): return not s._h or s._h.is_empty

def theorem1_check(h, d, min_overlap_area=2.0):
    """Safe iff H̃ ∩ DangerZone has negligible area.
    min_overlap_area: ignore slivers smaller than this (m²) to avoid
    false EVADING from residual shadow noise near the junction edge."""
    if not h or h.is_empty: return True
    h,d = safe_geom(h),safe_geom(d)
    if h.is_empty or d.is_empty: return True
    return h.intersection(d).area < min_overlap_area

class EvasivePolicy:
    def __init__(s, p): s.p = p
    def _sim(s, e, n, af):
        t,st=[],EgoState(e.x,e.y,e.yaw,e.speed)
        for _ in range(n):
            ns=max(0,min(s.p.max_speed,st.speed+af(st)*s.p.dt))
            st=EgoState(st.x+ns*math.cos(st.yaw)*s.p.dt,st.y+ns*math.sin(st.yaw)*s.p.dt,st.yaw,ns)
            t.append(st)
            if ns==0 and af(st)<=0: break
        return t
    def select_primitive(s, e, ht, rp):
        for nm,fn in [('BRAKE',lambda _:s.p.max_decel),('PUSH',lambda _:s.p.max_accel)]:
            ok,hs=True,copy.deepcopy(ht)
            for f in s._sim(e,s.p.horizon_steps,fn):
                hs.propagate(s.p.dt)
                if not theorem1_check(hs.polygon,compute_danger_zone(f,s.p)):ok=False;break
            if ok: return nm
        return 'BRAKE'

# ═══════════════════════════════════════════════════════════════════════════════
#  LiDAR
# ═══════════════════════════════════════════════════════════════════════════════

class LidarPerception:
    def __init__(s, r): s.r,s._hits,s._bins = r,[],[r]*360
    def callback(s, pc): s._hits=[(d.point.x,d.point.y) for d in pc if abs(d.point.z)<2.5]
    def attach(s, w, p):
        bp=w.get_blueprint_library().find('sensor.lidar.ray_cast')
        for k,v in [('range',s.r),('rotation_frequency',20),('channels',32),
                     ('points_per_second',100000),('upper_fov',10),('lower_fov',-30)]:
            bp.set_attribute(k,str(v))
        se=w.spawn_actor(bp,carla.Transform(carla.Location(x=0,z=2.0)),attach_to=p)
        se.listen(s.callback); print(f"[LiDAR] listening:{se.is_listening()}"); return se

    def _build(s):
        """Build per-degree distance bins in SENSOR-LOCAL frame."""
        bd=[s.r]*360
        for hx,hy in s._hits:
            d=math.hypot(hx,hy)
            i=int(math.degrees(math.atan2(hy,hx))%360)
            bd[i]=min(bd[i],d)
        s._bins=bd

    def is_visible(s, target_world_xy, ego_world_xy, ego_yaw, threshold=4.0):
        """Return True if any LiDAR hit (transformed to world frame) falls
        within `threshold` metres of `target_world_xy`.

        This is the correct LiDAR-only visibility check: the closed-loop
        switch should fire when the sensor physically detects the pursuer,
        not when geometry heuristics guess that it might be unoccluded.

        Hit points are in sensor-local frame (x forward, y left).
        World frame: rotate by ego_yaw, then translate by ego position.
        """
        if len(s._hits) < 3:
            return False
        ex, ey = ego_world_xy
        tx, ty = target_world_xy
        cy, sy = math.cos(ego_yaw), math.sin(ego_yaw)
        for hx, hy in s._hits:
            # Sensor-local → world
            wx = ex + hx * cy - hy * sy
            wy = ey + hx * sy + hy * cy
            if math.hypot(wx - tx, wy - ty) < threshold:
                return True
        return False

    def get_shadow_polygon(s, el, road_poly, ego_yaw=0.0):
        """Shadow = occluded region behind LiDAR hits, clipped to road.

        CRITICAL: LiDAR points are in sensor-local frame. Angular bins
        are sensor-local. To project shadow boundary into world coords,
        each sensor-local angle must be rotated by ego_yaw.
        Without this rotation, the shadow polygon is rotated wrong and
        never overlaps the road polygon, making H̃ always empty.
        """
        if len(s._hits)<3: return Polygon()
        ex,ey=el; s._build()
        pts=[]
        for i in range(360):
            if s._bins[i]<s.r-0.5:
                # Sensor-local angle → world angle
                world_angle = math.radians(i) + ego_yaw
                pts.append((ex + s.r*math.cos(world_angle),
                            ey + s.r*math.sin(world_angle)))
        if len(pts)<3: return Polygon()
        sh=safe_geom(Polygon(pts).convex_hull); rp=safe_geom(road_poly)
        return safe_geom(sh.intersection(rp)) if not(sh.is_empty or rp.is_empty) else Polygon()

    def get_clear_polygon(s, el, ego_yaw=0.0):
        """Clear = confirmed-empty region (unblocked rays), in world frame."""
        if len(s._hits)<3: return Polygon()
        ex,ey=el
        pts=[(ex,ey)]
        for i in range(360):
            if s._bins[i]>=s.r-0.5:
                world_angle = math.radians(i) + ego_yaw
                pts.append((ex + s.r*math.cos(world_angle),
                            ey + s.r*math.sin(world_angle)))
        return safe_geom(Polygon(pts).convex_hull) if len(pts)>=4 else Polygon()

# ═══════════════════════════════════════════════════════════════════════════════
#  Road geometry from waypoints
# ═══════════════════════════════════════════════════════════════════════════════

def _perp(wps, i):
    n=len(wps); j=min(i+1,n-1); k=max(i,0) if j==i else i
    dx,dy=wps[j][0]-wps[k][0],wps[j][1]-wps[k][1]; d=math.hypot(dx,dy)
    return (-dy/d, dx/d) if d>0.01 else (0,1)

def build_road_polygon(wps, hw=5.0):
    L,R=[],[]
    for i in range(len(wps)):
        px,py=_perp(wps,i); x,y=wps[i]
        L.append((x+hw*px,y+hw*py)); R.append((x-hw*px,y-hw*py))
    R.reverse(); return safe_geom(Polygon(L+R))


# ═══════════════════════════════════════════════════════════════════════════════
#  Controls
# ═══════════════════════════════════════════════════════════════════════════════

def accel_to_ctrl(a, spd):
    c=carla.VehicleControl()
    c.throttle=min(1,a/2.5) if a>=0 else 0; c.brake=min(1,abs(a)/8) if a<0 else 0
    c.hand_brake=False; return c

def pp_steer(e, tgt, wb):
    dx,dy=tgt[0]-e.x,tgt[1]-e.y; ld=math.hypot(dx,dy)
    if ld<0.1: return 0
    a=(math.atan2(dy,dx)-e.yaw+math.pi)%(2*math.pi)-math.pi
    return max(-1,min(1,math.atan2(2*wb*math.sin(a),ld)/math.radians(70)))

def pp_steer_rev(e, tgt, wb):
    """Pure-pursuit steer for reversing: treats yaw+π as the reference heading."""
    dx,dy=tgt[0]-e.x,tgt[1]-e.y; ld=math.hypot(dx,dy)
    if ld<0.1: return 0
    rev_yaw=e.yaw+math.pi
    a=(math.atan2(dy,dx)-rev_yaw+math.pi)%(2*math.pi)-math.pi
    return -max(-1,min(1,math.atan2(2*wb*math.sin(a),ld)/math.radians(70)))

# ═══════════════════════════════════════════════════════════════════════════════
#  Pursuer follower
# ═══════════════════════════════════════════════════════════════════════════════

class WaypointFollower:
    def __init__(s, wps, speed, delay_s=0.0):
        s.wps,s.speed,s.idx = wps,speed,0
        s.delay_ticks = int(delay_s / 0.05)
        s._tick = 0
    def tick(s, v):
        s._tick += 1
        if s._tick < s.delay_ticks: return  # sit still during delay
        loc=v.get_transform().location
        while s.idx<len(s.wps)-1:
            if math.hypot(loc.x-s.wps[s.idx][0],loc.y-s.wps[s.idx][1])<8: s.idx+=1
            else: break
        if s.idx>=len(s.wps): return
        tx,ty=s.wps[s.idx]; dx,dy=tx-loc.x,ty-loc.y; d=math.hypot(dx,dy)
        if d<0.1: return
        # Drive position directly via set_transform (works with physics=OFF).
        # Advance by speed*dt toward the target waypoint, capped so we never
        # overshoot. set_target_velocity is not used: it requires physics=ON.
        step=min(s.speed*0.05, d)
        v.set_transform(carla.Transform(
            carla.Location(x=loc.x+dx/d*step, y=loc.y+dy/d*step, z=loc.z),
            carla.Rotation(yaw=math.degrees(math.atan2(dy,dx)))))
    @property
    def finished(s): return s.idx>=len(s.wps)-1

# ═══════════════════════════════════════════════════════════════════════════════
#  FSM + Planner
# ═══════════════════════════════════════════════════════════════════════════════

class ST:
    APPROACHING="APPROACHING"; PEEKING="PEEKING"; YIELDING="YIELDING"
    PROCEEDING="PROCEEDING"; EVADING="EVADING"

class Planner:
    def __init__(s, p, rp, entry):
        s.p,s.rp,s.entry=p,rp,entry
        s.ht=HiddenSetTracker(rp,p); s.ev=EvasivePolicy(p)
        s.lidar=LidarPerception(p.lidar_range)
        s.state=ST.APPROACHING; s._dbg_count=0; s._evade_ticks=0
        s._resume_ticks=0   # counts ticks since last EVADING→PROCEEDING transition
        s._mode='OPEN'    # 'OPEN' = hidden-set phase, 'CLOSED' = visible pursuer
        s._cl_logged=False  # True once we've printed the closed-loop entry banner
        s._nr_alerted=False   # True while pursuer is within near-range zone
        s._pur_converging=False  # True when pursuer known + approaching junction
        s._pause=False        # set by run_scenario via --pause flag
        s._dodge_active=False # True while executing lateral bounded-swerve dodge
        s._dodge_start_y=0.0  # ego y when dodge began; used for phase-2 return
        s._recovering=False   # True during curb-recovery reverse phase
        s._recover_ticks=0
        s._recover_attempts=0 # counts recoveries; breaks scenario if ≥ 3 fail
        s._post_recover=False # True after recovery; caps speed until heading corrects
        s._nr_reversing=False # True during NR last-resort reverse-to-safe-waypoint
        s._rev_wi=0           # current reverse target waypoint index
        s._rev_wi_min=0       # minimum (oldest) waypoint allowed in reverse
        # OBB half-dims; overwritten from actual bounding_box after spawn
        s.ego_hl, s.ego_hw = EGO_HL, EGO_HW
        s.pur_hl, s.pur_hw = PUR_HL, PUR_HW

    def _observed_threat(s, e, pur_obs):
        """Closed-loop check: is the pursuer visible in the LiDAR point cloud
        AND inside the danger zone / closing fast?

        Paper §V-C / Def 2: the open→closed-loop transition fires when the
        pursuer's footprint enters the ego's field of view Z(x_e, O^t).
        In a LiDAR-only stack this means the sensor must physically return
        hits on the pursuer's body — checked via LidarPerception.is_visible().
        The shadow-polygon heuristic is NOT used here: it over-approximates
        the occluded region (full-range extension + convex hull) and would
        classify a visible pursuer as occluded when the LiDAR hits it."""
        if pur_obs is None: return False
        px, py, pvx, pvy = pur_obs
        dx, dy = e.x - px, e.y - py
        dist = math.hypot(dx, dy)
        # ── Gate 1: range ────────────────────────────────────────────────────
        if dist > s.p.lidar_range: return False
        # ── Hard proximity: vehicle-footprint guard (bypasses LiDAR gate) ────
        # ego half-width ≈ 0.9 m, pursuer half-width ≈ 0.9 m (vehicle) or
        # 0.25 m (pedestrian) → use 3.0 m to include a 1.2 m safety margin.
        # At this range a real LiDAR always sees the target; is_visible()
        # can fail here when closing velocity flips sign as the pursuer curves
        # past (simulation artefact).  In hardware this guard corresponds to
        # the last tracker estimate held while the sensor update is unstable.
        if dist < 3.0: return True
        # ── Gate 2: LiDAR hit check (true sensor-based visibility) ───────────
        if not s.lidar.is_visible((px, py), (e.x, e.y), e.yaw, threshold=4.0):
            return False   # no LiDAR returns near pursuer → still occluded
        # ── Gate 3: threat assessment (danger zone or TTC) ───────────────────
        dz = safe_geom(compute_danger_zone(e, s.p))
        if dz.is_empty: return False
        if dz.contains(Point(px, py)): return True
        closing = (pvx * dx + pvy * dy) / dist  # pursuer vel component toward ego
        if closing > 1.0 and dist / closing < 3.0: return True  # TTC < 3 s
        return False

    def _push_clears_conflict(s, e, pur_obs):
        """True if ego can sprint at max_speed past the conflict zone before the
        pursuer arrives.  When pursuer is observed, uses actual kinematics;
        otherwise falls back to H̃ forward simulation (select_primitive).

        Key insight: H̃-based simulation fails when pursuer is observed (H̃≈0
        makes PUSH always look safe, and H̃>0 sliver makes BRAKE look safe via
        the min_overlap filter).  Direct kinematics are more reliable."""
        if pur_obs is not None:
            px, py, pvx, pvy = pur_obs
            dx, dy = e.x - px, e.y - py
            dist = math.hypot(dx, dy)
            if dist < 0.1: return False
            closing = (pvx * dx + pvy * dy) / dist
            if closing <= 0.5: return True   # pursuer not closing → sprint freely
            # Head-on check: pursuer is in front of ego AND heading toward ego.
            # Sprinting in this configuration increases closing rate → BRAKE.
            ego_fx, ego_fy = math.cos(e.yaw), math.sin(e.yaw)
            in_front = ego_fx * (px - e.x) + ego_fy * (py - e.y)  # > 0 if ahead
            pur_spd = math.hypot(pvx, pvy)
            if in_front > 0 and pur_spd > 0.5:
                toward_ego = dx * pvx + dy * pvy  # > 0 if pursuer heads toward ego
                if toward_ego > 0:
                    return False  # head-on: PUSH makes collision worse → BRAKE
            ttc = dist / closing             # pursuer ETA to ego's current position
            # Time for ego to clear the conflict zone at max speed.
            # d_clear depends on whether ego is before or after the junction entry:
            #   PEEKING  → ego approaching entry: must traverse to entry + crossing width
            #   PROCEEDING → ego at/past entry: only the remaining crossing width
            # Typical intersection crossing width ~8 m; safe_stop_margin adds buffer.
            CROSSING_WIDTH = 8.0  # m
            jx, jy = s.entry
            dist_to_entry = math.hypot(e.x - jx, e.y - jy)
            if s.state == ST.PEEKING:
                d_clear = dist_to_entry + CROSSING_WIDTH + s.p.safe_stop_margin
            else:  # PROCEEDING — ego is at or past the entry
                d_clear = max(CROSSING_WIDTH - dist_to_entry, 0.0) + s.p.safe_stop_margin
            t_clear = d_clear / s.p.max_speed
            # Safety margin: combined vehicle half-widths / pursuer_max_speed ≈ 0.44 s.
            # Use 0.5 s (was 1.0 s, which caused BRAKE when PUSH would have cleared).
            return t_clear + 0.5 < ttc
        return s.ev.select_primitive(e, s.ht, s.rp) == 'PUSH'

    def step(s, e, wps, wi, pur_obs=None):
        p=s.p
        # Curb-recovery: reverse east to un-wedge from south kerb, then fall
        # through to normal step for fresh waypoint alignment.
        if s._recovering:
            s._recover_ticks += 1
            if s._recover_ticks < 40:   # 2 s of reverse
                c = carla.VehicleControl()
                c.throttle = 0.4; c.brake = 0; c.reverse = True; c.hand_brake = False
                return c, s.state, wi
            s._recovering = False; s._recover_ticks = 0; s._resume_ticks = 0
            s._post_recover = True  # heading is ~207° from dodge; cap speed until corrected
            print("[RECOVER-done] resuming forward drive (post-recover slow mode)")
        # NR last-resort reverse: ego reverses along prior waypoints until OBB gap
        # exceeds NR_CLEAR_GAP, then falls back to EVADING hold for normal resume.
        if s._nr_reversing:
            if pur_obs is not None:
                _rgap = obb_gap((e.x,e.y), e.yaw, s.ego_hl, s.ego_hw,
                                (pur_obs[0],pur_obs[1]),
                                math.atan2(pur_obs[3],pur_obs[2]) if math.hypot(pur_obs[2],pur_obs[3])>0.1 else 0.0,
                                s.pur_hl, s.pur_hw)
                if _rgap > NR_CLEAR_GAP:
                    s._nr_reversing = False
                    s._nr_alerted = False
                    print(f"[NR-REV-done] gap={_rgap:.1f}m — pursuer clear, resuming EVADING hold")
            if s._nr_reversing:
                rev_tgt = wps[s._rev_wi]
                if math.hypot(e.x-rev_tgt[0], e.y-rev_tgt[1]) < 2.0 and s._rev_wi > s._rev_wi_min:
                    s._rev_wi -= 1; rev_tgt = wps[s._rev_wi]
                c = carla.VehicleControl()
                c.throttle=0.4; c.brake=0; c.reverse=True; c.hand_brake=False
                c.steer = pp_steer_rev(e, rev_tgt, p.wheelbase)
                return c, s.state, wi
        # Paper §V-B: each tick, compute shadow on road, incorporate into H̃,
        # propagate by pursuer reachability, prune by confirmed-clear FOV.
        shadow=s.lidar.get_shadow_polygon((e.x,e.y), s.rp, ego_yaw=e.yaw)
        clear=s.lidar.get_clear_polygon((e.x,e.y), ego_yaw=e.yaw)

        # Debug: print shadow/clear area for first 5 non-empty shadows
        if not shadow.is_empty and s._dbg_count<5:
            s._dbg_count+=1
            print(f"  [DBG] shadow area={shadow.area:.1f}  clear area="
                  f"{clear.area if not clear.is_empty else 0:.1f}  "
                  f"ego_yaw={math.degrees(e.yaw):.1f}°  "
                  f"hits={len(s.lidar._hits)}")

        # Only incorporate shadow during APPROACHING and PEEKING (pre-junction
        # phases). During PROCEEDING the ego is past the junction and obstacles
        # ahead would re-inflate H with irrelevant road shadows, causing false
        # EVADING cycles. During EVADING, shadow is also suppressed so H can
        # drain via propagate + prune alone and the ego can resume.
        if s.state in (ST.APPROACHING, ST.PEEKING):
            s.ht.incorporate_shadow(shadow)  # union new occluded road into H̃
        s.ht.propagate(p.dt)             # grow by pursuer_max_speed × dt
        s.ht.prune_observed(clear)       # subtract confirmed-empty FOV
        s.ht.prune_by_velocity_bound(15) # Neel & Saripalli [19]

        dz=compute_danger_zone(e,p); safe=theorem1_check(s.ht.polygon,dz)
        de=math.hypot(e.x-s.entry[0],e.y-s.entry[1]); ri=wi

        # Closed-loop check: pursuer detected in LiDAR point cloud and dangerous
        obs_threat = s._observed_threat(e, pur_obs)

        # ── Mode-transition logging (paper Algorithm 1) ──────────────────────
        if obs_threat and s._mode == 'OPEN':
            s._mode = 'CLOSED'
            if pur_obs is not None:
                px, py, pvx, pvy = pur_obs
                dx, dy = e.x - px, e.y - py
                dist = math.hypot(dx, dy)
                closing = (pvx*dx + pvy*dy)/dist if dist > 0 else 0
                print(f"  [MODE→ClosedLoop] Pursuer first detected at ({px:.1f},{py:.1f})"
                      f"  dist={dist:.1f}m  closing={closing:.1f}m/s  ego=({e.x:.1f},{e.y:.1f})")
        elif not obs_threat and s._mode == 'CLOSED' and s.state == ST.PROCEEDING:
            # Pursuer has left the danger zone and ego resumed; back to open-loop.
            s._mode = 'OPEN'
            print(f"  [MODE→OpenLoop]   Pursuer cleared — ego=({e.x:.1f},{e.y:.1f})")
        # ─────────────────────────────────────────────────────────────────────

        if s.state==ST.APPROACHING:
            if de<25: s.state=ST.PEEKING
        elif s.state==ST.PEEKING:
            # Extended-range pursuer gate: pur_obs gives us the pursuer's position
            # even when it is beyond LiDAR range (e.g. from camera/GPS).  If the
            # pursuer is known to be converging toward the junction within 60 m,
            # stay in PEEKING even when H̃=0 — LiDAR cleared the hidden set but
            # the known vehicle hasn't yet crossed the junction threshold.
            pur_converging = False
            if pur_obs is not None:
                _ppx, _ppy, _ppvx, _ppvy = pur_obs
                _pur_dist = math.hypot(e.x - _ppx, e.y - _ppy)
                _pur_spd  = math.hypot(_ppvx, _ppvy)
                # Converging = moving AND east of junction turn-out (ppx < entry+5),
                # i.e. pursuer has not yet turned south past the crossing centre.
                if _pur_spd > 0.3 and _pur_dist < 60.0 and _ppx < s.entry[0] + 5:
                    pur_converging = True
                    if s.ht.is_empty() and not obs_threat and not s._pur_converging:
                        print(f"  [PEEKING-HOLD] pur_obs within 60m and converging "
                              f"pur=({_ppx:.1f},{_ppy:.1f}) dist={_pur_dist:.1f}m "
                              f"spd={_pur_spd:.1f}m/s — stopping to wait")
            s._pur_converging = pur_converging
            # Proceed condition: strict H̃=∅ while approaching, theorem-1 (H̃∩D=∅)
            # once held at junction approach (de < PEEK_HOLD_DIST).  The original
            # is_empty() gate is too conservative: it keeps the ego in PEEKING until
            # H̃ drains fully, which only happens after the ego has crept deep into
            # the conflict zone.  Theorem 1 (paper §V-B) requires only H̃∩D=∅, not
            # H̃=∅; using `safe` here is the correct implementation.
            PEEK_HOLD_DIST = 4.0   # m from junction; switch to theorem-1 when here
            _proceed_ok = (s.ht.is_empty() if de >= PEEK_HOLD_DIST else safe)
            if _proceed_ok and not obs_threat and not pur_converging:
                s.state=ST.PROCEEDING
            elif not safe or obs_threat:
                # Evaluate PUSH vs BRAKE before committing to EVADING.
                _obs = pur_obs if obs_threat else None
                if s._push_clears_conflict(e, _obs):
                    print(f"  [PUSH-THROUGH] Sprint clears conflict — proceeding at max speed")
                    s.state=ST.PROCEEDING
                else:
                    s.state=ST.EVADING; s._evade_ticks=0
        elif s.state==ST.PROCEEDING:
            if not safe or obs_threat:
                _obs = pur_obs if obs_threat else None
                if not s._push_clears_conflict(e, _obs):
                    s.state=ST.EVADING; s._evade_ticks=0
                elif s._mode == 'CLOSED' and pur_obs is not None:
                    # pursuer visible but obs_threat gating failed — do kinematic check anyway
                    if not s._push_clears_conflict(e, pur_obs):
                        s.state = ST.EVADING; s._evade_ticks=0
        elif s.state==ST.YIELDING:
            if s.ht.is_empty() and safe: s.state=ST.PROCEEDING
        elif s.state==ST.EVADING:
            s._evade_ticks += 1
            # Require ego to nearly stop AND hold EVADING for ≥1 s (20 ticks)
            # AND confirm no visible-pursuer threat remains.
            # Use Theorem-1 (H̃∩D=∅, paper §V-B) rather than H̃=∅: the hidden set
            # can degrade to a zero-area degenerate polygon (Shapely is_empty=False
            # even at area≈0) which would permanently block resume if we check
            # is_empty().  `safe` correctly captures the paper's safety condition.
            stopped = e.speed < 0.5
            held_long_enough = s._evade_ticks >= 20
            if (safe and stopped and held_long_enough and not obs_threat):
                ds=[math.hypot(e.x-w[0],e.y-w[1]) for w in wps]
                ri=_aligned_wp(e, wps, int(np.argmin(ds)))
                s._evade_ticks = 0
                s._resume_ticks = 0   # start re-alignment phase
                s._dodge_active = False   # clear dodge so lk override doesn't persist
                s._post_recover = False
                s._nr_reversing = False
                s.state=ST.PROCEEDING

        # ── Near-range proximity sensor (front 2D LiDAR / ultrasonic) ────────
        # Covers the LiDAR near-range blind-spot.  When the pursuer enters the
        # combined vehicle footprint zone this sensor fires regardless of whether
        # the main LiDAR point-cloud registered a hit.
        # • FRONT threat: pursuer is ahead in ego's direction of travel — lateral
        #   dodge: steer south-west to open clearance while the pursuer passes.
        #   Triggered at NR_DODGE_EDGE (4 m) for ~1 s lead time; the closer
        #   NR_EDGE (2 m) fires the proximity alert only.
        # • REAR threat while stopped: pursuer closing from behind while ego is at
        #   standstill → emergency sprint forward to clear the path before impact.
        NR_EDGE = 2.0        # m — proximity alert / emergency-sprint threshold
        NR_DODGE_EDGE = 4.0  # m — front lateral-dodge advance trigger
        if pur_obs is not None:
            _nr_px, _nr_py, _nr_pvx, _nr_pvy = pur_obs
            _nr_spd  = math.hypot(_nr_pvx, _nr_pvy)
            _pur_yaw = math.atan2(_nr_pvy, _nr_pvx) if _nr_spd > 0.1 else 0.0
            _nr_gap  = obb_gap((e.x,e.y), e.yaw, s.ego_hl, s.ego_hw,
                               (_nr_px,_nr_py), _pur_yaw, s.pur_hl, s.pur_hw)
            _nr_fx   = math.cos(e.yaw); _nr_fy = math.sin(e.yaw)
            _nr_in_front = _nr_fx*(_nr_px-e.x) + _nr_fy*(_nr_py-e.y)
            if _nr_gap < NR_EDGE:
                _nr_side = "FRONT" if _nr_in_front > 0 else "REAR"
                if not s._nr_alerted:
                    print(f"  [NR-SENSOR !!] Pursuer edge-gap={_nr_gap:.2f}m {_nr_side} — "
                          f"pur=({_nr_px:.1f},{_nr_py:.1f}) ego=({e.x:.1f},{e.y:.1f})")
                    s._nr_alerted = True
                    if s._pause: input("  [paused — press Enter to continue]")
                if _nr_in_front < 0 and e.speed < 0.5 and s.state == ST.EVADING:
                    print(f"  [EMERGENCY-SPRINT] Rear near-range threat — sprinting to clear")
                    _ds = [math.hypot(e.x-w[0], e.y-w[1]) for w in wps]
                    ri = _aligned_wp(e, wps, int(np.argmin(_ds)))
                    s.state = ST.PROCEEDING
                    s._evade_ticks = 0; s._resume_ticks = 0; s._dodge_active = False; s._post_recover = False
            # Front near-range: pursuer closing while ego is in EVADING (BRAKE active).
            # Priority: 1) PUSH (sprint) if still viable; 2) lateral dodge if
            # push is also unsafe and there is enough lane south; 3) hold BRAKE.
            # Gate on _nr_gap > 0: once overlapping (gap ≤ 0), PUSH/dodge evaluation
            # is meaningless — let the EVADING resume logic handle clearing instead.
            if 0 < _nr_gap < NR_DODGE_EDGE and _nr_in_front > 0 and s.state == ST.EVADING:
                if not s._dodge_active:
                    _push_safe = s._push_clears_conflict(e, pur_obs)
                    if _push_safe:
                        # PUSH can still clear the conflict — sprint through rather than dodge.
                        print(f"  [NR-PUSH] gap={_nr_gap:.2f}m — PUSH clears, sprint through")
                        _ds = [math.hypot(e.x-w[0], e.y-w[1]) for w in wps]
                        ri = _aligned_wp(e, wps, int(np.argmin(_ds)))
                        s.state = ST.PROCEEDING
                        s._evade_ticks = 0; s._resume_ticks = 0
                        s._dodge_active = False; s._post_recover = False
                    else:
                        # BRAKE + PUSH both unsafe; lateral dodge if lane allows.
                        # South clearance = distance from ego to south curb (positive = room).
                        _south_clearance = e.y - ROAD_SOUTH_Y
                        if _south_clearance > DODGE_SOUTH_LIMIT + 0.5:
                            # Normal dodge: enough room to complete the full bounded swerve.
                            print(f"  [LATERAL-DODGE] gap={_nr_gap:.2f}m PUSH+BRAKE unsafe, "
                                  f"south={_south_clearance:.2f}m — bounded swerve")
                            s._dodge_active = True
                            s._dodge_start_y = e.y
                        elif _south_clearance > 0:
                            # Last-resort curb-climb dodge: lane too narrow for a clean swerve
                            # but curb contact is acceptable when no pedestrian is present.
                            # (No pedestrian sensor in this scenario — gate always passes.)
                            print(f"  [LATERAL-DODGE-CURB] gap={_nr_gap:.2f}m last-resort "
                                  f"curb-climb, south={_south_clearance:.2f}m")
                            s._dodge_active = True
                            s._dodge_start_y = e.y
                        elif not s._nr_reversing:
                            # No room for any dodge — reverse to a prior safe waypoint.
                            print(f"  [NR-REVERSE] gap={_nr_gap:.2f}m no dodge room "
                                  f"({_south_clearance:.2f}m) — reversing to safe waypoint")
                            s._nr_reversing = True
                            s._rev_wi = max(0, wi - 1)
                            s._rev_wi_min = max(0, wi - REV_WPS_BACK)
                            s._nr_alerted = True
            elif _nr_gap >= NR_DODGE_EDGE:
                s._nr_alerted = False
                s._dodge_active = False   # pursuer clear; end dodge

        # Adaptive lookahead: short on curves, long on straights.
        # After EVADING→PROCEEDING, force tight look-ahead for 2 s to re-align
        # the heading before full-speed cruise resumes (avoids S-curve overshoot).
        RESUME_PHASE = 40   # ticks ≈ 2 s at 20 Hz
        if s.state == ST.PROCEEDING and s._resume_ticks < RESUME_PHASE:
            s._resume_ticks += 1
        def _curvature(i):
            if i < 1 or i >= len(wps)-1: return 0
            ax,ay = wps[i][0]-wps[i-1][0], wps[i][1]-wps[i-1][1]
            bx,by = wps[i+1][0]-wps[i][0], wps[i+1][1]-wps[i][1]
            cross = abs(ax*by - ay*bx)
            return cross / max(math.hypot(ax,ay)*math.hypot(bx,by), 0.01)
        curv = _curvature(wi)
        if s._resume_ticks > 0 and s._resume_ticks < RESUME_PHASE:
            look_ahead = 1   # track nearest waypoint tightly during re-alignment
        else:
            look_ahead = 2 if curv > 0.15 else (3 if curv > 0.05 else 5)
        lk=wps[min(wi+look_ahead,len(wps)-1)]
        # Lateral-dodge override: steer south-west to open clearance while the
        # pursuer passes on the adjacent northbound lane.
        # Target is 2 m west + 1 m south of ego, giving ~27° heading error for a
        # westbound ego (pure south gives ~180° error and steers the wrong way).
        # The 1 m south offset moves ego ~0.3 m south over 1 s, clearing the
        # combined OBB half-width overlap without going off-road.
        if s._dodge_active:
            if e.y < s._dodge_start_y - DODGE_SOUTH_LIMIT:
                # Phase 2: south limit reached — steer back north to recover lane.
                # North + south phases nearly cancel, leaving heading close to -180°.
                lk = (e.x - 2.0, e.y + 1.0)
            else:
                # Phase 1: steer south-west to open lateral clearance.
                lk = (e.x - 2.0, e.y - 1.0)
        elif s._post_recover:
            # After curb-recovery (fallback path), force close due-west target for
            # maximum heading-correction authority.
            lk = (e.x - 2.0, e.y)
        if s.state==ST.APPROACHING:
            a=np.clip((p.max_speed*0.6-e.speed)*2,p.max_decel*0.3,p.max_accel)
        elif s.state==ST.PEEKING:
            if s._pur_converging:
                # Known pursuer approaching junction — hold position until it clears.
                a = p.max_decel * 0.5 if e.speed > 0.05 else 0
            elif not s.ht.is_empty() and de < 4.0:
                # H̃ still active at junction approach — hold; don't enter turn arc.
                # Theorem-1 will clear the ego once H̃∩D=∅ at this vantage point.
                a = p.max_decel * 0.5 if e.speed > 0.05 else 0
            else:
                a=np.clip((p.max_speed*0.3-e.speed)*1.5,p.max_decel*0.4,p.max_accel)
        elif s.state==ST.YIELDING: a=p.max_decel*0.5
        elif s.state==ST.PROCEEDING:
            # Junction zone: keep PEEKING speed while ego is still crossing the
            # junction (de < crossing_width + margin). This prevents high-speed
            # entry into the conflict zone immediately after PEEKING→PROCEEDING.
            JUNCTION_ZONE = 10.0  # m from entry; matches CROSSING_WIDTH + safe margin
            if s._post_recover:
                # After curb-recovery, heading is ~27° south of west (-153° CARLA).
                # lk override (due west, 2 m) gives steer≈0.56 → 12°/s correction.
                # Clear when heading is within 10° of due west; then reset
                # _resume_ticks so the slow-crawl phase re-runs before full speed.
                _heading_err = abs((math.pi - e.yaw + math.pi) % (2*math.pi) - math.pi)
                if _heading_err < math.radians(10):
                    s._post_recover = False
                    s._resume_ticks = 1   # re-enter slow crawl (> 0 triggers the branch)
                    print(f"[POST-RECOVER] heading={math.degrees(e.yaw):.1f}° aligned")
                a = np.clip((0.5 - e.speed)*3.0, p.max_decel*0.4, p.max_accel*0.3)
            elif de < JUNCTION_ZONE:
                a=np.clip((p.max_speed*0.3-e.speed)*1.5,p.max_decel*0.4,p.max_accel)
            # During re-alignment phase: slow crawl to let heading correct before
            # full acceleration. After RESUME_PHASE ticks, resume normal speed.
            elif s._resume_ticks > 0 and s._resume_ticks < RESUME_PHASE:
                a=np.clip((p.max_speed*0.3-e.speed)*1.5,p.max_decel*0.4,p.max_accel)
            else:
                a=np.clip((p.max_speed-e.speed)*1.0,p.max_decel*0.3,p.max_accel)
        elif s.state==ST.EVADING:
            if s._dodge_active:
                # Gentle creep so pure-pursuit can execute the south-west dodge.
                a = np.clip((1.2 - e.speed)*3.0, p.max_decel*0.3, p.max_accel*0.5)
            else:
                a=p.max_decel if e.speed>0.1 else 0
                reason = "obs" if obs_threat else "hidden"
                print(f"  [EVADING] BRAKE v={e.speed:.1f} reason={reason}")
        else: a=0

        if e.speed>=p.max_speed and a>0: a=0
        # Curvature speed cap: slow down on turns
        if curv > 0.05:
            max_turn_speed = 2.0 if curv > 0.15 else 3.5
            if e.speed > max_turn_speed:
                a = min(a, (max_turn_speed - e.speed) * 2.0)
        ctrl=accel_to_ctrl(a,e.speed); ctrl.steer=pp_steer(e,lk,p.wheelbase)
        return ctrl,s.state,ri

# ═══════════════════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def get_ego_state(v):
    t,vel=v.get_transform(),v.get_velocity()
    try: ax=v.get_telemetry_data().acceleration_x
    except: ax=0
    return EgoState(t.location.x,t.location.y,math.radians(t.rotation.yaw),math.hypot(vel.x,vel.y),ax)

def spawn_occluder(w, tf):
    for n in ['vehicle.carlamotors.firetruck','vehicle.carlamotors.carlacola',
              'vehicle.mercedes.sprinter','vehicle.volkswagen.t2']:
        f=w.get_blueprint_library().filter(n)
        if f:
            a=w.try_spawn_actor(f[0],tf)
            if a: a.set_simulate_physics(False); print(f"[Occluder] {n}"); return a
    return None

# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════

def run_scenario(host='127.0.0.1', port=2000, slow=0.0, pause=False):
    print("="*70)
    print("  BLIND INTERSECTION — Town03 junction ~(147,-74), RHD")
    print("="*70)

    client=carla.Client(host,port); client.set_timeout(20.0)
    world=client.load_world('Town03')
    settings=world.get_settings()
    settings.synchronous_mode=True; settings.fixed_delta_seconds=0.05
    settings.spectator_as_ego=False; world.apply_settings(settings)
    client.get_trafficmanager(8000).set_synchronous_mode(True)

    cmap=world.get_map(); p=PlannerParams()
    bpl=world.get_blueprint_library(); actors=[]

    try:
        # ── Ego ──
        ebp=bpl.filter('vehicle.pixloop.hooke')
        if not ebp: raise RuntimeError("Hooke not found")
        ebp=ebp[0]
        if ebp.has_attribute('role_name'): ebp.set_attribute('role_name','ego_vehicle')
        if ebp.has_attribute('color'): ebp.set_attribute('color','30,144,255')
        ex,ey=EGO_WAYPOINTS[0]
        ez=cmap.get_waypoint(carla.Location(x=ex,y=ey,z=0.3),project_to_road=True).transform.location.z
        ego=world.spawn_actor(ebp,carla.Transform(
            carla.Location(x=ex,y=ey,z=ez+0.3),carla.Rotation(yaw=EGO_SPAWN_YAW)))
        actors.append(ego); print(f"[Ego] ({ex:.1f},{ey:.1f}) yaw={EGO_SPAWN_YAW}")

        # ── Occluder ──
        oz=cmap.get_waypoint(carla.Location(x=OCCLUDER_X,y=OCCLUDER_Y,z=0.3),
                             project_to_road=True).transform.location.z
        occ=spawn_occluder(world,carla.Transform(
            carla.Location(x=OCCLUDER_X,y=OCCLUDER_Y,z=oz+0.1),
            carla.Rotation(yaw=OCCLUDER_YAW)))
        if occ: actors.append(occ)

        # ── Pursuer ──
        px,py=PURSUER_WAYPOINTS[0]
        pz=cmap.get_waypoint(carla.Location(x=px,y=py,z=0.3),
                             project_to_road=True).transform.location.z
        pbps=bpl.filter('vehicle.tesla.model3') or bpl.filter('vehicle.audi.tt')
        if not pbps: raise RuntimeError("No pursuer bp")
        pbp=pbps[0]
        if pbp.has_attribute('color'): pbp.set_attribute('color','220,30,30')
        pur=world.try_spawn_actor(pbp,carla.Transform(
            carla.Location(x=px,y=py,z=pz+0.3),carla.Rotation(yaw=PURSUER_SPAWN_YAW)))
        if pur: pur.set_autopilot(False); pur.set_simulate_physics(False); actors.append(pur); print(f"[Pursuer] ({px:.1f},{py:.1f}) yaw={PURSUER_SPAWN_YAW} physics=OFF")
        else: print("[Pursuer] FAILED")

        world.tick()
        for nm,ac in [("Ego",ego),("Pursuer",pur)]:
            if ac and ac.is_alive:
                at=ac.get_transform()
                print(f"[{nm}] tick: ({at.location.x:.1f},{at.location.y:.1f}) yaw={at.rotation.yaw:.1f}")

        # ── Geometry ──
        # Road polygon covers all driveable area (both ego and pursuer paths).
        # H̃ is initialized from shadow ∩ road_poly (paper Eq. 4), not from
        # a pre-defined oncoming lane polygon. Theorem 1 check (H̃ ∩ D = ∅)
        # naturally filters spatial relevance — D is only ahead of ego.
        rpoly=build_road_polygon(EGO_WAYPOINTS+PURSUER_WAYPOINTS, hw=6.0)

        planner=Planner(p,rpoly,(JUNCTION_X,JUNCTION_Y))
        planner._pause=pause
        # Overwrite OBB half-dims from actual CARLA bounding boxes.
        # bounding_box.extent: x=half-length, y=half-width (local frame).
        _ebb = ego.bounding_box.extent
        planner.ego_hl, planner.ego_hw = _ebb.x, _ebb.y
        if pur and pur.is_alive:
            _pbb = pur.bounding_box.extent
            planner.pur_hl, planner.pur_hw = _pbb.x, _pbb.y
        print(f"[OBB] ego hl={planner.ego_hl:.3f} hw={planner.ego_hw:.3f} | "
              f"pur hl={planner.pur_hl:.3f} hw={planner.pur_hw:.3f}")
        lidar=planner.lidar.attach(world,ego); actors.append(lidar)
        world.tick()

        pfol=WaypointFollower(PURSUER_WAYPOINTS, 4.17, delay_s=14.0) if pur else None
        spec=world.get_spectator()

        print("[CARLA] Running.")
        wi,tick,ls=0,0,None
        collision_logged = False   # latched per collision event; resets when pursuer clears
        prev_pur_pos = None  # for effective-velocity estimation (physics=OFF)
        # Position-window stuck detection: immune to physics micro-jitter that
        # resets a consecutive-tick counter (0.11–0.16 m/s noise seen in logs).
        # Keep last 200 ticks (10 s) of ego positions; flag stuck when total
        # displacement over the window is < 0.5 m.
        from collections import deque
        pos_history = deque(maxlen=200)

        while True:
            world.tick(); tick+=1
            es=get_ego_state(ego)

            spec.set_transform(carla.Transform(
                carla.Location(x=130.0, y=-76.0, z=80.0),
                carla.Rotation(pitch=-90, yaw=0, roll=0)))

            if pur and pur.is_alive and pfol: pfol.tick(pur)

            # Compute observed pursuer state for closed-loop threat check.
            # With physics=OFF, get_velocity() is unreliable; estimate from
            # position delta instead (position is correct via set_transform).
            pur_obs = None
            if pur and pur.is_alive:
                pt = pur.get_transform()
                curr_pur = (pt.location.x, pt.location.y)
                if prev_pur_pos is not None:
                    pvx = (curr_pur[0] - prev_pur_pos[0]) / p.dt
                    pvy = (curr_pur[1] - prev_pur_pos[1]) / p.dt
                    pur_obs = (curr_pur[0], curr_pur[1], pvx, pvy)
                prev_pur_pos = curr_pur

            if wi<len(EGO_WAYPOINTS)-6:
                if math.hypot(es.x-EGO_WAYPOINTS[wi][0],es.y-EGO_WAYPOINTS[wi][1])<3: wi+=1

            ctrl,fsm,ri=planner.step(es,EGO_WAYPOINTS,wi,pur_obs)
            if ri!=wi: print(f"  [RESUME] wp {wi}->{ri}  [{planner._mode}]"); wi=ri
            ego.apply_control(ctrl)

            # ── Ground-truth collision detector (OBB overlap) ────────────────
            # Fires on actual footprint overlap, not raw centre distance.
            if pur_obs is not None:
                _cs  = math.hypot(pur_obs[2], pur_obs[3])
                _cpy = math.atan2(pur_obs[3], pur_obs[2]) if _cs > 0.1 else 0.0
                _cgap = obb_gap((es.x,es.y), es.yaw, planner.ego_hl, planner.ego_hw,
                                (pur_obs[0],pur_obs[1]), _cpy, planner.pur_hl, planner.pur_hw)
                if _cgap <= 0:
                    if not collision_logged:
                        print(f"  [!! COLLISION !!] t={tick*p.dt:.1f}s "
                              f"overlap={-_cgap:.2f}m "
                              f"ego=({es.x:.1f},{es.y:.1f}) "
                              f"pur=({pur_obs[0]:.1f},{pur_obs[1]:.1f}) "
                              f"impact_v=({pur_obs[2]:.1f},{pur_obs[3]:.1f})")
                        collision_logged = True
                        if pause: input("  [paused — press Enter to continue]")
                else:
                    collision_logged = False   # pursuer cleared; ready for next event

            ch=fsm!=ls; ls=fsm
            if ch or tick%20==0:
                ha=planner.ht.polygon.area if not planner.ht.is_empty() else 0
                sf=theorem1_check(planner.ht.polygon,compute_danger_zone(es,p))
                ps=""
                if pur and pur.is_alive and pfol:
                    pt=pur.get_transform(); pv=pur.get_velocity()
                    ps=(f" | pur=({pt.location.x:+7.1f},{pt.location.y:+6.1f})"
                        f" v={math.hypot(pv.x,pv.y):.1f} wp={pfol.idx}/{len(pfol.wps)}")
                mode_tag = f"[{planner._mode}]"
                print(f"t={tick*0.05:6.1f}s | {fsm:12s} | {mode_tag:7s} | v={es.speed:5.2f} | "
                      f"({es.x:+7.1f},{es.y:+7.1f}) | |H|={ha:5.1f} | "
                      f"{'OK' if sf else 'UNSAFE'}{ps}{'***' if ch else ''}")

            # Termination: ego heading west, x < END_X
            if es.x < END_X and fsm in (ST.PROCEEDING,ST.APPROACHING):
                print(f"[Done] x={es.x:.1f}"); break
            # Position-window stuck detection: append current position and check
            # whether the ego has moved at least 0.5 m over the last 10 s window.
            # This is immune to the physics micro-jitter (0.1–0.2 m/s noise spikes)
            # that indefinitely resets a consecutive-tick speed counter.
            if fsm == ST.PEEKING or planner._recovering:
                pos_history.clear()   # intentional hold / recovery ≠ stuck
            else:
                pos_history.append((es.x, es.y))
                if len(pos_history) == pos_history.maxlen:
                    ox, oy = pos_history[0]
                    if math.hypot(es.x - ox, es.y - oy) < 0.5:
                        if fsm == ST.PROCEEDING and planner._recover_attempts < 3:
                            planner._recover_attempts += 1
                            print(f"[Stuck-RECOVER] attempt {planner._recover_attempts}/3 — reversing off curb")
                            planner._recovering = True
                            planner._recover_ticks = 0
                            pos_history.clear()
                        else:
                            print("[Stuck]"); break
            if tick>4000: print("[Limit]"); break
            if slow: import time; time.sleep(slow)

    finally:
        print("[Cleanup]")
        try: settings.synchronous_mode=False; settings.spectator_as_ego=True; world.apply_settings(settings)
        except Exception as e: print(f"  {e}")
        for a in reversed(actors):
            try:
                if a and a.is_alive: a.destroy()
            except: pass
        print("[Done]")

if __name__=='__main__':
    pa=argparse.ArgumentParser()
    pa.add_argument('--host',default='127.0.0.1')
    pa.add_argument('--port',type=int,default=2000)
    pa.add_argument('--slow',type=float,default=0.0,
                    metavar='SEC',help='sleep SEC seconds after each tick (e.g. 0.1)')
    pa.add_argument('--pause',action='store_true',
                    help='freeze on NR-sensor / collision events until Enter')
    a=pa.parse_args()
    run_scenario(a.host,a.port,slow=a.slow,pause=a.pause)
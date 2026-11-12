#!/usr/bin/env python3
"""Generate synthetic rides for the Smart Helmet HUD app (stand-in for the Python prototype's recordings).

    python3 sim/generate_rides.py                 # writes sim/rides/*.jsonl and sim/rides/index.json
    python3 sim/generate_rides.py --seeds 5       # five noise seeds per scenario (name_s0 ... name_s4)

Everything here is SYNTHETIC. The LiDAR packets use the exact wire format of
firmware/bike/qtpy_lidar_udp (730 bytes, "HL" header, 360 x uint16 mm, index = degrees clockwise
from forward). Camera detections are what a YOLO model would output (class, confidence, pixel box).
The sensor geometry (camera field of view, rider-body shape, LiDAR height) is assumed, not measured.

Frame: rider/LiDAR at (0, 0), x = right, y = forward, metres. Yaw = degrees clockwise from forward.
Standard library only.
"""
import argparse
import base64
import json
import math
import os
import random
import struct

DT = 0.1                      # one LiDAR revolution / one frame
LIDAR_MIN_M = 0.05
LIDAR_MAX_M = 12.0            # light surfaces
LIDAR_MAX_DARK_M = 6.0        # black cars
LIDAR_SIGMA_M = 0.02
LIDAR_DROP_P = 0.02
LIDAR_HEIGHT_M = 1.0
CONTACT_GAP_M = 0.4           # outline closer than this to the rider = contact
CAM_HEIGHT_M = 1.0
REAR_CAM = {"w": 320, "h": 240, "hfov_deg": 62.0}
FRONT_CAM = {"w": 640, "h": 480, "hfov_deg": 68.0}
CAM_MAX_M = 50.0
START_LAT, START_LON, HEADING_DEG = 42.3398, -71.0892, 20.0

DIMS = {  # width, length, height (m)
    "car": (1.8, 4.5, 1.5), "truck": (2.5, 8.0, 3.2), "bus": (2.55, 12.0, 3.1),
    "bicycle": (0.6, 1.8, 1.7), "person": (0.5, 0.5, 1.7), "door": (0.08, 1.0, 1.2),
}


# ---------------------------------------------------------------- geometry
def rect_corners(cx, cy, w, l, yaw_deg):
    yaw = math.radians(yaw_deg)
    lx, ly = math.sin(yaw), math.cos(yaw)          # length axis
    wx, wy = math.cos(yaw), -math.sin(yaw)         # width axis
    out = []
    for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        out.append((cx + sl * l / 2 * lx + sw * w / 2 * wx, cy + sl * l / 2 * ly + sw * w / 2 * wy))
    return out


def ray_segment(dx, dy, ax, ay, bx, by):
    """Distance along the ray from the origin in direction (dx, dy) to segment AB, or None."""
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = (ax * ey - ay * ex) / den
    u = (ax * dy - ay * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def ray_circle(dx, dy, cx, cy, r):
    b = dx * cx + dy * cy
    disc = b * b - (cx * cx + cy * cy - r * r)
    if disc < 0:
        return None
    t = b - math.sqrt(disc)
    return t if t > 0 else None


def point_segment_dist(ax, ay, bx, by):
    ex, ey = bx - ax, by - ay
    k = max(0.0, min(1.0, -(ax * ex + ay * ey) / (ex * ex + ey * ey + 1e-12)))
    return math.hypot(ax + k * ex, ay + k * ey)


# ---------------------------------------------------------------- objects
class Obj:
    def __init__(self, oid, cls, pose, dark=False, camera=True, dims=None):
        self.id, self.cls, self.pose, self.dark, self.camera = oid, cls, pose, dark, camera
        self.w, self.l, self.h = dims or DIMS[cls]
        self.circle = cls == "person"

    def rel(self, t, rider_y):
        x, y, yaw = self.pose(t)
        return x, y - rider_y, yaw

    def outline_gap(self, x, y, yaw):
        if self.circle:
            return max(0.0, math.hypot(x, y) - self.w / 2)
        c = rect_corners(x, y, self.w, self.l, yaw)
        # inside test: all cross products same sign
        signs = [(c[(i + 1) % 4][0] - c[i][0]) * (-c[i][1]) - (c[(i + 1) % 4][1] - c[i][1]) * (-c[i][0])
                 for i in range(4)]
        if all(s >= 0 for s in signs) or all(s <= 0 for s in signs):
            return 0.0
        return min(point_segment_dist(*c[i], *c[(i + 1) % 4]) for i in range(4))


def straight(x0, y0, vy, vx=0.0, yaw=0.0):
    return lambda t: (x0 + vx * t, y0 + vy * t, yaw)


def static(x, y, yaw=0.0):
    return lambda t: (x, y, yaw)


def right_hook_path(x0, y0, v_fast, v_turn, rider_v, lead_m, radius, duration):
    """Car overtakes on the left, then turns right across the rider's path once it is lead_m ahead."""
    h = 0.01
    x, y, yaw, v, turning = x0, y0, 0.0, v_fast, False
    samples = []
    for i in range(int(duration / h) + 2):
        samples.append((x, y, math.degrees(yaw)))
        if not turning and y - rider_v * i * h > lead_m:
            turning = True
        if turning:
            v = max(v_turn, v - 8.0 * h)
            if yaw < math.pi / 2:
                yaw = min(math.pi / 2, yaw + v / radius * h)
        x += v * math.sin(yaw) * h
        y += v * math.cos(yaw) * h
    return lambda t: samples[max(0, min(len(samples) - 1, int(round(t / h))))]


def door_pose(hinge_x, hinge_y, t_open, open_s, max_deg):
    """Door hinged at its front edge on the traffic side of a car parked on the right; swings toward -x."""
    def pose(t):
        a = math.radians(max_deg * max(0.0, min(1.0, (t - t_open) / open_s)))
        dx, dy = -math.sin(a), -math.cos(a)
        return hinge_x + 0.5 * dx, hinge_y + 0.5 * dy, math.degrees(math.atan2(dx, dy))
    return pose


def parked_row(first_id, gap, y_first, n, spacing=6.0, dark_every=0):
    x = gap + DIMS["car"][0] / 2
    return [Obj(first_id + i, "car", static(x, y_first + i * spacing),
                dark=bool(dark_every and i % dark_every == 0)) for i in range(n)]


# ---------------------------------------------------------------- sensors
def rider_body(t):
    """Seat tube and pedalling legs as seen by a LiDAR on the rear rack (assumed shape)."""
    ph = 2 * math.pi * 1.2 * t
    return [(0.0, 0.20, 0.03),
            (-0.14, 0.45 + 0.15 * math.sin(ph), 0.07),
            (0.14, 0.45 - 0.15 * math.sin(ph), 0.07)]


def lidar_scan(objs_rel, t, lean_deg, max_m, drop_p, rng):
    body = rider_body(t)
    out = []
    for i in range(360):
        th = math.radians(i + 0.5)
        dx, dy = math.sin(th), math.cos(th)
        best = None
        for cx, cy, r in body:
            d = ray_circle(dx, dy, cx, cy, r)
            if d is not None and (best is None or d < best):
                best = d
        for o, (x, y, yaw) in objs_rel:
            limit = min(max_m, LIDAR_MAX_DARK_M if o.dark else LIDAR_MAX_M)
            if o.circle:
                d = ray_circle(dx, dy, x, y, o.w / 2)
            else:
                c = rect_corners(x, y, o.w, o.l, yaw)
                hits = [ray_segment(dx, dy, *c[k], *c[(k + 1) % 4]) for k in range(4)]
                hits = [h for h in hits if h is not None]
                d = min(hits) if hits else None
            if d is not None and d <= limit and (best is None or d < best):
                best = d
        # leaning tilts the scan plane: on the low side it hits the road
        s = dx * math.sin(math.radians(lean_deg))
        if s > 1e-6:
            g = LIDAR_HEIGHT_M / s
            if g <= max_m and (best is None or g < best):
                best = g
        if best is None or best < LIDAR_MIN_M or rng.random() < drop_p:
            out.append(0)
        else:
            out.append(max(1, min(65535, int(round((best + rng.gauss(0, LIDAR_SIGMA_M)) * 1000)))))
    return out


def lidar_packet(seq, t, ranges_mm):
    return b"HL" + struct.pack("<BBHI", 1, 0, seq & 0xFFFF, int(t * 1000)) + struct.pack("<360H", *ranges_mm)


def camera_boxes(objs_rel, cam, facing, rng):
    """facing = +1 front camera, -1 rear camera. Rear boxes are NOT mirrored (rider's left = image right)."""
    w, h = cam["w"], cam["h"]
    f = (w / 2) / math.tan(math.radians(cam["hfov_deg"]) / 2)
    out = []
    for o, (x, y, yaw) in objs_rel:
        if not o.camera:
            continue
        pts = [(x, y)] if o.circle else rect_corners(x, y, o.w, o.l, yaw)
        if o.circle:
            pts = [(x - o.w / 2, y), (x + o.w / 2, y)]
        front = [(px * facing, py * facing) for px, py in pts if py * facing > 0.3]
        if len(front) < 2:
            continue
        z = min(p[1] for p in front)
        if z > CAM_MAX_M:
            continue
        us = [w / 2 + f * px / pz for px, pz in front]
        u0, u1 = max(0.0, min(us)), min(float(w), max(us))
        if u1 - u0 < 6:
            continue
        v1 = min(float(h), h / 2 + f * CAM_HEIGHT_M / z)
        v0 = max(0.0, h / 2 + f * (CAM_HEIGHT_M - o.h) / z)
        p_miss = 0.03 if z < 20 else 0.15 if z < 40 else 0.4
        if rng.random() < p_miss or v1 - v0 < 4:
            continue
        bw = (u1 - u0) * rng.gauss(1, 0.03)
        bh = (v1 - v0) * rng.gauss(1, 0.03)
        cu = (u0 + u1) / 2 + rng.gauss(0, 1)
        cv = (v0 + v1) / 2 + rng.gauss(0, 1)
        conf = max(0.3, min(0.97, 0.9 - 0.008 * z + rng.gauss(0, 0.05)))
        out.append({"cls": o.cls, "conf": round(conf, 2),
                    "box": [round(cu - bw / 2, 1), round(cv - bh / 2, 1), round(bw, 1), round(bh, 1)]})
    return out


# ---------------------------------------------------------------- scenarios
def overtake(cls, gap, start_y=-40.0, rel_v=5.0, rider_v=5.0, dark=False, oid=1):
    return Obj(oid, cls, straight(-(gap + DIMS[cls][0] / 2), start_y, rider_v + rel_v), dark=dark)


def lean_profile(t):
    if t < 3:
        return 0.0
    if t < 4:
        return 15.0 * (t - 3)
    if t < 7:
        return 15.0
    return max(0.0, 15.0 * (8 - t))


def scenarios():
    S = []

    def add(name, desc, duration, objs, expect, speed=5.0, **kw):
        S.append(dict(name=name, desc=desc, duration=duration, objs=objs, expect=expect, speed=speed, **kw))

    add("clear_road", "Empty road. Only the rider's own legs and seat tube are in the scan.", 10, [],
        {"peak_max": 0, "basis": "design: rider's legs must not raise a warning"})
    add("lean_turn", "Empty road, bike leans 15 degrees right for 3 s so the scan plane hits the road.", 10, [],
        {"peak_max": 0, "basis": "design: far points are dropped while leaning"}, lean=lean_profile)
    add("overtake_wide_2.0m", "Car overtakes on the left with a 2.0 m gap.", 12, [overtake("car", 2.0)],
        {"peak_min": 1, "peak_max": 1, "dir": "left", "pass_gap_m": 2.0,
         "basis": "design: APPROACHING when closing > 2 m/s within 30 m; no close pass at 2.0 m"})
    add("overtake_1.2m", "Car overtakes on the left with a 1.2 m gap.", 12, [overtake("car", 1.2)],
        {"peak_min": 2, "dir": "left", "pass_gap_m": 1.2, "basis": "design: CAUTION when a car will pass closer than 1.5 m"})
    add("overtake_close_0.8m", "Car overtakes on the left with a 0.8 m gap.", 12, [overtake("car", 0.8)],
        {"peak_min": 2, "dir": "left", "pass_gap_m": 0.8, "basis": "design: CAUTION when a car will pass closer than 1.5 m"})
    add("truck_overtake_1.2m", "Truck overtakes on the left with a 1.2 m gap.", 13,
        [overtake("truck", 1.2, start_y=-45.0)],
        {"peak_min": 2, "dir": "left", "pass_gap_m": 1.2, "basis": "design: trucks and buses warn 1.3x earlier"})
    add("black_car_overtake_1.0m", "Black car overtakes at 1.0 m. The LiDAR only sees it inside 6 m; the camera sees it earlier.",
        12, [overtake("car", 1.0, dark=True)],
        {"peak_min": 2, "dir": "left", "pass_gap_m": 1.0, "basis": "design: camera tracks far cars, LiDAR confirms close in"})
    add("sun_washout_overtake_1.0m", "Car overtakes at 1.0 m in bright sun: LiDAR range cut to 4 m with 30% dropouts.",
        12, [overtake("car", 1.0)],
        {"peak_min": 2, "dir": "left", "pass_gap_m": 1.0, "basis": "design: cameras still cover range in bright sun"},
        lidar_max=4.0, lidar_drop=0.30)
    add("lidar_dropout_overtake_1.0m", "Car overtakes at 1.0 m while the LiDAR is silent from 5 s to 9 s.",
        12, [overtake("car", 1.0)],
        {"peak_min": 2, "dir": "left", "sensor_flag": "lidar", "basis": "design: if the LiDAR stops, cameras keep warning"},
        lidar_off=(5.0, 9.0))
    add("camera_dropout_overtake_1.0m", "Car overtakes at 1.0 m while the rear camera is silent from 2 s to 9 s.",
        12, [overtake("car", 1.0)],
        {"peak_min": 2, "dir": "left", "pass_gap_m": 1.0, "sensor_flag": "rear_camera",
         "basis": "design: if the rear camera stops, the LiDAR keeps warning"},
        rear_off=(2.0, 9.0))
    add("rear_collision", "Car directly behind closes at 6 m/s and does not move over.", 9,
        [Obj(1, "car", straight(0.0, -40.0, 11.0))],
        {"peak_min": 3, "dir": "rear", "danger_lead_s_min": 1.0, "caution_lead_s_min": 2.5,
         "basis": "design pass threshold: DANGER >= 1.0 s and first CAUTION >= 2.5 s before contact"})
    add("stopped_car_ahead", "Car stopped in the lane ahead; the rider does not brake.", 10,
        [Obj(1, "car", static(0.0, 45.0))],
        {"peak_min": 3, "dir": "front", "danger_lead_s_min": 1.0, "caution_lead_s_min": 2.5,
         "basis": "design pass threshold: DANGER >= 1.0 s and first CAUTION >= 2.5 s before contact"})
    add("door_zone_parked", "Row of parked cars 1.0 m to the right. No door opens.", 12,
        parked_row(1, 1.0, 10.0, 8, dark_every=3),
        {"peak_min": 2, "peak_max": 2, "dir": "right",
         "basis": "design: CAUTION when a parked car within 1.5 m puts the rider in the door zone"})
    add("parked_far_3.0m", "Row of parked cars 3.0 m to the right, outside the door zone.", 12,
        parked_row(1, 3.0, 10.0, 8),
        {"peak_max": 1, "basis": "design: door-zone CAUTION only inside 1.5 m"})
    add("door_opens", "Parked cars 1.0 m to the right; a door swings into the rider's path 11 m ahead.", 9,
        parked_row(1, 1.0, 10.0, 8) + [Obj(20, "door", door_pose(1.0, 32.6, (32.6 - 11.0) / 5.0, 0.7, 70.0), camera=False)],
        {"peak_min": 3, "dir": "right", "danger_lead_s_min": 1.0,
         "basis": "design: DANGER when anything solid is in the path covered in the next 1.5 s"})
    add("right_hook", "Car overtakes on the left at 1.8 m, then turns right across the rider's path, missing by about 0.7 m.", 12,
        [Obj(1, "car", right_hook_path(-2.7, -25.0, 10.0, 3.0, 5.0, 5.0, 5.0, 12.0))],
        {"peak_min": 3, "dir": "front", "basis": "design: DANGER when anything solid is in the path covered in the next 1.5 s"})
    add("cyclist_overtake_1.0m", "Another cyclist overtakes on the left at 1.0 m, 2.5 m/s faster.", 12,
        [Obj(1, "bicycle", straight(-(1.0 + 0.3), -15.0, 7.5))],
        {"peak_min": 1, "dir": "left", "pass_gap_m": 1.0, "basis": "informational: the design gives no rule for cyclists"})
    add("table_demo_walkers", "Bike stationary on a stand. One person walks past on the left at 1.0 m, one on the right at 1.2 m, one walks up from behind and stops.",
        14, [Obj(1, "person", straight(-1.25, -6.0, 1.3)),
             Obj(2, "person", lambda t: (1.45, 8.0 - 1.3 * max(0.0, t - 3.0), 0.0)),
             Obj(3, "person", lambda t: (0.0, min(-0.9, -9.0 + 1.3 * max(0.0, t - 6.0)), 0.0))],
        {"peak_min": 1, "basis": "informational: the live table demo; the design gives no rule for pedestrians"},
        speed=0.0)
    return S


# ---------------------------------------------------------------- run
def in_window(t, win):
    return win is not None and win[0] <= t < win[1]


def run(sc, seed):
    rng = random.Random(seed * 1000 + sum(map(ord, sc["name"])))
    v = sc["speed"]
    lean_fn = sc.get("lean", lambda t: 0.0)
    frames, summary = [], {}
    n = int(round(sc["duration"] / DT))
    contact_t = None
    for i in range(n):
        t = round(i * DT, 3)
        rider_y = v * t
        rel = [(o, o.rel(t, rider_y)) for o in sc["objs"]]
        truth = []
        for o, (x, y, yaw) in rel:
            gap = o.outline_gap(x, y, yaw)
            x2, y2, _ = o.rel(t + 0.05, v * (t + 0.05))
            x1, y1, _ = o.rel(t - 0.05, v * (t - 0.05))
            truth.append({"id": o.id, "cls": o.cls, "x": round(x, 2), "y": round(y, 2), "w": o.w, "l": o.l,
                          "yaw": round(yaw, 1), "vx": round((x2 - x1) / 0.1, 2), "vy": round((y2 - y1) / 0.1, 2),
                          "gap": round(gap, 2)})
            s = summary.setdefault(o.id, {"id": o.id, "cls": o.cls, "min_gap_m": 1e9, "min_gap_t": 0.0})
            if gap < s["min_gap_m"]:
                s["min_gap_m"], s["min_gap_t"] = round(gap, 2), t
            if gap < CONTACT_GAP_M and contact_t is None:
                contact_t = t
        lean = lean_fn(t)
        lidar = None
        if not in_window(t, sc.get("lidar_off")):
            ranges = lidar_scan(rel, t, lean, sc.get("lidar_max", LIDAR_MAX_M), sc.get("lidar_drop", LIDAR_DROP_P), rng)
            lidar = base64.b64encode(lidar_packet(i, t, ranges)).decode()
        rear = None if in_window(t, sc.get("rear_off")) else camera_boxes(rel, REAR_CAM, -1, rng)
        front = None if in_window(t, sc.get("front_off")) else camera_boxes(rel, FRONT_CAM, +1, rng)
        hd = math.radians(HEADING_DEG)
        frames.append({"type": "frame", "i": i, "t": t, "lidar": lidar, "rear": rear, "front": front,
                       "phone": {"speed": round(max(0.0, v + (rng.gauss(0, 0.2) if v else 0.0)), 2),
                                 "heading": round(HEADING_DEG + rng.gauss(0, 1.0), 1),
                                 "lean": round(lean + rng.gauss(0, 0.5), 1),
                                 "lat": round(START_LAT + rider_y * math.cos(hd) / 111320.0, 7),
                                 "lon": round(START_LON + rider_y * math.sin(hd) / (111320.0 * math.cos(math.radians(START_LAT))), 7)},
                       "truth": truth})
        if contact_t is not None and t >= contact_t + 0.5:
            break
    meta = {"type": "meta", "format": "ride-log", "version": 1, "synthetic": True, "name": sc["name"],
            "description": sc["desc"], "seed": seed, "dt": DT, "frames": len(frames), "rider_speed_mps": v,
            "lidar": {"packet": "qtpy_lidar_udp v1, base64", "zero_deg": "forward", "clockwise": True,
                      "height_m": LIDAR_HEIGHT_M},
            "rear_cam": dict(REAR_CAM, mirrored=False), "front_cam": FRONT_CAM,
            "levels": ["CLEAR", "APPROACHING", "CAUTION", "DANGER"],
            "expect": sc["expect"],
            "truth_summary": {"contact_t": contact_t, "objects": list(summary.values())}}
    return meta, frames


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "rides"))
    ap.add_argument("--seeds", type=int, default=1)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    index = []
    for sc in scenarios():
        for seed in range(a.seeds):
            meta, frames = run(sc, seed)
            fname = sc["name"] + (f"_s{seed}" if a.seeds > 1 else "") + ".jsonl"
            with open(os.path.join(a.out, fname), "w") as fh:
                fh.write(json.dumps(meta) + "\n")
                for fr in frames:
                    fh.write(json.dumps(fr, separators=(",", ":")) + "\n")
            index.append({"file": fname, "name": meta["name"], "description": meta["description"], "seed": seed,
                          "frames": meta["frames"], "expect": meta["expect"], "truth_summary": meta["truth_summary"]})
            print(f"{fname:40s} {meta['frames']:4d} frames  contact_t={meta['truth_summary']['contact_t']}")
    with open(os.path.join(a.out, "index.json"), "w") as fh:
        json.dump(index, fh, indent=1)


if __name__ == "__main__":
    main()

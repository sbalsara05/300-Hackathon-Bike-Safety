# Synthetic rides

Stand-in data for building and testing the app before the hardware and the Python prototype are available.
**Everything in `rides/` is generated, not recorded.** It shows whether the app's rules behave as designed on clean inputs; it says nothing about false alarms on a real street.

## Use

```bash
python3 sim/generate_rides.py              # rewrite sim/rides/ (add --seeds 5 for five noise seeds each)
```

```bash
python3 sim/play_ride_udp.py sim/rides/overtake_close_0.8m.jsonl --host 172.20.10.1 --loop
```

The second command is a fake bike unit: it sends the ride's LiDAR packets over Wi-Fi exactly as `qtpy_lidar_udp` would (port 5005). Leave out `--host` to send to this Mac, for the iOS Simulator.

## Ride file format (`ride-log` v1)

One JSON object per line. Line 1 is the header, every other line is one 0.1 s frame.

Header (`"type": "meta"`): `name`, `description`, `seed`, `dt`, `rider_speed_mps`, camera sizes and fields of view, plus:

- `expect`: what the warning rules should do. `peak_min` / `peak_max` are warning levels (0 CLEAR, 1 APPROACHING, 2 CAUTION, 3 DANGER), `dir` is the helmet direction, `danger_lead_s_min` / `caution_lead_s_min` are seconds before `truth_summary.contact_t`, `pass_gap_m` is the true passing gap (the design allows ±0.15 m), `sensor_flag` is the sensor the app should report as lost. `basis` says which design-doc rule the expectation comes from; "informational" means the design gives no rule.
- `truth_summary`: `contact_t` (first time anything is within 0.4 m of the rider, or null) and each object's closest approach.

Frame (`"type": "frame"`):

| Field | Content |
|---|---|
| `t` | Seconds from ride start |
| `lidar` | Base64 of the exact 730-byte firmware packet (`HL`, version, flags, seq, t_ms, 360 × uint16 mm; index = degrees clockwise from forward; 0 = no return). `null` = LiDAR silent |
| `rear`, `front` | Camera detections as a YOLO model would give them: `cls`, `conf`, `box` = x, y, width, height in pixels. `null` = camera silent. Rear boxes are not mirrored: the rider's left is the image's right |
| `phone` | `speed` (m/s), `heading` and `lean` (degrees, lean positive = right), `lat`, `lon` |
| `truth` | Where each object really is: centre `x`, `y` (m, rider at 0,0, x right, y forward), `w`, `l`, `yaw`, relative `vx`, `vy`, and `gap` to its outline. For tests only; the app must not read it when deciding warnings |

## Assumptions to replace with measurements

- LiDAR at the rider's position, 1 m high, 0° pointing forward; 2 cm range noise, 2% dropouts; 12 m range, 6 m on black cars.
- The rider's legs and seat tube appear 0.2–0.6 m ahead of the LiDAR within about ±35°.
- Rear camera 320×240 with a 62° field of view; front camera 640×480 with 68°. Detections miss more often with distance. Doors are never detected by the cameras.
- The rider holds a straight line at constant speed and never brakes. The lean scenario tilts the scan without turning the bike.

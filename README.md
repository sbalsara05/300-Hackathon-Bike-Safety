# 300-Hackathon-Bike-Safety
Companion app for bike safety tool for 300 Hackathon

## App

`ios/HelmetHUD.xcodeproj` is the iPhone app. Its code lives in the `ios/HelmetCore` package: `HelmetCore` holds the warning rules and data formats, `HelmetApp` holds the screens, the Bluetooth link to the helmet and the LiDAR receiver.

Check the rules against every synthetic ride, from `ios/HelmetCore`:

```bash
swift run ridecheck
```

Watch the live warning level from LiDAR packets arriving on this Mac (pair it with the fake bike unit below):

```bash
swift run livecheck
```

## The 18 test rides

`swift run ridecheck` plays each ride through the warning rules and compares the result with what the design doc says should happen. Every overtake also has its measured passing gap checked to within 0.15 m.

| # | Ride | What happens | Must do |
|---|---|---|---|
| 1 | `clear_road` | Empty road; only the rider's own legs are in the scan | Stay CLEAR |
| 2 | `lean_turn` | Empty road; bike leans 15° so the scan hits the road | Stay CLEAR |
| 3 | `parked_far_3.0m` | Parked cars 3.0 m to the right | No CAUTION or DANGER |
| 4 | `overtake_wide_2.0m` | Car passes on the left at 2.0 m | APPROACHING only |
| 5 | `overtake_1.2m` | Car passes on the left at 1.2 m | CAUTION, left |
| 6 | `overtake_close_0.8m` | Car passes on the left at 0.8 m | CAUTION, left |
| 7 | `truck_overtake_1.2m` | Truck passes on the left at 1.2 m | CAUTION, left |
| 8 | `cyclist_overtake_1.0m` | Another cyclist passes on the left at 1.0 m | At least APPROACHING |
| 9 | `black_car_overtake_1.0m` | Black car at 1.0 m; LiDAR only sees it inside 6 m | CAUTION, left |
| 10 | `sun_washout_overtake_1.0m` | Car at 1.0 m in bright sun; LiDAR cut to 4 m | CAUTION, left |
| 11 | `lidar_dropout_overtake_1.0m` | Car at 1.0 m while the LiDAR is silent | CAUTION, left; report LiDAR lost |
| 12 | `camera_dropout_overtake_1.0m` | Car at 1.0 m while the rear camera is silent | CAUTION, left; report camera lost |
| 13 | `rear_collision` | Car closes from directly behind and doesn't move over | CAUTION 2.5 s and DANGER 1.0 s before contact, rear |
| 14 | `stopped_car_ahead` | Car stopped in the lane ahead | CAUTION 2.5 s and DANGER 1.0 s before contact, front |
| 15 | `right_hook` | Car overtakes, then turns right across the rider's path | DANGER, front |
| 16 | `door_zone_parked` | Parked cars 1.0 m to the right, no door opens | CAUTION, right; never DANGER |
| 17 | `door_opens` | A parked car's door swings into the rider's path | DANGER 1.0 s before contact, right |
| 18 | `table_demo_walkers` | Bike on a stand; three people walk past | At least APPROACHING |

**Current result: 17 of 18 pass.** `door_opens` fails: in the simulation the rider's own body hides a door ahead from the rear-mounted LiDAR until about 0.4 s before contact, so only the door-zone CAUTION fires. This rests on an assumed body shape and needs checking on the real bike.

Rides 8 and 18 have loose targets because the design doc gives no rule for cyclists or pedestrians.

## Test data

`sim/` holds 18 synthetic rides for building and testing the app before the hardware arrives: overtakes, a door opening, a right hook, sensor dropouts and more. Each ride has LiDAR scans in the bike unit's exact packet format, camera detections, phone readings and the expected warning.

The rides are generated, not recorded. They check that the warning rules behave as designed; they are not evidence of real-world performance.

```bash
python3 sim/generate_rides.py
```

```bash
python3 sim/play_ride_udp.py sim/rides/overtake_close_0.8m.jsonl --loop
```

The first command regenerates the rides; the second plays one over Wi-Fi as a fake bike unit. See [sim/README.md](sim/README.md) for the file format and the assumptions behind the data.

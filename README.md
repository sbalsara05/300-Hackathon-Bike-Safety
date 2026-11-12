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

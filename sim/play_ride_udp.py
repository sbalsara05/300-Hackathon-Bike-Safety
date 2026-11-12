#!/usr/bin/env python3
"""Fake bike unit: play a ride file's LiDAR packets over UDP, exactly as qtpy_lidar_udp would.

    python3 sim/play_ride_udp.py sim/rides/overtake_close_0.8m.jsonl --host 172.20.10.1 --loop

--host is the phone's address (an iPhone hotspot is normally 172.20.10.1). Default is this machine,
which works with firmware/tools/udp_lidar_listen.py and with the iOS Simulator.
"""
import argparse
import base64
import json
import socket
import time

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("ride")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=5005)
ap.add_argument("--loop", action="store_true")
a = ap.parse_args()

lines = [json.loads(line) for line in open(a.ride)]
meta, frames = lines[0], lines[1:]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"{meta['name']}: {len(frames)} frames to {a.host}:{a.port}")
while True:
    t0 = time.time()
    for fr in frames:
        if fr["lidar"]:                       # null = LiDAR silent in this scenario
            sock.sendto(base64.b64decode(fr["lidar"]), (a.host, a.port))
        delay = t0 + fr["t"] + meta["dt"] - time.time()
        if delay > 0:
            time.sleep(delay)
    if not a.loop:
        break

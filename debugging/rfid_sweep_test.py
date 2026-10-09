#!/usr/bin/env python3
"""
rfid_sweep_test.py — does the reader catch a tag that passes by quickly?
(That is what matters on a driving AGV, not how often a held tag repeats.)

Stop the controller first:  sudo systemctl stop agv-controller
Run:                        python3 debugging/rfid_sweep_test.py [seconds]
Sweep ONE tag under the antenna about as fast as the AGV drives, pause ~1 s,
sweep again... (e.g. 10 sweeps in 30 s). Every sweep the reader catches
prints "CAUGHT #n". Compare the count with how many sweeps you made.
"""
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from drivers.rfid_tcp import parse_frames, tag_from_frame   # same parser as the controller

READER = ("192.168.3.200", 2022)
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
NEW_PASS_GAP = 0.7      # a read after this much silence counts as a new sweep

s = socket.create_connection(READER, timeout=0.5)
s.sendall(bytes.fromhex("CFFF0070002415"))
print(f"Connected. Sweep a tag under the antenna, pause ~1 s, repeat — {DURATION:.0f} s...\n")

buf, t0, last, caught, reads_in_pass = bytearray(), time.time(), None, 0, 0
while time.time() - t0 < DURATION:
    try:
        data = s.recv(1024)
    except socket.timeout:
        continue
    if not data:
        break
    buf.extend(data)
    for f in parse_frames(buf):
        tag = tag_from_frame(f)
        if tag is None:
            continue
        now = time.time()
        rssi = int.from_bytes(f[6:8], "big", signed=True) / 10
        if last is None or now - last > NEW_PASS_GAP:
            caught += 1
            print(f"{now - t0:7.2f}s  CAUGHT #{caught}  tag {tag}  {rssi:.0f} dBm")
        last = now
s.close()
print(f"\nCaught {caught} sweep(s) in {DURATION:.0f} s. "
      "If that equals the number of sweeps you made, the reader is fast enough.")

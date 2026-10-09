#!/usr/bin/env python3
"""
rfid_rate_test.py — how fast does the reader report a tag?

Stop the controller first:  sudo systemctl stop agv-controller
Run:                        python3 debugging/rfid_rate_test.py [seconds]
Hold ONE tag still under the antenna for the whole test (default 15 s).
Prints every read and a summary: reads per second and the longest gap.
Good result: 10+ reads/s and no gap over ~0.2 s.
"""
import socket
import sys
import time

sys.path.insert(0, __import__("os").path.join(__import__("os").path.dirname(__file__), ".."))
from drivers.rfid_tcp import parse_frames, tag_from_frame   # same parser as the controller

READER = ("192.168.3.200", 2022)
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

s = socket.create_connection(READER, timeout=1)
s.sendall(bytes.fromhex("CFFF0070002415"))   # same start command as the controller
print(f"Connected. Hold a tag under the antenna — measuring for {DURATION:.0f} s...\n")

buf, t0, last, times, tags = bytearray(), time.time(), None, [], {}
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
        gap = "" if last is None else f"  (+{now - last:.3f}s)"
        rssi = int.from_bytes(f[6:8], "big", signed=True) / 10
        print(f"{now - t0:7.3f}s  tag {tag}  {rssi:6.1f} dBm{gap}")
        times.append(now); tags[tag] = tags.get(tag, 0) + 1; last = now
s.close()

print("\n── Summary ─────────────────────────────")
if len(times) < 2:
    print(f"Only {len(times)} read(s) in {DURATION:.0f} s — the reader is barely reporting.")
else:
    gaps = [b - a for a, b in zip(times, times[1:])]
    span = times[-1] - times[0]
    print(f"Reads: {len(times)}  ({len(times) / DURATION:.1f} per second)   tags: {tags}")
    print(f"Longest gap: {max(gaps):.3f} s   average gap: {sum(gaps) / len(gaps):.3f} s")
    print("GOOD" if max(gaps) < 0.3 and len(times) / DURATION >= 5 else
          "SLOW — the AGV can drive past a tag inside one of these gaps")

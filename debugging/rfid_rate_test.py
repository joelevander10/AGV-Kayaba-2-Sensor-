#!/usr/bin/env python3
"""
rfid_rate_test.py — how fast does the reader report a tag?

Standalone — copy just this file to any AGV PC.
Stop the controller first:  sudo systemctl stop agv-controller
Run:                        python3 debugging/rfid_rate_test.py [seconds]
Hold ONE tag still under the antenna for the whole test (default 15 s).
Prints every read and a summary: reads per second and the longest gap.
Good result: 10+ reads/s and no gap over ~0.2 s.
"""
import socket
import sys
import time

# ── Frame parser (self-contained copy of drivers/rfid_tcp.py, so this script
#    also runs on another AGV's PC with no project files) ──────────────────
import binascii as _ba
def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc
def _frame_at(buf, i):
    if len(buf) - i < 5:
        return 0
    total = 7 + buf[i + 4]
    if len(buf) - i < total:
        return 0
    fr = buf[i:i + total]
    return total if _crc16(fr[:-2]) == (fr[-2] << 8 | fr[-1]) else -1
def parse_frames(buf):
    out = []
    while True:
        i = buf.find(0xCF)
        if i < 0:
            buf.clear(); break
        if i:
            del buf[:i]
        n = _frame_at(buf, 0)
        if n > 0:
            out.append(bytes(buf[:n])); del buf[:n]; continue
        if n < 0:
            del buf[:1]; continue
        j, skipped = buf.find(0xCF, 1), False
        while j > 0:
            if _frame_at(buf, j) > 0:
                del buf[:j]; skipped = True; break
            j = buf.find(0xCF, j + 1)
        if not skipped:
            if len(buf) > 4096:
                buf.clear()
            break
    return out
def tag_from_frame(fr):
    if len(fr) < 17 or (fr[2] << 8 | fr[3]) != 0x0001 or fr[5] != 0:
        return None
    return _ba.hexlify(fr[13:15]).decode().upper()

READER = ("192.168.3.200", 2022)
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

s = socket.create_connection(READER, timeout=1)
s.sendall(bytes.fromhex("CFFF0070002415"))   # same start command as the controller
print(f"Connected. Hold a tag under the antenna — measuring for {DURATION:.0f} s...\n")

buf, t0, last, times, tags = bytearray(), time.time(), None, [], {}
rx_bytes, frame_bytes, other_frames = 0, 0, 0
while time.time() - t0 < DURATION:
    try:
        data = s.recv(1024)
    except socket.timeout:
        continue
    if not data:
        break
    buf.extend(data)
    rx_bytes += len(data)
    for f in parse_frames(buf):
        frame_bytes += len(f)
        tag = tag_from_frame(f)
        if tag is None:
            other_frames += 1
            continue
        now = time.time()
        gap = "" if last is None else f"  (+{now - last:.3f}s)"
        rssi = int.from_bytes(f[6:8], "big", signed=True) / 10
        print(f"{now - t0:7.3f}s  tag {tag}  {rssi:6.1f} dBm{gap}")
        times.append(now); tags[tag] = tags.get(tag, 0) + 1; last = now
s.close()

print("\n── Summary ─────────────────────────────")
junk = rx_bytes - frame_bytes
print(f"Bytes received: {rx_bytes}   in valid frames: {frame_bytes}   "
      f"corrupted/unusable: {junk} ({100 * junk / max(rx_bytes, 1):.0f}%)   other frames: {other_frames}")
if junk > 0.1 * max(rx_bytes, 1):
    print("WARNING: many corrupted bytes — reads are being lost on the reader→network link "
          "(RS232 converter, cable, baud rate or interference), not in the reader.")
if len(times) < 2:
    print(f"Only {len(times)} read(s) in {DURATION:.0f} s — the reader is barely reporting.")
else:
    gaps = [b - a for a, b in zip(times, times[1:])]
    span = times[-1] - times[0]
    print(f"Reads: {len(times)}  ({len(times) / DURATION:.1f} per second)   tags: {tags}")
    print(f"Longest gap: {max(gaps):.3f} s   average gap: {sum(gaps) / len(gaps):.3f} s")
    print("GOOD" if max(gaps) < 0.3 and len(times) / DURATION >= 5 else
          "SLOW — the AGV can drive past a tag inside one of these gaps")

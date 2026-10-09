#!/usr/bin/env python3
"""
rfid_sweep_test.py — does the reader catch a tag that passes by quickly?
(That is what matters on a driving AGV, not how often a held tag repeats.)

Standalone — copy just this file to any AGV PC.
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

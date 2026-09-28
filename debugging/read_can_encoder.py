"""
read_can_encoder.py
===================
Reads absolute position and angular velocity from a CALT CAX60 encoder
over CANopen (CiA DS-406) using python-can with a slcan USB adapter.

Hardware
--------
  PC (CANable v2.0 Pro, slcan firmware) ── CALT CAX60
  COM12, 125 kbit/s, 120 Ω termination at each end

CANopen framing used here
-------------------------
  NMT (Network Management)
    COB-ID 0x000  data [0x01, 0x00]  →  Start Remote Node (broadcast)
    Devices boot into Pre-Operational — PDOs are silent until you send this.

  TPDO1 (Transmit Process Data Object 1) from encoder
    COB-ID  0x180 + node_id  =  0x181   (node_id = 0x01)
    Payload  4 bytes, little-endian UINT32  =  absolute position in counts
    Rate     set by the encoder's event timer (default ~10 ms)

  Absolute position
    Range   0 … COUNTS_PER_REV-1, wraps on overflow
    12-bit encoder → 4096 counts/revolution (verify via SDO object 0x6501)

Velocity calculation
--------------------
  v(rad/s) = Δangle_rad / Δt
  Δangle is rollover-corrected: if raw jump > half a revolution,
  a wrap occurred — subtract or add one full revolution to the delta.
  rpm = v * 60 / (2π)
"""

import struct
import time
import can  # pip install python-can

# ── configuration ──────────────────────────────────────────────────────────────
PORT           = "/dev/ttyACM0"
BITRATE        = 125_000
NODE_ID        = 0x01
TPDO1_COB_ID   = 0x180 + NODE_ID        # 0x181
COUNTS_PER_REV = 4096                   # 12-bit encoder; read 0x6501 to verify
RAD_PER_COUNT  = 2.0 * 3.141592653589793 / COUNTS_PER_REV
HALF_REV       = COUNTS_PER_REV / 2.0

# ── open bus ───────────────────────────────────────────────────────────────────
bus = can.interface.Bus(
    interface        = "slcan",
    channel          = PORT,
    bitrate          = BITRATE,
    sleep_after_open = 2.0,     # slcan firmware needs ~2 s after serial open
)

# ── NMT: move all nodes from Pre-Operational → Operational ────────────────────
# Without this the encoder never sends TPDO1.
# COB-ID 0x000, data[0]=0x01 (start), data[1]=0x00 (all nodes)
bus.send(can.Message(
    arbitration_id = 0x000,
    data           = [0x01, 0x00],
    is_extended_id = False,
))
time.sleep(0.1)   # give nodes time to transition

print(f"Listening on {PORT} @ {BITRATE//1000} kbit/s  (Ctrl+C to stop)\n")
print(f"{'Position (counts)':>18}  {'Angle (°)':>10}  {'Speed (rad/s)':>14}  {'Speed (RPM)':>12}")
print("-" * 62)

# ── velocity state ─────────────────────────────────────────────────────────────
prev_count = None
prev_time  = None
cum_angle  = 0.0   # unwrapped cumulative angle in radians

# ── main loop ─────────────────────────────────────────────────────────────────
try:
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            print("  (no message — check wiring / NMT state)")
            continue

        # ignore frames that are not our encoder's TPDO1
        if msg.arbitration_id != TPDO1_COB_ID:
            continue
        if len(msg.data) < 4:
            continue

        now = time.perf_counter()

        # decode payload: 4-byte little-endian unsigned int = position in counts
        count = struct.unpack_from("<I", msg.data, 0)[0]

        # ── rollover-aware velocity ────────────────────────────────────────────
        if prev_count is None:
            # first sample: initialise, no velocity yet
            cum_angle  = count * RAD_PER_COUNT
            prev_count = count
            prev_time  = now
            omega      = None
        else:
            dt = now - prev_time

            # raw count delta — may cross the 0/4095 boundary
            delta = count - prev_count

            # if the jump is larger than half a revolution, a rollover happened
            if delta >  HALF_REV:
                delta -= COUNTS_PER_REV   # e.g. jumped from 4090 → 5 CW
            elif delta < -HALF_REV:
                delta += COUNTS_PER_REV   # e.g. jumped from 5 → 4090 CCW

            cum_angle += delta * RAD_PER_COUNT
            prev_count = count
            prev_time  = now

            omega = (delta * RAD_PER_COUNT) / dt if dt > 0 else 0.0

        # ── display ───────────────────────────────────────────────────────────
        angle_deg = (cum_angle * 180.0) / 3.141592653589793

        if omega is None:
            print(f"{count:>18}  {angle_deg:>10.2f}  {'---':>14}  {'---':>12}")
        else:
            rpm = omega * 60.0 / (2.0 * 3.141592653589793)
            print(f"{count:>18}  {angle_deg:>10.2f}  {omega:>+14.4f}  {rpm:>+12.3f}")

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    bus.shutdown()

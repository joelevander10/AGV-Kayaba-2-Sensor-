"""
verify_wheel_speed.py
=====================
Manual bench tool — verify ACTUAL AGV wheel ground speed against the commanded
speed, using a CALT CAX60 absolute encoder coupled to the AGV wheel by a small
friction wheel (image: encoder wheel rim pressed against the AGV wheel rim).

Run it yourself:  python3 debugging/verify_wheel_speed.py

Procedure
---------
  1. Mount the encoder friction wheel against the RIGHT AGV wheel.
  2. Start this script.  It prints live readings and records every sample.
  3. Drive the AGV straight, stepping the commanded speed 0 → 0.8 m/s in
     0.02 m/s increments, holding each step ~3-5 s.
  4. Ctrl+C to stop — a CSV and a PNG plot are written to OUT_DIR.
  5. Re-mount on the LEFT wheel and repeat.

Coupling geometry  (why we trust v = omega_enc * r_enc)
-------------------------------------------------------
  The encoder wheel and the AGV wheel are in rolling contact, so their rim
  (tangential) speeds match at the contact point:
        omega_enc * r_enc  =  omega_agv * r_agv  =  v_ground
  The AGV wheel rolls on the floor, so its rim speed IS the ground speed.
  Therefore the AGV linear ground speed equals the encoder RIM speed:
        v_agv = omega_enc * r_enc          (r_enc = 30 mm)
  The AGV wheel diameter (180 mm) does NOT enter the linear-speed calc; it is
  only used here to report the AGV wheel's own RPM for reference.

  Caveats (acknowledged "limited mounting"):
    * any slip at the friction contact under-reads speed,
    * the effective encoder rim diameter scales v linearly — measure it well,
    * external contact spins the encoder OPPOSITE the AGV wheel (sign only).

CANopen framing (CiA DS-406) — same as the reference reader
-----------------------------------------------------------
  NMT  0x000 [0x01,0x00]              start all nodes (Pre-Op -> Operational)
  TPDO1 0x180+node (=0x181)          4-byte LE UINT32 absolute position (counts)
  SDO upload req 0x600+node          read object dictionary (used to verify res.)
  SDO upload rsp 0x580+node
"""

import os
import csv
import math
import struct
import time
from datetime import datetime

import can  # pip install python-can

# ── CAN / encoder configuration ─────────────────────────────────────────────
PORT           = "/dev/ttyACM0"
BITRATE        = 125_000
NODE_ID        = 0x01
TPDO1_COB_ID   = 0x180 + NODE_ID          # 0x181
SDO_TX_COB_ID  = 0x600 + NODE_ID          # 0x601  (PC -> encoder)
SDO_RX_COB_ID  = 0x580 + NODE_ID          # 0x581  (encoder -> PC)

# Per debugging/encoder.eds this is a MULTI-TURN encoder: TPDO1 maps object
# 0x6004 (Position value, 24-bit) which accumulates across revolutions and wraps
# only at the TOTAL measuring range (~16.7M counts) — NOT every revolution.
# So counts-per-rev (0x6001) sizes the angle per count, while the rollover
# boundary is the total range (0x6002).  Both are read over SDO at startup;
# these are only fallbacks if the read fails.
COUNTS_PER_REV  = 4096                      # single-turn resolution (0x6001), confirmed at runtime
ROLLOVER_RANGE  = 0x1000000                 # 24-bit total range (0x6002); wrap boundary

# ── coupling geometry ───────────────────────────────────────────────────────
ENC_WHEEL_DIAMETER_M = 0.060               # encoder friction wheel
AGV_WHEEL_DIAMETER_M = 0.180               # AGV drive wheel (reference only)
ENC_RADIUS_M = ENC_WHEEL_DIAMETER_M / 2.0  # 0.030 m -> the only term in v_agv
AGV_RADIUS_M = AGV_WHEEL_DIAMETER_M / 2.0

# Flip so that DRIVING FORWARD reads a POSITIVE velocity.  Run forward once,
# look at the live sign, set this to -1 if forward comes out negative.
DIRECTION_SIGN = +1.0

# ── velocity filtering ──────────────────────────────────────────────────────
# Single-count quantization (0.088 deg) over a ~10 ms window is noisy at low
# speed, so we report BOTH the raw per-sample velocity and an EMA-filtered one.
EMA_ALPHA = 0.30                           # 0..1, higher = less smoothing

# ── commanded-speed reference overlay (optional) ────────────────────────────
# The script cannot command the motors; it just draws the staircase you intend
# to drive so you can eyeball actual-vs-commanded. Timing starts when you press
# ENTER (a banner tells you when) — start driving at that moment.
OVERLAY_COMMAND = True
CMD_V_START = 0.00                         # m/s
CMD_V_STEP  = 0.02                         # m/s per step
CMD_V_MAX   = 0.50                         # m/s
CMD_DWELL_S = 2.0                          # seconds held per step

# ── output ──────────────────────────────────────────────────────────────────
OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "analysis_plot"
)

# Derived from the constants above; read_encoder_geometry() overwrites these
# from the live device at startup (the SDO read is authoritative).
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV
HALF_RANGE    = ROLLOVER_RANGE / 2.0


# ── helpers ─────────────────────────────────────────────────────────────────
def commanded_speed(t_s: float) -> float:
    """Reference staircase: v_start, stepping by v_step every dwell, capped."""
    n = int(t_s // CMD_DWELL_S)
    return min(CMD_V_START + n * CMD_V_STEP, CMD_V_MAX)


def _sdo_read(bus, index, sub):
    """Expedited SDO upload → int, or None on abort/timeout."""
    bus.send(can.Message(
        arbitration_id=SDO_TX_COB_ID,
        data=[0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0],
        is_extended_id=False,
    ))
    deadline = time.time() + 0.5
    while time.time() < deadline:
        rsp = bus.recv(timeout=0.5)
        if rsp is None:
            return None
        if rsp.arbitration_id != SDO_RX_COB_ID:
            continue
        cmd = rsp.data[0]
        if cmd == 0x80:                                 # SDO abort
            return None
        n = 4 - ((cmd >> 2) & 0x03) if (cmd & 0x02) else 4
        return int.from_bytes(bytes(rsp.data[4:4 + n]), "little")
    return None


def read_encoder_geometry(bus) -> None:
    """Read counts-per-rev (0x6001) and total range (0x6002) from the device and
    update the global angle-per-count + rollover boundary.  Falls back to the
    configured constants if the device does not answer.  Never fatal."""
    global COUNTS_PER_REV, ROLLOVER_RANGE, RAD_PER_COUNT, HALF_RANGE
    try:
        cpr = _sdo_read(bus, 0x6001, 0x00)
        rng = _sdo_read(bus, 0x6002, 0x00)
        if cpr:
            COUNTS_PER_REV = cpr
            RAD_PER_COUNT  = 2.0 * math.pi / cpr
        if rng:
            # 0x6002 is the max value; the modulo range is one greater.
            ROLLOVER_RANGE = rng + 1
            HALF_RANGE     = ROLLOVER_RANGE / 2.0
        print(f"  counts/rev (0x6001)  : {cpr}  (using {COUNTS_PER_REV})")
        print(f"  total range (0x6002) : {rng}  -> wrap at {ROLLOVER_RANGE}")
        if cpr and rng:
            print(f"  -> {'MULTI-turn (~%d rev)' % (ROLLOVER_RANGE / cpr) if rng + 1 > 1.5 * cpr else 'single-turn'}")
    except Exception as exc:                            # noqa: BLE001
        print(f"  geometry read failed ({exc}) — using configured fallbacks")


# ── open bus + bring node operational ───────────────────────────────────────
bus = can.interface.Bus(
    interface="slcan", channel=PORT, bitrate=BITRATE, sleep_after_open=2.0,
)
bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00], is_extended_id=False))
time.sleep(0.1)

print(f"Encoder verify  |  {PORT} @ {BITRATE // 1000} kbit/s  node 0x{NODE_ID:02X}")
print(f"enc wheel d={ENC_WHEEL_DIAMETER_M * 1000:.0f} mm  "
      f"agv wheel d={AGV_WHEEL_DIAMETER_M * 1000:.0f} mm  "
      f"v_agv = omega_enc * {ENC_RADIUS_M:.3f} m")
print("Reading encoder geometry over SDO (best effort):")
read_encoder_geometry(bus)

input("\n>>> Press ENTER, then immediately start driving the AGV straight...\n")
print(f"{'t (s)':>7}  {'count':>6}  {'enc rpm':>9}  {'v_agv (m/s)':>11}  "
      f"{'v_filt':>8}  {'cmd':>6}")
print("-" * 60)

# ── recording state ─────────────────────────────────────────────────────────
rows          = []
prev_count    = None
prev_time     = None
v_filt        = 0.0
max_count     = 0
multiturn     = False
t0            = time.perf_counter()

try:
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            print("  (no message — check wiring / NMT state)")
            continue
        if msg.arbitration_id != TPDO1_COB_ID or len(msg.data) < 4:
            continue

        now   = time.perf_counter()
        t_s   = now - t0
        count = struct.unpack_from("<I", msg.data, 0)[0]

        max_count = max(max_count, count)
        if count >= COUNTS_PER_REV:
            multiturn = True

        if prev_count is None:
            prev_count, prev_time = count, now
            continue

        dt    = now - prev_time
        delta = count - prev_count
        # Wrap only at the TOTAL measuring range (multi-turn), not every rev.
        if   delta >  HALF_RANGE: delta -= 2.0 * HALF_RANGE
        elif delta < -HALF_RANGE: delta += 2.0 * HALF_RANGE
        prev_count, prev_time = count, now

        omega   = DIRECTION_SIGN * (delta * RAD_PER_COUNT) / dt if dt > 0 else 0.0
        enc_rpm = omega * 60.0 / (2.0 * math.pi)
        v_agv   = omega * ENC_RADIUS_M                     # = ground speed
        v_filt += EMA_ALPHA * (v_agv - v_filt)
        agv_rpm = (v_agv / AGV_RADIUS_M) * 60.0 / (2.0 * math.pi)
        v_cmd   = commanded_speed(t_s) if OVERLAY_COMMAND else ""

        rows.append({
            "t_s": round(t_s, 4), "raw_count": count, "delta_counts": delta,
            "dt_s": round(dt, 5), "omega_rad_s": round(omega, 5),
            "enc_rpm": round(enc_rpm, 4), "v_agv_ms": round(v_agv, 5),
            "v_agv_filt_ms": round(v_filt, 5), "agv_wheel_rpm": round(agv_rpm, 4),
            "v_cmd_ms": v_cmd,
        })

        cmd_str = f"{v_cmd:6.2f}" if OVERLAY_COMMAND else "    --"
        print(f"{t_s:7.2f}  {count:6d}  {enc_rpm:9.2f}  "
              f"{v_agv:11.4f}  {v_filt:8.4f}  {cmd_str}")

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    bus.shutdown()

    print("\n── resolution check ──")
    print(f"  max raw count seen : {max_count}")
    print(f"  count exceeded {COUNTS_PER_REV - 1}: "
          f"{'YES -> multi-turn' if multiturn else 'no -> single-turn (per-rev wrap)'}")

    if not rows:
        print("No samples recorded — nothing to save.")
    else:
        os.makedirs(OUT_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(OUT_DIR, f"{stamp}_wheelverify.csv")
        png_path = os.path.join(OUT_DIR, f"{stamp}_wheelverify.png")

        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nCSV saved : {csv_path}")

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            t      = [r["t_s"] for r in rows]
            v_raw  = [r["v_agv_ms"] for r in rows]
            v_sm   = [r["v_agv_filt_ms"] for r in rows]

            fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
            ax[0].plot(t, v_raw, color="0.7", lw=0.8, label="actual (raw)")
            ax[0].plot(t, v_sm, color="C0", lw=1.6, label="actual (EMA)")
            if OVERLAY_COMMAND:
                ax[0].plot(t, [commanded_speed(x) for x in t],
                           color="C3", lw=1.4, ls="--", label="commanded (ref)")
            ax[0].set_ylabel("v_agv (m/s)")
            ax[0].set_title(f"AGV wheel speed verification — {stamp}")
            ax[0].grid(True, alpha=0.3)
            ax[0].legend(loc="upper left")

            ax[1].plot(t, [r["raw_count"] for r in rows], color="C2", lw=0.8)
            ax[1].set_ylabel("raw count")
            ax[1].set_xlabel("time (s)")
            ax[1].grid(True, alpha=0.3)

            fig.tight_layout()
            fig.savefig(png_path, dpi=110)
            print(f"Plot saved: {png_path}")
        except Exception as exc:                          # noqa: BLE001
            print(f"Plot failed ({exc}) — CSV is still saved.")

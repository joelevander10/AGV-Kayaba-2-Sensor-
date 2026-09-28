"""
motor_test.py — Direct motor command for wiring verification and manual testing.

Usage:
    python3 motor_test.py <left_rpm>,<right_rpm>,<duration_s>

    left_rpm / right_rpm:
        positive  → forward
        negative  → reverse
        0         → stopped (direction signals off, zero voltage)

Examples:
    python3 motor_test.py 500,500,3       # both forward at 500 RPM for 3s
    python3 motor_test.py -300,300,2      # spin left in place for 2s
    python3 motor_test.py 0,0,1           # stop both for 1s (useful as a reset)
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymodbus.client import ModbusTcpClient
import config
from motion import rpm_to_voltage

# ── Channel map from profile ──────────────────────────────────────────────────
L = config.MOTOR_CHANNELS["left"]
R = config.MOTOR_CHANNELS["right"]

CH_L_FWD   = L["do_fwd"]
CH_L_REV   = L["do_rev"]
CH_L_BRK   = L["do_brake"]
CH_L_ALARM = L.get("do_alarm")
CH_R_FWD   = R["do_fwd"]
CH_R_REV   = R["do_rev"]
CH_R_BRK   = R["do_brake"]
CH_R_ALARM = R.get("do_alarm")

CH_AO_LEFT  = L["ao_speed"]
CH_AO_RIGHT = R["ao_speed"]


# ── Helpers ───────────────────────────────────────────────────────────────────

def rpm_to_volts(rpm: float) -> float:
    if rpm == 0:
        return 0.0
    return max(0.0, min(rpm_to_voltage(abs(rpm)), config.V_RANGE))

def volts_to_dac(v: float) -> int:
    return int(v / config.V_RANGE * config.DAC_RES)

def coil(client, channel, value):
    result = client.write_coil(
        address=config.DO_BASE + channel, value=bool(value), device_id=config.DEVICE_ID)
    state = "ON " if value else "OFF"
    status = "OK" if not result.isError() else "ERR"
    print(f"  DO{channel:02d} = {state}  [{status}]")

def ao(client, channel, volts):
    dac = volts_to_dac(volts)
    result = client.write_register(
        address=config.AO_BASE + channel, value=dac, device_id=config.DEVICE_ID)
    status = "OK" if not result.isError() else "ERR"
    print(f"  AO{channel}  = {volts:.3f}V (DAC={dac})  [{status}]")

def stop_all(dio, ao_client):
    print("\n[STOP] Zeroing outputs...")
    ao(ao_client, CH_AO_LEFT,  0.0)
    ao(ao_client, CH_AO_RIGHT, 0.0)
    for ch in [CH_L_FWD, CH_L_REV, CH_R_FWD, CH_R_REV]:
        coil(dio, ch, False)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 motor_test.py <left_rpm>,<right_rpm>,<duration_s>")
        print("  e.g. python3 motor_test.py 500,500,3")
        sys.exit(1)

    try:
        parts = sys.argv[1].split(",")
        left_rpm  = float(parts[0])
        right_rpm = float(parts[1])
        duration  = float(parts[2])
    except (ValueError, IndexError):
        print("Error: expected format left_rpm,right_rpm,duration  (e.g. 500,-300,2)")
        sys.exit(1)

    if duration <= 0:
        print("Error: duration must be > 0")
        sys.exit(1)

    left_v  = rpm_to_volts(left_rpm)
    right_v = rpm_to_volts(right_rpm)

    def dir_label(rpm):
        if rpm > 0:   return "FWD"
        if rpm < 0:   return "REV"
        return "STOP"

    print("=" * 52)
    print(f"  Left  : {left_rpm:+.0f} RPM  ({dir_label(left_rpm)})  {left_v:.3f}V")
    print(f"  Right : {right_rpm:+.0f} RPM  ({dir_label(right_rpm)})  {right_v:.3f}V")
    print(f"  Duration: {duration}s")
    print("=" * 52)

    dio = ModbusTcpClient(config.DIO_IP, port=config.MODBUS_PORT)
    if not dio.connect():
        sys.exit(f"FATAL: Could not connect to DIO at {config.DIO_IP}")

    ao_client = ModbusTcpClient(config.AO_IP, port=config.MODBUS_PORT)
    if not ao_client.connect():
        dio.close()
        sys.exit(f"FATAL: Could not connect to AO at {config.AO_IP}")

    try:
        # Clear everything first
        stop_all(dio, ao_client)
        time.sleep(0.1)

        # Set direction signals
        print("\n[DIR] Setting direction...")
        coil(dio, CH_L_FWD, left_rpm  > 0)
        coil(dio, CH_L_REV, left_rpm  < 0)
        coil(dio, CH_R_FWD, right_rpm > 0)
        coil(dio, CH_R_REV, right_rpm < 0)

        # Apply speed
        print("\n[RUN] Applying voltage...")
        ao(ao_client, CH_AO_LEFT,  left_v)
        ao(ao_client, CH_AO_RIGHT, right_v)

        print(f"\n  >>> RUNNING for {duration}s — Ctrl+C to abort <<<\n")
        time.sleep(duration)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_all(dio, ao_client)
        dio.close()
        ao_client.close()
        print("Done.")


if __name__ == "__main__":
    main()

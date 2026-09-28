import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import can
import serial.tools.list_ports as list_ports
import config

RETRIES  = 3
INTERVAL = 2.0


def _find_canable_port(VID=0x16D0, PID=0x117E):
    for port in list_ports.comports():
        if port.vid == VID and port.pid == PID:
            return port.device
    return None


async def _try(label, attempt_fn):
    print(f"\n[{label}]")
    for i in range(1, RETRIES + 1):
        try:
            ok, detail = await attempt_fn()
            if ok:
                print(f"  Attempt {i}: OK  — {detail}")
                return True
            else:
                print(f"  Attempt {i}: FAIL — {detail}")
        except Exception as e:
            print(f"  Attempt {i}: ERROR — {e}")
        if i < RETRIES:
            await asyncio.sleep(INTERVAL)
    print(f"  → {label} UNREACHABLE after {RETRIES} attempts.")
    return False


async def check_ping(label, host):
    async def attempt():
        proc = await asyncio.create_subprocess_exec(
            "ping", "-c", "1", "-W", "1", host,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        await proc.wait()
        ok = proc.returncode == 0
        return ok, host
    return await _try(label, attempt)


async def check_can():
    async def attempt():
        channel = await asyncio.get_event_loop().run_in_executor(None, _find_canable_port)
        if channel is None:
            return False, "CANable2 USB device not found"
        bus = can.interface.Bus(
            channel=channel, interface="slcan",
            bitrate=500_000, ttyBaudrate=3_000_000)
        bus.shutdown()
        return True, f"CANable2 on {channel}"
    return await _try("CAN MGS1600", attempt)


async def main():
    print("=" * 40)
    print(" AGV KIM2A — Connection Checker")
    print("=" * 40)

    results = {}
    results["DIO Modbus"]  = await check_ping("DIO Modbus (DI/DO)", config.DIO_IP)
    results["AO Modbus"]   = await check_ping("AO Modbus",          config.AO_IP)
    results["RFID TCP"]    = await check_ping("RFID TCP",           config.RFID_IP)
    results["CAN MGS1600"] = await check_can()

    print("\n" + "=" * 40)
    print(" Summary")
    print("=" * 40)
    for name, ok in results.items():
        status = "OK  " if ok else "FAIL"
        print(f"  [{status}]  {name}")
    print()


if __name__ == "__main__":
    asyncio.run(main())

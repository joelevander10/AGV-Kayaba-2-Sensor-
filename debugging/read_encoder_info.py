"""
read_encoder_info.py
====================
One-time diagnostic — read the CALT/CANopen encoder's identity and resolution
objects over SDO so we KNOW (not assume) counts-per-revolution, total range,
single- vs multi-turn, and the TPDO event timer.

Run:  python3 debugging/read_encoder_info.py

Requires the CANable adapter free (stop agv-controller.service first, or run
with the service's CAN driver disabled — see notes in verify_wheel_speed.py).

Objects read (per debugging/encoder.eds):
  0x1008      Manufacturer device name        (string)
  0x1018:1..3 Identity (vendor/product/rev)
  0x6001      Measuring units per revolution   -> counts per rev
  0x6002      Total measuring range            -> single- vs multi-turn
  0x6501      SingleTurn resolution            (RO, fixed)
  0x1800:5    TPDO1 event timer (ms)           -> sample rate
"""

import time
import can

PORT          = "/dev/ttyACM0"
BITRATE       = 125_000            # verify: encoder may be 125k while MGS1600 bus is 500k
NODE_ID       = 0x01
SDO_TX        = 0x600 + NODE_ID
SDO_RX        = 0x580 + NODE_ID


def sdo_read(bus, index, sub):
    """Expedited SDO upload. Returns (value:int|str|None, raw:bytes|None)."""
    bus.send(can.Message(
        arbitration_id=SDO_TX,
        data=[0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0],
        is_extended_id=False,
    ))
    deadline = time.time() + 1.0
    while time.time() < deadline:
        rsp = bus.recv(timeout=1.0)
        if rsp is None or rsp.arbitration_id != SDO_RX:
            continue
        cmd = rsp.data[0]
        if cmd == 0x80:
            return None, bytes(rsp.data[4:8])          # SDO abort
        if cmd & 0x02:                                  # expedited, size specified
            n = 4 - ((cmd >> 2) & 0x03)
        else:
            n = 4
        raw = bytes(rsp.data[4:4 + n])
        return int.from_bytes(raw, "little"), raw
    return None, None


def main():
    bus = can.interface.Bus(interface="slcan", channel=PORT,
                            bitrate=BITRATE, sleep_after_open=2.0)
    bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00], is_extended_id=False))
    time.sleep(0.1)

    print(f"Encoder info  |  {PORT} @ {BITRATE // 1000} kbit/s  node 0x{NODE_ID:02X}\n")

    # device name (string object)
    val, raw = sdo_read(bus, 0x1008, 0x00)
    name = raw.decode("ascii", "replace") if raw else "?"
    print(f"  Device name (0x1008)            : {name}")

    for idx, sub, label in [
        (0x1018, 1, "Vendor ID (0x1018:1)           "),
        (0x1018, 2, "Product code (0x1018:2)        "),
        (0x1018, 3, "Revision (0x1018:3)            "),
    ]:
        val, _ = sdo_read(bus, idx, sub)
        print(f"  {label}: 0x{val:08X}" if val is not None else f"  {label}: (no response)")

    print()
    cpr, _   = sdo_read(bus, 0x6001, 0x00)
    rng, _   = sdo_read(bus, 0x6002, 0x00)
    st,  _   = sdo_read(bus, 0x6501, 0x00)
    evt, _   = sdo_read(bus, 0x1800, 0x05)

    print(f"  Counts per revolution (0x6001)  : {cpr}")
    print(f"  Total measuring range (0x6002)  : {rng}"
          + (f"   (0x{rng:06X})" if rng is not None else ""))
    print(f"  SingleTurn resolution (0x6501)  : {st}")
    print(f"  TPDO1 event timer (0x1800:5)    : {evt} ms"
          + (f"  -> {1000.0/evt:.1f} Hz" if evt else ""))

    print("\n── interpretation ──")
    if cpr and rng:
        turns = rng / cpr
        if turns > 1.5:
            print(f"  MULTI-TURN: ~{turns:.0f} revolutions before wrap "
                  f"(count climbs to {rng}, wraps there — NOT every rev).")
        else:
            print(f"  SINGLE-TURN: count wraps every revolution at {cpr}.")
        print(f"  Use COUNTS_PER_REV = {cpr}, ROLLOVER_RANGE = {rng} in verify_wheel_speed.py")

    bus.shutdown()


if __name__ == "__main__":
    main()

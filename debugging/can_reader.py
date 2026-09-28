import asyncio
import struct
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import can
import serial.tools.list_ports as list_ports

import config

DT = config.DT  # same loop rate as main program (0.01s)


def _find_canable_port(VID=0x16D0, PID=0x117E):
    for port in list_ports.comports():
        if port.vid == VID and port.pid == PID:
            return port.device
    return None


async def main():
    loop = asyncio.get_event_loop()
    print("Searching for CANable2 adapter...")
    print("Press Ctrl+C to stop.\n")

    while True:
        channel = await loop.run_in_executor(None, _find_canable_port)
        if channel is None:
            print("CANable2 not found. Retrying in 2s...")
            await asyncio.sleep(2)
            continue

        bus      = None
        notifier = None
        try:
            bus = can.interface.Bus(
                channel=channel,
                interface="slcan",
                bitrate=500_000,
                ttyBaudrate=3_000_000,
            )
            reader = can.AsyncBufferedReader()
            notifier = can.Notifier(bus, [reader])

            bus.send(can.Message(
                arbitration_id=0x000,
                data=[0x01, config.CAN_NODE_ID],
                is_extended_id=False,
            ))
            print(f"Connected on {channel}. Receiving frames...\n")

            async for msg in reader:
                if msg.arbitration_id != config.SENSOR_COB_ID:
                    continue
                if len(msg.data) < 5:
                    continue

                left, right = struct.unpack_from("<hh", msg.data, 0)
                flags = msg.data[4]
                tape    = bool(flags & config.FLAG_TAPE_DETECT)
                l_mark  = bool(flags & config.FLAG_LEFT_MARKER)
                r_mark  = bool(flags & config.FLAG_RIGHT_MARKER)
                fail    = bool(flags & config.FLAG_SENSOR_FAIL)

                print(
                    f"Left={left:+5d}mm  Right={right:+5d}mm  "
                    f"Tape={'YES' if tape else 'NO ':3}  "
                    f"L_Mark={'YES' if l_mark else 'NO ':3}  "
                    f"R_Mark={'YES' if r_mark else 'NO ':3}  "
                    f"{'[SENSOR FAIL]' if fail else ''}"
                )

                await asyncio.sleep(DT)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"CAN error: {e}. Retrying in 2s...")
        finally:
            if notifier is not None:
                notifier.stop()
            if bus is not None:
                bus.shutdown()

        await asyncio.sleep(2)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

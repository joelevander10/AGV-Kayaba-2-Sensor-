import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymodbus.client import AsyncModbusTcpClient
import config


async def main(channels: list[int]):
    for ch in channels:
        if ch < 0 or ch >= config.NUM_DO:
            print(f"Error: channel {ch} out of range (0–{config.NUM_DO - 1})")
            sys.exit(1)

    client = AsyncModbusTcpClient(config.DIO_IP, port=config.MODBUS_PORT)
    await client.connect()
    if not client.connected:
        print(f"Error: could not connect to DIO at {config.DIO_IP}:{config.MODBUS_PORT}")
        sys.exit(1)

    async def set_outputs(high_channels: list[int]):
        for i in range(config.NUM_DO):
            value = i in high_channels
            await client.write_coil(
                address=config.DO_BASE + i, value=value, device_id=config.DEVICE_ID)

    try:
        await set_outputs(channels)
        active = ", ".join(f"DO{ch}" for ch in sorted(channels))
        print(f"HIGH: {active}  |  rest LOW — press Ctrl+C to zero all.")
        while True:
            await asyncio.sleep(1)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await set_outputs([])
        client.close()
        print("All outputs zeroed.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python3 write_do.py <channel[,channel,...]>")
        print(f"  e.g. python3 write_do.py 0,3,7")
        sys.exit(1)

    try:
        channels = [int(x.strip()) for x in sys.argv[1].split(",")]
    except ValueError:
        print("Error: channels must be integers, e.g. 0,3,7")
        sys.exit(1)

    asyncio.run(main(channels))

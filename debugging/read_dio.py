import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymodbus.client import AsyncModbusTcpClient
import config


async def main():
    client = AsyncModbusTcpClient(config.DIO_IP, port=config.MODBUS_PORT)
    await client.connect()
    if not client.connected:
        print(f"Failed to connect to {config.DIO_IP}:{config.MODBUS_PORT}")
        return

    print("Reading DI every 320 ms — press Ctrl+C to stop.\n")
    try:
        while True:
            result = await client.read_discrete_inputs(
                address=config.DI_BASE, count=16, device_id=config.DEVICE_ID
            )

            if result.isError():
                print("Modbus read error:", result)
            else:
                bits = result.bits[:16]
                if config.DI_FLIPPED:
                    bits = [not b for b in bits]
                print(f"{'Index':<8} {'State'}")
                print("-" * 20)
                for i, bit in enumerate(bits):
                    print(f"DI[{i:02d}]   {'ON' if bit else 'OFF'}")
                print()

            await asyncio.sleep(0.320)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    asyncio.run(main())

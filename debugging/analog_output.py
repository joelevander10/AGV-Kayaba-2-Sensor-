import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymodbus.client import AsyncModbusTcpClient
import config


async def main(channel: int, voltage: float):
    if not (0 <= channel < config.NUM_AO):
        print(f"Error: channel must be 0–{config.NUM_AO - 1}")
        sys.exit(1)
    if not (0.0 <= voltage <= config.V_RANGE):
        print(f"Error: voltage must be 0–{config.V_RANGE}V")
        sys.exit(1)

    dac_value = int(voltage / config.V_RANGE * config.DAC_RES)

    client = AsyncModbusTcpClient(config.AO_IP, port=config.MODBUS_PORT)
    await client.connect()
    if not client.connected:
        print(f"Failed to connect to AO module at {config.AO_IP}:{config.MODBUS_PORT}")
        sys.exit(1)

    async def set_voltage(v: float):
        dac = int(v / config.V_RANGE * config.DAC_RES)
        await client.write_register(
            address=config.AO_BASE + channel,
            value=dac,
            device_id=config.DEVICE_ID
        )

    try:
        await set_voltage(voltage)
        print(f"AO[{channel}] = {voltage}V (DAC={dac_value}) — press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        print(f"\nReverting AO[{channel}] to 0V...")
        await set_voltage(0.0)
        client.close()
        print("Done.")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 analog_output.py <channel> <voltage>")
        print("       python3 analog_output.py 0 5")
        sys.exit(1)

    try:
        ch  = int(sys.argv[1])
        vol = float(sys.argv[2])
    except ValueError:
        print("Error: channel must be an integer, voltage must be a number.")
        sys.exit(1)

    asyncio.run(main(ch, vol))

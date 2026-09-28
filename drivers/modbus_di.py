import asyncio
import logging

from pymodbus.client import AsyncModbusTcpClient

import config
from drivers.base import SensorDriver

logger = logging.getLogger(__name__)


class DIReader(SensorDriver):

    def __init__(self):
        super().__init__("DI Modbus")

    async def run(self, state):
        client = AsyncModbusTcpClient(config.DIO_IP, port=config.MODBUS_PORT)

        while True:
            try:
                if not client.connected:
                    await client.connect()
                    if not client.connected:
                        logger.warning("DIO not ready at %s. Retrying...", config.DIO_IP)
                        await asyncio.sleep(2)
                        continue

                result = await client.read_discrete_inputs(
                    address=config.DI_BASE, count=config.NUM_DI)

                if not result.isError():
                    bits = result.bits[:config.NUM_DI]
                    if config.DI_FLIPPED:
                        bits = [not b for b in bits]
                    state.latest_di = bits
                    await state.di_queue.put(state.latest_di)
                    self._record_rx()
                else:
                    logger.warning("Modbus DI read error, reconnecting...")
                    client.close()

            except Exception as e:
                logger.error("DI Reader exception: %s", e)
                client.close()

            await asyncio.sleep(config.DI_POLL_INTERVAL)


# ── Module-level shim so io_hardware.di_reader still works ──────────────────

_instance = DIReader()

async def di_reader(state):
    await _instance.run(state)

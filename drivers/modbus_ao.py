import asyncio
import logging

from pymodbus.client import AsyncModbusTcpClient

import config
from drivers.base import ActuatorDriver

logger = logging.getLogger(__name__)


class AOWriter(ActuatorDriver):

    async def run(self, state):
        client = AsyncModbusTcpClient(config.AO_IP, port=config.MODBUS_PORT)

        while True:
            await state.ao_dirty.wait()
            state.ao_dirty.clear()
            # Snapshot the latest setpoints — collapses any backlog to the
            # newest value per channel (no stale replay after a reconnect).
            setpoints = dict(state.ao_setpoints)
            try:
                if not client.connected:
                    await client.connect()

                for channel_no, target_v in setpoints.items():
                    dac_value = int(target_v / config.V_RANGE * config.DAC_RES)
                    await client.write_register(
                        address=config.AO_BASE + channel_no,
                        value=dac_value,
                        device_id=config.DEVICE_ID)

            except Exception as e:
                logger.error("AO Writer error: %s. Reconnecting...", e)
                client.close()
                await asyncio.sleep(1)
                state.ao_dirty.set()   # re-assert latest setpoints on recovery


# ── Module-level shim ────────────────────────────────────────────────────────

async def ao_writer(state):
    await AOWriter().run(state)

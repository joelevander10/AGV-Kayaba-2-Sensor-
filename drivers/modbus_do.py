import asyncio
import logging

from pymodbus.client import AsyncModbusTcpClient

import config
from drivers.base import ActuatorDriver

logger = logging.getLogger(__name__)


class DOWriter(ActuatorDriver):

    async def run(self, state):
        client = AsyncModbusTcpClient(config.DIO_IP, port=config.MODBUS_PORT)

        while True:
            await state.do_dirty.wait()
            state.do_dirty.clear()
            # Snapshot the latest setpoints — collapses any backlog to the
            # newest value per channel (no stale replay after a reconnect).
            setpoints = dict(state.do_setpoints)
            try:
                if not client.connected:
                    await client.connect()

                for channel_no, active_state in setpoints.items():
                    await client.write_coil(
                        address=config.DO_BASE + channel_no,
                        value=bool(active_state),
                        device_id=config.DEVICE_ID)
                    state.latest_do[channel_no] = bool(active_state)

            except Exception as e:
                logger.error("DO Writer error: %s. Reconnecting...", e)
                client.close()
                await asyncio.sleep(1)
                state.do_dirty.set()   # re-assert latest setpoints on recovery


# ── Module-level shim ────────────────────────────────────────────────────────

async def do_writer(state):
    await DOWriter().run(state)

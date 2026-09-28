import asyncio
import binascii
import logging
import socket

import config
from drivers.base import SensorDriver

logger = logging.getLogger(__name__)


class RFIDReader(SensorDriver):

    def __init__(self):
        super().__init__("RFID TCP")

    async def run(self, state):
        loop = asyncio.get_running_loop()

        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            try:
                await loop.sock_connect(sock, (config.RFID_IP, config.RFID_PORT))
                await loop.sock_sendall(sock, config.RFID_INIT_CMD)
                logger.info("RFID Connected.")
                self._record_rx()   # mark healthy on connect

                while True:
                    try:
                        data = await asyncio.wait_for(
                            loop.sock_recv(sock, 1024),
                            timeout=2.0)
                    except asyncio.TimeoutError:
                        # No tag in range — connection still alive, keep watchdog happy
                        self._record_rx()
                        continue

                    if not data:
                        logger.warning("RFID connection closed by peer.")
                        break

                    hex_data = binascii.hexlify(data).decode().upper()
                    for packet in hex_data.split("CF")[1:]:
                        packet = "CF" + packet
                        if len(packet) >= 34:
                            tag = packet[26:30]
                            logger.info("RFID tag read: %s (dec=%d)", tag, int(tag, 16))
                            await state.rfid_queue.put(tag)
                            self._record_rx()

            except Exception as e:
                logger.warning("RFID connection failed/lost: %s. Retrying in 2s...", e)
            finally:
                sock.close()

            await asyncio.sleep(2)


# ── Module-level shim ────────────────────────────────────────────────────────

_instance = RFIDReader()

async def rfid_reader(state):
    await _instance.run(state)

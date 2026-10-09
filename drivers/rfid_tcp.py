import asyncio
import binascii
import logging
import socket
import time

import config
from drivers.base import SensorDriver

logger = logging.getLogger(__name__)

# ── Reader frame format ──────────────────────────────────────────────────────
#   [0] 0xCF  [1] addr  [2..3] cmd  [4] LEN  [5 .. 5+LEN-1] data  [last 2] CRC
#   CRC = CRC-16/MCRF4XX over every byte before it, sent high byte first
#   (checked against the init command CF FF 00 70 00 -> 24 15).
# The tag ID is bytes 13..14 of a tag frame (same position as before).
_HEADER     = 0xCF
_MIN_FRAME  = 7          # header+addr+cmd(2)+len+crc(2), no data
_TAG_FRAME  = 17         # shortest frame that carries a tag
_MAX_BUFFER = 4096


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def _frame_at(buf: bytearray, i: int):
    """Length of a complete, CRC-valid frame starting at buf[i], 0 if the
    frame is not complete yet, -1 if buf[i] is not a real frame start."""
    if len(buf) - i < 5:
        return 0
    total = _MIN_FRAME + buf[i + 4]
    if len(buf) - i < total:
        return 0
    frame = buf[i:i + total]
    if _crc16(frame[:-2]) == (frame[-2] << 8 | frame[-1]):
        return total
    return -1


def parse_frames(buf: bytearray) -> list:
    """Pull every complete frame out of buf (bytes stay in buf until a whole
    frame has arrived, so frames split across TCP reads are not lost, and a
    0xCF byte inside the data is never mistaken for a new frame)."""
    frames = []
    while True:
        i = buf.find(_HEADER)
        if i < 0:
            buf.clear()
            break
        if i:
            del buf[:i]
        n = _frame_at(buf, 0)
        if n > 0:
            frames.append(bytes(buf[:n]))
            del buf[:n]
            continue
        if n < 0:                      # false header — resync on the next 0xCF
            del buf[:1]
            continue
        # Incomplete at 0: if a valid frame already sits further on, the
        # byte at 0 was a false header (with a bogus length) — skip to it.
        j = buf.find(_HEADER, 1)
        skipped = False
        while j > 0:
            if _frame_at(buf, j) > 0:
                del buf[:j]
                skipped = True
                break
            j = buf.find(_HEADER, j + 1)
        if not skipped:
            if len(buf) > _MAX_BUFFER:
                buf.clear()
            break
    return frames


def tag_from_frame(frame: bytes):
    if len(frame) < _TAG_FRAME:
        return None
    return binascii.hexlify(frame[13:15]).decode().upper()


class RFIDReader(SensorDriver):

    def __init__(self):
        super().__init__("RFID TCP")

    async def run(self, state):
        loop = asyncio.get_running_loop()

        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            buf = bytearray()
            last_tag, last_log_t = None, 0.0
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
                        buf.clear()
                        self._record_rx()
                        continue

                    if not data:
                        logger.warning("RFID connection closed by peer.")
                        break

                    buf.extend(data)
                    for frame in parse_frames(buf):
                        tag = tag_from_frame(frame)
                        if tag is None:
                            continue
                        state.rfid_queue.put_nowait(tag)
                        self._record_rx()
                        # Log a tag once while it stays in range (not every read).
                        now = time.monotonic()
                        if tag != last_tag or now - last_log_t > 2.0:
                            logger.info("RFID tag read: %s (dec=%d)", tag, int(tag, 16))
                            last_log_t = now
                        last_tag = tag

            except Exception as e:
                logger.warning("RFID connection failed/lost: %s. Retrying in 2s...", e)
            finally:
                sock.close()

            await asyncio.sleep(2)


# ── Module-level shim ────────────────────────────────────────────────────────

_instance = RFIDReader()

async def rfid_reader(state):
    await _instance.run(state)

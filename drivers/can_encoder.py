"""
drivers/can_encoder.py — CANopen absolute encoder reader (wheel-speed calibration)
==================================================================================
Reads a CALT/CANopen multi-turn encoder over slcan (CANable) to measure the
ACTUAL wheel ground speed during calibration. Mirrors drivers/can_mgs1600.py but
speaks CANopen TPDO/SDO instead of the MGS1600 frame.

IMPORTANT — this shares the single CANable adapter with the MGS1600 reader. Only
one may own the port at a time, so the calibration flow stops the MGS1600 CAN
driver before starting this one (see calibration.py / DriverManager).

Geometry: a small friction wheel (config.ENCODER_RADIUS_M) rolls on the AGV
wheel rim, so rim speeds match and the AGV ground speed is:
    v_ground = omega_enc * ENCODER_RADIUS_M
The encoder is multi-turn (TPDO1 maps object 0x6004, 24-bit), so the raw count
accumulates across revolutions and wraps only at the TOTAL measuring range — the
rollover correction therefore uses ENCODER_TOTAL_RANGE, not counts-per-rev.
"""

import asyncio
import logging
import math
import struct
import time

import can
import serial.tools.list_ports as list_ports

import config
from drivers.base import SensorDriver

logger = logging.getLogger(__name__)


def _find_canable_port(VID=0x16D0, PID=0x117E):
    for port in list_ports.comports():
        if port.vid == VID and port.pid == PID:
            return port.device
    return None


class EncoderReader(SensorDriver):

    def __init__(self):
        super().__init__("CAN Encoder")
        self._counts_per_rev = config.ENCODER_COUNTS_PER_REV
        self._total_range    = config.ENCODER_TOTAL_RANGE
        self._rad_per_count  = 2.0 * math.pi / self._counts_per_rev
        self._half_range     = self._total_range / 2.0
        self._tpdo_cob_id    = config.ENCODER_TPDO_COB_ID_BASE + config.ENCODER_NODE_ID

    def _sdo_read(self, bus, index, sub):
        """Blocking expedited SDO upload (used once at startup). Returns int|None."""
        tx = 0x600 + config.ENCODER_NODE_ID
        rx = 0x580 + config.ENCODER_NODE_ID
        bus.send(can.Message(
            arbitration_id=tx,
            data=[0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0],
            is_extended_id=False))
        deadline = time.time() + 0.5
        while time.time() < deadline:
            rsp = bus.recv(timeout=0.5)
            if rsp is None:
                return None
            if rsp.arbitration_id != rx:
                continue
            cmd = rsp.data[0]
            if cmd == 0x80:
                return None
            n = 4 - ((cmd >> 2) & 0x03) if (cmd & 0x02) else 4
            return int.from_bytes(bytes(rsp.data[4:4 + n]), "little")
        return None

    def _read_geometry(self, bus):
        """Best-effort: confirm counts/rev (0x6001) and total range (0x6002) from
        the live device, overriding the config fallbacks. Never fatal."""
        try:
            cpr = self._sdo_read(bus, 0x6001, 0x00)
            rng = self._sdo_read(bus, 0x6002, 0x00)
            if cpr:
                self._counts_per_rev = cpr
                self._rad_per_count  = 2.0 * math.pi / cpr
            if rng:
                self._total_range = rng + 1   # 0x6002 is the max value; range = max+1
                self._half_range  = self._total_range / 2.0
            logger.info("[Encoder] geometry: counts/rev=%s total_range=%s",
                        self._counts_per_rev, self._total_range)
        except Exception as exc:                       # noqa: BLE001
            logger.warning("[Encoder] geometry read failed (%s) — using config", exc)

    async def run(self, state):
        loop = asyncio.get_running_loop()
        while True:
            channel = _find_canable_port()
            if channel is None:
                logger.warning("[Encoder] CANable not found, retrying in 2s...")
                state.encoder_connected = False
                await asyncio.sleep(2)
                continue

            bus      = None
            notifier = None
            prev_count = None
            prev_time  = None
            try:
                bus = await loop.run_in_executor(None, lambda: can.interface.Bus(
                    channel=channel,
                    interface="slcan",
                    bitrate=config.ENCODER_BITRATE,
                    ttyBaudrate=3_000_000))

                # NMT: Pre-Operational -> Operational (start all nodes), else silent
                bus.send(can.Message(arbitration_id=0x000, data=[0x01, 0x00],
                                     is_extended_id=False))
                await asyncio.sleep(0.1)
                await loop.run_in_executor(None, self._read_geometry, bus)

                reader   = can.AsyncBufferedReader()
                notifier = can.Notifier(bus, [reader])
                logger.info("[Encoder] connected on %s @ %d kbit/s (node 0x%02X)",
                            channel, config.ENCODER_BITRATE // 1000, config.ENCODER_NODE_ID)
                state.encoder_connected = True
                state.encoder_last_rx   = time.time()

                async for msg in reader:
                    if msg.arbitration_id != self._tpdo_cob_id or len(msg.data) < 4:
                        continue

                    now   = time.perf_counter()
                    count = struct.unpack_from("<I", msg.data, 0)[0]
                    self._record_rx()
                    state.encoder_count   = count
                    state.encoder_last_rx = time.time()

                    if prev_count is None:
                        prev_count, prev_time = count, now
                        continue

                    dt    = now - prev_time
                    delta = count - prev_count
                    # Wrap only at the total measuring range (multi-turn).
                    if   delta >  self._half_range: delta -= self._total_range
                    elif delta < -self._half_range: delta += self._total_range
                    prev_count, prev_time = count, now

                    if dt > 0:
                        omega = (delta * self._rad_per_count) / dt
                        state.encoder_rpm = omega * 60.0 / (2.0 * math.pi)
                        state.encoder_v   = omega * config.ENCODER_RADIUS_M

            except asyncio.CancelledError:
                raise
            except Exception as exc:                   # noqa: BLE001
                logger.error("[Encoder] error (%s) — reconnecting in 2s", exc)
                await asyncio.sleep(2)
            finally:
                state.encoder_connected = False
                if notifier is not None:
                    try: notifier.stop()
                    except Exception: pass
                if bus is not None:
                    try: bus.shutdown()
                    except Exception: pass

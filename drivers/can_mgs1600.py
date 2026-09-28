import asyncio
import logging
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
            logger.debug("CANable2 found at: %s", port.device)
            return port.device
    return None


class CANReader(SensorDriver):

    def __init__(self):
        super().__init__("CAN MGS1600")

    async def run(self, state):
        while True:
            channel = _find_canable_port()
            if channel is None:
                logger.warning("CANable2 not found, retrying in 2s...")
                await asyncio.sleep(2)
                continue

            bus      = None
            notifier = None
            try:
                loop = asyncio.get_running_loop()
                bus = await loop.run_in_executor(None, lambda: can.interface.Bus(
                    channel=channel,
                    interface="slcan",
                    bitrate=500_000,
                    ttyBaudrate=3_000_000))
                reader = can.AsyncBufferedReader()

                notifier = can.Notifier(bus, [reader])
                bus.send(can.Message(
                    arbitration_id=0x000,
                    data=[0x01, config.CAN_NODE_ID],
                    is_extended_id=False))
                # Rear MGS1600 lives on the same physical bus, just a different
                # CANopen node — NMT-start it too (no-op wire traffic if the
                # profile has no rear sensor and REAR_CAN_NODE_ID is None, since
                # that branch is skipped entirely).
                if config.REAR_SENSOR_ENABLED:
                    bus.send(can.Message(
                        arbitration_id=0x000,
                        data=[0x01, config.REAR_CAN_NODE_ID],
                        is_extended_id=False))

                logger.info("CAN connected on %s", channel)
                state.can_last_rx = time.time()
                if config.REAR_SENSOR_ENABLED:
                    state.rear_can_last_rx = time.time()

                async for msg in reader:
                    if (msg.arbitration_id == config.SENSOR_COB_ID
                            and len(msg.data) >= 5):
                        left, right = struct.unpack_from("<hh", msg.data, 0)
                        flags = msg.data[4]
                        state.can_last_rx = time.time()
                        frame = {
                            "left_mm":        left,
                            "right_mm":       right,
                            "tape_detected":  bool(flags & config.FLAG_TAPE_DETECT),
                            "left_marker":    bool(flags & config.FLAG_LEFT_MARKER),
                            "right_marker":   bool(flags & config.FLAG_RIGHT_MARKER),
                            "sensor_failure": bool(flags & config.FLAG_SENSOR_FAIL),
                        }
                        state.latest_sensor = frame
                        await state.sensor_queue.put(frame)
                        self._record_rx()

                    elif (config.REAR_SENSOR_ENABLED
                            and msg.arbitration_id == config.REAR_SENSOR_COB_ID
                            and len(msg.data) >= 5):
                        # Same MGS1600 frame layout, just the rear unit's node.
                        left, right = struct.unpack_from("<hh", msg.data, 0)
                        flags = msg.data[4]
                        state.rear_can_last_rx = time.time()
                        rear_frame = {
                            "left_mm":        left,
                            "right_mm":       right,
                            "tape_detected":  bool(flags & config.FLAG_TAPE_DETECT),
                            "left_marker":    bool(flags & config.FLAG_LEFT_MARKER),
                            "right_marker":   bool(flags & config.FLAG_RIGHT_MARKER),
                            "sensor_failure": bool(flags & config.FLAG_SENSOR_FAIL),
                        }
                        state.latest_rear_sensor = rear_frame
                        await state.rear_sensor_queue.put(rear_frame)
                        self._record_rx()

            except Exception as e:
                logger.warning("CAN error or disconnect: %s, retrying in 2s...", e)

            finally:
                if notifier is not None:
                    notifier.stop()
                if bus is not None:
                    bus.shutdown()

            await asyncio.sleep(2)


# ── Module-level shim ────────────────────────────────────────────────────────

_instance = CANReader()

async def can_reader(state):
    await _instance.run(state)
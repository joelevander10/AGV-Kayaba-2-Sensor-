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

    def _send_nmt_start(self, bus):
        """CANopen NMT 'start remote node' to the front (and rear) MGS1600.
        A sensor that rebooted (e.g. supply dip) comes back pre-operational
        and sends nothing until it gets this again."""
        nodes = [config.CAN_NODE_ID]
        if config.REAR_SENSOR_ENABLED:
            nodes.append(config.REAR_CAN_NODE_ID)
        for node in nodes:
            bus.send(can.Message(arbitration_id=0x000, data=[0x01, node],
                                 is_extended_id=False))

    async def run(self, state):
        # Auto-reconnect ("hard"):
        #   * no frame from a sensor for NMT_RESEND_S  -> re-send NMT start
        #     (every NMT_RESEND_S while it stays silent)
        #   * no frame from ANY sensor for REOPEN_S    -> close and reopen the
        #     CANable adapter (full reconnect), retrying every RETRY_S
        #   * adapter unplugged / error                -> reopen, same retry
        # The safety watchdog still stops the AGV after 1 s without data.
        nmt_resend_s = config.CAN_NMT_RESEND_S
        reopen_s     = config.CAN_REOPEN_S
        retry_s      = config.CAN_RETRY_S
        reconnects   = 0
        while True:
            channel = _find_canable_port()
            if channel is None:
                logger.warning("CANable2 not found, retrying in %.1fs...", retry_s)
                await asyncio.sleep(retry_s)
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
                self._send_nmt_start(bus)

                logger.info("CAN connected on %s%s", channel,
                            f" (reconnect #{reconnects})" if reconnects else "")
                now = time.time()
                state.can_last_rx = now
                if config.REAR_SENSOR_ENABLED:
                    state.rear_can_last_rx = now
                front_rx = rear_rx = any_rx = now
                last_nmt     = now
                outage_start = None
                outage_who   = ""

                while True:
                    try:
                        msg = await asyncio.wait_for(reader.get_message(), timeout=0.05)
                    except asyncio.TimeoutError:
                        msg = None

                    now = time.time()
                    if msg is not None:
                        if (msg.arbitration_id == config.SENSOR_COB_ID
                                and len(msg.data) >= 5):
                            left, right = struct.unpack_from("<hh", msg.data, 0)
                            flags = msg.data[4]
                            state.can_last_rx = now
                            front_rx = any_rx = now
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
                            state.rear_can_last_rx = now
                            rear_rx = any_rx = now
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

                    # ── Silence detection ──────────────────────────────────
                    front_quiet = now - front_rx > nmt_resend_s
                    rear_quiet  = (config.REAR_SENSOR_ENABLED
                                   and now - rear_rx > nmt_resend_s)
                    if front_quiet or rear_quiet:
                        who = "+".join(n for n, q in (("front", front_quiet),
                                                      ("rear", rear_quiet)) if q)
                        if outage_start is None:
                            outage_start = now - nmt_resend_s
                            outage_who   = who
                            logger.warning("[CAN] no data from %s sensor — re-sending NMT start", who)
                        elif who not in outage_who:
                            outage_who = "front+rear"
                        if now - last_nmt >= nmt_resend_s:
                            self._send_nmt_start(bus)
                            last_nmt = now
                    elif outage_start is not None:
                        logger.warning("[CAN] sensor data back after %.2f s (%s)",
                                       now - outage_start, outage_who)
                        outage_start = None
                        outage_who   = ""

                    if now - any_rx > reopen_s:
                        raise RuntimeError(
                            f"no data from any sensor for {now - any_rx:.1f} s — reopening adapter")

            except Exception as e:
                logger.warning("CAN error or disconnect: %s — reconnecting in %.1fs", e, retry_s)

            finally:
                if notifier is not None:
                    try:
                        notifier.stop()
                    except Exception:
                        pass
                if bus is not None:
                    try:
                        bus.shutdown()
                    except Exception:
                        pass

            reconnects += 1
            await asyncio.sleep(retry_s)


# ── Module-level shim ────────────────────────────────────────────────────────

_instance = CANReader()

async def can_reader(state):
    await _instance.run(state)
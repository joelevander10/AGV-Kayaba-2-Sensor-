"""
safety_watchdog.py — Hardware driver health monitor
====================================================
Polls a list of (SensorDriver, timeout_s) pairs every 50 ms.

If any driver has not received data within its timeout threshold:
  - sets state.system_error = True
  - immediately zeroes both AO speed outputs

When all drivers recover:
  - clears state.system_error

mode_manager reads state.system_error and forces idle on the next cycle.
The DO/AO writer is output-only and is not monitored here.
"""

import asyncio
import logging
import time

logger = logging.getLogger(__name__)


async def safety_watchdog(state, watched: list):
    """
    watched: list of (SensorDriver, timeout_s) tuples.
    Example:
        safety_watchdog(state, watched=[
            (di_drv,   config.WATCHDOG_DI_TIMEOUT_S),
            (can_drv,  config.WATCHDOG_CAN_TIMEOUT_S),
            (rfid_drv, config.WATCHDOG_RFID_TIMEOUT_S),
        ])
    """
    while True:
        now = time.time()
        fault_detail = None

        for driver, timeout_s in watched:
            h = driver.get_health()
            if (now - h["last_rx"]) > timeout_s:
                fault_detail = h["detail"]
                break

        if fault_detail is not None:
            if not state.system_error:
                logger.error(
                    "WATCHDOG: hardware lost — '%s' stale (no data) — forcing idle",
                    fault_detail,
                )
                state.system_error = True
                state.system_error_detail = fault_detail
                # Zero speed outputs immediately without waiting for mode_manager
                state.set_ao(0, 0.0)
                state.set_ao(1, 0.0)
        else:
            if state.system_error:
                logger.info("WATCHDOG: all drivers recovered — clearing system_error")
                state.system_error = False
                state.system_error_detail = None

        await asyncio.sleep(0.05)
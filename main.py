import asyncio
import logging
import signal
import threading
import time
from pymodbus.client import AsyncModbusTcpClient

import config
from logger import setup_logging
from state import AMRState
from core.sequence_engine import SequenceEngine
from drivers.modbus_di   import DIReader
from drivers.modbus_do   import DOWriter
from drivers.modbus_ao   import AOWriter
from drivers.can_mgs1600 import CANReader
from drivers.rfid_tcp    import RFIDReader 
from safety_watchdog import safety_watchdog
import modes
from rfid_processor import rfid_processor
from pusher import pusher_controller
from app.app import run_server

logger = logging.getLogger(__name__)


class DriverManager:
    """Starts/stops the toggleable features (NAV, SEQ) at runtime.

    Dependency hierarchy:  NAV (base) -> SEQ.
      - NAV  drives the RFID reader (factory). SEQ also needs RFID tags, so the
        reader runs whenever NAV is on; SEQ has no driver of its own — it is a
        pure gate flag the sequence engine reads.
    Enabling a child requires its parent on; disabling a parent cascades down to
    its dependents.

    The Flask server runs in a separate thread, so set_enabled() marshals the
    start/stop onto the asyncio loop via run_coroutine_threadsafe. Toggles are
    NOT persisted — on restart the controller boots from the profile JSON.
    """

    # factory=None means a pure gate flag (no driver task).
    # CAN is managed (not a fixed core task) so the calibration flow can release
    # the shared CANable adapter for the encoder, then restore it.
    _SPEC = {
        "CAN_ENABLED": {"factory": CANReader, "watch": True, "requires": None, "watch_timeout": config.WATCHDOG_CAN_TIMEOUT_S},
        "NAV_ENABLED": {"factory": RFIDReader, "watch": True, "requires": None, "watch_timeout": config.WATCHDOG_RFID_TIMEOUT_S},
        "SEQ_ENABLED": {"factory": None, "watch": False, "requires": "NAV_ENABLED"},
    }
    # Direct dependents that must be disabled when a flag is disabled.
    _DEPENDENTS = {
        "CAN_ENABLED":  [],
        "NAV_ENABLED":  ["SEQ_ENABLED"],
        "SEQ_ENABLED":  [],
    }

    def __init__(self, state, watched):
        self._state   = state
        self._watched = watched          # shared list read by safety_watchdog
        self._loop    = None
        self._tasks   = {}               # flag -> asyncio.Task
        self._drivers = {}               # flag -> driver instance

    def bind_loop(self, loop):
        self._loop = loop

    def is_running(self, flag):
        t = self._tasks.get(flag)
        return t is not None and not t.done()

    async def _start(self, flag):
        spec = self._SPEC[flag]
        req  = spec["requires"]
        if req is not None and not getattr(config, req):
            raise ValueError(f"{flag} requires {req} to be enabled first")
        if spec["factory"] is not None and not self.is_running(flag):
            drv = spec["factory"]()
            self._drivers[flag] = drv
            self._tasks[flag]   = asyncio.create_task(drv.run(self._state))
            if spec["watch"]:
                self._watched.append((drv, spec.get("watch_timeout",
                                                     config.WATCHDOG_RFID_TIMEOUT_S)))
        setattr(config, flag, True)
        logger.info("[DriverManager] %s ENABLED (live)", flag)

    async def _stop(self, flag):
        # Cascade: disable dependents first so a child never outlives its parent.
        for dep in self._DEPENDENTS.get(flag, []):
            if getattr(config, dep):
                await self._stop(dep)
        # Remove from the watchdog first so a cancelled driver can't trip a
        # stale-data fault between cancellation and removal.
        drv = self._drivers.pop(flag, None)
        if drv is not None:
            self._watched[:] = [(d, t) for (d, t) in self._watched if d is not drv]
        task = self._tasks.pop(flag, None)
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        setattr(config, flag, False)
        logger.info("[DriverManager] %s DISABLED (live)", flag)

    async def _apply(self, flag, enabled):
        if enabled:
            await self._start(flag)
        else:
            await self._stop(flag)

    def driver_health(self):
        """Health + observed frame rate for every watched sensor driver.

        Read by the web layer from the Flask thread, so it only reads driver
        state — the drivers themselves keep their counters up to date on the
        event loop.
        """
        now = time.time()
        out = []
        for drv, timeout_s in list(self._watched):
            h = drv.get_health()
            out.append({
                "name":      h["detail"],
                "ok":        h["ok"],
                "rate_hz":   round(h.get("rate_hz", 0.0), 1),
                "total":     h.get("total", 0),
                "age_s":     round(now - h["last_rx"], 2),
                "timeout_s": timeout_s,
            })
        return out

    def set_enabled(self, flag, enabled):
        """Thread-safe entry point for the Flask server. Blocks until applied."""
        if flag not in self._SPEC:
            raise ValueError(f"unknown feature flag: {flag}")
        if self._loop is None:
            raise RuntimeError("driver manager loop not bound")
        fut = asyncio.run_coroutine_threadsafe(
            self._apply(flag, bool(enabled)), self._loop)
        return fut.result(timeout=10)


async def shutdown():
    """Zero all DO and AO outputs via direct Modbus writes, bypassing the queues."""
    logger.info("Shutting down — zeroing all outputs...")
    try:
        ao_client = AsyncModbusTcpClient(config.AO_IP, port=config.MODBUS_PORT)
        await ao_client.connect()
        for i in range(config.NUM_AO):
            await ao_client.write_register(address=config.AO_BASE + i, value=0)
        ao_client.close()

        if config.DIO_ENABLED:
            dio_client = AsyncModbusTcpClient(config.DIO_IP, port=config.MODBUS_PORT)
            await dio_client.connect()
            for i in range(config.NUM_DO):
                await dio_client.write_coil(address=config.DO_BASE + i, value=False)
            dio_client.close()
        logger.info("All outputs zeroed. Shutdown complete.")
    except Exception as e:
        logger.error("Shutdown error: %s", e)

async def run():
    setup_logging()

    loop = asyncio.get_running_loop()

    state  = AMRState()
    engine = SequenceEngine(state, config.SEQUENCES)

    logger.info("Loaded profile: %s  (%d sequence(s) defined)",
                config.AGV_ID, len(config.SEQUENCES))

    # ── Instantiate drivers (feature-flag gated) ──────────────────────────────
    # DIO and CAN are fixed at boot. RFID is managed by DriverManager
    # so they can be toggled live from the HMI.
    # CAN is started via the DriverManager (see boot block below) so calibration
    # can release the shared adapter for the encoder; DIO and AO stay fixed.
    di_drv   = DIReader()   if config.DIO_ENABLED  else None
    do_drv   = DOWriter()   if config.DIO_ENABLED  else None
    ao_drv   = AOWriter()

    for name, enabled in [
        ("DIO (DI+DO)", config.DIO_ENABLED),
        ("CAN sensor",  config.CAN_ENABLED),
        ("RFID NAV",    config.NAV_ENABLED),
        ("RFID SEQ",    config.SEQ_ENABLED),
    ]:
        logger.info("%-12s %s", name, "ENABLED" if enabled else "DISABLED")

    watched = []
    manager = DriverManager(state, watched)
    manager.bind_loop(loop)

    threading.Thread(target=run_server, args=(state, engine, manager), daemon=True).start()

    def terminate_gracefully():
        logger.info("Termination signal received. Cancelling tasks...")
        for task in asyncio.all_tasks(loop):
            task.cancel()

    loop.add_signal_handler(signal.SIGTERM, terminate_gracefully)
    loop.add_signal_handler(signal.SIGINT, terminate_gracefully)

    # ── Build task list ───────────────────────────────────────────────────────
    core_tasks = [ao_drv.run(state)]

    if do_drv:
        core_tasks.append(do_drv.run(state))
    if di_drv:
        core_tasks.append(di_drv.run(state))
        watched.append((di_drv, config.WATCHDOG_DI_TIMEOUT_S))

    core_tasks.append(safety_watchdog(state, watched=watched))
    core_tasks.append(rfid_processor(state, engine))
    core_tasks.append(modes.mode_manager(state, engine, manager))
    core_tasks.append(pusher_controller(state))

    # NAV/SEQ run under the manager so the HMI can toggle them live.
    # Boots from the profile JSON values (hierarchy already enforced in config);
    # toggles do not persist across restart. Order matters: parents before
    # children so the dependency guard in _start is satisfied.
    if config.CAN_ENABLED:
        await manager._apply("CAN_ENABLED", True)
    if config.NAV_ENABLED:
        await manager._apply("NAV_ENABLED", True)
    if config.SEQ_ENABLED:
        await manager._apply("SEQ_ENABLED", True)

    tasks = asyncio.gather(*core_tasks)

    try:
        await tasks
    except asyncio.CancelledError:
        logger.info("Main loops cancelled. Executing Modbus hardware shutdown...")
        await shutdown()

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
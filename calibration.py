"""
calibration.py — open-loop wheel-speed calibration ramp
=======================================================
Drives both wheels straight at a stepped open-loop speed (0 → CAL_V_MAX, stepping
CAL_V_STEP every CAL_DWELL_S) while a CANopen encoder measures the ACTUAL ground
speed of one wheel. Logs commanded-vs-actual to CSV + PNG for analysis.

This runs as a mode task launched by mode_manager when state.calibration_request
is set from the HMI (only from the ARMED state). It owns the CAN adapter for the
duration: the MGS1600 CAN driver is stopped first (single shared adapter) and
restored on exit.

Safety: emergency / manual switch / RESET / web Stop all cancel this task; the
finally block idles the motors, stops the encoder, and restores the CAN driver.
"""

import asyncio
import csv
import logging
import os
import threading
import time
from collections import deque
from datetime import datetime

import config
import motion
from drivers.can_encoder import EncoderReader

logger = logging.getLogger(__name__)

_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analysis_plot")
_MAX_SAMPLES = 60000   # ~50 min at 20 Hz; bounds memory


class _CalRecorder:
    """Bounded in-memory recorder; saves CSV + PNG off-thread on stop()."""

    _FIELDS = ["t_s", "v_cmd_ms", "voltage_v", "encoder_v_ms",
               "encoder_rpm", "encoder_count"]

    def __init__(self, wheel: str):
        self._wheel = wheel
        self._rows  = deque(maxlen=_MAX_SAMPLES)
        self._stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(_OUT_DIR, f"{self._stamp}_{wheel}_wheelcal.csv")
        self.png_path = os.path.join(_OUT_DIR, f"{self._stamp}_{wheel}_wheelcal.png")

    def record(self, **kw):
        self._rows.append([kw.get(f) for f in self._FIELDS])

    def stop(self):
        rows = list(self._rows)
        if not rows:
            logger.info("[Calib] no samples recorded — nothing to save")
            return
        threading.Thread(target=self._save, args=(rows,), daemon=True).start()

    def _save(self, rows):
        try:
            os.makedirs(_OUT_DIR, exist_ok=True)
            with open(self.csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(self._FIELDS)
                w.writerows(rows)
            logger.info("[Calib] CSV saved: %s", self.csv_path)
        except Exception as exc:                       # noqa: BLE001
            logger.error("[Calib] CSV save failed: %s", exc)
            return
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            t    = [r[0] for r in rows]
            cmd  = [r[1] for r in rows]
            act  = [r[3] for r in rows]
            fig, ax = plt.subplots(figsize=(11, 5))
            ax.plot(t, cmd, color="C3", lw=1.4, ls="--", label="commanded (m/s)")
            ax.plot(t, act, color="C0", lw=1.2, label="actual encoder (m/s)")
            ax.set_xlabel("time (s)"); ax.set_ylabel("v (m/s)")
            ax.set_title(f"Wheel-speed calibration ({self._wheel}) — {self._stamp}")
            ax.grid(True, alpha=0.3); ax.legend(loc="upper left")
            fig.tight_layout(); fig.savefig(self.png_path, dpi=110)
            plt.close(fig)
            logger.info("[Calib] plot saved: %s", self.png_path)
        except Exception as exc:                       # noqa: BLE001
            logger.error("[Calib] plot failed (CSV still saved): %s", exc)


def _commanded_speed(elapsed: float) -> float:
    n = int(elapsed // config.CAL_DWELL_S)
    return min(config.CAL_V_START + n * config.CAL_V_STEP, config.CAL_V_MAX)


async def calibrate_mode(state, manager=None):
    """Open-loop straight speed ramp with synchronous encoder logging."""
    wheel = state.calibration_wheel
    logger.info("[Calib] START — wheel=%s, ramp %.2f→%.2f m/s step %.2f every %.1fs",
                wheel, config.CAL_V_START, config.CAL_V_MAX,
                config.CAL_V_STEP, config.CAL_DWELL_S)

    can_was_enabled = bool(getattr(config, "CAN_ENABLED", False)) and manager is not None
    enc_reader = EncoderReader()
    enc_task   = None
    recorder   = _CalRecorder(wheel)

    status = state.calibration_status
    status.update({"active": True, "v_cmd": 0.0, "voltage": 0.0,
                   "elapsed": 0.0, "wheel": wheel, "csv_path": recorder.csv_path})

    try:
        # ── Free the shared CAN adapter from the MGS1600 reader ───────────────
        if can_was_enabled:
            logger.info("[Calib] releasing CAN (MGS1600) driver for the encoder")
            await manager._apply("CAN_ENABLED", False)

        # ── Start the encoder reader and wait briefly for first data ──────────
        enc_task = asyncio.create_task(enc_reader.run(state))
        wait_deadline = time.time() + 3.0
        while time.time() < wait_deadline and not state.encoder_connected:
            await asyncio.sleep(0.1)
        if not state.encoder_connected:
            logger.warning("[Calib] encoder not connected yet — continuing, "
                           "data will appear once it links")

        # ── Ramp loop ─────────────────────────────────────────────────────────
        # End one dwell after reaching V_MAX so the top step is captured.
        n_steps   = max(1, round((config.CAL_V_MAX - config.CAL_V_START)
                                 / config.CAL_V_STEP))
        total_t   = (n_steps + 1) * config.CAL_DWELL_S
        t0        = time.perf_counter()

        while True:
            elapsed = time.perf_counter() - t0
            if elapsed > total_t:
                logger.info("[Calib] ramp complete (%.1fs)", elapsed)
                break
            if state.emergency_active:
                # mode_manager will cancel us; bail out cleanly meanwhile.
                break

            v_cmd = _commanded_speed(elapsed)
            rpm   = motion.mps_to_rpm(v_cmd)
            # Voltage logged is the command on the wheel the encoder is on.
            volt  = motion.rpm_to_voltage(rpm, wheel)
            if v_cmd <= 0.0:
                await motion.idle(state)
                volt = 0.0
            else:
                await motion.set_forward(state, rpm)

            status.update({"v_cmd": round(v_cmd, 3), "voltage": round(volt, 3),
                           "elapsed": round(elapsed, 1)})
            recorder.record(t_s=round(elapsed, 3), v_cmd_ms=round(v_cmd, 4),
                            voltage_v=round(volt, 4),
                            encoder_v_ms=round(state.encoder_v, 4),
                            encoder_rpm=round(state.encoder_rpm, 3),
                            encoder_count=state.encoder_count)
            await asyncio.sleep(0.05)

    except asyncio.CancelledError:
        logger.info("[Calib] cancelled")
        raise
    finally:
        # Shield cleanup so a cancellation still idles motors + restores CAN.
        await asyncio.shield(_cleanup(state, manager, enc_task, recorder,
                                      can_was_enabled))


async def _cleanup(state, manager, enc_task, recorder, can_was_enabled):
    try:
        await motion.idle(state)
    except Exception as exc:                           # noqa: BLE001
        logger.error("[Calib] idle on cleanup failed: %s", exc)

    if enc_task is not None and not enc_task.done():
        enc_task.cancel()
        try:
            await enc_task
        except (asyncio.CancelledError, Exception):
            pass
    state.encoder_connected = False

    recorder.stop()
    state.calibration_status["active"] = False
    # Clear the request so mode_manager returns to ARMED after a natural finish
    # (on web Stop / mode switch it is already being cleared by the caller).
    state.calibration_request = False

    # Restore the MGS1600 CAN driver we released at start.
    if can_was_enabled and manager is not None and not getattr(config, "CAN_ENABLED", False):
        logger.info("[Calib] restoring CAN (MGS1600) driver")
        try:
            await manager._apply("CAN_ENABLED", True)
        except Exception as exc:                       # noqa: BLE001
            logger.error("[Calib] failed to restore CAN driver: %s", exc)
    logger.info("[Calib] cleanup done")

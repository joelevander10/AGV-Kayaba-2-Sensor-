"""
plotter.py — AGV PID Run Analysis Plotter
==========================================
Records data during an AUTO run and saves a plot and CSV when the run ends.

This module is imported by modes.py. It does NOT run standalone.

USAGE
-----
Set PLOTTING_ENABLED = True to activate recording and plotting.
Set PLOTTING_ENABLED = False to disable with zero overhead — no data
is collected, no files are written, no matplotlib is even imported.

OUTPUT (both files share the same timestamp prefix)
------
PNG : analysis_plot/[YYYYMMDD_HHMMSS]_analysisplot.png
      - Left Y axis  : Left RPM and Right RPM (0–2000)
      - Right Y axis : Tracking error in mm (-100 to +100)
      - X axis       : Time in seconds from run start

CSV : analysis_plot/[YYYYMMDD_HHMMSS]_data.csv
      Columns: time_s, error_mm, left_rpm, right_rpm, pid_output, d_term

CALL SITE IN modes.py
---------------------
recorder.record(
    error_mm=e,
    left_rpm=left_rpm,
    right_rpm=right_rpm,
    pid_output=output,
    d_term=filtered_d
)
"""

import csv as csv_module
import logging
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

logger = logging.getLogger(__name__)

# ══════════════════════════════════════════════════════════════════════════════
#  USER SETTING
# ══════════════════════════════════════════════════════════════════════════════

PLOTTING_ENABLED = True

# Cap on samples kept in memory (≈20 min @100 Hz). Bounds RAM regardless of run
# length; on a longer run the plot/CSV reflect the most recent window.
PLOT_MAX_SAMPLES = 120000

# Repo-relative: <repo>/analysis_plot  (debugging/ is one level under the repo)
PLOT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "analysis_plot")


# ══════════════════════════════════════════════════════════════════════════════
#  RECORDER CLASS
# ══════════════════════════════════════════════════════════════════════════════

class RunRecorder:
    """
    Collects one data point per PID cycle during an AUTO run.
    Call start() when the run begins, record() each cycle, stop() when done.
    All methods are no-ops when PLOTTING_ENABLED is False.
    """

    def __init__(self):
        self._t0             = None
        self._times          = deque(maxlen=PLOT_MAX_SAMPLES)
        self._errors         = deque(maxlen=PLOT_MAX_SAMPLES)
        self._left_rpms      = deque(maxlen=PLOT_MAX_SAMPLES)
        self._right_rpms     = deque(maxlen=PLOT_MAX_SAMPLES)
        self._pid_outputs    = deque(maxlen=PLOT_MAX_SAMPLES)
        self._d_terms        = deque(maxlen=PLOT_MAX_SAMPLES)
        self._target_speeds  = deque(maxlen=PLOT_MAX_SAMPLES)
        self._active         = False

    def start(self):
        if not PLOTTING_ENABLED:
            return
        self._t0             = time.time()
        self._times          = deque(maxlen=PLOT_MAX_SAMPLES)
        self._errors         = deque(maxlen=PLOT_MAX_SAMPLES)
        self._left_rpms      = deque(maxlen=PLOT_MAX_SAMPLES)
        self._right_rpms     = deque(maxlen=PLOT_MAX_SAMPLES)
        self._pid_outputs    = deque(maxlen=PLOT_MAX_SAMPLES)
        self._d_terms        = deque(maxlen=PLOT_MAX_SAMPLES)
        self._target_speeds  = deque(maxlen=PLOT_MAX_SAMPLES)
        self._active         = True
        logger.info("[PLOT] Recording started.")

    def record(self, error_mm: float, left_rpm: float, right_rpm: float,
               pid_output: float, d_term: float, target_speed_ms: float = 0.0):
        """
        Call once per PID cycle inside auto_mode.

        Args:
            error_mm       — lateral error in mm
            left_rpm       — commanded left wheel RPM
            right_rpm      — commanded right wheel RPM
            pid_output     — combined PID output
            d_term         — filtered derivative term
            target_speed_ms — current desired speed setpoint in m/s
        """
        if not PLOTTING_ENABLED or not self._active:
            return
        self._times.append(time.time() - self._t0)
        self._errors.append(error_mm)
        self._left_rpms.append(left_rpm)
        self._right_rpms.append(right_rpm)
        self._pid_outputs.append(pid_output)
        self._d_terms.append(d_term)
        self._target_speeds.append(target_speed_ms)

    def stop(self):
        """
        Stop recording and save PNG + CSV.
        Safe to call multiple times — subsequent calls are no-ops.
        Does not raise even if recording was never started or had no data.
        """
        if not PLOTTING_ENABLED or not self._active:
            return
        self._active = False
        n = len(self._times)
        if n < 2:
            # Run was too short — this is normal for a tape-not-found abort
            # or an emergency stop in the first cycle. Not an error.
            logger.info("[PLOT] Run too short (%d samples) — no file saved.", n)
            return
        logger.info("[PLOT] Run ended — %d samples over %.1fs. Saving (background)...",
                    n, self._times[-1])
        # Copy to plain lists and save off-thread. stop() is called from
        # auto_mode's finally (often during cancellation), so it MUST NOT block
        # the event loop — matplotlib rendering can take hundreds of ms.
        args = (list(self._times), list(self._errors),
                list(self._left_rpms), list(self._right_rpms),
                list(self._pid_outputs), list(self._d_terms),
                list(self._target_speeds))
        threading.Thread(target=_save_safe, args=args, daemon=True).start()


# ══════════════════════════════════════════════════════════════════════════════
#  SAVE FUNCTION
# ══════════════════════════════════════════════════════════════════════════════

def _save_safe(times, errors, left_rpms, right_rpms, pid_outputs, d_terms,
               target_speeds):
    """Thread entry point — never let a plotting failure escape the worker."""
    try:
        _save_outputs(times, errors, left_rpms, right_rpms, pid_outputs,
                      d_terms, target_speeds)
    except Exception as e:
        logger.error("[PLOT] ERROR saving output: %s", e)


def _save_outputs(times, errors, left_rpms, right_rpms, pid_outputs, d_terms,
                  target_speeds):
    os.makedirs(PLOT_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path  = os.path.join(PLOT_DIR, f"{timestamp}_data.csv")
    png_path  = os.path.join(PLOT_DIR, f"{timestamp}_analysisplot.png")

    # ── CSV first — data is on disk before matplotlib even starts ─────────────
    with open(csv_path, "w", newline="") as f:
        writer = csv_module.writer(f)
        writer.writerow(["time_s", "error_mm", "left_rpm", "right_rpm",
                         "pid_output", "d_term", "target_speed_ms"])
        for row in zip(times, errors, left_rpms, right_rpms, pid_outputs,
                       d_terms, target_speeds):
            writer.writerow([
                f"{row[0]:.4f}",
                f"{row[1]:.3f}",
                f"{row[2]:.3f}",
                f"{row[3]:.3f}",
                f"{row[4]:.3f}",
                f"{row[5]:.3f}",
                f"{row[6]:.3f}",
            ])
    logger.info("[PLOT] CSV  saved → %s", csv_path)

    # ── PNG ───────────────────────────────────────────────────────────────────
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax_rpm = plt.subplots(figsize=(14, 6))
    ax_err = ax_rpm.twinx()

    line_l, = ax_rpm.plot(times, left_rpms,  color="#2196F3", linewidth=1.0,
                          label="Left RPM",  alpha=0.85)
    line_r, = ax_rpm.plot(times, right_rpms, color="#4CAF50", linewidth=1.0,
                          label="Right RPM", alpha=0.85)

    _k = 60 * 30 / (math.pi * 0.18)   # mps_to_rpm factor (wheel dia=0.18m, ratio=30)
    line_t, = ax_rpm.plot(times, [s * _k for s in target_speeds],
                          color="#FF9800", linewidth=1.3, linestyle="--",
                          label="Target RPM", alpha=0.9)

    ax_rpm.set_ylim(0, 2500)
    ax_rpm.set_ylabel("Wheel RPM", color="#333333")
    ax_rpm.tick_params(axis="y", labelcolor="#333333")
    ax_rpm.set_xlabel("Time (s)")
    ax_rpm.grid(True, linestyle="--", alpha=0.4)

    line_e, = ax_err.plot(times, errors, color="#F44336", linewidth=1.2,
                          label="Error (mm)", alpha=0.9)
    ax_err.axhline(0, color="#F44336", linewidth=0.6, linestyle=":")
    ax_err.set_ylim(-100, 100)
    ax_err.set_ylabel("Tracking Error (mm)", color="#F44336")
    ax_err.tick_params(axis="y", labelcolor="#F44336")

    lines  = [line_l, line_r, line_t, line_e]
    labels = [l.get_label() for l in lines]
    ax_rpm.legend(lines, labels, loc="upper left", fontsize=9)

    duration  = times[-1] if times else 0
    max_err   = max(abs(e) for e in errors) if errors else 0
    avg_left  = sum(left_rpms)  / len(left_rpms)  if left_rpms  else 0
    avg_right = sum(right_rpms) / len(right_rpms) if right_rpms else 0

    plt.title(
        f"AGV Auto Run  |  {timestamp}  |  "
        f"Duration={duration:.1f}s  "
        f"MaxErr={max_err:.1f}mm  "
        f"AvgL={avg_left:.0f}rpm  AvgR={avg_right:.0f}rpm",
        fontsize=10
    )

    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    logger.info("[PLOT] PNG  saved → %s", png_path)
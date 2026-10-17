"""
plotter.py — AGV PID run recorder (CSV + PNG per AUTO / autotune run)
=====================================================================
Records one row per PID cycle during an AUTO run (and a PID auto-tune run) and
saves a CSV + PNG when the run ends. Download them from the web PARAMS page
(RUN LOGS) to analyse oscillation.

OUTPUT (same timestamp prefix; <tag> = "auto" or "autotune")
    analysis_plot/YYYYMMDD_HHMMSS_<tag>_data.csv
    analysis_plot/YYYYMMDD_HHMMSS_<tag>_plot.png

CSV columns: see COLUMNS below. Missing values are left empty.

Set PLOTTING_ENABLED = False to disable with zero overhead.
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

PLOTTING_ENABLED = True

# Cap on rows kept in memory (≈10 min @100 Hz). On a longer run the files hold
# the most recent window.
PLOT_MAX_SAMPLES = 60000

# Repo-relative: <repo>/analysis_plot  (debugging/ is one level under the repo)
PLOT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "analysis_plot")

COLUMNS = [
    "time_s",            # s since run start
    "phase",             # autotune: warmup / relay / recover; auto: speed mode
    "speed_mode",
    "target_speed_ms",   # speed the mode/lidar asks for
    "cmd_speed_ms",      # ramped speed actually commanded
    "track",             # left / right track followed
    "raw_left_mm",       # MGS1600 left track position (as received)
    "raw_right_mm",      # MGS1600 right track position (as received)
    "pv_mm",             # position used by the PID (after range / slew checks)
    "error_mm",          # e = ERROR_SIGN * pv
    "p_term", "i_term", "d_term",
    "pid_output",        # clamped P+I+D (rpm differential)
    "ff_rpm",            # curvature feedforward
    "speed_red_rpm",     # PID speed reduction
    "left_rpm", "right_rpm",
    "kp", "td", "n", "v_red",
    "lidar_zone",
]


class RunRecorder:
    """record(**fields) once per cycle; stop() saves CSV + PNG off-thread."""

    def __init__(self, tag="auto"):
        self._tag = tag
        self._t0 = None
        self._rows = deque(maxlen=PLOT_MAX_SAMPLES)
        self._active = False

    def start(self):
        if not PLOTTING_ENABLED:
            return
        self._t0 = time.time()
        self._rows = deque(maxlen=PLOT_MAX_SAMPLES)
        self._active = True
        logger.info("[PLOT] Recording started (%s).", self._tag)

    def record(self, **fields):
        if not PLOTTING_ENABLED or not self._active:
            return
        fields["time_s"] = time.time() - self._t0
        self._rows.append(tuple(fields.get(c) for c in COLUMNS))

    def stop(self):
        if not PLOTTING_ENABLED or not self._active:
            return
        self._active = False
        n = len(self._rows)
        if n < 2:
            logger.info("[PLOT] Run too short (%d samples) — no file saved.", n)
            return
        logger.info("[PLOT] Run ended — %d samples over %.1fs. Saving (background)...",
                    n, self._rows[-1][0])
        threading.Thread(target=_save_safe, args=(list(self._rows), self._tag),
                         daemon=True).start()


def _save_safe(rows, tag):
    try:
        _save_outputs(rows, tag)
    except Exception as e:
        logger.error("[PLOT] ERROR saving output: %s", e)


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.4f}"
    return str(v)


def _save_outputs(rows, tag):
    os.makedirs(PLOT_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(PLOT_DIR, f"{stamp}_{tag}_data.csv")
    png_path = os.path.join(PLOT_DIR, f"{stamp}_{tag}_plot.png")

    with open(csv_path, "w", newline="") as f:
        w = csv_module.writer(f)
        w.writerow(COLUMNS)
        for r in rows:
            w.writerow([_fmt(v) for v in r])
    logger.info("[PLOT] CSV saved → %s", csv_path)

    col = {c: i for i, c in enumerate(COLUMNS)}
    def series(name):
        i = col[name]
        return [(r[i] if r[i] is not None else float("nan")) for r in rows]
    t = series("time_s")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_e, ax_u, ax_v) = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    ax_e.plot(t, series("error_mm"), color="#D32F2F", lw=1.1, label="error (mm)")
    ax_e.axhline(0, color="#999", lw=0.6)
    ax_e.set_ylabel("error mm"); ax_e.grid(True, ls="--", alpha=0.4); ax_e.legend(loc="upper right", fontsize=8)

    ax_u.plot(t, series("p_term"), color="#1976D2", lw=0.9, label="P")
    ax_u.plot(t, series("d_term"), color="#7B1FA2", lw=0.9, label="D")
    ax_u.plot(t, series("i_term"), color="#388E3C", lw=0.9, label="I")
    ax_u.plot(t, series("pid_output"), color="#000", lw=1.1, label="output")
    ax_u.axhline(0, color="#999", lw=0.6)
    ax_u.set_ylabel("steer rpm"); ax_u.grid(True, ls="--", alpha=0.4); ax_u.legend(loc="upper right", fontsize=8, ncol=4)

    k = 60 * 30 / (math.pi * 0.18)   # m/s → motor rpm (wheel 0.18 m, gear 30)
    ax_v.plot(t, series("left_rpm"), color="#2196F3", lw=0.9, label="left rpm")
    ax_v.plot(t, series("right_rpm"), color="#4CAF50", lw=0.9, label="right rpm")
    ax_v.plot(t, [v * k for v in series("cmd_speed_ms")], color="#FF9800", lw=1.2, ls="--", label="commanded")
    ax_v.set_ylabel("motor rpm"); ax_v.set_xlabel("time (s)")
    ax_v.grid(True, ls="--", alpha=0.4); ax_v.legend(loc="upper right", fontsize=8, ncol=3)

    errs = [abs(x) for x in series("error_mm") if x == x]
    fig.suptitle(f"AGV {tag} run {stamp} · {t[-1]:.1f}s · max |e| {max(errs) if errs else 0:.1f} mm", fontsize=10)
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)
    logger.info("[PLOT] PNG saved → %s", png_path)

"""
trip_logger.py — timestamped trip / event logs per branch (CSV)
================================================================
Folder layout (data/trip_logs/, downloadable from the web LOGS page):

    data/trip_logs/
        branch1_UBC/      2026-10-22_trips.csv   2026-10-22_events.csv
        branch2_Piston/   ...
        branch3_ITG/      ...
        branch4/          ...
        no_branch/        trips started without a branch (e.g. CALL START)
        general/          events while no trip is open (idle, manual, ...)

*_trips.csv  — one row per trip, homepost → homepost:
               start/end time, duration, end reason, RFID reads, track
               losses, errors.
*_speed.csv  — speed every SPEED_SAMPLE_S during a trip: target speed,
               commanded speed (wheel average), left / right wheel, distance.
*_events.csv — every event of the trip with a timestamp (and the speed):
               TRIP_START / TRIP_END, RFID (tag + sequences fired),
               TRACK_LOST / TRACK_BACK (+ how long), MODE changes and every
               WARNING / ERROR / CRITICAL log line (CAN sensor, watchdog,
               emergency, ...).

All file writing happens on a background thread (event() only queues), so
the control loop is never blocked by the disk.
"""
import asyncio
import csv
import logging
import os
import queue
import re
import threading
import time
from datetime import datetime, timedelta

import config

logger = logging.getLogger(__name__)

EVENT_COLS = ["time", "trip_id", "branch", "type", "detail", "mode", "speed_mode", "speed_ms"]
TRIP_COLS  = ["trip_id", "branch", "branch_name", "start_time", "end_time",
              "duration_s", "duration", "driving", "stopped", "start_point", "end_reason",
              "distance_m", "avg_speed_ms", "max_speed_ms",
              "rfid_reads", "track_lost", "track_lost_s", "errors"]
SPEED_COLS = ["time", "trip_id", "branch", "mode", "speed_mode", "target_ms", "speed_ms",
              "left_ms", "right_ms", "distance_m"]
FILE_RE    = re.compile(r"^\d{4}-\d{2}-\d{2}_(trips|events|speed)(-old\d+)?\.csv$")
FOLDER_RE  = re.compile(r"^[A-Za-z0-9_\-]+$")

_q      = queue.Queue()
_lock   = threading.Lock()
_trip   = None          # open trip dict, or None
_state  = None          # AMRState, for mode / speed columns
_thread = None


def _now_str(ts=None):
    return datetime.fromtimestamp(ts or time.time()).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _hms(sec):
    sec = int(round(sec))
    return f"{sec // 3600:02d}:{sec % 3600 // 60:02d}:{sec % 60:02d}"


def folder_for(branch):
    if branch is None:
        return "no_branch"
    b = str(branch)
    name = config.TRIP_BRANCH_NAMES.get(b)
    return f"branch{b}_{name}" if name else f"branch{b}"


# ── Writer thread ─────────────────────────────────────────────────────────────

def _append(folder, fname, cols, row):
    d = os.path.join(config.TRIP_LOG_DIR, folder)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, fname)
    new = not os.path.exists(path)
    if not new:
        # File from an older version with other columns: keep it as -oldN
        # and start a fresh one, so columns never shift.
        with open(path, encoding="utf-8-sig") as f:
            header = f.readline().strip()
        if header != ",".join(cols):
            stem = fname[:-4]
            n = 1
            while os.path.exists(os.path.join(d, f"{stem}-old{n}.csv")):
                n += 1
            os.rename(path, os.path.join(d, f"{stem}-old{n}.csv"))
            new = True
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        if new:
            f.write("\ufeff")          # BOM so Excel opens the UTF-8 file correctly
            w.writeheader()
        w.writerow(row)


def _cleanup():
    """Delete daily files older than KEEP_DAYS (by the date in the name)."""
    if config.TRIP_KEEP_DAYS <= 0:
        return
    cutoff = (datetime.now() - timedelta(days=config.TRIP_KEEP_DAYS)).strftime("%Y-%m-%d")
    try:
        for folder in os.listdir(config.TRIP_LOG_DIR):
            d = os.path.join(config.TRIP_LOG_DIR, folder)
            if not os.path.isdir(d):
                continue
            for n in os.listdir(d):
                if FILE_RE.match(n) and n[:10] < cutoff:
                    os.remove(os.path.join(d, n))
    except FileNotFoundError:
        pass


def _writer():
    _cleanup()
    last_cleanup = time.time()
    while True:
        kind, folder, row = _q.get()
        day = row.get("time", row.get("end_time", ""))[:10] or datetime.now().strftime("%Y-%m-%d")
        try:
            if kind == "event":
                _append(folder, f"{day}_events.csv", EVENT_COLS, row)
            elif kind == "speed":
                _append(folder, f"{day}_speed.csv", SPEED_COLS, row)
            else:
                _append(folder, f"{day}_trips.csv", TRIP_COLS, row)
        except Exception as e:      # never let logging kill the thread
            print(f"[trip_logger] write failed: {e}")
        if time.time() - last_cleanup > 6 * 3600:
            _cleanup()
            last_cleanup = time.time()


def init(state):
    """Start the writer thread. Call once from main.py."""
    global _state, _thread
    _state = state
    if _thread is None:
        os.makedirs(config.TRIP_LOG_DIR, exist_ok=True)
        _thread = threading.Thread(target=_writer, name="trip-logger", daemon=True)
        _thread.start()


# ── Public API ────────────────────────────────────────────────────────────────

def event(etype, detail=""):
    """Queue one timestamped event (any thread, never blocks)."""
    if not config.TRIP_LOG_ENABLED:
        return
    with _lock:
        trip = _trip
        if trip is not None:
            if etype == "RFID":
                trip["rfid_reads"] += 1
            elif etype == "TRACK_LOST":
                trip["track_lost"] += 1
            elif etype in ("ERROR", "CRITICAL", "WARNING"):
                trip["errors"] += 1
        folder = folder_for(trip["branch"]) if trip else "general"
        row = {
            "time":       _now_str(),
            "trip_id":    trip["trip_id"] if trip else "",
            "branch":     (trip["branch"] or "") if trip else "",
            "type":       etype,
            "detail":     detail,
            "mode":       getattr(_state, "current_mode", "") if _state else "",
            "speed_mode": getattr(_state, "speed_mode", "") if _state else "",
            "speed_ms":   round(_speed_now()[1], 3) if _state else "",
        }
    _q.put(("event", folder, row))


def track_lost_time(seconds):
    with _lock:
        if _trip is not None:
            _trip["track_lost_s"] += seconds


def start_trip(branch, start_point="homepost"):
    """Open a trip (AUTO start). An open trip is closed first as 'not back at
    homepost' when a new one starts from homepost."""
    global _trip
    if _trip is not None:
        end_trip("restarted before reaching homepost")
    now = time.time()
    b = str(branch) if branch is not None else None
    with _lock:
        _trip = {
            "trip_id":     datetime.fromtimestamp(now).strftime("%Y%m%d-%H%M%S") + f"-B{b or '0'}",
            "branch":      b,
            "start_ts":    now,
            "start_point": start_point,
            "rfid_reads":  0, "track_lost": 0, "track_lost_s": 0.0, "errors": 0,
            "distance_m":  0.0, "driving_s": 0.0, "stopped_s": 0.0, "max_speed": 0.0,
        }
    event("TRIP_START", f"branch {b or '-'} ({config.TRIP_BRANCH_NAMES.get(b or '', '')}) from {start_point}")


def set_branch(branch):
    """Branch chosen after the trip started (e.g. CALL START run, then a
    branch button at B0). Later events/summary go to that branch's folder."""
    with _lock:
        if _trip is not None and _trip["branch"] is None and branch is not None:
            _trip["branch"] = str(branch)
            changed = True
        else:
            changed = False
    if changed:
        event("BRANCH", f"branch {branch} selected during trip")


def end_trip(reason="homepost"):
    global _trip
    with _lock:
        trip = _trip
    if trip is None:
        return
    event("TRIP_END", reason)
    now = time.time()
    dur = now - trip["start_ts"]
    b = trip["branch"]
    row = {
        "trip_id":      trip["trip_id"],
        "branch":       b or "",
        "branch_name":  config.TRIP_BRANCH_NAMES.get(b or "", ""),
        "start_time":   _now_str(trip["start_ts"]),
        "end_time":     _now_str(now),
        "duration_s":   round(dur, 1),
        "duration":     _hms(dur),
        "driving":      _hms(trip["driving_s"]),
        "stopped":      _hms(trip["stopped_s"]),
        "start_point":  trip["start_point"],
        "end_reason":   reason,
        "distance_m":   round(trip["distance_m"], 1),
        "avg_speed_ms": round(trip["distance_m"] / trip["driving_s"], 3) if trip["driving_s"] > 0 else 0.0,
        "max_speed_ms": round(trip["max_speed"], 3),
        "rfid_reads":   trip["rfid_reads"],
        "track_lost":   trip["track_lost"],
        "track_lost_s": round(trip["track_lost_s"], 1),
        "errors":       trip["errors"],
    }
    with _lock:
        _trip = None
    _q.put(("trip", folder_for(b), row))
    logger.info("[TRIP] %s — branch %s, %s (%s)", reason, b or "-", _hms(dur), row["trip_id"])


def current():
    """Snapshot of the open trip for the web page (or None)."""
    with _lock:
        if _trip is None:
            return None
        t = dict(_trip)
    t["elapsed"] = _hms(time.time() - t.pop("start_ts"))
    t["driving"] = _hms(t.pop("driving_s"))
    t["stopped"] = _hms(t.pop("stopped_s"))
    t["distance_m"] = round(t["distance_m"], 1)
    t["max_speed"] = round(t["max_speed"], 3)
    t["speed_ms"] = round(_speed_now()[1], 3) if _state else 0.0
    t["branch_name"] = config.TRIP_BRANCH_NAMES.get(t["branch"] or "", "")
    return t


# ── Speed ─────────────────────────────────────────────────────────────────────

_MOVING_MODES = ("running", "reversing", "autotune", "rotating")


def _rpm_to_ms(rpm):
    return rpm / config.GEAR_RATIO * config.WHEEL_CIRCUMFERENCE / 60.0


def _speed_now():
    """(target m/s, commanded m/s, left m/s, right m/s) — commanded = wheel
    average (includes the PID speed reduction). 0 when not driving.
    Negative = reversing."""
    s = _state
    if s is None or getattr(s, "current_mode", None) not in _MOVING_MODES \
            or getattr(s, "emergency_active", False):
        return 0.0, 0.0, 0.0, 0.0
    tgt = float(getattr(s, "target_speed", 0.0) or 0.0)
    if abs(tgt) < 1e-6:
        return 0.0, 0.0, 0.0, 0.0
    sign = -1.0 if tgt < 0 else 1.0
    l = abs(_rpm_to_ms(float(getattr(s, "left_rpm", 0.0) or 0.0))) * sign
    r = abs(_rpm_to_ms(float(getattr(s, "right_rpm", 0.0) or 0.0))) * sign
    return tgt, (l + r) / 2.0, l, r


async def speed_sampler(state):
    """Integrates distance / driving / stopped time every 0.2 s and writes a
    speed row every SPEED_SAMPLE_S while a trip is open. Run as a task."""
    step = 0.2
    last = time.time()
    last_row = 0.0
    while True:
        await asyncio.sleep(step)
        now = time.time()
        dt = min(now - last, 1.0)
        last = now
        tgt, v, l, r = _speed_now()
        with _lock:
            trip = _trip
            if trip is None:
                continue
            av = abs(v)
            trip["distance_m"] += av * dt
            if av > 0.02:
                trip["driving_s"] += dt
            else:
                trip["stopped_s"] += dt
            trip["max_speed"] = max(trip["max_speed"], av)
            dist = trip["distance_m"]
            tid, b = trip["trip_id"], trip["branch"]
        if not config.TRIP_LOG_ENABLED or now - last_row < config.TRIP_SPEED_SAMPLE_S:
            continue
        last_row = now
        _q.put(("speed", folder_for(b), {
            "time": _now_str(now), "trip_id": tid, "branch": b or "",
            "mode": getattr(state, "current_mode", ""), "speed_mode": getattr(state, "speed_mode", ""),
            "target_ms": round(tgt, 3), "speed_ms": round(v, 3),
            "left_ms": round(l, 3), "right_ms": round(r, 3), "distance_m": round(dist, 2),
        }))


class TripLogHandler(logging.Handler):
    """Copies WARNING / ERROR / CRITICAL log lines into the trip events."""
    def emit(self, record):
        if record.name == __name__:
            return
        try:
            event(record.levelname, f"{record.name}: {record.getMessage()}")
        except Exception:
            pass

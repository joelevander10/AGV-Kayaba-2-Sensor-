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
*_events.csv — every event of the trip with a timestamp:
               TRIP_START / TRIP_END, RFID (tag + sequences fired),
               TRACK_LOST / TRACK_BACK (+ how long), MODE changes and every
               WARNING / ERROR / CRITICAL log line (CAN sensor, watchdog,
               emergency, ...).

All file writing happens on a background thread (event() only queues), so
the control loop is never blocked by the disk.
"""
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

EVENT_COLS = ["time", "trip_id", "branch", "type", "detail", "mode", "speed_mode"]
TRIP_COLS  = ["trip_id", "branch", "branch_name", "start_time", "end_time",
              "duration_s", "duration", "start_point", "end_reason",
              "rfid_reads", "track_lost", "track_lost_s", "errors"]
FILE_RE    = re.compile(r"^\d{4}-\d{2}-\d{2}_(trips|events)\.csv$")
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
        "start_point":  trip["start_point"],
        "end_reason":   reason,
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
    t["branch_name"] = config.TRIP_BRANCH_NAMES.get(t["branch"] or "", "")
    return t


class TripLogHandler(logging.Handler):
    """Copies WARNING / ERROR / CRITICAL log lines into the trip events."""
    def emit(self, record):
        if record.name == __name__:
            return
        try:
            event(record.levelname, f"{record.name}: {record.getMessage()}")
        except Exception:
            pass

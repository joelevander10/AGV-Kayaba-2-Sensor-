"""
app/app.py — AGV Web Monitoring Dashboard
==========================================
Flask server running in a daemon thread alongside the asyncio main loop.
Shares the AMRState object directly (GIL-safe for reads; single-attribute
writes for individual state fields).

Access from any device on 192.168.2.x: http://192.168.2.100:5000
"""

import os
import re
import subprocess
import time
import threading
import functools
import secrets
import sys
from flask import Flask, render_template, jsonify, request, session, send_from_directory, abort
# Make the project root importable even when this file is started from the
# app/ folder or by an editor (engineer_auth, logger and config live there).
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _root not in sys.path:
    sys.path.insert(0, _root)
import engineer_auth
import keyboard_control
from logger import get_error_log
import logging

logger = logging.getLogger(__name__)

# Flask looks for templates relative to the app.py file location
_here = os.path.dirname(os.path.abspath(__file__))
app   = Flask(__name__, template_folder=os.path.join(_here, "templates"))
# Session cookie for the engineer login. A fresh random key per start means
# every login ends when the controller restarts.
app.secret_key = secrets.token_bytes(32)
app.config.update(SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE="Strict")

# ── Engineer login (protects tuning / IO test / bypass / calibration) ────────
ENGINEER_SESSION_S = 30 * 60
_login_fail = {"count": 0, "locked_until": 0.0}

def _engineer_ok():
    return session.get("eng_until", 0) > time.time()

def _locked_response():
    return jsonify({"error": "Engineer login required — tap ENGINEER in the bottom bar.",
                    "locked": True}), 401

def engineer_required(fn):
    @functools.wraps(fn)
    def wrapper(*a, **k):
        if not _engineer_ok():
            return _locked_response()
        return fn(*a, **k)
    return wrapper


@app.route("/api/engineer/status")
def api_engineer_status():
    remaining = max(0, int(session.get("eng_until", 0) - time.time()))
    return jsonify({"logged_in": remaining > 0, "remaining_s": remaining,
                    "password_set": engineer_auth.is_set()})


@app.route("/api/engineer/login", methods=["POST"])
def api_engineer_login():
    now = time.time()
    if now < _login_fail["locked_until"]:
        return jsonify({"error": f"Too many wrong passwords — wait {int(_login_fail['locked_until'] - now)} s."}), 429
    if not engineer_auth.is_set():
        return jsonify({"error": "No engineer password set on this AGV yet."}), 409
    data = request.get_json(silent=True) or {}
    if engineer_auth.verify(data.get("password", "")):
        _login_fail["count"] = 0
        session["eng_until"] = now + ENGINEER_SESSION_S
        logger.warning("[ENGINEER] login from %s", request.remote_addr)
        return jsonify({"ok": True, "remaining_s": ENGINEER_SESSION_S})
    _login_fail["count"] += 1
    time.sleep(1.0)                           # slow down guessing
    if _login_fail["count"] >= 5:
        _login_fail["count"] = 0
        _login_fail["locked_until"] = time.time() + 60
        logger.warning("[ENGINEER] 5 wrong passwords from %s — login blocked 60 s", request.remote_addr)
    return jsonify({"error": "Wrong password."}), 403


@app.route("/api/engineer/logout", methods=["POST"])
def api_engineer_logout():
    session.pop("eng_until", None)
    return jsonify({"ok": True})

_state   = None   # set once by run_server()
_engine  = None   # set once by run_server()
_manager = None   # set once by run_server() — toggles the RFID driver live

def _build_di_names():
    import config
    names = ["[SPARE]"] * config.NUM_DI
    mapping = {
        config.DI_START:       "START",
        config.DI_RESET:       "RESET",
        config.DI_MODE_SWITCH: "MODE SWITCH",
        config.DI_EMERGENCY:   "EMERGENCY",
        config.DI_FWD:         "FWD",
        config.DI_REV:         "REV",
        config.DI_LEFT:        "LEFT",
        config.DI_RIGHT:       "RIGHT",
    }
    for idx, label in mapping.items():
        if 0 <= idx < config.NUM_DI:
            names[idx] = label
    for i, idx in enumerate(config.DI_BRANCH_SELECT):
        if 0 <= idx < config.NUM_DI:
            names[idx] = f"BRANCH {i + 1} SELECT"
    _tl = config.DI_TROLLEY_LIMIT
    if len(_tl) == 2 and _tl[0] == _tl[1]:
        # Both switches wired (in parallel) onto one input.
        if 0 <= _tl[0] < config.NUM_DI:
            names[_tl[0]] = "TROLLEY LIMIT 1+2 (NC)"
    else:
        for i, idx in enumerate(_tl):
            if 0 <= idx < config.NUM_DI:
                names[idx] = f"TROLLEY LIMIT {i + 1} (NC)"
    if 0 <= config.DI_PUSHER_PROX_TOP < config.NUM_DI:
        names[config.DI_PUSHER_PROX_TOP] = "PUSHER PROX TOP"
    if 0 <= config.DI_PUSHER_PROX_BOTTOM < config.NUM_DI:
        names[config.DI_PUSHER_PROX_BOTTOM] = "PUSHER PROX BOTTOM"
    _nc = " NC" if config.LIDAR_NORMALLY_CLOSED else ""
    for _idx, _lbl in ((config.LIDAR_DI_OUTER, f"LIDAR OUTER{_nc} ({config.LIDAR_OUTER_PERCENT:g}%)"),
                       (config.LIDAR_DI_MIDDLE, f"LIDAR MIDDLE{_nc} ({config.LIDAR_MIDDLE_PERCENT:g}%)"),
                       (config.LIDAR_DI_INNER, f"LIDAR INNER{_nc} (STOP)")):
        if 0 <= _idx < config.NUM_DI:
            names[_idx] = _lbl
    if 0 <= config.DI_PUSHER_BUTTON < config.NUM_DI:
        names[config.DI_PUSHER_BUTTON] = "PUSHER BUTTON (UP/DOWN)"
    if 0 <= config.DI_REVERSE_MARKER < config.NUM_DI:
        names[config.DI_REVERSE_MARKER] = "REVERSE STOP MARKER"
    if 0 <= config.DI_CALL_START < config.NUM_DI:
        names[config.DI_CALL_START] = "CALL START (to B0)"
    return names

def _build_do_names():
    import config
    names = ["[SPARE]"] * config.NUM_DO
    mc = config.MOTOR_CHANNELS
    do_map = {
        mc["left"]["do_fwd"]:    "LEFT FWD",
        mc["left"]["do_rev"]:    "LEFT REV",
        mc["left"]["do_brake"]:  "LEFT BRAKE",
        mc["right"]["do_fwd"]:   "RIGHT FWD",
        mc["right"]["do_rev"]:   "RIGHT REV",
        mc["right"]["do_brake"]: "RIGHT BRAKE",
    }
    for idx, label in do_map.items():
        if 0 <= idx < config.NUM_DO:
            names[idx] = label
    for idx in config.PUSHER_DO_UP:
        if 0 <= idx < config.NUM_DO:
            names[idx] = "PUSHER UP"
    for idx in config.PUSHER_DO_DOWN:
        if 0 <= idx < config.NUM_DO:
            names[idx] = "PUSHER DOWN"
    if 0 <= config.DO_HORN_AUTO < config.NUM_DO:
        names[config.DO_HORN_AUTO] = "HORN (AUTO)"
    if 0 <= config.DO_HORN_ALARM < config.NUM_DO:
        names[config.DO_HORN_ALARM] = "HORN (ALARM)"
    for _k, _ch in enumerate(config.LIDAR_DO_AREA):
        if 0 <= _ch < config.NUM_DO:
            names[_ch] = f"LIDAR AREA SEL {_k + 1}"
    if 0 <= config.DO_AT_HOME < config.NUM_DO:
        names[config.DO_AT_HOME] = "AT HOMEPOST"
    return names


def _build_state_snapshot():
    """Derive the full JSON payload from the shared state object."""
    import config
    s = _state
    mode = s.current_mode

    # ── AGV summary ───────────────────────────────────────────────────────────
    agv = {
        "mode":                 mode,
        "emergency":            s.emergency_active,
        "system_error":         s.system_error,
        "system_error_detail":  s.system_error_detail,
        "speed_mode":           s.speed_mode,
        "track_follow":         s.track_follow,
        "reverse_track":        s.reverse_track,
        "active_branch":        s.active_branch,
        "at_homepost":          s.at_homepost,
        "lidar_enabled":        config.LIDAR_ENABLED,
        "lidar_zone":           s.lidar_zone,
        "lidar_area":           s.lidar_area,
        "call_run":             s.call_run,
        "sequence_stop":        s.sequence_stop,
        "latest_rfid_tag":      s.latest_rfid_tag,
        "web_button_mode":      s.web_button_mode,
        "kb_estop":             s.kb_estop,
        "kb_mode_override":     s.kb_mode_override,
        "keypad_devices":       list(s.keypad_devices),
        "web_btn_manual":       s.web_btn_manual,
        "web_btn_emergency":    s.web_btn_emergency,
        "trolley_hooked":       s.trolley_hooked,
        "pusher_status":        s.pusher_status,
        "pusher_up_request":    s.pusher_up_request,
        "pusher_down_request":  s.pusher_down_request,
        "pusher_jog_cmd":       s.pusher_jog_cmd,
        "pusher_estop":         s.pusher_estop,
        "alarm_active":         s.alarm_active,
    }

    # ── Magnetic sensor (MGS1600 via CAN) ────────────────────────────────────
    raw = s.latest_sensor
    sensor = {
        "left_mm":        raw["left_mm"]        if raw else None,
        "tape_detected":  raw["tape_detected"]   if raw else False,
        "left_marker":    raw["left_marker"]     if raw else False,
        "right_marker":   raw["right_marker"]    if raw else False,
        "sensor_failure": raw["sensor_failure"]  if raw else False,
        "left_rpm":       round(s.left_rpm,  1),
        "right_rpm":      round(s.right_rpm, 1),
        # Actual per-wheel linear velocity (m/s) from motor RPM:
        #   wheel_rpm = motor_rpm / GEAR_RATIO; v = wheel_rpm * circ / 60
        "v_left":         round(s.left_rpm  / config.GEAR_RATIO * config.WHEEL_CIRCUMFERENCE / 60.0, 3),
        "v_right":        round(s.right_rpm / config.GEAR_RATIO * config.WHEEL_CIRCUMFERENCE / 60.0, 3),
        "pid_output":     round(s.pid_output, 1),
        "target_speed":   round(s.target_speed, 3),
        "speed_mode":     s.speed_mode,
    }

    # ── Rear magnetic sensor (second MGS1600, REVERSE only) ───────────────────
    # No track_follow here — REVERSE always follows right_mm, unconditionally.
    # "active" is NOT just "did we ever see a frame" — it's "did a frame land
    # recently" (age <= CAN_TIMEOUT). Without this, a rear sensor that died
    # would keep showing its last-known TAPE/position forever on the
    # dashboard, since latest_rear_sensor is never cleared on its own.
    raw_rear      = s.latest_rear_sensor
    rear_age_s    = (time.time() - s.rear_can_last_rx) if s.rear_can_last_rx else None
    rear_active   = bool(raw_rear) and rear_age_s is not None and rear_age_s <= config.CAN_TIMEOUT
    rear_sensor = {
        "enabled":        config.REAR_SENSOR_ENABLED,
        "active":         rear_active,
        "age_s":          round(rear_age_s, 2) if rear_age_s is not None else None,
        "left_mm":        raw_rear["left_mm"]       if raw_rear else None,
        "right_mm":       raw_rear["right_mm"]      if raw_rear else None,
        "tape_detected":  raw_rear["tape_detected"]  if raw_rear else False,
        "sensor_failure": raw_rear["sensor_failure"] if raw_rear else False,
    }

    # ── Sequence engine status ────────────────────────────────────────────────
    sequences = _engine.status() if _engine is not None else {
        "active": None, "armed": None, "cooldowns": {}, "last_event": None
    }

    # ── Driver frame rates (DI / CAN / RFID) ──────────────────────────────────
    # The CAN rate is the one that matters for PID tuning: the control loop only
    # advances when a sensor frame lands, so this IS the effective loop rate.
    drivers = _manager.driver_health() if _manager is not None else []

    return {
        "agv": agv, "sensor": sensor, "rear_sensor": rear_sensor,
        "sequences": sequences, "drivers": drivers,
    }


_VALID_COMMANDS = {"forward", "fwd_left", "fwd_right", "reverse",
                   "rvs_left", "rvs_right", "left", "right"}  # manual pendant jog

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/manual")
def manual():
    return render_template("manual.html")

@app.route("/api/manual/command", methods=["POST"])
def api_manual_command():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if _state.current_mode != "manual":
        return jsonify({"error": f"AGV not in manual mode (current: {_state.current_mode})"}), 403

    data = request.get_json(silent=True) or {}
    cmd  = data.get("command")   # None = stop

    if cmd is not None and cmd not in _VALID_COMMANDS:
        return jsonify({"error": f"unknown command: {cmd}"}), 400

    _state.web_manual_command = cmd
    _state.web_manual_expire  = time.time() + 0.4   # 400 ms watchdog
    return jsonify({"ok": True, "command": cmd})


@app.route("/api/keyboard", methods=["POST"])
def api_keyboard():
    """One-press keyboard action from the MANUAL page (laptop keyboard):
    {"key": "1".."5" | "7".."9" | "0" | "z" | "x" | "r" | "q" | "e"}.
    Movement keys (WASD) use /api/manual/command like the jog buttons.
    Same key map as the USB keypad on the AGV PC — see keyboard_control.py."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    data = request.get_json(silent=True) or {}
    key = str(data.get("key", "")).lower()
    if key not in keyboard_control.ACTION_KEYS:
        return jsonify({"error": f"unknown key '{key}'"}), 400
    ok, msg = keyboard_control.handle_key(_state, key, source="web keyboard")
    return jsonify({"ok": ok, "message": msg}), (200 if ok else 409)


# ── Pusher manual control (web dashboard) ──────────────────────────────────
# Three independent controls, see pusher.py's module docstring for how they're
# arbitrated (estop > emergency > jog > full-travel > autonomous sequence):
#   /api/pusher/move  — "full travel": one press, runs to the proximity sensor
#   /api/pusher/jog   — hold-to-run: browser must keep re-sending while held
#   /api/pusher/estop — pusher-only emergency stop (latches until cleared)

@app.route("/api/pusher/move", methods=["POST"])
def api_pusher_move():
    """Full-travel UP/DOWN — one press latches the request; pusher.py runs to
    the respective proximity sensor and clears it automatically on arrival
    (or on timeout/estop)."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if _state.pusher_estop:
        return jsonify({"error": "pusher e-stop is active — clear it first"}), 403

    data      = request.get_json(silent=True) or {}
    direction = data.get("direction")
    if direction not in ("up", "down"):
        return jsonify({"error": "direction must be 'up' or 'down'"}), 400

    if direction == "up":
        _state.pusher_up_request = True
    else:
        _state.pusher_down_request = True
    return jsonify({"ok": True, "direction": direction})


@app.route("/api/pusher/jog", methods=["POST"])
def api_pusher_jog():
    """Hold-to-run UP/DOWN. The browser must keep calling this (e.g. every
    ~150-200ms) for as long as the button is held — each call refreshes a
    400ms watchdog; pusher.py stops the instant the deadline lapses without a
    fresh call, same as it would on release. Call with direction=null once on
    release for an immediate stop instead of waiting out the watchdog."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    data      = request.get_json(silent=True) or {}
    direction = data.get("direction")   # "up" | "down" | None (release)
    if direction not in ("up", "down", None):
        return jsonify({"error": "direction must be 'up', 'down', or null"}), 400

    if direction is None:
        _state.pusher_jog_cmd = None
        return jsonify({"ok": True, "direction": None})

    if _state.pusher_estop:
        return jsonify({"error": "pusher e-stop is active — clear it first"}), 403

    _state.pusher_jog_cmd    = direction
    _state.pusher_jog_expire = time.time() + 0.4   # 400 ms watchdog
    return jsonify({"ok": True, "direction": direction})


@app.route("/api/reverse", methods=["POST"])
def api_reverse():
    """AUTO REVERSE (PID tape-follow backwards) from the MANUAL page.
    {"action": "start"} — only in ARMED, needs tape under the reverse sensor.
    {"action": "stop"}  — only while REVERSING; drops back to ARMED."""
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    data   = request.get_json(silent=True) or {}
    action = data.get("action")
    mode   = _state.current_mode

    if action == "start":
        if mode != "armed":
            return jsonify({"error": f"AGV must be ARMED (AUTO selector) to start reverse — now {mode}"}), 409
        # Same sensor reverse_mode will follow: rear if fitted, else front.
        if config.REAR_SENSOR_ENABLED:
            raw   = _state.latest_rear_sensor
            age   = (time.time() - _state.rear_can_last_rx) if _state.rear_can_last_rx else None
            fresh = age is not None and age <= config.CAN_TIMEOUT
            tape  = bool(raw) and fresh and raw.get("tape_detected", False)
            which = "rear"
        else:
            raw   = _state.latest_sensor
            tape  = bool(raw) and raw.get("tape_detected", False)
            which = "front"
        if not tape:
            return jsonify({"error": f"no tape under the {which} sensor"}), 409
        _state.web_reverse_start = True
        logger.info("[Reverse] START requested via web MANUAL page")
        return jsonify({"ok": True, "action": "start"})

    if action == "stop":
        if mode != "reversing":
            return jsonify({"error": f"AGV is not reversing — now {mode}"}), 409
        _state.web_reverse_stop = True
        logger.info("[Reverse] STOP requested via web MANUAL page")
        return jsonify({"ok": True, "action": "stop"})

    return jsonify({"error": "action must be 'start' or 'stop'"}), 400


@app.route("/api/pusher/estop", methods=["POST"])
def api_pusher_estop():
    """Pusher-only emergency stop — latches until explicitly cleared. While
    active, pusher.py de-energizes immediately and drops any pending
    up/down/jog request so nothing resumes on its own once cleared."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    data   = request.get_json(silent=True) or {}
    enable = data.get("enable")
    if not isinstance(enable, bool):
        return jsonify({"error": "enable must be true or false"}), 400

    _state.pusher_estop = enable
    logger.warning("[Pusher] E-STOP %s via web dashboard", "ENGAGED" if enable else "CLEARED")
    return jsonify({"ok": True, "pusher_estop": enable})


@app.route("/api/state")
def api_state():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    return jsonify(_build_state_snapshot())


_VALID_BRANCHES = {"1", "2", "3", "4"}

@app.route("/api/branch", methods=["POST"])
def api_branch():
    """Select (or clear) the active destination branch. Gates RFID fork tags
    (AA/AB/AC — see profile 'requires_branch') so the AGV only reacts to the
    tag for the branch it was actually dispatched to. Normally cleared by the
    BB merge-tag sequence action, not by this endpoint — but null is accepted
    here too for a manual override / abort."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    mode = _state.current_mode
    if mode in ("running", "reversing", "emergency", "autotune"):
        return jsonify({"error": f"Cannot change destination while AGV is {mode}"}), 403

    data   = request.get_json(silent=True) or {}
    branch = data.get("branch")

    if branch is not None and branch not in _VALID_BRANCHES:
        return jsonify({"error": f"branch must be one of {sorted(_VALID_BRANCHES)} or null"}), 400

    _state.active_branch = branch
    return jsonify({"ok": True, "branch": branch})


@app.route("/errors")
def errors():
    return render_template("errors.html")


@app.route("/api/errors")
def api_errors():
    return jsonify(get_error_log())


@app.route("/io")
def io_monitor():
    return render_template("io_monitor.html")


@app.route("/params")
def params():
    return render_template("params.html")


@app.route("/api/io")
def api_io():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    di = _state.latest_di or [False] * config.NUM_DI
    do_raw = _state.latest_do
    return jsonify({
        "di":       list(di[:config.NUM_DI]),
        "di_names": _build_di_names(),
        "do":       [do_raw.get(i, False) for i in range(config.NUM_DO)],
        "do_names": _build_do_names(),
        "num_di":   config.NUM_DI,
        "num_do":   config.NUM_DO,
        "io_test_mode": _state.io_test_mode,
        "do_forced":    {str(k): v for k, v in _state.do_override.items()},
        "mode":         _state.current_mode,
    })


# ── IO TEST MODE (web IO page) ──────────────────────────────────────────────
# Lets an operator force individual DO channels ON/OFF to check wiring.
# Only allowed while the AGV is not driving itself; mode_manager also
# force-exits test mode the moment it enters RUNNING/REVERSING/CALIBRATE.
_IO_TEST_ALLOWED_MODES = ("armed", "manual", "emergency")

@app.route("/api/io/test", methods=["POST"])
def api_io_test():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    data   = request.get_json(silent=True) or {}
    enable = data.get("enable")
    if not isinstance(enable, bool):
        return jsonify({"error": "enable must be true or false"}), 400
    if enable and not _engineer_ok():
        return _locked_response()
    if enable:
        mode = _state.current_mode
        if mode not in _IO_TEST_ALLOWED_MODES:
            return jsonify({"error": f"IO test only in ARMED / MANUAL / EMERGENCY — now {mode}"}), 409
        _state.io_test_mode = True
        logger.warning("[IO TEST] ENTERED via web IO page (mode=%s)", mode)
    else:
        _state.io_test_mode = False
        _state.clear_do_overrides()
        logger.warning("[IO TEST] EXITED via web IO page — all DO overrides released")
    return jsonify({"ok": True, "io_test_mode": _state.io_test_mode})


@app.route("/api/io/do", methods=["POST"])
@engineer_required
def api_io_do():
    """Force one DO: {"channel": n, "value": true|false|null}. null releases
    the channel back to the program."""
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if not _state.io_test_mode:
        return jsonify({"error": "turn IO TEST MODE on first"}), 409
    data  = request.get_json(silent=True) or {}
    ch    = data.get("channel")
    value = data.get("value")
    if not isinstance(ch, int) or isinstance(ch, bool) or not (0 <= ch < config.NUM_DO):
        return jsonify({"error": f"channel must be 0..{config.NUM_DO - 1}"}), 400
    if value is None:
        if ch in _state.do_override:
            _state.do_override.pop(ch)
            _state.do_setpoints.setdefault(ch, False)
        logger.info("[IO TEST] DO%d released to program", ch)
    elif isinstance(value, bool):
        _state.do_override[ch] = value
        logger.info("[IO TEST] DO%d forced %s", ch, "ON" if value else "OFF")
    else:
        return jsonify({"error": "value must be true, false or null"}), 400
    _state.do_dirty.set()
    return jsonify({"ok": True, "channel": ch, "value": value})


@app.route("/api/params")
@engineer_required
def api_params():
    import config, math
    return jsonify({
        "Networking": {
            "AGV ID":        config.AGV_ID,
            "Device ID":     config.DEVICE_ID,
            "Local IP":      config.LOCAL_IP,
            "DIO IP":        config.DIO_IP,
            "AO IP":         config.AO_IP,
            "Modbus Port":   config.MODBUS_PORT,
            "RFID IP":       config.RFID_IP,
            "RFID Port":     config.RFID_PORT,
        },
        "IO Mapping": {
            "DI Base":        config.DI_BASE,
            "DO Base":        config.DO_BASE,
            "Num DI":         config.NUM_DI,
            "Num DO":         config.NUM_DO,
            "AO Base":        config.AO_BASE,
            "Num AO":         config.NUM_AO,
            "DI Flipped":     config.DI_FLIPPED,
            "DI Emergency":   config.DI_EMERGENCY,
            "DI Fwd":         config.DI_FWD,
            "DI Rev":         config.DI_REV,
            "DI Left":        config.DI_LEFT,
            "DI Right":       config.DI_RIGHT,
            "DI Mode Switch": config.DI_MODE_SWITCH,
            "DI Start":       config.DI_START,
            "DI Reset":       config.DI_RESET,
            "DI Prox":        config.DI_PROX,
        },
        "Motor Channels": {
            "Left Fwd (DO)":    config.MOTOR_CHANNELS["left"]["do_fwd"],
            "Left Rev (DO)":    config.MOTOR_CHANNELS["left"]["do_rev"],
            "Left Brake (DO)":  config.MOTOR_CHANNELS["left"]["do_brake"],
            "Left Speed (AO)":  config.MOTOR_CHANNELS["left"]["ao_speed"],
            "Right Fwd (DO)":   config.MOTOR_CHANNELS["right"]["do_fwd"],
            "Right Rev (DO)":   config.MOTOR_CHANNELS["right"]["do_rev"],
            "Right Brake (DO)": config.MOTOR_CHANNELS["right"]["do_brake"],
            "Right Speed (AO)": config.MOTOR_CHANNELS["right"]["ao_speed"],
        },
        "Kinematics": {
            "Wheel Diameter (m)":      config.WHEEL_DIAMETER,
            "Gear Ratio":              config.GEAR_RATIO,
            "Wheel Circumference (m)": round(config.WHEEL_CIRCUMFERENCE, 4),
        },
        "Speeds": {
            "Manual High (m/s)":          config.MANUAL_TARGET_HIGH_SPEED,
            "Manual Slow (m/s)":          config.MANUAL_TARGET_SLOW_SPEED,
            "Auto High (m/s)":            config.AUTO_TARGET_HIGH_SPEED,
            "Auto Max (m/s)":             config.AUTO_TARGET_MAX_SPEED,
            "Auto Medium (m/s)":          config.AUTO_TARGET_MEDIUM_SPEED,
            "Auto Slow (m/s)":            config.AUTO_TARGET_SLOW_SPEED,
            "Auto Approach (m/s)":        config.AUTO_TARGET_APPROACH_SPEED,
            "Accel Rate (m/s²)":          config.ACCEL_RATE,
            "Decel Rate (m/s²)":          config.DECEL_RATE,
            "Approach Decel Rate (m/s²)": config.APPROACH_DECEL_RATE,
        },
        "PID Tuning": {
            "KP (High)":                        config.KP,
            "TD (High)":                        config.TD,
            "N (High)":                         config.N,
            "KP (Max)":                         config.KP_MAX,
            "TD (Max)":                         config.TD_MAX,
            "N (Max)":                          config.N_MAX,
            "KP (Medium)":                      config.KP_MEDIUM,
            "TD (Medium)":                      config.TD_MEDIUM,
            "N (Medium)":                       config.N_MEDIUM,
            "KP (Slow)":                        config.KP_SLOW,
            "TD (Slow)":                        config.TD_SLOW,
            "N (Slow)":                         config.N_SLOW,
            "KP (Approach)":                    config.KP_APPROACH,
            "TD (Approach)":                    config.TD_APPROACH,
            "N (Approach)":                     config.N_APPROACH,
            "KP (Reverse)":                     config.KP_REVERSE,
            "TD (Reverse)":                     config.TD_REVERSE,
            "N (Reverse)":                      config.N_REVERSE,
            "TI":                               config.TI,
            "TI Deadband (mm)":                 config.TI_DEADBAND,
            "TI Max":                           config.TI_MAX,
            "DT (s)":                           config.DT,
            "V Red Coef (High)":                config.V_RED_COEF,
            "V Red Coef (Max)":                 config.V_RED_COEF_MAX,
            "V Red Coef (Medium)":              config.V_RED_COEF_MEDIUM,
            "V Red Coef (Slow)":                config.V_RED_COEF_SLOW,
            "V Red Coef (Approach)":            config.V_RED_COEF_APPROACH,
            "V Red Coef (Reverse)":             config.V_RED_COEF_REVERSE,
            "Output Clamp (RPM)":               config.OUTPUT_CLAMP_RPM,
            "Output Clamp Into-Bar (Approach)": config.OUTPUT_CLAMP_INTO_BAR_APPROACH,
            "Output Clamp Away (Approach)":     config.OUTPUT_CLAMP_AWAY_APPROACH,
            "SR Alpha":                         config.SR_ALPHA,
            "SR Cap":                           config.SR_CAP,
        },
        "CAN Sensor": {
            "COB ID":               config.SENSOR_COB_ID,
            "Timeout (s)":          config.CAN_TIMEOUT,
            "Node ID":              config.CAN_NODE_ID,
            "Flag Tape Detect":     config.FLAG_TAPE_DETECT,
            "Flag Left Marker":     config.FLAG_LEFT_MARKER,
            "Flag Right Marker":    config.FLAG_RIGHT_MARKER,
            "Flag Sensor Fail":     config.FLAG_SENSOR_FAIL,
            "Sensor Max (mm)":      config.SENSOR_MAX_MM,
            "Sensor Max Step (mm)": config.SENSOR_MAX_STEP_MM,
            "Rear Sensor Fitted":       config.REAR_SENSOR_ENABLED,
            "Rear COB ID":              config.REAR_SENSOR_COB_ID,
            "Rear Node ID":             config.REAR_CAN_NODE_ID,
            "Rear Sensor Max (mm)":      config.REAR_SENSOR_MAX_MM,
            "Rear Sensor Max Step (mm)": config.REAR_SENSOR_MAX_STEP_MM,
        },
        "RFID": {
            "Sequence Stop Delay (s)": config.SEQUENCE_STOP_DELAY,
            "Init Cmd (hex)":          config.RFID_INIT_CMD.hex().upper(),
        },
        "Feedforward": {
            "Enabled":           config.FF_ENABLED,
            "Track Width (m)":   config.TRACK_WIDTH,
            "Curve Radius (m)":  config.CURVE_RADIUS,
            "Direction Sign":    config.FF_DIRECTION_SIGN,
            "Curve Speed Modes": ", ".join(sorted(config.FF_CURVE_MODES)),
            "FF Alpha":          config.FF_ALPHA,
            "FF Exit Alpha":     config.FF_EXIT_ALPHA,
            "FF Scale":          config.FF_SCALE,
        },
        "Timing": {
            "DI Poll Interval (s)": config.DI_POLL_INTERVAL,
        },
        "Watchdog": {
            "DI Timeout (s)":   config.WATCHDOG_DI_TIMEOUT_S,
            "CAN Timeout (s)":  config.WATCHDOG_CAN_TIMEOUT_S,
            "RFID Timeout (s)": config.WATCHDOG_RFID_TIMEOUT_S,
        },
        "Encoder": {
            "Enabled":            config.ENCODER_ENABLED,
            "Node ID":            config.ENCODER_NODE_ID,
            "Bitrate":            config.ENCODER_BITRATE,
            "Counts / Rev":       config.ENCODER_COUNTS_PER_REV,
            "Total Range":        config.ENCODER_TOTAL_RANGE,
            "Wheel Diameter (m)": config.ENCODER_WHEEL_DIAMETER_M,
            "TPDO COB-ID Base":   config.ENCODER_TPDO_COB_ID_BASE,
        },
        "Calibration Ramp": {
            "V Start (m/s)": config.CAL_V_START,
            "V Step (m/s)":  config.CAL_V_STEP,
            "V Max (m/s)":   config.CAL_V_MAX,
            "Dwell (s)":     config.CAL_DWELL_S,
        },
        "Features": {
            "DIO Enabled":        config.DIO_ENABLED,
            "CAN Enabled":        config.CAN_ENABLED,
            "RFID Navigation":    config.NAV_ENABLED,
            "RFID Sequence":      config.SEQ_ENABLED,
            "Lidar":              config.LIDAR_ENABLED,
        },
        "Hardware": {
            "V Range":            config.V_RANGE,
            "DAC Res":            config.DAC_RES,
            "RPM/Volt (Left)":    config.RPM_PER_VOLT_LEFT,
            "RPM Offset (Left)":  config.RPM_VOLT_OFFSET_LEFT,
            "RPM/Volt (Right)":   config.RPM_PER_VOLT_RIGHT,
            "RPM Offset (Right)": config.RPM_VOLT_OFFSET_RIGHT,
        },
    })


@app.route("/api/params/features", methods=["POST"])
@engineer_required
def api_set_features():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    mode = _state.current_mode
    if mode in ("running", "reversing", "emergency", "autotune"):
        return jsonify({"error": f"Cannot change settings while AGV is {mode}"}), 403

    if _manager is None:
        return jsonify({"error": "driver manager unavailable"}), 503

    data = request.get_json(silent=True) or {}
    allowed = {"CAN_ENABLED", "NAV_ENABLED", "SEQ_ENABLED", "LIDAR_ENABLED"}
    updates = {k: v for k, v in data.items() if k in allowed}
    if not updates:
        return jsonify({"error": "no valid feature flags provided"}), 400

    # Lidar is a plain flag read live by auto_mode (no driver to start/stop).
    # Live only — back to the profile's lidar.ENABLED on restart.
    if "LIDAR_ENABLED" in updates:
        config.LIDAR_ENABLED = bool(updates.pop("LIDAR_ENABLED"))
        _state.lidar_zone = None
        if config.LIDAR_ENABLED:
            logger.info("[HMI] Lidar safety fields ENABLED")
        else:
            logger.warning("[HMI] Lidar safety fields DISABLED from the web")

    # The manager enforces the NAV -> SEQ hierarchy: disabling a parent
    # cascades to children; enabling a child whose parent is off is rejected.
    # CAN is independent (the magnetic sensor — can be disabled when disconnected).
    try:
        for k, v in updates.items():
            _manager.set_enabled(k, v)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": f"failed to apply: {e}"}), 500

    return jsonify({
        "ok": True,
        "CAN_ENABLED":  config.CAN_ENABLED,
        "NAV_ENABLED":  config.NAV_ENABLED,
        "SEQ_ENABLED":  config.SEQ_ENABLED,
        "LIDAR_ENABLED": config.LIDAR_ENABLED
    })


def _tuning_guard():
    """Shared guard for the live-tuning endpoints.

    Returns an error (body, status) tuple, or None when the request may proceed.
    Tuning is allowed only from idle ARMED / idle MANUAL — never while the AGV is
    driving a line or sitting in a fault. That restriction is what lets the values
    be applied with a plain setattr: auto_mode is rebuilt from scratch on the next
    START, so it re-reads every one of them."""
    if _state is None:
        return {"error": "state not initialised"}, 503
    mode = _state.current_mode
    if mode in ("running", "reversing", "emergency", "autotune"):
        return {"error": f"Cannot change tuning while AGV is {mode}"}, 403
    return None


def _tuning_values(config):
    return {k: getattr(config, k) for k in config.TUNABLE_SPECS}


def _save_tuning(config):
    """Persist every tunable that differs from the profile to
    config.TUNING_SAVE_FILE (atomic write), so it is the default after a
    restart. Nothing differs → the file is removed. Returns (ok, error)."""
    import json
    path = config.TUNING_SAVE_FILE
    values = {k: getattr(config, k) for k in config.TUNABLE_SPECS
              if getattr(config, k) != config.TUNABLE_DEFAULTS[k]}
    try:
        if not values:
            if os.path.exists(path):
                os.remove(path)
        else:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "values": values}, f, indent=2, sort_keys=True)
            os.replace(tmp, path)
        config.TUNING_SAVED = values
        return True, None
    except OSError as e:
        logger.error("TUNING: could not save to %s — %s", path, e)
        return False, str(e)


@app.route("/api/params/tuning", methods=["GET", "POST"])
@engineer_required
def api_tuning():
    import config, math

    if request.method == "GET":
        mode = _state.current_mode if _state is not None else None
        return jsonify({
            "values":   _tuning_values(config),
            "specs":    {k: list(v) for k, v in config.TUNABLE_SPECS.items()},
            "defaults": config.TUNABLE_DEFAULTS,
            "saved":    sorted(config.TUNING_SAVED),
            "locked":   mode in ("running", "reversing", "emergency", "autotune"),
        })

    err = _tuning_guard()
    if err:
        return jsonify(err[0]), err[1]

    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict) or not data:
        return jsonify({"error": "no parameters provided"}), 400

    # Validate the WHOLE batch before mutating anything, so a single rejected
    # field cannot leave a gain set half-applied.
    updates = {}
    for k, raw in data.items():
        if k not in config.TUNABLE_SPECS:
            return jsonify({"error": f"'{k}' is not a tunable parameter"}), 400
        try:
            v = float(raw)
        except (TypeError, ValueError):
            return jsonify({"error": f"{k} must be a number (got {raw!r})"}), 400
        if not math.isfinite(v):
            return jsonify({"error": f"{k} must be finite (got {raw!r})"}), 400
        lo, hi = config.TUNABLE_SPECS[k]
        if not (lo <= v <= hi):
            return jsonify({"error": f"{k} must be in [{lo}, {hi}] (got {v})"}), 400
        updates[k] = v

    for k, v in updates.items():
        setattr(config, k, v)

    saved, save_err = _save_tuning(config)
    logger.info("TUNING: applied%s %s", " and saved" if saved else " (NOT saved)",
                ", ".join(f"{k}={v}" for k, v in sorted(updates.items())))
    return jsonify({"ok": True, "applied": sorted(updates),
                    "saved": saved, "save_error": save_err,
                    "values": _tuning_values(config)})


@app.route("/api/params/tuning/reset", methods=["POST"])
@engineer_required
def api_tuning_reset():
    import config

    err = _tuning_guard()
    if err:
        return jsonify(err[0]), err[1]

    for k, v in config.TUNABLE_DEFAULTS.items():
        setattr(config, k, v)
    saved, save_err = _save_tuning(config)      # nothing differs → file removed

    logger.info("TUNING: reset all tunable parameters to profile defaults")
    return jsonify({"ok": True, "saved": saved, "save_error": save_err,
                    "values": _tuning_values(config)})


_RESTART_CMD = ["/usr/bin/systemctl", "restart", "agv-controller.service"]

@app.route("/api/web_button/mode", methods=["POST"])
def api_web_button_mode():
    """Enable or disable web button bypass mode (MANUAL page). Open to
    operators — no engineer login needed."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    data = request.get_json(silent=True) or {}
    enable = data.get("enable")
    if not isinstance(enable, bool):
        return jsonify({"error": "enable must be true or false"}), 400
    _state.web_button_mode = enable
    if not enable:
        _state.web_btn_start     = False
        _state.web_btn_reset     = False
        _state.web_btn_manual    = False
        _state.web_btn_emergency = False
    logger.info("[WebBtn] bypass mode %s", "ON" if enable else "OFF")
    return jsonify({"ok": True, "web_button_mode": enable})


@app.route("/api/web_button/press", methods=["POST"])
def api_web_button_press():
    """Send a virtual button press. Requires web_button_mode active."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if not _state.web_button_mode:
        return jsonify({"error": "web button mode not active"}), 403

    data   = request.get_json(silent=True) or {}
    button = data.get("button")   # "start" | "reset" | "manual" | "emergency"
    value  = data.get("value", True)

    if button == "start":
        _state.web_btn_start = bool(value)
    elif button == "reset":
        _state.web_btn_reset = bool(value)
    elif button == "manual":
        _state.web_btn_manual = bool(value)
    elif button == "emergency":
        _state.web_btn_emergency = bool(value)
    else:
        return jsonify({"error": f"unknown button: {button}"}), 400

    return jsonify({"ok": True, "button": button, "value": bool(value)})


@app.route("/api/restart", methods=["POST"])
def api_restart():
    """Restart the controller systemd service (operator button on /errors).

    Allowed only when the AGV is not actively moving under auto and not in
    emergency — i.e. mode must be manual / armed / idle. Requires a passwordless
    sudoers rule for `systemctl restart agv-controller.service`."""
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503

    mode = _state.current_mode
    if mode in ("running", "reversing", "emergency", "autotune"):
        return jsonify({"error": f"Cannot restart while AGV is {mode}"}), 403

    # Verify passwordless sudo is configured for exactly this command before
    # firing, so we can return a clear error instead of silently doing nothing.
    check = subprocess.run(["sudo", "-n", "-l"] + _RESTART_CMD,
                           capture_output=True, text=True)
    if check.returncode != 0:
        return jsonify({"error": "sudo not configured for systemctl restart — "
                                 "add a /etc/sudoers.d rule"}), 500

    logger.warning("[Restart] Operator requested controller restart via HMI (mode=%s)", mode)
    # Fire and return: systemctl hands the restart job to PID 1, which performs
    # it even after this process is terminated. Popen so we don't block/wait.
    subprocess.Popen(["sudo", "-n"] + _RESTART_CMD)
    return jsonify({"ok": True})


# ── Wheel-speed calibration (open-loop encoder ramp) ──────────────────────────

@app.route("/api/calibration/status")
def api_calibration_status():
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    s = _state
    enc_age = (time.time() - s.encoder_last_rx) if s.encoder_last_rx else None
    return jsonify({
        "available":   config.ENCODER_ENABLED,
        "mode":        s.current_mode,
        "wheel":       s.calibration_wheel,
        "requested":   s.calibration_request,
        "calibration": dict(s.calibration_status),
        "encoder": {
            "connected": s.encoder_connected,
            "rpm":       round(s.encoder_rpm, 2),
            "v":         round(s.encoder_v, 4),
            "count":     s.encoder_count,
            "age_s":     round(enc_age, 2) if enc_age is not None else None,
        },
    })


@app.route("/api/calibration/start", methods=["POST"])
@engineer_required
def api_calibration_start():
    """Start the open-loop calibration ramp. Allowed only from ARMED (AUTO
    selector, safe, not running/manual/emergency)."""
    import config
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if not config.ENCODER_ENABLED:
        return jsonify({"error": "encoder disabled in profile"}), 403

    mode = _state.current_mode
    if mode != "armed":
        return jsonify({"error": f"Calibration starts only from ARMED (current: {mode}). "
                                 "Select AUTO and release START first."}), 403

    data  = request.get_json(silent=True) or {}
    wheel = data.get("wheel", _state.calibration_wheel)
    if wheel not in ("left", "right"):
        return jsonify({"error": "wheel must be 'left' or 'right'"}), 400

    _state.calibration_wheel   = wheel
    _state.calibration_request = True
    logger.warning("[Calib] start requested via HMI (wheel=%s)", wheel)
    return jsonify({"ok": True, "wheel": wheel})


# ── PID auto-tune (relay test, see autotune.py) ─────────────────────────────

@app.route("/api/autotune", methods=["GET"])
def api_autotune_status():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    return jsonify({"mode": _state.current_mode, "running": _state.current_mode == "autotune",
                    **_state.autotune_status})


@app.route("/api/autotune/start", methods=["POST"])
@engineer_required
def api_autotune_start():
    import autotune, math
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    if _state.current_mode != "armed":
        return jsonify({"error": f"Auto-tune starts only from ARMED (now {_state.current_mode})"}), 403
    raw = _state.latest_sensor
    if not (raw and raw.get("tape_detected")):
        return jsonify({"error": "no tape under the front sensor"}), 409
    data = request.get_json(silent=True) or {}
    mode = str(data.get("speed_mode", "HIGH")).upper()
    if mode not in autotune.MODE_INFO:
        return jsonify({"error": f"speed_mode must be one of {sorted(autotune.MODE_INFO)}"}), 400
    limits = {"relay_rpm": (5, 400), "err_limit_mm": (10, 90), "max_s": (5, 120), "cycles": (2, 10)}
    params = {"speed_mode": mode}
    for k, (lo, hi) in limits.items():
        try:
            v = float(data.get(k, autotune.DEFAULTS[k]))
        except (TypeError, ValueError):
            return jsonify({"error": f"{k} must be a number"}), 400
        if not (math.isfinite(v) and lo <= v <= hi):
            return jsonify({"error": f"{k} must be in [{lo}, {hi}]"}), 400
        params[k] = v
    _state.autotune_params = params
    _state.autotune_status.update(autotune.new_status())
    _state.autotune_status["phase"] = "starting"
    _state.autotune_request = True
    logger.warning("[AUTOTUNE] start requested via HMI: %s", params)
    return jsonify({"ok": True, "params": params})


@app.route("/api/autotune/stop", methods=["POST"])
def api_autotune_stop():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    _state.autotune_request = False
    return jsonify({"ok": True})


# ── Run logs (CSV / PNG written by debugging/plotter.py) ────────────────────

_RUN_FILE_RE = re.compile(r"^[0-9]{8}_[0-9]{6}_[A-Za-z0-9_]+\.(csv|png)$")

@app.route("/api/runs")
@engineer_required
def api_runs():
    from debugging.plotter import PLOT_DIR
    try:
        names = [n for n in os.listdir(PLOT_DIR) if _RUN_FILE_RE.match(n)]
    except FileNotFoundError:
        names = []
    names.sort(reverse=True)
    out = []
    for n in names[:60]:
        st = os.stat(os.path.join(PLOT_DIR, n))
        out.append({"name": n, "bytes": st.st_size})
    return jsonify({"files": out})


@app.route("/api/runs/<name>")
@engineer_required
def api_run_file(name):
    from debugging.plotter import PLOT_DIR
    if not _RUN_FILE_RE.match(name):
        abort(404)
    return send_from_directory(PLOT_DIR, name, as_attachment=name.endswith(".csv"))


@app.route("/api/calibration/stop", methods=["POST"])
def api_calibration_stop():
    if _state is None:
        return jsonify({"error": "state not initialised"}), 503
    _state.calibration_request = False
    logger.info("[Calib] stop requested via HMI")
    return jsonify({"ok": True})


@app.route("/api/wifi")
def api_wifi():
    try:
        out = subprocess.check_output(
            ["nmcli", "-f", "IN-USE,SIGNAL,SSID", "dev", "wifi"],
            timeout=2, text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            if line.strip().startswith("*"):
                parts = line.split()
                signal = int(parts[1])
                ssid   = " ".join(parts[2:])
                return jsonify({"ok": True, "signal": signal, "ssid": ssid})
        return jsonify({"ok": False, "signal": None, "ssid": None})
    except Exception as e:
        return jsonify({"ok": False, "signal": None, "ssid": None, "error": str(e)})


def run_server(state, engine=None, manager=None):
    global _state, _engine, _manager
    import config
    _state   = state
    _engine  = engine
    _manager = manager
    logger.info("[Flask] Dashboard at http://%s:5000", config.LOCAL_IP)
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
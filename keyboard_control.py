"""
keyboard_control.py — keyboard / keypad commands for the AGV
=============================================================
One place that turns a key into an action, used by BOTH:
  * the laptop keyboard on the web MANUAL page  (POST /api/keyboard)
  * a USB keypad plugged into the AGV PC        (drivers/keypad_evdev.py)

Movement (hold-to-drive, MANUAL mode only)
    W forward   S reverse   A spin left   D spin right
    (arrow keys / remote D-pad do the same)
    W+A / W+D / S+A / S+D = curve. Releasing the keys stops.
Actions (one press)
    1 APPROACH  2 SLOW  3 MEDIUM  4 HIGH  5 MAX   (AUTO speed mode; ARMED/RUNNING)
    Z pusher full UP      X pusher full DOWN       (ARMED/MANUAL only)
    R AUTO REVERSE start (ARMED) / stop (REVERSING)
    Q switch MANUAL <-> AUTO (overrides the selector until it is moved)
    7 / 8 / 9 / 0  start AUTO to branch 1 / 2 / 3 / 4  (ARMED; else just selects)
    E EMERGENCY STOP — latched, released only by RESET
"""
import logging
import time

import config

logger = logging.getLogger(__name__)

MOVE_KEYS = {"w", "a", "s", "d"}
SPEED_KEYS = {"1": "APPROACH", "2": "SLOW", "3": "MEDIUM", "4": "HIGH", "5": "MAX"}
BRANCH_KEYS = {"7": "1", "8": "2", "9": "3", "0": "4"}
ACTION_KEYS = set(SPEED_KEYS) | set(BRANCH_KEYS) | {"z", "x", "r", "q", "e"}
MOVE_WATCHDOG_S = 0.4      # same 400 ms watchdog as the web jog buttons


def move_command(held):
    """Manual jog command for the set of movement keys held, or None."""
    up, down, left, right = "w" in held, "s" in held, "a" in held, "d" in held
    if up and down:
        up = down = False          # contradictory — ignore both
    if left and right:
        left = right = False
    if up and left:    return "fwd_left"
    if up and right:   return "fwd_right"
    if down and left:  return "rvs_left"
    if down and right: return "rvs_right"
    if up:             return "forward"
    if down:           return "reverse"
    if left:           return "left"
    if right:          return "right"
    return None


def apply_move(state, cmd):
    """Drive (or stop with cmd=None) through the same channel as the web jog
    buttons. Only takes effect in MANUAL; the watchdog stops the AGV if the
    caller stops refreshing (lost connection, unplugged keypad)."""
    if cmd is not None and state.current_mode != "manual":
        return False
    state.web_manual_command = cmd
    state.web_manual_expire = time.time() + MOVE_WATCHDOG_S
    return True


def _tape_for_reverse(state):
    """Same check as /api/reverse: tape under the sensor reverse will follow."""
    if config.REAR_SENSOR_ENABLED:
        raw = state.latest_rear_sensor
        age = (time.time() - state.rear_can_last_rx) if state.rear_can_last_rx else None
        fresh = age is not None and age <= config.CAN_TIMEOUT
        return bool(raw) and fresh and raw.get("tape_detected", False), "rear"
    raw = state.latest_sensor
    return bool(raw) and raw.get("tape_detected", False), "front"


def handle_key(state, key, source="keyboard"):
    """Run the one-press action for `key`. Returns (ok, message)."""
    key = (key or "").lower()
    mode = state.current_mode

    if key == "e":
        if not state.kb_estop:
            state.kb_estop = True
            logger.critical("[%s] E — EMERGENCY STOP (press RESET to release)", source)
        return True, "EMERGENCY STOP — press RESET to release"

    if key in SPEED_KEYS:
        speed = SPEED_KEYS[key]
        if mode not in ("armed", "running"):
            return False, f"speed keys work in ARMED/RUNNING (now {mode})"
        state.speed_mode = speed
        logger.info("[%s] %s — AUTO speed mode %s", source, key, speed)
        return True, f"speed {speed}"

    if key in ("z", "x"):
        direction = "up" if key == "z" else "down"
        if mode not in ("armed", "manual"):
            return False, f"pusher keys work in ARMED/MANUAL (now {mode})"
        if state.pusher_estop:
            return False, "pusher e-stop is active"
        if direction == "up":
            state.pusher_down_request, state.pusher_up_request = False, True
        else:
            state.pusher_up_request, state.pusher_down_request = False, True
        logger.info("[%s] %s — pusher full %s", source, key.upper(), direction.upper())
        return True, f"pusher full {direction.upper()}"

    if key == "r":
        if mode == "reversing":
            state.web_reverse_stop = True
            logger.info("[%s] R — stop AUTO REVERSE", source)
            return True, "reverse STOP"
        if mode != "armed":
            return False, f"R starts reverse only in ARMED (now {mode})"
        tape, which = _tape_for_reverse(state)
        if not tape:
            return False, f"no tape under the {which} sensor"
        state.web_reverse_start = True
        logger.info("[%s] R — start AUTO REVERSE", source)
        return True, "reverse START"

    if key == "q":
        want_manual = mode != "manual"
        if state.web_button_mode:
            state.web_btn_manual = want_manual
        else:
            state.kb_mode_override = want_manual
        logger.info("[%s] Q — switch to %s", source, "MANUAL" if want_manual else "AUTO")
        return True, "MANUAL" if want_manual else "AUTO (ARMED)"

    if key in BRANCH_KEYS:
        branch = BRANCH_KEYS[key]
        if mode in ("running", "reversing", "emergency"):
            return False, f"branch keys blocked while {mode}"
        state.kb_branch_request = branch
        logger.info("[%s] %s — branch %s%s", source, key, branch,
                    " (start AUTO)" if mode == "armed" else " (select)")
        return True, f"branch {branch}" + (" — START" if mode == "armed" else " selected")

    return False, f"unknown key '{key}'"

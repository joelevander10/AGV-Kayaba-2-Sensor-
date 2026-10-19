"""
rotate.py — 180° spin-turn on the magnetic tape
================================================
The AGV spins in place (left wheel one way, right wheel the other) and stops
when the FRONT sensor is centred on the tape again, facing the other way.

How the stop is found
    The front MGS1600 sits SENSOR_LOOKAHEAD_M (~7.6 cm) ahead of the drive
    axle, so while spinning it sweeps a small circle: it LOSES the tape after
    ~45° and FINDS it again from ~135°, the reading moving towards 0 mm, which
    is reached exactly at 180°. So:
      1. SPIN   at ROTATE_SPEED_MPS until the tape has been lost and found
                again (and the estimated angle is past ROTATE_MIN_ANGLE_DEG,
                so the start tape or a nearby crossing tape is not mistaken
                for the target);
      2. CREEP  at ROTATE_CREEP_MPS until |position| <= ROTATE_CENTER_TOL_MM
                (or the reading crosses 0);
      3. BRAKE  and hold.
    The angle estimate (from the commanded wheel speed and TRACK_WIDTH_M) is
    only a guard: past ROTATE_MAX_ANGLE_DEG, or after ROTATE_TIMEOUT_S, the
    turn is aborted and the AGV brakes.

Started from MANUAL or ARMED by the T / Y keys (keyboard + AGV keypad) or
the ROTATE 180° buttons on the manual page — see mode_manager in modes.py.
After the turn the AGV stays braked in the mode it came from (stop and wait).
Use it on a plain straight tape, away from forks, crossings and RFID tags.
"""
import asyncio
import logging
import math
import time

import config
import motion

logger = logging.getLogger(__name__)


def _status(state, phase, message="", **extra):
    st = dict(state.rotate_status or {})
    st.update({"phase": phase, "message": message, "t": time.time()})
    st.update(extra)
    state.rotate_status = st


def _front(state):
    """(fresh, tape_detected, position_mm) from the latest front-sensor frame."""
    raw = state.latest_sensor
    age = (time.time() - state.can_last_rx) if state.can_last_rx else None
    fresh = bool(raw) and age is not None and age <= config.CAN_TIMEOUT
    if not fresh:
        return False, False, None
    tape = bool(raw.get("tape_detected", False))
    pos = raw.get("right_mm")
    if pos is None:
        pos = raw.get("left_mm")
    return True, tape, (float(pos) if pos is not None else None)


def check_start(state):
    """Can a rotation start now? Returns (ok, reason)."""
    if not config.ROTATE_ENABLED:
        return False, "rotate 180° is disabled in the profile"
    if state.current_mode not in ("manual", "armed"):
        return False, f"rotate works in MANUAL or ARMED (now {state.current_mode})"
    if config.ROTATE_BLOCK_WITH_TROLLEY and state.trolley_hooked:
        return False, "trolley is hooked — unhook before rotating"
    fresh, tape, _ = _front(state)
    if not fresh:
        return False, "front sensor not responding"
    if not tape:
        return False, "no tape under the front sensor"
    return True, "ok"


async def rotate_mode(state, direction):
    """Run one 180° turn. direction: "left" (CCW) or "right" (CW).
    Returns True when centred on the tape, False on abort."""
    sign_l, sign_r = (-1.0, 1.0) if direction == "left" else (1.0, -1.0)
    from modes import _lidar_zone      # local import: modes imports this module
    track_w = max(config.TRACK_WIDTH, 0.05)

    v_cmd       = 0.0
    angle       = 0.0            # estimated turn so far (deg)
    phase       = "spin"
    lost_frames = 0
    seen_frames = 0
    lost_seen   = False
    prev_pos    = None
    lidar_hold  = False
    clear_since = None
    t0          = time.perf_counter()
    last_t      = t0
    _status(state, "spinning", f"turning {direction.upper()}", direction=direction, angle=0.0)
    logger.info("[ROTATE] 180° turn %s started", direction.upper())

    try:
        while True:
            now = time.perf_counter()
            dt = min(max(now - last_t, 0.0), 0.1)
            last_t = now

            if now - t0 > config.ROTATE_TIMEOUT_S:
                logger.warning("[ROTATE] timeout after %.0fs — aborting", config.ROTATE_TIMEOUT_S)
                await motion.set_brake(state)
                _status(state, "failed", "timeout — tape not found", angle=round(angle))
                return False
            if angle > config.ROTATE_MAX_ANGLE_DEG:
                logger.warning("[ROTATE] turned ~%.0f° without centring on the tape — aborting", angle)
                await motion.set_brake(state)
                _status(state, "failed", f"tape not found after ~{angle:.0f}°", angle=round(angle))
                return False

            # ── Lidar INNER field (only if lidar is enabled): pause ──────────
            if _lidar_zone(state) == "inner":
                if not lidar_hold:
                    logger.warning("[ROTATE] LIDAR INNER — pausing")
                    await motion.set_brake(state)
                    lidar_hold = True
                    v_cmd = 0.0
                    _status(state, "paused", "lidar inner field", angle=round(angle))
                clear_since = None
                t0 += dt                               # pause doesn't count toward timeout
                await asyncio.sleep(0.02)
                continue
            if lidar_hold:
                clear_since = clear_since or now
                if now - clear_since < config.LIDAR_RESUME_DELAY_S:
                    t0 += dt
                    await asyncio.sleep(0.02)
                    continue
                lidar_hold = False
                _status(state, "spinning" if phase == "spin" else "searching",
                        "resumed", angle=round(angle))

            fresh, tape, pos = _front(state)
            if not fresh:
                logger.error("[ROTATE] front sensor lost — aborting")
                await motion.set_brake(state)
                _status(state, "failed", "front sensor lost", angle=round(angle))
                return False

            if phase == "spin":
                if tape:
                    lost_frames = 0
                    seen_frames += 1
                else:
                    seen_frames = 0
                    lost_frames += 1
                    if lost_frames >= 3 and not lost_seen:
                        lost_seen = True
                        logger.info("[ROTATE] tape left the sensor at ~%.0f°", angle)
                if lost_seen and seen_frames >= 2 and angle >= config.ROTATE_MIN_ANGLE_DEG:
                    phase = "creep"
                    prev_pos = pos
                    logger.info("[ROTATE] tape found again at ~%.0f° (%.1f mm) — creeping",
                                angle, pos if pos is not None else float("nan"))
                    _status(state, "searching", "tape found — centring", angle=round(angle))
            else:  # creep → stop on centre
                if tape and pos is not None:
                    crossed = (prev_pos is not None and prev_pos * pos < 0
                               and abs(prev_pos) < 40 and abs(pos) < 40)
                    if abs(pos) <= config.ROTATE_CENTER_TOL_MM or crossed:
                        await motion.set_brake(state)
                        logger.info("[ROTATE] centred on tape (%.1f mm) — ~%.0f° turned", pos, angle)
                        await asyncio.sleep(config.ROTATE_SETTLE_S)
                        fresh, tape, pos2 = _front(state)
                        msg = (f"done — {pos2:+.0f} mm" if (tape and pos2 is not None)
                               else "done")
                        _status(state, "done", msg, angle=round(angle))
                        return True
                    prev_pos = pos

            # ── Speed ramp + drive ───────────────────────────────────────────
            target = config.ROTATE_SPEED_MPS if phase == "spin" else config.ROTATE_CREEP_MPS
            step = config.ROTATE_ACCEL_MPS2 * dt
            v_cmd = min(v_cmd + step, target) if v_cmd < target else max(v_cmd - step, target)
            rpm = motion.mps_to_rpm(v_cmd)
            await motion.set_wheels(state, sign_l * max(rpm, 1e-9), sign_r * max(rpm, 1e-9))

            angle += math.degrees(2.0 * v_cmd / track_w * dt)
            st = state.rotate_status or {}
            if abs(st.get("angle", 0) - angle) >= 5:
                st = dict(st); st["angle"] = round(angle); state.rotate_status = st

            await asyncio.sleep(0.01)
    except asyncio.CancelledError:
        logger.info("[ROTATE] cancelled at ~%.0f°", angle)
        _status(state, "aborted", "stopped", angle=round(angle))
        raise
    finally:
        await motion.idle(state)

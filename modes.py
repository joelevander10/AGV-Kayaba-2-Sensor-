import asyncio
import logging
import time

import config
import motion
import calibration
import autotune
import rotate

from core.pid import PIDController
from debugging.plotter import RunRecorder

logger = logging.getLogger(__name__)


def _ramp_dt(last_t):
    """Real time since the previous speed-ramp step (s), clamped to 0.2 s.
    The control loop only runs when a sensor frame arrives (~10 Hz from the
    MGS1600, not every DT = 10 ms), so stepping the ramp by a fixed DT made
    every accel / decel ~10x slower than the rate set in the profile."""
    now = time.perf_counter()
    if last_t is None:
        return now, config.DT
    return now, min(max(now - last_t, 0.0), 0.2)

# ══════════════════════════════════════════════════════════════════════════════
#  AUTO MODE  (forward tape-following only)
# ══════════════════════════════════════════════════════════════════════════════

def _lidar_zone(state):
    """Innermost lidar field currently occupied ("inner"/"middle"/"outer")
    or None. NORMALLY_CLOSED: DI OFF = object (ON = clear); otherwise
    DI ON = object. Values as seen after DI_FLIPPED. Disabled → None."""
    if not config.LIDAR_ENABLED:
        return None
    di = state.latest_di
    if di is None:
        return None
    def on(idx):
        # True = object in this field. An unassigned field (-1) never triggers.
        if not (0 <= idx < len(di)):
            return False
        return (not di[idx]) if config.LIDAR_NORMALLY_CLOSED else bool(di[idx])
    if on(config.LIDAR_DI_INNER):
        return "inner"
    if on(config.LIDAR_DI_MIDDLE):
        return "middle"
    if on(config.LIDAR_DI_OUTER):
        return "outer"
    return None


async def auto_mode(state, engine=None):
    """Forward tape-following PID loop.

    Sensor at front, speed modes active, sequence stops active, RFID + proximity
    marker hooks active. The AGV only ever runs auto forward.

    Args:
        engine: SequenceEngine instance. The marker-hook calls are guarded with
                `if engine`.
    """
    await motion.idle(state)
    await motion.set_forward(state, 0.0)
    # Steering direction, from profile pid_tuning.ERROR_SIGN_FORWARD
    # (-1.0 = e = -pv, original; +1.0 = flipped because the AGV steered away
    # from the tape).
    error_sign = config.ERROR_SIGN_FORWARD

    # Initialise PID with SLOW gains; forward mode will switch to HIGH on first
    # cycle if speed_mode == "HIGH".
    pid = PIDController(
        kp           = config.KP_SLOW,
        td           = config.TD_SLOW,
        n            = config.N_SLOW,
        ti           = config.TI,
        ti_deadband  = config.TI_DEADBAND,
        ti_max       = config.TI_MAX,
        dt           = config.DT,
        output_clamp = config.OUTPUT_CLAMP_RPM,
        v_red_coef   = config.V_RED_COEF_SLOW,
        sr_alpha     = config.SR_ALPHA,
        sr_cap       = config.SR_CAP,
    )

    current_target_speed = 0.0
    ramp_t               = None   # last speed-ramp step (perf_counter), see _ramp_dt
    tape_was_lost        = False
    was_sequence_stopped = False
    last_speed_mode      = None   # tracks when gain set must change
    last_valid_pv        = None   # last accepted lateral offset (mm) for slew limit
    _dbg_count           = 0      # throttles the per-cycle PID debug log
    ff_filtered          = 0.0    # low-pass filtered curvature feedforward (rpm)
    prox_was_active      = False  # edge-detect for the DI proximity station marker
    lidar_stopped        = False  # braked by the lidar INNER field
    lidar_clear_since    = None   # when INNER cleared (resume after RESUME_DELAY_S)

    recorder = RunRecorder()
    recorder.start()

    try:
        while True:

            # ── Emergency guard ───────────────────────────────────────────────
            # idle(): brake follows brake.ACTIVE_WHEN_STOPPED (default ON).
            if state.emergency_active:
                await motion.idle(state)
                current_target_speed = 0.0
                pid.reset()
                last_valid_pv = None
                ff_filtered   = 0.0
                await asyncio.sleep(config.DT)
                continue

            # ── Sequence stop ─────────────────────────────────────────────────
            if state.sequence_stop:
                if not was_sequence_stopped:
                    logger.info("[AUTO] Sequence stop — holding.")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    was_sequence_stopped = True
                await asyncio.sleep(config.DT)
                continue

            if was_sequence_stopped:
                logger.info("[AUTO] Sequence stop ended — resuming.")
                await motion.set_forward(state, 0.0)
                was_sequence_stopped = False

            # ── Lidar safety fields (AUTO forward only) ──────────────────────
            zone = _lidar_zone(state)
            state.lidar_zone = zone
            if zone == "inner":
                if not lidar_stopped:
                    logger.warning("[AUTO] LIDAR INNER field — stopping")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    lidar_stopped = True
                lidar_clear_since = None
                await asyncio.sleep(config.DT)
                continue
            if lidar_stopped:
                now_t = time.perf_counter()
                if lidar_clear_since is None:
                    lidar_clear_since = now_t
                    logger.info("[AUTO] LIDAR INNER clear — resuming in %.1fs",
                                config.LIDAR_RESUME_DELAY_S)
                if now_t - lidar_clear_since < config.LIDAR_RESUME_DELAY_S:
                    await asyncio.sleep(config.DT)
                    continue
                logger.info("[AUTO] LIDAR — resuming")
                await motion.set_forward(state, 0.0)
                lidar_stopped     = False
                lidar_clear_since = None

            # ── Target speed and PID gain selection ───────────────────────────
            if state.speed_mode == "APPROACH":
                target_speed = config.AUTO_TARGET_APPROACH_SPEED
            elif state.speed_mode == "SLOW":
                target_speed = config.AUTO_TARGET_SLOW_SPEED
            elif state.speed_mode == "MAX":
                target_speed = config.AUTO_TARGET_MAX_SPEED
            elif state.speed_mode == "MEDIUM":
                target_speed = config.AUTO_TARGET_MEDIUM_SPEED
            else:
                target_speed = config.AUTO_TARGET_HIGH_SPEED
            # Lidar: run at a percentage of the current speed mode's speed.
            lidar_cap = None
            if zone == "middle":
                lidar_cap = target_speed * config.LIDAR_MIDDLE_PERCENT / 100.0
            elif zone == "outer":
                lidar_cap = target_speed * config.LIDAR_OUTER_PERCENT / 100.0
            if lidar_cap is not None and target_speed > lidar_cap:
                target_speed = lidar_cap

            # Only switch gains when speed_mode actually changes — avoids
            # redundant recomputation of the derivative filter coefficient.
            # SLOW branch gains (AA / AB / AC) only while SLOW and that tag set them.
            branch_gains = state.speed_mode == "SLOW" and state.gain_set == "BRANCH"
            gain_key = (state.speed_mode, branch_gains)
            if gain_key != last_speed_mode:
                if branch_gains:
                    pid.update_gains(config.KP_SLOW_BRANCH, config.TD_SLOW_BRANCH,
                                     config.N_SLOW_BRANCH, config.V_RED_COEF_SLOW_BRANCH,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                    logger.info("[AUTO] PID gains → SLOW BRANCH")
                elif state.speed_mode == "MAX":
                    pid.update_gains(config.KP_MAX, config.TD_MAX, config.N_MAX,
                                     config.V_RED_COEF_MAX,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                elif state.speed_mode == "MEDIUM":
                    pid.update_gains(config.KP_MEDIUM, config.TD_MEDIUM, config.N_MEDIUM,
                                     config.V_RED_COEF_MEDIUM,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                elif state.speed_mode == "HIGH":
                    pid.update_gains(config.KP, config.TD, config.N,
                                     config.V_RED_COEF,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                elif state.speed_mode == "APPROACH":
                    # Asymmetric clamp: barely steer into the one-sided guide bar
                    # (let it center the AGV mechanically), keep authority to steer
                    # off the bar. Positive output = yaw right = into the bar.
                    pid.update_gains(config.KP_APPROACH, config.TD_APPROACH,
                                     config.N_APPROACH, config.V_RED_COEF_APPROACH,
                                     clamp_hi=config.OUTPUT_CLAMP_INTO_BAR_APPROACH,
                                     clamp_lo=-config.OUTPUT_CLAMP_AWAY_APPROACH)
                else:
                    pid.update_gains(config.KP_SLOW, config.TD_SLOW,
                                     config.N_SLOW, config.V_RED_COEF_SLOW,
                                     clamp_hi=config.OUTPUT_CLAMP_RPM,
                                     clamp_lo=-config.OUTPUT_CLAMP_RPM)
                last_speed_mode = gain_key

            # ── Drain sensor queue — keep only the latest frame ───────────────
            sensor = None
            while not state.sensor_queue.empty():
                sensor = await state.sensor_queue.get()

            if sensor is not None:

                logger.debug("[AUTO] left_marker=%s", sensor["left_marker"])

                # ── Tape-loss guard ───────────────────────────────────────────
                if not sensor["tape_detected"]:
                    logger.warning("[AUTO] LOST TAPE — stopping")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    tape_was_lost = True
                    await asyncio.sleep(0.01)
                    continue

                if tape_was_lost:
                    logger.info("[AUTO] TAPE REACQUIRED — resuming")
                    await motion.set_forward(state, 0.0)
                    tape_was_lost = False

                # ── Marker hook (sequence engine) ──────────────────────────────
                if engine is not None:
                    if sensor["left_marker"]:
                        await engine.on_marker("left")
                    if sensor["right_marker"]:
                        await engine.on_marker("right")

                # ── Acceleration / deceleration ramp ──────────────────────────
                # Stepped by the REAL time between frames (see _ramp_dt).
                ramp_t, rdt = _ramp_dt(ramp_t)
                if current_target_speed < target_speed:
                    current_target_speed += config.ACCEL_RATE * rdt
                    if current_target_speed > target_speed:
                        current_target_speed = target_speed
                elif current_target_speed > target_speed:
                    # APPROACH crawl uses a gentler decel so the RFID-armed
                    # slow-down is smooth before the prox marker hard-stops;
                    # corner/HIGH->SLOW slowdowns use the normal decel rate.
                    decel_rate = (config.APPROACH_DECEL_RATE
                                  if state.speed_mode == "APPROACH"
                                  else config.DECEL_RATE)
                    if lidar_cap is not None and target_speed == lidar_cap:
                        decel_rate = config.LIDAR_DECEL_RATE
                    current_target_speed -= decel_rate * rdt
                    if current_target_speed < target_speed:
                        current_target_speed = target_speed

                base_rpm = motion.mps_to_rpm(current_target_speed)

                # ── Sensor sanity + slew limit ────────────────────────────────
                # Reject physically impossible readings (a glitch frame would
                # otherwise jerk the steering). Out-of-range → skip this frame
                # and hold the last command; a large jump → clamp toward the
                # last accepted value.
                # Fork selection: follow whichever track a set_track sequence
                # chose (default left). On a single line both readings are
                # superimposed, so switching there is seamless; mid-fork the
                # slew clamp below bounds any residual step.
                pv = (sensor["right_mm"] if state.track_follow == "right"
                      else sensor["left_mm"])
                if pv is None or abs(pv) > config.SENSOR_MAX_MM:
                    _dbg_count += 1
                    if _dbg_count % 25 == 0:
                        logger.warning("[AUTO] sensor pv out of range (%s mm) — skipping frame", pv)
                    await asyncio.sleep(config.DT)
                    continue
                if last_valid_pv is not None:
                    step = pv - last_valid_pv
                    if abs(step) > config.SENSOR_MAX_STEP_MM:
                        pv = last_valid_pv + (config.SENSOR_MAX_STEP_MM
                                              if step > 0 else -config.SENSOR_MAX_STEP_MM)
                last_valid_pv = pv

                # ── Curvature feedforward (forward auto, corner zones only) ───
                # Supply the geometric turn so the PID doesn't need a standing
                # error to hold the curve. Low-pass filtered to smooth the
                # step at corner entry/exit. ff = sign*0.5*base*(W/R).
                ff_target = 0.0
                if config.FF_ENABLED and state.nav_in_corner:
                    ff_target = (config.FF_SCALE * config.FF_DIRECTION_SIGN
                                 * 0.5 * base_rpm
                                 * (config.TRACK_WIDTH / config.CURVE_RADIUS))
                # Gentle ramp when entering/deepening a corner; fast collapse on
                # exit so the residual turn command doesn't bleed into the straight
                # and over-rotate the AGV. ff_target is 0 outside corners (always
                # same sign as ff_filtered), so abs(ff_target) > abs(ff_filtered)
                # is true only while ramping up.
                _ff_alpha = (config.FF_ALPHA if abs(ff_target) > abs(ff_filtered)
                             else config.FF_EXIT_ALPHA)
                ff_filtered += _ff_alpha * (ff_target - ff_filtered)

                # ── PID compute ───────────────────────────────────────────────
                left_rpm, right_rpm, dbg = pid.compute(pv, base_rpm, error_sign,
                                                       feedforward=ff_filtered)
                # Near top speed keep the steering difference instead of
                # clipping the faster wheel at 5 V (see motion.keep_steering).
                left_rpm, right_rpm = motion.keep_steering(left_rpm, right_rpm)

                left_v  = motion.rpm_to_voltage(left_rpm,  "left")
                right_v = motion.rpm_to_voltage(right_rpm, "right")

                state.set_ao(0, left_v)
                state.set_ao(1, right_v)

                state.left_rpm     = left_rpm
                state.right_rpm    = right_rpm
                state.pid_output   = dbg["output"]
                state.target_speed = current_target_speed

                recorder.record(
                    phase=state.speed_mode, speed_mode=state.speed_mode,
                    target_speed_ms=target_speed, cmd_speed_ms=current_target_speed,
                    track=state.track_follow,
                    raw_left_mm=sensor.get("left_mm"), raw_right_mm=sensor.get("right_mm"),
                    pv_mm=pv, error_mm=dbg["e"],
                    p_term=dbg["p"], i_term=dbg["i"], d_term=dbg["d"],
                    pid_output=dbg["output"], ff_rpm=dbg["feedforward"],
                    speed_red_rpm=dbg["speed_reduction"],
                    left_rpm=left_rpm, right_rpm=right_rpm,
                    kp=pid._kp, td=pid._td, n=pid._n, v_red=pid._v_red_coef,
                    lidar_zone=state.lidar_zone)

                # Throttle the per-cycle PID debug line (~every 25 cycles) so an
                # accidental console DEBUG level can't flood at 100 Hz.
                _dbg_count += 1
                if _dbg_count % 25 == 0:
                    logger.debug(
                        "[%s] Target=%.2fm/s e=%+.1fmm P=%+.1f I=%+.1f D=%+.1f "
                        "out=%+.1f L=%.1f R=%.1frpm Vred=%.1frpm",
                        state.speed_mode, current_target_speed, dbg["e"],
                        dbg["p"], dbg["i"], dbg["d"], dbg["output"],
                        left_rpm, right_rpm, dbg["speed_reduction"],
                    )

            # ── DI proximity station marker ───────────────────────────────────
            # A magnetic proximity sensor on DI_PROX acts as a station marker for
            # sequences with marker_side "prox" (e.g. trolley_load_00). Fire on
            # the rising edge so one pass triggers one marker event. Independent
            # of the CAN tape frame, so it works even between sensor frames.
            if (engine is not None and config.DI_PROX >= 0
                    and config.SEQ_ENABLED):
                di = state.latest_di
                prox_active = bool(di and len(di) > config.DI_PROX and di[config.DI_PROX])
                if prox_active and not prox_was_active:
                    await engine.on_marker("prox")
                prox_was_active = prox_active

            # ── CAN timeout ───────────────────────────────────────────────────
            if time.time() - state.can_last_rx > config.CAN_TIMEOUT:
                if not tape_was_lost:
                    logger.warning("[AUTO] CAN TIMEOUT — sensor lost, stopping")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    ff_filtered   = 0.0
                    tape_was_lost = True

                await asyncio.sleep(config.DT)
                continue

            await asyncio.sleep(config.DT)

    finally:
        recorder.stop()


# ══════════════════════════════════════════════════════════════════════════════
#  REVERSE MODE  (dead-end backing maneuver — tape-following, direction flipped)
# ══════════════════════════════════════════════════════════════════════════════

async def reverse_mode(state, engine=None):
    """Backing-out maneuver: PID tape-follow with error_sign flipped (-1.0),
    driven via motion.set_reverse. For dead-end sidings (e.g. the C1 loading
    bay) where the AGV can't turn around and must back out the way it came.

    error_sign here corrects for TWO stacked geometry flips, not one:
      1) the rear sensor's own left/right reporting is mirrored relative to
         the front sensor (same physical unit, mounted facing the opposite
         way along the chassis);
      2) both wheels are driven via motion.set_reverse (DO_rev, not DO_fwd),
         so for a *given* PID output sign, "left wheel faster than right"
         yields the OPPOSITE yaw direction compared to auto_mode's forward
         drive (DO_fwd) — the PID's left/right-rpm formula itself doesn't
         know which direction bit is active.
    Flipping error_sign only corrects (1). If the loop is left at +1.0 (as
    it was before), (2) is uncorrected and the controller ends up with net
    POSITIVE feedback — error grows instead of shrinking, output saturates at
    the clamp, and the AGV just keeps turning instead of tracking straight.
    -1.0 corrects both. Field-verify before trusting on the real track (see
    note below) — the exact sign still depends on this AGV's physical rear
    sensor mounting, which wasn't confirmed against hardware here.

    Entered from "running" when state.reverse_request is set True
    (SequenceEngine's "start_reverse" action — tag "1"). Exits back to
    "running" when reverse_request is cleared ("stop_reverse" — normally
    fired by the C1 tag once the AGV backs into it), or faults to a held
    brake on a REVERSE_TIMEOUT_S safety timeout if C1 never shows up
    (reverse_request is left set, so mode_manager will NOT silently resume —
    operator RESET is required to recover from a timeout).

    Deliberately simpler than auto_mode: fixed conservative speed
    (AUTO_TARGET_REVERSE_SPEED, independent of speed_mode), no curvature
    feedforward, no corner/APPROACH gain switching — this is a narrow,
    single-purpose maneuver, not general-purpose driving.

    Sensor source: a REAR-mounted MGS1600 on its own CAN node
    (config.REAR_SENSOR_COB_ID), separate from the front sensor auto_mode
    uses, with its own PID gains (KP_REVERSE/TD_REVERSE/N_REVERSE/
    V_RED_COEF_REVERSE). If a profile hasn't got a rear sensor configured yet
    (config.REAR_SENSOR_ENABLED is False), this falls back to the front
    sensor — the same behaviour as before the rear sensor existed — so
    upgrading this code doesn't strand an AGV that hasn't had the second
    sensor fitted yet.

    Track side: state.reverse_track ("right" by default, reset at the start
    of every reverse). RFID tags with a "set_reverse_track" action (AE → left,
    BB → right while REVERSING) switch it — the REVERSE equivalent of the
    forward track_follow fork selection.
    """
    await motion.idle(state)
    await motion.set_reverse(state, 0.0)
    error_sign = -1.0             # reverse: sensor-mirror flip AND reverse-drive
                                   # wheel-direction flip both corrected — see
                                   # docstring above. Verify on hardware; flip
                                   # back to +1.0 if this makes it worse, not
                                   # better.

    use_rear = config.REAR_SENSOR_ENABLED
    if use_rear:
        logger.info("[REVERSE] Using REAR sensor (CAN node %s) for tape-follow",
                    config.REAR_CAN_NODE_ID)
    else:
        logger.warning("[REVERSE] No rear sensor configured (REAR_SENSOR_COB_ID "
                       "unset) — falling back to the FRONT sensor")
    rev_queue      = state.rear_sensor_queue if use_rear else state.sensor_queue
    sensor_max_mm      = config.REAR_SENSOR_MAX_MM      if use_rear else config.SENSOR_MAX_MM
    sensor_max_step_mm = config.REAR_SENSOR_MAX_STEP_MM if use_rear else config.SENSOR_MAX_STEP_MM

    pid = PIDController(
        kp           = config.KP_REVERSE,
        td           = config.TD_REVERSE,
        n            = config.N_REVERSE,
        ti           = config.TI,
        ti_deadband  = config.TI_DEADBAND,
        ti_max       = config.TI_MAX,
        dt           = config.DT,
        output_clamp = config.OUTPUT_CLAMP_RPM,
        v_red_coef   = config.V_RED_COEF_REVERSE,
        sr_alpha     = config.SR_ALPHA,
        sr_cap       = config.SR_CAP,
    )

    current_target_speed = 0.0
    ramp_t               = None   # last speed-ramp step (perf_counter), see _ramp_dt
    tape_was_lost         = False
    last_valid_pv          = None
    deadline    = time.perf_counter() + config.REVERSE_TIMEOUT_S
    timed_out   = False
    # Reverse stop marker (DI_REVERSE_MARKER): edge-triggered. Start from the
    # CURRENT reading so an AGV already sitting on the marker (or an unwired
    # input that reads ON) does not fire the stop the moment reverse starts.
    _mk = config.DI_REVERSE_MARKER
    def _marker_on():
        di = state.latest_di
        return bool(_mk >= 0 and di is not None and len(di) > _mk and di[_mk])
    marker_prev = _marker_on()
    state.reverse_speed_override = None   # CA sets it again on this run
    # Track for this reverse: fixed by the destination branch when the profile
    # lists it (reverse_track_by_branch — AE / BB are then ignored), otherwise
    # start RIGHT and let AE / BB switch it during this run.
    # A track forced by the start tag (tag 3 → left) beats the branch rule.
    if state.reverse_track_fixed:
        _fixed = state.reverse_track_fixed
        logger.info("[REVERSE] Start tag forces reverse track %s (AE/BB ignored)", _fixed.upper())
    else:
        _fixed = config.REVERSE_TRACK_BY_BRANCH.get(str(state.active_branch))
        if _fixed:
            logger.info("[REVERSE] Branch %s — reverse track fixed %s (AE/BB ignored)",
                        state.active_branch, _fixed.upper())
    state.reverse_track = _fixed or "right"
    state.reverse_marker_armed = False     # tag BA arms the DI10 stop marker
    held        = False   # sequence_stop hold (e.g. C1: waiting for pusher)
    held_remain = 0.0     # timeout budget left when the hold started

    logger.info("[REVERSE] START — backing out toward C1 (timeout %.0fs)",
                config.REVERSE_TIMEOUT_S)

    try:
        while True:
            # ── Exit: stop_reverse cleared the flag (normal completion) ───────
            if not state.reverse_request:
                logger.info("[REVERSE] reverse_request cleared — ending maneuver")
                break

            # ── Emergency guard ────────────────────────────────────────────────
            if state.emergency_active:
                await motion.idle(state)
                current_target_speed = 0.0
                pid.reset()
                last_valid_pv = None
                await asyncio.sleep(config.DT)
                continue

            # ── Reverse stop marker (rising edge) ─────────────────────────────
            marker_now = _marker_on()
            if marker_now and not marker_prev and engine is not None:
                if state.reverse_marker_armed:
                    logger.info("[REVERSE] Stop marker DI%d detected", _mk)
                    await engine.on_reverse_marker()
                else:
                    logger.info("[REVERSE] Stop marker DI%d ignored — tag BA not read yet", _mk)
            marker_prev = marker_now

            # ── Sequence stop — hold in place, still in REVERSE mode ──────────
            # C1 stops the AGV here (stop_agv) and keeps REVERSE active while
            # the pusher goes up; stop_reverse only runs once DI14 confirms.
            # The safety timeout is frozen while held.
            if state.sequence_stop:
                if not held:
                    logger.info("[REVERSE] Sequence stop — holding (brake).")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    held_remain   = max(0.0, deadline - time.perf_counter())
                    held = True
                deadline = time.perf_counter() + held_remain
                while not rev_queue.empty():
                    rev_queue.get_nowait()
                await asyncio.sleep(config.DT)
                continue

            if held:
                logger.info("[REVERSE] Sequence stop ended — resuming reverse.")
                await motion.set_reverse(state, 0.0)
                held = False

            # ── Safety timeout — C1 never showed up ────────────────────────────
            if not timed_out and time.perf_counter() > deadline:
                logger.error(
                    "[REVERSE] timeout after %.0fs — no C1 tag seen, braking and "
                    "holding (operator RESET required)", config.REVERSE_TIMEOUT_S)
                await motion.set_brake(state)
                current_target_speed = 0.0
                pid.reset()
                timed_out = True

            if timed_out:
                # Hold the brake; wait for RESET (mode_manager clears
                # reverse_request via _reset_sequence_state) or emergency.
                await asyncio.sleep(config.DT)
                continue

            target_speed = (state.reverse_speed_override
                            if state.reverse_speed_override is not None
                            else config.AUTO_TARGET_REVERSE_SPEED)

            # ── Drain sensor queue — keep only the latest frame ───────────────
            sensor = None
            while not rev_queue.empty():
                sensor = await rev_queue.get()

            if sensor is not None:
                if not sensor["tape_detected"]:
                    logger.warning("[REVERSE] LOST TAPE — stopping")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    tape_was_lost = True
                    await asyncio.sleep(0.01)
                    continue

                if tape_was_lost:
                    logger.info("[REVERSE] TAPE REACQUIRED — resuming")
                    await motion.set_reverse(state, 0.0)
                    tape_was_lost = False

                ramp_t, rdt = _ramp_dt(ramp_t)
                if current_target_speed < target_speed:
                    current_target_speed += config.ACCEL_RATE * rdt
                    if current_target_speed > target_speed:
                        current_target_speed = target_speed
                elif current_target_speed > target_speed:
                    current_target_speed -= config.DECEL_RATE * rdt
                    if current_target_speed < target_speed:
                        current_target_speed = target_speed

                base_rpm = motion.mps_to_rpm(current_target_speed)

                # Track chosen by RFID AE / BB while reversing (default right).
                pv = (sensor["left_mm"] if state.reverse_track == "left"
                      else sensor["right_mm"])
                if pv is None or abs(pv) > sensor_max_mm:
                    await asyncio.sleep(config.DT)
                    continue
                if last_valid_pv is not None:
                    step = pv - last_valid_pv
                    if abs(step) > sensor_max_step_mm:
                        pv = last_valid_pv + (sensor_max_step_mm
                                              if step > 0 else -sensor_max_step_mm)
                last_valid_pv = pv

                left_rpm, right_rpm, dbg = pid.compute(pv, base_rpm, error_sign)

                left_v  = motion.rpm_to_voltage(left_rpm,  "left")
                right_v = motion.rpm_to_voltage(right_rpm, "right")
                state.set_ao(0, left_v)
                state.set_ao(1, right_v)

                state.left_rpm     = left_rpm
                state.right_rpm    = right_rpm
                state.pid_output   = dbg["output"]
                # Negative marks reverse motion for the HMI/telemetry.
                state.target_speed = -current_target_speed

            # ── CAN timeout ───────────────────────────────────────────────────
            last_rx = state.rear_can_last_rx if use_rear else state.can_last_rx
            if time.time() - last_rx > config.CAN_TIMEOUT:
                if not tape_was_lost:
                    logger.warning("[REVERSE] CAN TIMEOUT — %s sensor lost, stopping",
                                   "rear" if use_rear else "front")
                    await motion.set_brake(state)
                    current_target_speed = 0.0
                    pid.reset()
                    last_valid_pv = None
                    tape_was_lost = True
                await asyncio.sleep(config.DT)
                continue

            await asyncio.sleep(config.DT)

    finally:
        await motion.idle(state)
        logger.info("[REVERSE] STOP")


# ══════════════════════════════════════════════════════════════════════════════
#  MANUAL MODE
# ══════════════════════════════════════════════════════════════════════════════

async def manual_mode(state):
    """Handles pendant jogging and web remote.
    Web remote (state.web_manual_command) takes priority over physical DI buttons.
    Emergency is fully owned by mode_manager — this task does not check it."""
    current_motion = None
    # Signed per-wheel rpm actually commanded (+ fwd / − rev), ramped toward
    # the jog target at MANUAL_ACCEL_RATE (speeding up) / MANUAL_DECEL_RATE
    # (slowing down, stopping, or reversing through zero).
    cur_l = cur_r = 0.0
    last_out = None
    last_t   = time.perf_counter()

    while True:
        # Target motor RPM for each speed; motion helpers convert to per-wheel
        # volts. Re-read every cycle so live speed edits from the params page
        # apply immediately — this task keeps running for the whole time the
        # selector is in MANUAL, so caching these before the loop would pin them
        # to whatever the profile held at task start.
        rpm_high = motion.mps_to_rpm(config.MANUAL_TARGET_HIGH_SPEED)
        rpm_slow = motion.mps_to_rpm(config.MANUAL_TARGET_SLOW_SPEED)

        # ── Drain DI queue — keep only the latest frame ───────────────────────
        di = None
        while not state.di_queue.empty():
            di = await state.di_queue.get()

        # ── Web remote takes priority; fall back to physical DI ───────────────
        # Auto-expire stale web command (network lag / dropped connection safety)
        if state.web_manual_command is not None and time.time() > state.web_manual_expire:
            state.web_manual_command = None

        web_cmd = state.web_manual_command
        if web_cmd is not None:
            motion_state = web_cmd
        elif di is not None:
            # -1 = pendant pin not wired yet (see profile DI_FWD/DI_REV/DI_LEFT/
            # DI_RIGHT) — treat as never-pressed rather than let Python's
            # negative indexing silently read the LAST DI channel instead.
            def _pin(idx):
                return bool(idx >= 0 and idx < len(di) and di[idx])
            pb_fwd   = _pin(config.DI_FWD)
            pb_rvs   = _pin(config.DI_REV)
            pb_left  = _pin(config.DI_LEFT)
            pb_right = _pin(config.DI_RIGHT)

            if pb_fwd and pb_left:       motion_state = "fwd_left"
            elif pb_fwd and pb_right:    motion_state = "fwd_right"
            elif pb_rvs and pb_left:     motion_state = "rvs_left"
            elif pb_rvs and pb_right:    motion_state = "rvs_right"
            elif pb_fwd:                 motion_state = "forward"
            elif pb_rvs:                 motion_state = "reverse"
            elif pb_left:                motion_state = "left"
            elif pb_right:               motion_state = "right"
            else:                        motion_state = "idle"
        else:
            # No new DI frame and no web command: keep the last jog state so
            # the ramp keeps running.
            motion_state = current_motion or "idle"

        if motion_state != current_motion:
            logger.debug("Manual: %s", motion_state.upper())
            current_motion = motion_state

        # Signed per-wheel targets (left, right). Re-read every cycle so live
        # speed edits from the params page apply while a button is held.
        h, s = rpm_high, rpm_slow
        tgt_l, tgt_r = {
            "forward":   ( h,  h), "reverse":   (-h, -h),
            "left":      (-s,  s), "right":     ( s, -s),
            "fwd_left":  ( s,  h), "fwd_right": ( h,  s),
            "rvs_left":  (-s, -h), "rvs_right": (-h, -s),
        }.get(motion_state, (0.0, 0.0))

        now_t  = time.perf_counter()
        dt     = min(max(now_t - last_t, 0.0), 0.1)
        last_t = now_t
        up_step   = motion.mps_to_rpm(config.MANUAL_ACCEL_RATE) * dt
        down_step = motion.mps_to_rpm(config.MANUAL_DECEL_RATE) * dt

        def _ramp(cur, tgt):
            # Speeding up = same sign and |tgt| > |cur|; anything else
            # (slowing, stopping, reversing through zero) uses the decel step.
            speeding_up = (cur == 0 or (cur > 0) == (tgt > 0)) and abs(tgt) > abs(cur)
            step = up_step if speeding_up else down_step
            if cur < tgt:
                return min(cur + step, tgt)
            return max(cur - step, tgt)

        cur_l = _ramp(cur_l, tgt_l)
        cur_r = _ramp(cur_r, tgt_r)

        if cur_l == 0.0 and cur_r == 0.0 and tgt_l == 0.0 and tgt_r == 0.0:
            out = "idle"
            if out != last_out:
                await motion.idle(state)
        else:
            # At exactly 0 rpm hold the direction of the target so the
            # driver is already set the right way as the ramp starts.
            l_cmd = cur_l if cur_l != 0.0 else (1e-9 if tgt_l >= 0 else -1e-9)
            r_cmd = cur_r if cur_r != 0.0 else (1e-9 if tgt_r >= 0 else -1e-9)
            out = (round(l_cmd, 1), round(r_cmd, 1))
            if out != last_out:
                await motion.set_wheels(state, l_cmd, r_cmd)
        last_out = out

        await asyncio.sleep(0.02)


# ══════════════════════════════════════════════════════════════════════════════
#  MODE MANAGER
# ══════════════════════════════════════════════════════════════════════════════

async def mode_manager(state, engine=None, manager=None):
    """Central state machine.

    States: None | "manual" | "armed" | "running" | "calibrate" | "reversing" | "emergency"

    DI conventions (from parameters.json / profile)
    ------------------------------------------------
    DI_MODE_SWITCH : HIGH = MANUAL, LOW = AUTO
    DI_EMERGENCY   : NO contact — True = triggered, False = safe
    DI_START       : momentary NO — rising edge = start
    DI_RESET       : momentary NO — rising edge = reset
    DI_BRANCH_SELECT[i] : momentary NO — rising edge = select branch str(i+1).
                          In ARMED it ALSO starts AUTO (same tape check as
                          START; normal run, no B0 stop). In other modes it
                          only selects. Blocked while running/reversing/
                          emergency (same guard as the HMI's /api/branch).
                          Ignored entirely while web_button_mode is active.

    engine: SequenceEngine (Phase 3). Passed through to auto_mode and used for
            cancel_armed() on mode transitions. None until Phase 3 is wired up.
    """

    current_mode   = None
    active_task    = None
    last_start     = False
    last_reset     = False
    last_call      = False
    state.lidar_area = list(config.LIDAR_DEFAULT_AREA)
    # AGV is normally parked at homepost at power-up (profile homepost.
    # ASSUME_AT_HOME_ON_BOOT) — see state.at_homepost.
    state.at_homepost = config.ASSUME_AT_HOME_ON_BOOT
    last_branch_di = [False] * len(config.DI_BRANCH_SELECT)
    last_phys_manual = None   # physical MANUAL/AUTO selector, for the Q-key override
    rotate_from_manual = False   # 180° turn: return to MANUAL (True) or ARMED afterwards

    async def _cancel_active():
        nonlocal active_task
        if engine is not None:
            await engine.cancel_active()
        if active_task:
            active_task.cancel()
            try:
                await active_task
            except asyncio.CancelledError:
                pass
            active_task = None

    async def _flush_and_idle():
        # Setpoint tables are latest-wins, so asserting idle overwrites any
        # prior command — no queue to drain.
        await motion.idle(state)

    async def _flush_and_brake():
        await motion.set_brake(state)

    def _reset_sequence_state():
        state.speed_mode          = "SLOW"
        state.gain_set            = None
        state.sequence_stop       = False
        state.nav_in_corner       = False
        # Default is "right" — the front sensor's left/right reporting is
        # physically mirrored, see state.py's track_follow comment.
        state.track_follow        = "right"
        state.pending_sequence    = None
        state.calibration_request = False
        state.autotune_request    = False
        state.reverse_request     = False
        state.homepost_request    = False
        state.leaving_home        = False
        state.reverse_speed_override = None
        state.reverse_track       = "right"
        state.reverse_track_fixed = None
        state.reverse_marker_armed = False
        state.alarm_active        = False   # RESET / emergency / mode change silences the alarm horn
        state.lidar_area          = list(config.LIDAR_DEFAULT_AREA)
        state.web_reverse_start   = False
        state.web_reverse_stop    = False
        state.call_run            = False   # next run is a normal run unless DI_CALL_START launches it
        state.kb_branch_request   = None    # a branch key pressed before this transition is dropped
        state.rotate_request      = None    # a T/Y press before this transition is dropped
        # Don't touch state.trolley_hooked here — it reflects physical reality
        # (pusher.py keeps it live from the limit/prox sensors regardless of
        # AGV mode), not something a mode transition should reset. Pending
        # pusher commands, on the other hand, are things we issued — drop them
        # on any of these transitions so they can't fire unexpectedly later.
        # Deliberately NOT touching pusher_estop here — that's a separate,
        # explicit safety latch the operator sets/clears from the dashboard,
        # not something an AGV mode transition should silently release.
        state.pusher_down_request = False
        state.pusher_up_request   = False
        state.pusher_jog_cmd      = None
        if engine is not None:
            engine.cancel_armed()

    while True:
        if state.latest_di is None:
            await asyncio.sleep(0.01)
            continue

        di = state.latest_di

        # ── Horn ────────────────────────────────────────────────────────────
        # DO_HORN_AUTO: on for the duration of any autonomous motion. Driven
        # every cycle here (not just on mode change) so it always reflects
        # current_mode as of the end of the previous iteration — at most one
        # ~10ms loop tick stale, immaterial for a horn.
        if config.DO_HORN_AUTO >= 0:
            # The ALARM horn replaces the AUTO horn while it is on.
            state.set_do(config.DO_HORN_AUTO,
                         current_mode in ("running", "reversing", "autotune", "rotating") and not state.alarm_active)
        if config.DO_HORN_ALARM >= 0:
            state.set_do(config.DO_HORN_ALARM, state.alarm_active)
        # Lidar field-set select outputs (pattern from set_lidar_area).
        for _k, _ch in enumerate(config.LIDAR_DO_AREA):
            if _ch >= 0:
                state.set_do(_ch, state.lidar_area[_k])
        if current_mode != "running":
            state.lidar_zone = None
        # DO_AT_HOME: on while parked at homepost (off the moment AUTO drives away).
        if config.DO_AT_HOME >= 0:
            state.set_do(config.DO_AT_HOME,
                         state.at_homepost and current_mode not in ("running", "reversing"))

        # Physical emergency is always active regardless of web_button_mode.
        # Web emergency adds on top: either source can trigger emergency.
        phys_emergency = di[config.DI_EMERGENCY]
        if state.web_button_mode:
            emergency_safe = not phys_emergency and not state.web_btn_emergency
            switch_manual  = state.web_btn_manual
            btn_start      = state.web_btn_start
            btn_reset      = state.web_btn_reset
        else:
            emergency_safe = not phys_emergency
            phys_manual    = bool(di[config.DI_MODE_SWITCH])
            # Q key (keyboard/keypad) overrides the selector until the
            # selector itself is moved — then the physical switch rules again.
            if (last_phys_manual is not None and phys_manual != last_phys_manual
                    and state.kb_mode_override is not None):
                state.kb_mode_override = None
                logger.info("Mode selector moved — keyboard MANUAL/AUTO override cleared")
            last_phys_manual = phys_manual
            switch_manual  = (phys_manual if state.kb_mode_override is None
                              else state.kb_mode_override)
            # -1 = no physical START button (DI0 is the pusher button now);
            # AUTO is started by the branch buttons / CALL START instead.
            btn_start      = (0 <= config.DI_START < len(di)) and bool(di[config.DI_START])
            btn_reset      = di[config.DI_RESET]

        start_rising = btn_start and not last_start
        reset_rising = btn_reset and not last_reset

        last_start = btn_start
        last_reset = btn_reset

        # ENTER key (keyboard/keypad) = one RESET press, in any mode.
        if state.kb_reset_request:
            state.kb_reset_request = False
            reset_rising = True

        # E key emergency stop: latched until RESET. The same RESET press then
        # also performs the normal emergency recovery below.
        if state.kb_estop and reset_rising:
            state.kb_estop = False
            logger.info("Keyboard EMERGENCY STOP released by RESET.")
        if state.kb_estop:
            emergency_safe = False

        # CALL START (DI_CALL_START): physical-only, ignored in web_button_mode
        # like the other panel inputs. Only honoured at homepost (checked in
        # the ARMED start branch below).
        btn_call = (not state.web_button_mode
                    and 0 <= config.DI_CALL_START < len(di)
                    and bool(di[config.DI_CALL_START]))
        call_rising = btn_call and not last_call
        last_call   = btn_call

        # IO TEST MODE must never survive into autonomous motion.
        if state.io_test_mode and current_mode in ("running", "reversing", "calibrate", "autotune", "rotating"):
            state.io_test_mode = False
            state.clear_do_overrides()
            logger.warning("IO TEST MODE exited automatically — AGV is %s", current_mode)

        # Web AUTO REVERSE pulses (manual page). Consume once; a pulse that
        # arrives in the wrong mode is simply dropped, never queued.
        web_rev_start = state.web_reverse_start and current_mode == "armed"
        web_rev_stop  = state.web_reverse_stop  and current_mode == "reversing"
        state.web_reverse_start = False
        state.web_reverse_stop  = False

        # Clear momentary web buttons after reading so they act as single pulses
        if state.web_button_mode:
            if state.web_btn_start: state.web_btn_start = False
            if state.web_btn_reset: state.web_btn_reset = False

        # ── System error guard (watchdog — Phase 4) ───────────────────────────
        if state.system_error and current_mode not in ("emergency", None):
            logger.error("SYSTEM ERROR — hardware driver lost, stopping")
            await _cancel_active()
            await _flush_and_brake()
            _reset_sequence_state()
            # Hold here until watchdog clears system_error
            await asyncio.sleep(0.01)
            continue

        # ══════════════════════════════════════════════════════════════════════
        #  EMERGENCY
        # ══════════════════════════════════════════════════════════════════════

        if not emergency_safe:
            if current_mode != "emergency":
                logger.critical("!! EMERGENCY — all motion stopped")
                await _cancel_active()
                # idle(): brake ON when brake.ACTIVE_WHEN_STOPPED (default);
                # set it false in the profile to release the motors on
                # emergency so the AGV can be pushed by hand.
                await _flush_and_idle()
                _reset_sequence_state()
                state.emergency_active = True
                current_mode           = "emergency"
                state.current_mode     = current_mode
            await asyncio.sleep(0.01)
            continue

        # ══════════════════════════════════════════════════════════════════════
        #  EMERGENCY RECOVERY
        # ══════════════════════════════════════════════════════════════════════

        if current_mode == "emergency":
            if not reset_rising:
                await asyncio.sleep(0.01)
                continue

            logger.info("Emergency cleared by operator RESET.")
            state.emergency_active = False

            if switch_manual:
                logger.info("Entering MANUAL.")
                await _flush_and_idle()
                active_task        = asyncio.create_task(manual_mode(state))
                current_mode       = "manual"
                state.current_mode = current_mode
            else:
                logger.info("Entering ARMED. Press START to run AUTO.")
                await _flush_and_idle()
                current_mode       = "armed"
                state.current_mode = current_mode

            await asyncio.sleep(0.01)
            continue

        # ══════════════════════════════════════════════════════════════════════
        #  DESTINATION BRANCH SELECT (physical pushbuttons, DI_BRANCH_SELECT)
        # ══════════════════════════════════════════════════════════════════════
        # Independent of the mode state machine below — checked every cycle so
        # a press is never missed regardless of which mode branch matches this
        # iteration. Same guard as the /api/branch HMI endpoint: destination
        # can't change while the AGV is actually moving under auto. Skipped
        # entirely in web_button_mode, matching the other physical DI inputs
        # above — the panel isn't read as trustworthy while that bypass is on.
        branch_start = False   # a branch button pressed in ARMED also starts AUTO
        if not state.web_button_mode:
            branch_locked = current_mode in ("running", "reversing", "emergency", "rotating")
            for _i, _pin in enumerate(config.DI_BRANCH_SELECT):
                if _pin < 0 or _pin >= len(di):
                    continue
                _pressed = di[_pin]
                _rising  = _pressed and not last_branch_di[_i]
                if _rising:
                    if branch_locked:
                        logger.warning(
                            "Branch select button %d ignored — AGV is %s",
                            _i + 1, current_mode)
                    else:
                        state.active_branch = str(_i + 1)
                        logger.info("Branch select button %d pressed — active_branch = %s",
                                    _i + 1, state.active_branch)
                        if current_mode == "armed":
                            branch_start = True
                last_branch_di[_i] = _pressed

        # Keys 7/8/9/0 (keyboard/keypad): same as the branch buttons.
        _kb_branch = state.kb_branch_request
        if _kb_branch is not None:
            state.kb_branch_request = None
            if current_mode in ("running", "reversing", "emergency", "rotating"):
                logger.warning("Branch key %s ignored — AGV is %s", _kb_branch, current_mode)
            else:
                state.active_branch = _kb_branch
                logger.info("Branch key — active_branch = %s", _kb_branch)
                if current_mode == "armed":
                    branch_start = True

        # ══════════════════════════════════════════════════════════════════════
        #  NORMAL TRANSITIONS
        # ══════════════════════════════════════════════════════════════════════

        if current_mode is None:
            await _flush_and_idle()   # trigger DO/AO writer connections; they connect async
            # Always enter ARMED first so the DO/AO Modbus writers have time to
            # establish their TCP connections before manual_mode accepts pendant
            # input. The switch_manual check below fires on the very next loop
            # iteration and immediately transitions to MANUAL if the selector is
            # already in that position.
            logger.info("Startup: ARMED (initializing outputs)")
            current_mode       = "armed"
            state.current_mode = current_mode

        # ── 180° spin-turn (rotate.py) ────────────────────────────────────
        # Started by T / Y (keyboard + AGV keypad) or the manual page, from
        # MANUAL or ARMED. Ends centred on the tape, braked, back in the mode
        # it came from (stop and wait). RESET, T/Y again, the web STOP or
        # moving the selector stops it; emergency is handled above.
        elif current_mode == "rotating":
            done = active_task is None or active_task.done()
            stop_req = state.rotate_request == "stop"
            if reset_rising or stop_req or done or bool(switch_manual) != rotate_from_manual:
                if done:
                    if active_task is not None and not active_task.cancelled():
                        exc = active_task.exception()
                        if exc is not None:
                            logger.error("[ROTATE] task error: %r", exc)
                            state.rotate_status = {"phase": "failed", "message": f"error: {exc}",
                                                   "angle": 0, "t": time.time()}
                    active_task = None
                else:
                    logger.info("[ROTATE] %s — stopping the turn.",
                                "RESET" if reset_rising else
                                "STOP" if stop_req else "mode selector moved")
                    await _cancel_active()
                await _flush_and_idle()
                state.rotate_request = None
                if switch_manual:
                    logger.info("[ROTATE] finished — back to MANUAL (braked).")
                    active_task        = asyncio.create_task(manual_mode(state))
                    current_mode       = "manual"
                else:
                    logger.info("[ROTATE] finished — back to ARMED (braked).")
                    current_mode       = "armed"
                state.current_mode = current_mode

        elif (current_mode in ("manual", "armed") and state.rotate_request
              and (current_mode == "manual") == bool(switch_manual)):
            direction = state.rotate_request
            state.rotate_request = None
            ok, why = (False, "nothing to stop") if direction == "stop" else rotate.check_start(state)
            if not ok:
                if direction != "stop":
                    logger.warning("[ROTATE] %s ignored — %s", direction.upper(), why)
                    state.rotate_status = {"phase": "refused", "message": why,
                                           "angle": 0, "t": time.time()}
            else:
                rotate_from_manual = current_mode == "manual"
                await _cancel_active()          # stops manual_mode's jog loop
                await _flush_and_idle()
                state.web_manual_command = None
                state.at_homepost  = False      # now facing the other way
                active_task        = asyncio.create_task(rotate.rotate_mode(state, direction))
                current_mode       = "rotating"
                state.current_mode = current_mode

        elif switch_manual and current_mode != "manual":
            logger.info("Mode switch: MANUAL")
            state.at_homepost = False   # may be driven away by hand; re-set by reading the homepost tag
            await _cancel_active()
            await _flush_and_idle()
            _reset_sequence_state()
            active_task        = asyncio.create_task(manual_mode(state))
            current_mode       = "manual"
            state.current_mode = current_mode

        elif not switch_manual and current_mode == "manual":
            logger.info("Mode switch: AUTO — ARMED. Place AGV on tape and press START.")
            await _cancel_active()
            await _flush_and_idle()
            current_mode       = "armed"
            state.current_mode = current_mode

        elif (current_mode == "armed" and call_rising and not start_rising
              and not branch_start and not state.at_homepost):
            logger.warning("CALL START (DI%d) ignored — AGV is not at homepost.", config.DI_CALL_START)

        elif current_mode == "armed" and (start_rising or call_rising or branch_start):
            # Only a pure DI_CALL_START press makes a call run (B0 stop);
            # START or a branch button in the same tick wins as a normal run.
            by_call       = call_rising and not start_rising and not branch_start
            tape_present  = False
            latest_sensor = None
            while not state.sensor_queue.empty():
                latest_sensor = state.sensor_queue.get_nowait()
            if latest_sensor is not None:
                tape_present = latest_sensor.get("tape_detected", False)
                await state.sensor_queue.put(latest_sensor)

            if not tape_present:
                logger.warning("START ignored — tape not detected. Place AGV on tape first.")
            else:
                if by_call:
                    logger.info("CALL START (DI%d) — launching AUTO mode, will stop at B0.",
                                config.DI_CALL_START)
                elif branch_start:
                    logger.info("BRANCH %s button — launching AUTO mode to that branch.",
                                state.active_branch)
                else:
                    logger.info("START — launching AUTO mode.")
                state.call_run     = by_call
                state.leaving_home = state.at_homepost
                state.at_homepost  = False
                active_task        = asyncio.create_task(auto_mode(state, engine=engine))
                current_mode       = "running"
                state.current_mode = current_mode

        elif current_mode == "armed" and web_rev_start:
            # AUTO REVERSE started from the web MANUAL page: PID tape-follow
            # backwards (rear sensor) straight from ARMED. Ends like any
            # reverse — at the C1 tag (stop_reverse → resumes AUTO forward),
            # on REVERSE_TIMEOUT_S, or on RESET / web STOP (→ ARMED).
            logger.info("WEB REVERSE — launching REVERSE mode from ARMED.")
            state.reverse_track_fixed = None   # normal track rules for a manual reverse
            state.reverse_request = True
            state.at_homepost     = False
            active_task        = asyncio.create_task(reverse_mode(state, engine=engine))
            current_mode       = "reversing"
            state.current_mode = current_mode

        elif current_mode == "armed" and state.calibration_request:
            # Open-loop wheel-speed calibration ramp. No tape needed (no PID /
            # tape-following): the runner drives both wheels straight and logs
            # the encoder. Allowed only from ARMED (AUTO selector, safe).
            logger.info("CALIBRATION START — launching open-loop wheel-speed ramp.")
            await _cancel_active()
            await _flush_and_idle()
            active_task        = asyncio.create_task(
                calibration.calibrate_mode(state, manager))
            current_mode       = "calibrate"
            state.current_mode = current_mode

        elif current_mode == "armed" and state.autotune_request:
            # PID auto-tune (relay test) on a straight tape — see autotune.py.
            # Tape check is done by the web endpoint; RFID tags are ignored.
            logger.info("AUTOTUNE START — launching relay test.")
            state.at_homepost  = False
            await _cancel_active()
            await _flush_and_idle()
            active_task        = asyncio.create_task(autotune.autotune_mode(state))
            current_mode       = "autotune"
            state.current_mode = current_mode

        elif current_mode == "autotune" and (reset_rising or not state.autotune_request):
            logger.info("AUTOTUNE END — returning to ARMED.")
            state.autotune_request = False
            await _cancel_active()
            await _flush_and_brake()
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "calibrate" and (reset_rising or not state.calibration_request):
            logger.info("CALIBRATION STOP — returning to ARMED.")
            await _cancel_active()
            await _flush_and_idle()
            state.calibration_request = False
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "running" and state.homepost_request:
            # RFID "0" (homepost) — full exit from AUTO, same destination as a
            # manual RESET. Unlike sequence_stop (holds in place, resumable),
            # this actually leaves the running task and drops to ARMED —
            # operator must press START again to resume.
            logger.info("HOMEPOST / stop point reached — exiting AUTO, returning to ARMED.")
            await _cancel_active()
            await _flush_and_idle()
            _reset_sequence_state()
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "running" and state.reverse_request:
            # Tag "1" armed a dead-end backing maneuver — switch AUTO forward
            # over to REVERSE. auto_mode's own sequence-stop/tape-loss state
            # doesn't carry over; reverse_mode starts clean.
            logger.info("REVERSE requested — switching AUTO forward to REVERSE.")
            await _cancel_active()
            await _flush_and_idle()
            # Tag "1" stops the AGV (stop_agv) before start_reverse and never
            # resumes it — clear that here so reverse_mode (which now honours
            # sequence_stop for the C1 hold) actually starts backing up.
            state.sequence_stop = False
            active_task        = asyncio.create_task(reverse_mode(state, engine=engine))
            current_mode       = "reversing"
            state.current_mode = current_mode

        elif current_mode == "reversing" and (reset_rising or web_rev_stop):
            logger.info("%s — aborting REVERSE, returning to ARMED.",
                        "WEB STOP" if web_rev_stop else "RESET")
            await _cancel_active()
            await _flush_and_idle()
            _reset_sequence_state()
            current_mode       = "armed"
            state.current_mode = current_mode

        elif current_mode == "reversing" and not state.reverse_request:
            # Normal completion: the C1 tag fired "stop_reverse", clearing the
            # flag. Resume AUTO forward to continue the route.
            logger.info("REVERSE complete — resuming AUTO forward.")
            # Cancel ONLY the reverse task — NOT the engine's sequence. The C1
            # sequence that cleared reverse_request is still running (it goes
            # on to wait_hooked → resume); cancelling it here would skip the
            # hook wait. auto_mode starts held by sequence_stop until "resume".
            if active_task:
                active_task.cancel()
                try:
                    await active_task
                except asyncio.CancelledError:
                    pass
                active_task = None
            await _flush_and_idle()
            active_task        = asyncio.create_task(auto_mode(state, engine=engine))
            current_mode       = "running"
            state.current_mode = current_mode

        elif current_mode == "running" and reset_rising:
            logger.info("RESET — stopping AUTO, returning to ARMED.")
            await _cancel_active()
            await _flush_and_idle()
            _reset_sequence_state()
            current_mode       = "armed"
            state.current_mode = current_mode
            logger.info("ARMED. Press START to run again.")

        await asyncio.sleep(0.01)
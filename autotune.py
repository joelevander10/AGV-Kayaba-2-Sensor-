"""
autotune.py — PID auto-tune for tape following (relay excitation + model fit)
=============================================================================
Started from the web PARAMS page (engineer, AGV in ARMED on a STRAIGHT tape).

Why not plain Ziegler–Nichols: lateral tape following is a double integrator
(steering difference → turn rate → sideways offset). Classic relay/ZN rules
assume a self-settling process and give poor gains here. Instead:

1. WARMUP  normal PID following while ramping to the speed of the chosen mode,
           until the AGV runs straight at that speed.
2. RELAY   the PID is replaced by a small relay: steer +d rpm when the error is
           above +h mm, −d below −h mm. The AGV weaves gently.
3. FIT     from the recorded steering u(t) and sensor position pv(t) the model
               pv'' = Kg · ( u(t−T) + τL · u'(t−T) )
           is fitted (least squares, best delay T searched):
               Kg  how strongly steering moves the AGV sideways  (mm/s² per rpm)
               τL  sensor look-ahead time (sensor distance ahead of the axle / v)
               T   total lag of drivers, motors and sensor (s)
4. DESIGN  PD gains that give the closed loop a chosen damping ζ and natural
           frequency ωn (limited by the measured lag T):
               KP = ωn² / Kg          TD = 2ζ/ωn − τL          N ≈ TD / 0.04 s
           Three choices are offered (calm / recommended / responsive).

Safety: aborts (brake → ARMED) on tape loss, CAN timeout, lidar inner field,
RESET, emergency or the web STOP; an error above err_limit ends the test (the
data recorded up to then is still used if there is enough). RFID tags are not
acted on. Gains are only applied when the engineer presses APPLY.
"""
import asyncio
import logging
import math
import time

import config
import motion
from core.pid import PIDController
from debugging.plotter import RunRecorder

logger = logging.getLogger(__name__)

# speed mode → (target speed attr, gain keys KP/TD/N/V_RED)
MODE_INFO = {
    "MAX":    ("AUTO_TARGET_MAX_SPEED",    ("KP_MAX",  "TD_MAX",  "N_MAX",  "V_RED_COEF_MAX")),
    "HIGH":   ("AUTO_TARGET_HIGH_SPEED",   ("KP",      "TD",      "N",      "V_RED_COEF")),
    "MEDIUM": ("AUTO_TARGET_MEDIUM_SPEED", ("KP_MEDIUM", "TD_MEDIUM", "N_MEDIUM", "V_RED_COEF_MEDIUM")),
    "SLOW":   ("AUTO_TARGET_SLOW_SPEED",   ("KP_SLOW", "TD_SLOW", "N_SLOW", "V_RED_COEF_SLOW")),
}

DEFAULTS = {"relay_rpm": 20.0, "hyst_mm": 1.5, "err_limit_mm": 50.0,
            "max_s": 30.0, "cycles": 4}
MIN_RELAY_S = 3.0          # least relay data the fit needs
LEAD_S = 0.3               # relay switches on e + LEAD_S·de/dt (adds weaves → better fit)
RELAY_S = 10.0             # relay test length when the cycle count is not reached first

# (id, label, damping ζ, ωn = factor / T)
DESIGNS = [
    ("calm",        "Calm (least overshoot)",  0.90, 0.20),
    ("recommended", "Recommended",             0.80, 0.30),
    ("responsive",  "Responsive",              0.70, 0.40),
]


def new_status():
    return {"phase": "idle", "message": "", "cycles": 0, "elapsed_s": 0.0,
            "params": {}, "result": None}


def fit_model(ts, us, pvs):
    """Least-squares fit of pv = Kg·∬u(t−T) + Kg·τL·∫u(t−T) + y0 + v0·t + c·t²
    (the t² term absorbs a constant side-pull, e.g. a caster or trolley bias).
    Returns dict(kg, tau_l, lag, r2) for the best delay T."""
    import numpy as np
    t = np.asarray(ts) - ts[0]
    dt = 0.01
    tu = np.arange(0.0, t[-1], dt)
    u = np.interp(tu, t, np.asarray(us))
    pv = np.interp(tu, t, np.asarray(pvs))
    best = None
    for lag in np.arange(0.0, 0.41, 0.01):
        ud = np.interp(tu - lag, tu, u, left=u[0])
        i1 = np.cumsum(ud) * dt
        i2 = np.cumsum(i1) * dt
        A = np.column_stack([i2, i1, np.ones_like(tu), tu, tu * tu])
        coef, *_ = np.linalg.lstsq(A, pv, rcond=None)
        res = pv - A @ coef
        sse = float(res @ res)
        if best is None or sse < best[0]:
            best = (sse, lag, coef)
    sse, lag, coef = best
    sst = float(((pv - pv.mean()) ** 2).sum()) or 1e-9
    # Positive steering output REDUCES the error (negative feedback), so the
    # plant gain from u to e is −Kg.
    kg = -float(coef[0])
    tau_l = float(coef[1] / coef[0]) if coef[0] != 0 else float("nan")
    return {"kg": kg, "tau_l": tau_l, "lag": float(lag), "r2": 1.0 - sse / sst}


def design(mode, fit, speed):
    """Only the NET lead (look-ahead minus lag) is identifiable from the data;
    the drive lag is recovered with the known sensor look-ahead
    (config.SENSOR_LOOKAHEAD_M / speed) and limits how fast the loop may be."""
    keys = MODE_INFO[mode][1]
    kg = fit["kg"]
    tau_net = fit["tau_l"] - fit["lag"]
    tau_geom = config.SENSOR_LOOKAHEAD_M / max(speed, 0.05)
    t_lag = max(0.05, tau_geom - tau_net)
    out = []
    for did, label, zeta, f in DESIGNS:
        wn = min(3.0, max(0.5, f / t_lag))
        kp = wn * wn / kg
        td = max(0.0, 2.0 * zeta / wn - tau_net)
        n = min(30.0, max(5.0, td / 0.04)) if td > 0 else getattr(config, keys[2])
        out.append({"id": did, "label": label,
                    "zeta": zeta, "wn": round(wn, 2),
                    "values": {keys[0]: round(kp, 3), keys[1]: round(td, 3), keys[2]: round(n, 1)}})
    return out


def build_result(mode, d, h, ts, us, pvs, periods, amps, speed):
    fit = fit_model(ts, us, pvs)
    tau_net = fit["tau_l"] - fit["lag"]
    lag_est = max(0.0, config.SENSOR_LOOKAHEAD_M / max(speed, 0.05) - tau_net)
    keys = MODE_INFO[mode][1]
    warn = []
    if not (fit["kg"] > 0):
        warn.append("Fitted gain is not positive — check ERROR_SIGN_FORWARD / track; gains not reliable.")
    if fit["r2"] < 0.7:
        warn.append(f"Model fit is weak (R² {fit['r2']:.2f}) — repeat on a straighter tape or with a bigger relay.")
    if lag_est >= 0.35:
        warn.append(f"Drive lag ≈ {lag_est:.2f} s is large — the motor drivers react slowly "
                    "(check the drivers' own accel/decel time setting).")
    res = {"mode": mode, "relay_rpm": d, "hyst_mm": h,
           "relay_s": round(ts[-1] - ts[0], 2), "cycles": len(periods),
           "tu_s": round(sum(periods) / len(periods), 2) if periods else None,
           "amplitude_mm": round(sum(amps) / len(amps), 1) if amps else None,
           "kg": round(fit["kg"], 4), "net_lead_s": round(tau_net, 3),
           "lag_s": round(lag_est, 3), "r2": round(fit["r2"], 3),
           "current": {k: getattr(config, k) for k in keys[:3]},
           "suggestions": design(mode, fit, speed) if fit["kg"] > 0 else [],
           "warnings": warn}
    return res


async def autotune_mode(state):
    from modes import _lidar_zone          # lazy: modes imports this module
    st = state.autotune_status
    p = dict(DEFAULTS); p.update(state.autotune_params or {})
    mode = p.get("speed_mode", "HIGH")
    speed_attr, keys = MODE_INFO[mode]
    target_speed = float(getattr(config, speed_attr))
    d, h = float(p["relay_rpm"]), float(p["hyst_mm"])
    err_limit, max_s, want = float(p["err_limit_mm"]), float(p["max_s"]), int(p["cycles"])
    error_sign = config.ERROR_SIGN_FORWARD
    kp, td, n, vred = (getattr(config, k) for k in keys)

    st.update(new_status()); st["params"] = dict(p, speed_mode=mode, target_speed_ms=target_speed)
    st["phase"] = "warmup"; st["message"] = f"Accelerating to {target_speed:.2f} m/s ({mode})"
    logger.warning("[AUTOTUNE] start %s @ %.2f m/s, relay ±%.0f rpm, hyst %.1f mm, limit %.0f mm",
                   mode, target_speed, d, h, err_limit)

    pid = PIDController(kp=kp, td=td, n=n, ti=config.TI, ti_deadband=config.TI_DEADBAND,
                        ti_max=config.TI_MAX, dt=config.DT, output_clamp=config.OUTPUT_CLAMP_RPM,
                        v_red_coef=vred, sr_alpha=config.SR_ALPHA, sr_cap=config.SR_CAP)
    rec = RunRecorder(tag="autotune"); rec.start()
    await motion.idle(state)
    await motion.set_forward(state, 0.0)

    t0 = time.time()
    cmd_speed, last_pv, straight_since = 0.0, None, None
    relay_u, relay_t0 = 0.0, None
    last_cross, prev_e = None, None
    sig_prev_e, sig_prev_t, de_f = None, None, 0.0
    warm_out = []              # PID output while running straight (side-pull bias)
    u_bias = 0.0
    switches = 0
    cyc_max, cyc_min = -1e9, 1e9
    periods, amps = [], []
    ts, us, pvs = [], [], []

    def finish(phase, message, result=None):
        st["phase"] = phase; st["message"] = message; st["result"] = result
        logger.warning("[AUTOTUNE] %s — %s", phase.upper(), message)

    def finish_with_fit(reason):
        if relay_t0 is not None and switches < 3:
            finish("failed", f"{reason} — the relay never steered the AGV back to the tape "
                             f"(±{d:.0f} rpm is too weak against the side-pull at this speed, "
                             "or the tape is not straight). Increase Relay (rpm), e.g. "
                             f"{min(400, int(d * 2.5))}, and use a straight section.")
            return
        if relay_t0 is None or ts[-1] - ts[0] < MIN_RELAY_S:
            finish("failed", f"{reason} — not enough relay data (needs {MIN_RELAY_S:.0f} s). "
                             "Use a longer straight tape, a smaller relay or a longer time limit.")
            return
        try:
            r = build_result(mode, d, h, ts, us, pvs, periods[2:] or periods, amps[2:] or amps,
                             target_speed)
            finish("done", f"{reason} — model fitted (R² {r['r2']})", r)
        except Exception as ex:                      # never crash the mode on a bad fit
            finish("failed", f"Fit failed: {ex}")

    try:
        while True:
            now = time.time()
            st["elapsed_s"] = round(now - t0, 1)
            if now - t0 > max_s:
                if st["phase"] == "relay":
                    finish_with_fit("Time limit")
                else:
                    finish("failed", f"Time limit ({max_s:.0f} s) reached before the AGV ran straight at speed.")
                break
            if config.LIDAR_ENABLED and _lidar_zone(state) == "inner":
                finish("aborted", "Lidar inner field — stopped"); break
            if time.time() - state.can_last_rx > config.CAN_TIMEOUT:
                finish("aborted", "Magnetic sensor (CAN) timeout"); break

            sensor = None
            while not state.sensor_queue.empty():
                sensor = await state.sensor_queue.get()
            if sensor is None:
                await asyncio.sleep(config.DT); continue
            if not sensor["tape_detected"]:
                finish("aborted", "Tape lost"); break

            pv = sensor["right_mm"] if state.track_follow == "right" else sensor["left_mm"]
            if pv is None or abs(pv) > config.SENSOR_MAX_MM:
                await asyncio.sleep(config.DT); continue
            if last_pv is not None and abs(pv - last_pv) > config.SENSOR_MAX_STEP_MM:
                pv = last_pv + math.copysign(config.SENSOR_MAX_STEP_MM, pv - last_pv)
            last_pv = pv
            e = error_sign * pv
            if abs(e) > err_limit:
                if st["phase"] == "relay":
                    finish_with_fit(f"Error reached the {err_limit:.0f} mm limit")
                else:
                    finish("aborted", f"Error {e:+.0f} mm above the {err_limit:.0f} mm limit during warm-up")
                break

            if cmd_speed < target_speed:
                cmd_speed = min(target_speed, cmd_speed + config.ACCEL_RATE * config.DT)
            base = motion.mps_to_rpm(cmd_speed)

            if st["phase"] == "warmup":
                left, right, dbg = pid.compute(pv, base, error_sign)
                out, pterm, iterm, dterm = dbg["output"], dbg["p"], dbg["i"], dbg["d"]
                if cmd_speed >= target_speed and abs(e) < max(10.0, 3 * h):
                    straight_since = straight_since or now
                    warm_out.append(out)
                    if now - straight_since >= 1.0:
                        # Steering the AGV needed to run straight = constant
                        # side-pull; the relay swings ±d around it.
                        u_bias = sum(warm_out) / len(warm_out)
                        st["phase"] = "relay"; st["message"] = "Relay test running"
                        relay_t0 = now; relay_u = u_bias + (d if e > 0 else -d)
                        logger.info("[AUTOTUNE] relay phase started (bias %+.1f rpm)", u_bias)
                else:
                    straight_since = None
                    warm_out.clear()
            else:
                # switching signal with a little lead (filtered derivative)
                if sig_prev_e is not None and now > sig_prev_t:
                    de_f += 0.2 * ((e - sig_prev_e) / (now - sig_prev_t) - de_f)
                sig_prev_e, sig_prev_t = e, now
                sigma = e + LEAD_S * de_f
                new_u = relay_u
                if sigma > h:
                    new_u = u_bias + d
                elif sigma < -h:
                    new_u = u_bias - d
                if new_u != relay_u:
                    switches += 1
                relay_u = new_u
                left, right = base + relay_u, base - relay_u
                out, pterm, iterm, dterm = relay_u, None, None, None
                ts.append(now); us.append(relay_u); pvs.append(e)
                cyc_max, cyc_min = max(cyc_max, e), min(cyc_min, e)
                if prev_e is not None and prev_e < 0 <= e:
                    if last_cross is not None:
                        periods.append(now - last_cross)
                        amps.append((cyc_max - cyc_min) / 2.0)
                    last_cross, cyc_max, cyc_min = now, e, e
                prev_e = e
                usable = periods[2:]
                st["cycles"] = len(usable)
                st["message"] = (f"Relay test {now - relay_t0:.1f} s · {len(periods)} weave(s)"
                                 + (f" · Tu≈{sum(usable)/len(usable):.2f} s" if usable else ""))
                if len(usable) >= want or now - relay_t0 >= RELAY_S:
                    finish_with_fit(f"{len(periods)} weaves / {now - relay_t0:.1f} s recorded")
                    break

            left, right = motion.keep_steering(left, right)
            state.set_ao(0, motion.rpm_to_voltage(left, "left"))
            state.set_ao(1, motion.rpm_to_voltage(right, "right"))
            state.left_rpm, state.right_rpm, state.pid_output = left, right, out
            state.target_speed = cmd_speed
            rec.record(phase=st["phase"], speed_mode=mode, target_speed_ms=target_speed,
                       cmd_speed_ms=cmd_speed, track=state.track_follow,
                       raw_left_mm=sensor.get("left_mm"), raw_right_mm=sensor.get("right_mm"),
                       pv_mm=pv, error_mm=e, p_term=pterm, i_term=iterm, d_term=dterm,
                       pid_output=out, left_rpm=left, right_rpm=right,
                       kp=kp, td=td, n=n, v_red=vred)
            await asyncio.sleep(config.DT)
    except asyncio.CancelledError:
        if st["phase"] in ("warmup", "relay"):
            finish("aborted", "Stopped (STOP / RESET / emergency)")
        raise
    finally:
        await motion.set_brake(state)
        state.autotune_request = False
        rec.stop()

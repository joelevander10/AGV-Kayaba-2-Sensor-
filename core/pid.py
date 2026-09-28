"""
PID controller with filtered derivative, integral deadband, and speed reduction.

Extracted from modes.py to be reusable across all auto-mode variants (forward,
reverse, future docking modes, etc.).
"""

import time


class PIDController:

    def __init__(self, kp, td, n, ti, ti_deadband, ti_max, dt, output_clamp,
                 v_red_coef, sr_alpha, sr_cap):
        self._kp          = kp
        self._td          = td
        self._n           = n
        self._ti          = ti
        self._ti_deadband = ti_deadband
        self._ti_max      = ti_max
        self._dt          = dt
        # Output clamp as an asymmetric (low, high) pair. The constructor keeps a
        # single symmetric magnitude for backwards compatibility; update_gains can
        # swap in asymmetric bounds per speed mode (e.g. a tight into-the-guide-bar
        # limit during APPROACH).
        self._clamp_hi    = +output_clamp
        self._clamp_lo    = -output_clamp
        self._v_red_coef  = v_red_coef
        self._sr_alpha    = sr_alpha
        self._sr_cap      = sr_cap
        self.reset()

    # ── State management ──────────────────────────────────────────────────────

    def reset(self):
        """Zero all integrator / filter state. Call on tape loss, emergency, or
        mode change — replaces the 6-variable reset block scattered through the
        original auto_mode."""
        self.integral        = 0.0
        self.last_e          = 0.0
        self.filtered_d      = 0.0
        self.speed_reduction = 0.0
        self._last_t         = None   # perf_counter ts of previous compute()

    def update_gains(self, kp, td, n, v_red_coef, clamp_hi=None, clamp_lo=None):
        """Live gain switching (e.g. HIGH speed ↔ SLOW speed).

        Does NOT reset integrator or filter state — the switch should be smooth.
        The derivative filter coefficient is recomputed per cycle from the
        measured dt, so nothing to precompute here.

        clamp_hi/clamp_lo optionally swap the PID-output clamp bounds (rpm). They
        are asymmetric on purpose: in APPROACH the AGV is guided by a one-sided
        physical bar, so the bound on the into-the-bar steering direction is held
        much tighter than the off-the-bar direction. When None, the current bounds
        are kept."""
        self._kp         = kp
        self._td         = td
        self._n          = n
        self._v_red_coef = v_red_coef
        if clamp_hi is not None:
            self._clamp_hi = clamp_hi
        if clamp_lo is not None:
            self._clamp_lo = clamp_lo

    # ── Compute ───────────────────────────────────────────────────────────────

    def compute(self, pv, base_rpm, error_sign=-1.0, feedforward=0.0):
        """One PID cycle.

        Args:
            pv:          process variable — sensor left_mm (lateral offset in mm)
            base_rpm:    current target RPM from the acceleration ramp
            error_sign:  -1.0 for forward (sensor at front),
                         +1.0 for reverse  (sensor at rear — geometry inverted).
                         Every term (P, I, D) derives from `e`, so this factor
                         propagates through the whole controller automatically —
                         do not reintroduce a term computed from raw `pv`.
            feedforward: open-loop steering differential (rpm) added to the PID
                         output — e.g. curvature feedforward in a corner. Not
                         subject to the PID output clamp (it is a known-good
                         command); the final wheel voltages are still clamped
                         downstream.

        Returns:
            (left_rpm, right_rpm, debug_dict)
        """
        e = error_sign * pv

        # ── Measured loop period ──────────────────────────────────────────────
        # Use the real elapsed time, not the nominal config.DT — frame timing
        # jitters and the loop only runs on fresh sensor frames. Clamp to a sane
        # band so a scheduling hiccup or a resume-after-stop gap can't blow up
        # the I/D terms.
        now = time.perf_counter()
        if self._last_t is None:
            dt = self._dt
        else:
            dt = now - self._last_t
            dt = max(0.2 * self._dt, min(5.0 * self._dt, dt))
        self._last_t = now

        # ── P ─────────────────────────────────────────────────────────────────
        p_term = self._kp * e

        # ── I (with deadband) ─────────────────────────────────────────────────
        if self._ti is not None:
            if abs(e) < self._ti_deadband:
                self.integral += e * dt
                self.integral  = max(-self._ti_max, min(self._ti_max, self.integral))
            i_term = self._kp * (1.0 / self._ti) * self.integral
        else:
            i_term = 0.0

        # ── D (low-pass filtered) ─────────────────────────────────────────────
        # Differentiate the ERROR, not the raw measurement, so error_sign reaches
        # the D term the same way it reaches P and I. Differentiating pv with a
        # hardcoded minus only happens to be right when error_sign == -1; it
        # inverts D into positive feedback for error_sign == +1 (reverse).
        # Setpoint is fixed at 0 here, so there is no derivative-kick downside.
        de               = (e - self.last_e) / dt
        alpha            = dt / ((self._td / self._n) + dt)
        raw_d            = self._kp * self._td * de
        self.filtered_d += alpha * (raw_d - self.filtered_d)

        # ── Output clamp (PID steering only) ──────────────────────────────────
        output = p_term + i_term + self.filtered_d
        output = max(self._clamp_lo, min(self._clamp_hi, output))

        # ── Total steering differential = PID + curvature feedforward ─────────
        output_total = output + feedforward

        # ── Speed reduction (low-pass filtered, capped) ───────────────────────
        raw_sr               = abs(e * self._v_red_coef) + abs(de) * 0.5
        raw_sr               = min(raw_sr, base_rpm * self._sr_cap)
        self.speed_reduction += self._sr_alpha * (raw_sr - self.speed_reduction)

        left_rpm  = base_rpm - self.speed_reduction + output_total
        right_rpm = base_rpm - self.speed_reduction - output_total

        self.last_e = e

        return left_rpm, right_rpm, {
            "e":               e,
            "p":               p_term,
            "i":               i_term,
            "d":               self.filtered_d,
            "output":          output,
            "feedforward":     feedforward,
            "output_total":    output_total,
            "speed_reduction": self.speed_reduction,
        }

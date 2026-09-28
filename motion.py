import config

# ── Kinematic Math ────────────────────────────────────────────────────────────
# Motor map is per-wheel: rpm = RPM_PER_VOLT[side] * V + RPM_VOLT_OFFSET[side].
# Left and right differ ~6% (measured), so the inverse used to command voltage
# is side-aware — this corrects both the speed scale and the L/R drift.

def _cal(side):
    if side == "right":
        return config.RPM_PER_VOLT_RIGHT, config.RPM_VOLT_OFFSET_RIGHT
    return config.RPM_PER_VOLT_LEFT, config.RPM_VOLT_OFFSET_LEFT

def voltage_to_rpm(voltage, side="left"):
    k, o = _cal(side)
    return k * voltage + o

def rpm_to_voltage(rpm, side="left"):
    """Inverse motor map for one wheel, clamped to [0, 5] V. rpm <= 0 → 0 V so
    a zero/negative command is a true stop (no creep from the affine offset, and
    we never command reverse through the speed AO here)."""
    if rpm <= 0:
        return 0.0
    k, o = _cal(side)
    return max(0.0, min(5.0, (rpm - o) / k))

def mps_to_rpm(v_meter_per_second):
    v_meter_per_minute = v_meter_per_second * 60
    wheel_rpm = v_meter_per_minute / config.WHEEL_CIRCUMFERENCE
    motor_rpm = wheel_rpm * config.GEAR_RATIO
    return motor_rpm

# ── Motion Commands ───────────────────────────────────────────────────────────

async def _drive(state, left_fwd: bool, left_v: float, right_fwd: bool, right_v: float):
    """
    Core drive primitive. Sets direction DO and speed AO for both wheels.
    Channel mapping is read from config.MOTOR_CHANNELS (set in profile JSON).
    """
    lch = config.MOTOR_CHANNELS["left"]
    rch = config.MOTOR_CHANNELS["right"]

    state.set_do(lch["do_fwd"],   left_fwd)
    state.set_do(lch["do_rev"],   not left_fwd)
    state.set_do(lch["do_brake"], False)

    state.set_do(rch["do_fwd"],   right_fwd)
    state.set_do(rch["do_rev"],   not right_fwd)
    state.set_do(rch["do_brake"], False)

    state.set_ao(lch["ao_speed"], left_v)
    state.set_ao(rch["ao_speed"], right_v)

# High-level helpers take target motor RPM (not voltage) and convert per-wheel.
# A target of 0 → 0 V (true stop). Callers pass mps_to_rpm(speed) or 0.0.

async def set_forward(state, rpm):
    await _drive(state, True, rpm_to_voltage(rpm, "left"),
                        True, rpm_to_voltage(rpm, "right"))

async def set_reverse(state, rpm):
    await _drive(state, False, rpm_to_voltage(rpm, "left"),
                        False, rpm_to_voltage(rpm, "right"))

async def set_left(state, rpm):
    # spin left: left wheel reverse, right wheel forward
    await _drive(state, False, rpm_to_voltage(rpm, "left"),
                        True,  rpm_to_voltage(rpm, "right"))

async def set_right(state, rpm):
    # spin right: left wheel forward, right wheel reverse
    await _drive(state, True,  rpm_to_voltage(rpm, "left"),
                        False, rpm_to_voltage(rpm, "right"))

async def set_forward_left(state, rpm_fast, rpm_slow):
    # both wheels forward, right (outer) faster than left (inner)
    await _drive(state, True, rpm_to_voltage(rpm_slow, "left"),
                        True, rpm_to_voltage(rpm_fast, "right"))

async def set_forward_right(state, rpm_fast, rpm_slow):
    # both wheels forward, left (outer) faster than right (inner)
    await _drive(state, True, rpm_to_voltage(rpm_fast, "left"),
                        True, rpm_to_voltage(rpm_slow, "right"))

async def set_reverse_left(state, rpm_fast, rpm_slow):
    # both wheels reverse, right (outer) faster than left (inner)
    await _drive(state, False, rpm_to_voltage(rpm_slow, "left"),
                        False, rpm_to_voltage(rpm_fast, "right"))

async def set_reverse_right(state, rpm_fast, rpm_slow):
    # both wheels reverse, left (outer) faster than right (inner)
    await _drive(state, False, rpm_to_voltage(rpm_fast, "left"),
                        False, rpm_to_voltage(rpm_slow, "right"))

async def set_brake(state):
    lch = config.MOTOR_CHANNELS["left"]
    rch = config.MOTOR_CHANNELS["right"]
    state.set_ao(lch["ao_speed"], 0.0)
    state.set_ao(rch["ao_speed"], 0.0)
    state.set_do(lch["do_fwd"],   False)
    state.set_do(lch["do_rev"],   False)
    state.set_do(lch["do_brake"], True)
    state.set_do(rch["do_fwd"],   False)
    state.set_do(rch["do_rev"],   False)
    state.set_do(rch["do_brake"], True)

async def idle(state):
    lch = config.MOTOR_CHANNELS["left"]
    rch = config.MOTOR_CHANNELS["right"]
    state.set_ao(lch["ao_speed"], 0.0)
    state.set_ao(rch["ao_speed"], 0.0)
    state.set_do(lch["do_fwd"],   False)
    state.set_do(lch["do_rev"],   False)
    state.set_do(lch["do_brake"], False)
    state.set_do(rch["do_fwd"],   False)
    state.set_do(rch["do_rev"],   False)
    state.set_do(rch["do_brake"], False)

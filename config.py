"""
Configuration loader.

Profile selection
-----------------
Set the AGV_ID environment variable before starting:
    AGV_ID=agv1_kim python3 main.py

If AGV_ID is not set, defaults to "agv1_kim".
Looks for: profiles/{AGV_ID}.json
Falls back to: parameters.json  (backwards-compatibility)
"""

import json
import os

_dir = os.path.dirname(os.path.abspath(__file__))

# ── Profile selection ─────────────────────────────────────────────────────────

AGV_ID = os.environ.get("AGV_ID", "agv1_kim")
print(AGV_ID)
_profile_path   = os.path.join(_dir, "profiles", f"{AGV_ID}.json")
_fallback_path  = os.path.join(_dir, "parameters.json")

if os.path.exists(_profile_path):
    _param_path = _profile_path
elif os.path.exists(_fallback_path):
    _param_path = _fallback_path
else:
    raise FileNotFoundError(
        f"No profile found for AGV_ID='{AGV_ID}'. "
        f"Expected: {_profile_path} or {_fallback_path}"
    )

try:
    with open(_param_path, "r") as f:
        _params = json.load(f)
except json.JSONDecodeError as e:
    raise ValueError(f"CRITICAL: {_param_path} is malformed: {e}")

# ── Networking ────────────────────────────────────────────────────────────────
LOCAL_IP    = _params["networking"].get("LOCAL_IP", "0.0.0.0")
DIO_IP      = _params["networking"]["DIO_IP"]
AO_IP       = _params["networking"]["AO_IP"]
MODBUS_PORT = _params["networking"]["MODBUS_PORT"]
DEVICE_ID   = _params["networking"]["DEVICE_ID"]
RFID_IP     = _params["networking"]["RFID_IP"]
RFID_PORT   = _params["networking"]["RFID_PORT"]

# ── IO Mapping ────────────────────────────────────────────────────────────────
DI_FLIPPED     = bool(_params["io_mapping"].get("DI_FLIPPED", 0))
DI_BASE        = _params["io_mapping"]["DI_BASE"]
DO_BASE        = _params["io_mapping"]["DO_BASE"]
NUM_DI         = _params["io_mapping"]["NUM_DI"]
NUM_DO         = _params["io_mapping"]["NUM_DO"]
AO_BASE        = _params["io_mapping"]["AO_BASE"]
NUM_AO         = _params["io_mapping"]["NUM_AO"]
DI_EMERGENCY   = _params["io_mapping"]["DI_EMERGENCY"]
DI_FWD         = _params["io_mapping"]["DI_FWD"]
DI_REV         = _params["io_mapping"]["DI_REV"]
DI_LEFT        = _params["io_mapping"]["DI_LEFT"]
DI_RIGHT       = _params["io_mapping"]["DI_RIGHT"]
DI_MODE_SWITCH = _params["io_mapping"]["DI_MODE_SWITCH"]
DI_START       = _params["io_mapping"]["DI_START"]
DI_RESET       = _params["io_mapping"]["DI_RESET"]
# Magnetic proximity sensor used as a station marker for sequences (-1 = none).
DI_PROX        = _params["io_mapping"].get("DI_PROX", -1)

# Physical destination-branch select buttons (replaces the HMI-only /api/branch
# selector). 4-element list, index i -> branch str(i+1) ("1".."4"); a slot of -1
# means that branch has no physical button wired. Momentary NO pushbuttons,
# read as a rising edge in mode_manager (same pattern as DI_START/DI_RESET).
DI_BRANCH_SELECT = _params["io_mapping"].get("DI_BRANCH_SELECT", [-1, -1, -1, -1])

# Trolley-hitch limit switches (normally-closed). Per-index logical meaning:
# True = switch un-pressed = trolley NOT yet caught; False = pressed = caught.
# Watched continuously by pusher.pusher_controller (not just "during reverse").
DI_TROLLEY_LIMIT = _params["io_mapping"].get("DI_TROLLEY_LIMIT", [-1, -1])

# Pusher pin position — inductive proximity sensors, top/bottom of travel.
DI_PUSHER_PROX_TOP    = _params["io_mapping"].get("DI_PUSHER_PROX_TOP", -1)
DI_PUSHER_PROX_BOTTOM = _params["io_mapping"].get("DI_PUSHER_PROX_BOTTOM", -1)
# CALL START — momentary/NO input, rising edge starts AUTO from ARMED, but only
# while the AGV is at homepost (state.at_homepost). A run started this way sets
# state.call_run, which gates the profile's "stop_at_b0" sequence
# ("requires_flag": "call_run"). -1 = not wired.
DI_CALL_START = _params["io_mapping"].get("DI_CALL_START", -1)
# REVERSE stop marker — magnetic/prox marker read ONLY while REVERSING. A
# rising edge fires any sequence whose trigger has "reverse_marker": true
# (the CC stop), so the AGV stops on the marker or on RFID CC, whichever
# comes first. -1 = not wired.
DI_REVERSE_MARKER = _params["io_mapping"].get("DI_REVERSE_MARKER", -1)

# ── Motor Channels (parameterized — avoids hardcoded DO 0-5 in motion.py) ────
# Falls back to the hardcoded mapping if the profile doesn't have this section
# (backwards-compat with old parameters.json).
MOTOR_CHANNELS = _params.get("motor_channels", {
    "left":  {"do_fwd": 0, "do_rev": 1, "do_brake": 2, "ao_speed": 0},
    "right": {"do_fwd": 3, "do_rev": 4, "do_brake": 5, "ao_speed": 1},
})

# ── Hardware Specs ────────────────────────────────────────────────────────────
V_RANGE = _params["hardware"]["V_RANGE"]
DAC_RES = _params["hardware"]["DAC_RES"]

# Motor speed calibration: rpm = RPM_PER_VOLT * volts + RPM_VOLT_OFFSET.
# Per-wheel (left/right differ ~6%); each falls back to the shared value if a
# profile only provides the single pair.
RPM_PER_VOLT    = _params["hardware"].get("RPM_PER_VOLT", 646.59)
RPM_VOLT_OFFSET = _params["hardware"].get("RPM_VOLT_OFFSET", -101.2)
RPM_PER_VOLT_LEFT     = _params["hardware"].get("RPM_PER_VOLT_LEFT",     RPM_PER_VOLT)
RPM_VOLT_OFFSET_LEFT  = _params["hardware"].get("RPM_VOLT_OFFSET_LEFT",  RPM_VOLT_OFFSET)
RPM_PER_VOLT_RIGHT    = _params["hardware"].get("RPM_PER_VOLT_RIGHT",    RPM_PER_VOLT)
RPM_VOLT_OFFSET_RIGHT = _params["hardware"].get("RPM_VOLT_OFFSET_RIGHT", RPM_VOLT_OFFSET)

# ── Kinematics ────────────────────────────────────────────────────────────────
WHEEL_DIAMETER      = _params["kinematics"]["WHEEL_DIAMETER"]
GEAR_RATIO          = _params["kinematics"]["GEAR_RATIO"]
WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER

# ── Speeds ────────────────────────────────────────────────────────────────────
MANUAL_TARGET_HIGH_SPEED     = _params["speeds"]["MANUAL_TARGET_HIGH_SPEED"]
MANUAL_TARGET_SLOW_SPEED     = _params["speeds"]["MANUAL_TARGET_SLOW_SPEED"]
# Manual jog ramps (m/s²): each wheel ramps toward its jog target instead of
# jumping; releasing the button ramps down with MANUAL_DECEL_RATE.
MANUAL_ACCEL_RATE            = _params["speeds"].get("MANUAL_ACCEL_RATE", _params["speeds"]["ACCEL_RATE"])
MANUAL_DECEL_RATE            = _params["speeds"].get("MANUAL_DECEL_RATE", 0.8)
AUTO_TARGET_HIGH_SPEED       = _params["speeds"]["AUTO_TARGET_HIGH_SPEED"]
# MAX speed mode (set_speed "MAX", e.g. RFID 0018). Uses the HIGH PID gains.
AUTO_TARGET_MAX_SPEED        = _params["speeds"].get("AUTO_TARGET_MAX_SPEED", AUTO_TARGET_HIGH_SPEED)
AUTO_TARGET_SLOW_SPEED       = _params["speeds"]["AUTO_TARGET_SLOW_SPEED"]
AUTO_TARGET_APPROACH_SPEED   = _params["speeds"].get("AUTO_TARGET_APPROACH_SPEED", 0.1)
ACCEL_RATE                   = _params["speeds"]["ACCEL_RATE"]
DECEL_RATE                   = _params["speeds"]["DECEL_RATE"]
APPROACH_DECEL_RATE          = _params["speeds"]["APPROACH_DECEL_RATE"]
# Reverse-drive (mundur) speed — deliberately NOT tied to speed_mode/HIGH: a
# blind-ish backing maneuver toward a dead-end station is capped low for
# safety regardless of whatever speed the AGV was running before it hit the
# reverse trigger tag. Falls back to a conservative 0.15 m/s if the profile
# doesn't define it.
AUTO_TARGET_REVERSE_SPEED    = _params["speeds"].get("AUTO_TARGET_REVERSE_SPEED", 0.15)
# Safety cap on how long REVERSE mode may run before the exit tag (C1) shows
# up. If exceeded, the AGV brakes and holds — operator RESET required. Guards
# against reversing indefinitely if the exit tag is missing/misread.
REVERSE_TIMEOUT_S            = _params["speeds"].get("REVERSE_TIMEOUT_S", 30.0)

# ── PID Tuning ────────────────────────────────────────────────────────────────
# Steering direction for FORWARD auto: e = ERROR_SIGN_FORWARD * pv.
# -1.0 = original convention; +1.0 flips P/I/D together if the AGV steers
# AWAY from the tape instead of back onto it. Only -1.0 or +1.0.
ERROR_SIGN_FORWARD = float(_params["pid_tuning"].get("ERROR_SIGN_FORWARD", -1.0))
KP = _params["pid_tuning"]["KP"]
TD = _params["pid_tuning"]["TD"]
N  = _params["pid_tuning"]["N"]

KP_SLOW = _params["pid_tuning"]["KP_SLOW"]
TD_SLOW = _params["pid_tuning"]["TD_SLOW"]
N_SLOW  = _params["pid_tuning"]["N_SLOW"]

KP_APPROACH         = _params["pid_tuning"].get("KP_APPROACH",         1.6)
TD_APPROACH         = _params["pid_tuning"].get("TD_APPROACH",         0.12)
N_APPROACH          = _params["pid_tuning"].get("N_APPROACH",          12)

# PID gains — REVERSE (dead-end backing maneuver, rear sensor). The rear
# sensor's mounting offset/geometry differs from the front one, so the
# controller needs its own gain set rather than reusing SLOW verbatim.
# Falls back to the SLOW gains so a profile that hasn't been retuned yet
# behaves exactly as before this feature was added.
KP_REVERSE          = _params["pid_tuning"].get("KP_REVERSE",          _params["pid_tuning"]["KP_SLOW"])
TD_REVERSE          = _params["pid_tuning"].get("TD_REVERSE",          _params["pid_tuning"]["TD_SLOW"])
N_REVERSE           = _params["pid_tuning"].get("N_REVERSE",           _params["pid_tuning"]["N_SLOW"])
V_RED_COEF_REVERSE  = _params["pid_tuning"].get("V_RED_COEF_REVERSE",  _params["pid_tuning"]["V_RED_COEF_SLOW"])

TI          = _params["pid_tuning"]["TI"]
TI_DEADBAND = _params["pid_tuning"]["TI_DEADBAND"]
TI_MAX      = _params["pid_tuning"]["TI_MAX"]
DT          = _params["pid_tuning"]["DT"]

V_RED_COEF            = _params["pid_tuning"]["V_RED_COEF"]
V_RED_COEF_SLOW       = _params["pid_tuning"]["V_RED_COEF_SLOW"]
V_RED_COEF_APPROACH   = _params["pid_tuning"].get("V_RED_COEF_APPROACH", 0)
OUTPUT_CLAMP_RPM = _params["pid_tuning"]["OUTPUT_CLAMP_RPM"]
# Asymmetric APPROACH clamp (magnitudes, rpm). The station approach is guided by a
# one-sided physical bar on the AGV's right, so steering INTO the bar is held tight
# (the bar absorbs that correction mechanically) while steering off the bar keeps
# more authority. Default to the symmetric clamp so older profiles are unchanged.
OUTPUT_CLAMP_INTO_BAR_APPROACH = _params["pid_tuning"].get("OUTPUT_CLAMP_INTO_BAR_APPROACH", OUTPUT_CLAMP_RPM)
OUTPUT_CLAMP_AWAY_APPROACH     = _params["pid_tuning"].get("OUTPUT_CLAMP_AWAY_APPROACH",     OUTPUT_CLAMP_RPM)
SR_ALPHA         = _params["pid_tuning"]["SR_ALPHA"]
SR_CAP           = _params["pid_tuning"]["SR_CAP"]

# ── CAN Sensor ────────────────────────────────────────────────────────────────
SENSOR_COB_ID    = _params["can_sensor"]["SENSOR_COB_ID"]
FLAG_TAPE_DETECT  = _params["can_sensor"]["FLAG_TAPE_DETECT"]
FLAG_LEFT_MARKER  = _params["can_sensor"]["FLAG_LEFT_MARKER"]
FLAG_RIGHT_MARKER = _params["can_sensor"]["FLAG_RIGHT_MARKER"]
FLAG_SENSOR_FAIL  = _params["can_sensor"]["FLAG_SENSOR_FAIL"]
CAN_TIMEOUT      = _params["can_sensor"]["CAN_TIMEOUT"]
CAN_NODE_ID      = _params["can_sensor"]["CAN_NODE_ID"]

# Sensor sanity / slew limits for the lateral offset (left_mm) fed to the PID.
# Reject readings beyond ±SENSOR_MAX_MM (sensor half-width is ~80 mm); clamp
# per-cycle jumps larger than SENSOR_MAX_STEP_MM to reject glitch frames.
SENSOR_MAX_MM      = _params["can_sensor"].get("SENSOR_MAX_MM", 100.0)
SENSOR_MAX_STEP_MM = _params["can_sensor"].get("SENSOR_MAX_STEP_MM", 40.0)

# ── Rear CAN Sensor (second MGS1600 used only by REVERSE mode) ───────────────
# Same physical CAN bus/adapter as the front sensor (and the encoder) — just a
# distinct CANopen node, so its frames show up under their own COB-ID on the
# same wire. can_mgs1600.py filters on REAR_SENSOR_COB_ID in addition to
# SENSOR_COB_ID and publishes matching frames separately.
# REAR_SENSOR_COB_ID absent from the profile (= None) means no rear sensor is
# fitted: can_mgs1600.py then skips rear-frame parsing entirely, and
# reverse_mode (modes.py) simply never receives a rear frame — same as the
# pre-existing "no sensor" behaviour, no crash.
REAR_SENSOR_COB_ID  = _params["can_sensor"].get("REAR_SENSOR_COB_ID")
REAR_CAN_NODE_ID    = _params["can_sensor"].get("REAR_CAN_NODE_ID")
REAR_SENSOR_ENABLED = REAR_SENSOR_COB_ID is not None
# Rear frame sanity/slew limits — default to the front sensor's (same hardware
# model) unless the profile overrides them.
REAR_SENSOR_MAX_MM      = _params["can_sensor"].get("REAR_SENSOR_MAX_MM",      SENSOR_MAX_MM)
REAR_SENSOR_MAX_STEP_MM = _params["can_sensor"].get("REAR_SENSOR_MAX_STEP_MM", SENSOR_MAX_STEP_MM)

# ── RFID ──────────────────────────────────────────────────────────────────────
RFID_INIT_CMD       = bytes.fromhex(_params["rfid"]["RFID_INIT_CMD_HEX"])
# RFID_COMMANDS kept for backwards-compat if old parameters.json is used
RFID_COMMANDS       = _params.get("rfid_commands", {})
SEQUENCE_STOP_DELAY = _params["rfid"]["SEQUENCE_STOP_DELAY"]

# ── Sequences (Phase 3 — sequence engine) ────────────────────────────────────
SEQUENCES = _params.get("sequences", [])

# ── Feature flags ────────────────────────────────────────────────────────────
# RFID is split into two categories with a dependency hierarchy:
#   NAV  (navigation) — corner speed-zone tags (set_speed). Independent base.
#   SEQ  (sequence)   — stop/run/checkpoint sequences. Requires NAV.
# Cascade: disabling NAV disables SEQ.
# Legacy profiles with a single RFID_ENABLED flag map onto both NAV and SEQ.
_feat = _params.get("features", {})
_legacy_rfid = _feat.get("RFID_ENABLED", 1)
DIO_ENABLED  = bool(_feat.get("DIO_ENABLED", 1))
CAN_ENABLED  = bool(_feat.get("CAN_ENABLED", 1))
NAV_ENABLED  = bool(_feat.get("NAV_ENABLED", _legacy_rfid))
SEQ_ENABLED  = bool(_feat.get("SEQ_ENABLED", _legacy_rfid))

# Enforce the dependency hierarchy at load (a child can't outlive its parent).
if not NAV_ENABLED:
    SEQ_ENABLED = False

# ── Watchdog timeouts (Phase 4) ───────────────────────────────────────────────
_wd = _params.get("watchdog", {})
WATCHDOG_DI_TIMEOUT_S   = _wd.get("DI_TIMEOUT_S",   1.0)
WATCHDOG_CAN_TIMEOUT_S  = _wd.get("CAN_TIMEOUT_S",  1.0)
WATCHDOG_RFID_TIMEOUT_S = _wd.get("RFID_TIMEOUT_S", 5.0)

# ── Hardware poll intervals ────────────────────────────────────────────────────
_timing = _params.get("timing", {})
DI_POLL_INTERVAL = _timing.get("DI_POLL_INTERVAL", 0.02)

# ── Curvature feedforward (corner steering) ───────────────────────────────────
# In a sustained turn a pure-feedback controller settles with a standing
# cross-track error (it needs error to generate the turn). Feeding forward the
# geometric differential removes that lag and lets corners be taken faster.
#   output_ff = FF_DIRECTION_SIGN * 0.5 * base_rpm * (TRACK_WIDTH / CURVE_RADIUS)
# applied while speed_mode is one of FF_CURVE_MODES (the corner zones).
_ff = _params.get("feedforward", {})
FF_ENABLED        = bool(_ff.get("ENABLED", 0))
TRACK_WIDTH       = _ff.get("TRACK_WIDTH_M", 0.46)    # centre-to-centre wheel track
CURVE_RADIUS      = _ff.get("CURVE_RADIUS_M", 1.5)
_ff_dir           = str(_ff.get("CURVE_DIRECTION", "left")).lower()
FF_DIRECTION_SIGN = -1.0 if _ff_dir == "left" else 1.0   # left turn => output < 0
FF_CURVE_MODES    = set(_ff.get("CURVE_SPEED_MODES", ["SLOW"]))
FF_ALPHA          = _ff.get("FF_ALPHA", 0.15)            # smooths corner entry/exit
FF_EXIT_ALPHA     = _ff.get("FF_EXIT_ALPHA", FF_ALPHA)   # fast collapse on corner exit (else FF_ALPHA)
FF_SCALE          = _ff.get("FF_SCALE", 1.0)             # overall multiplier on the corner compensation


# ── Encoder (wheel-speed calibration) ─────────────────────────────────────────
# CANopen friction-wheel encoder used to measure ACTUAL wheel ground speed.
# Multi-turn: TPDO1 maps object 0x6004 (24-bit), so the count wraps at
# TOTAL_RANGE, not every revolution. Rolling contact => v_ground = omega * RADIUS.
_enc = _params.get("encoder", {})
ENCODER_ENABLED          = bool(_enc.get("ENABLED", 0))
ENCODER_NODE_ID          = int(_enc.get("NODE_ID", 1))
ENCODER_BITRATE          = int(_enc.get("BITRATE", 125000))
ENCODER_COUNTS_PER_REV   = int(_enc.get("COUNTS_PER_REV", 4096))
ENCODER_TOTAL_RANGE      = int(_enc.get("TOTAL_RANGE", 16777216))
ENCODER_WHEEL_DIAMETER_M = float(_enc.get("WHEEL_DIAMETER_M", 0.060))
ENCODER_RADIUS_M         = ENCODER_WHEEL_DIAMETER_M / 2.0
ENCODER_TPDO_COB_ID_BASE = int(_enc.get("TPDO_COB_ID_BASE", 0x180))

# ── Wheel-speed calibration ramp (open-loop straight) ─────────────────────────
_cal = _params.get("calibration", {})
CAL_V_START = float(_cal.get("V_START", 0.0))
CAL_V_STEP  = float(_cal.get("V_STEP", 0.02))
CAL_V_MAX   = float(_cal.get("V_MAX", 0.80))
CAL_DWELL_S = float(_cal.get("DWELL_S", 4.0))


# ── Pusher (trolley-hitch pin actuator) ───────────────────────────────────────
# See pusher.py for the full state machine. Both channels of a pair are driven
# together (2 DO per direction, per the physical wiring). [-1,-1] (the
# default) means this profile has no pusher hardware — pusher_controller()
# becomes a no-op in that case.
_pusher = _params.get("pusher", {})
PUSHER_DO_UP          = _pusher.get("DO_UP",   [-1, -1])
PUSHER_DO_DOWN        = _pusher.get("DO_DOWN", [-1, -1])
PUSHER_HOOK_DELAY_S   = float(_pusher.get("HOOK_DELAY_S", 2.0))
# Safety guard (not requested explicitly — added defensively): de-energize
# and abort a move if it hasn't reached its target proximity sensor within
# this long, instead of driving the actuator indefinitely against a jam.
PUSHER_MOVE_TIMEOUT_S = float(_pusher.get("MOVE_TIMEOUT_S", 15.0))
# Explicit, operator-controlled gate for the AUTONOMOUS hook sequence (the
# idle -> hook_wait -> moving_up chain that runs with no button pressed).
# Defaults to OFF. This is deliberately NOT inferred from whether
# DI_TROLLEY_LIMIT/DI_PUSHER_PROX_TOP have channel numbers assigned — a
# channel can be numbered in the profile before the physical sensor is
# actually wired/mounted, and an unwired or misread limit switch can false-
# positive "caught" and fire the sequence with no trolley present. Flip this
# to true in the profile only once the limit switches AND top proximity
# sensor are physically installed and verified. While false, DO8-11 only
# ever move in response to an explicit button command (FULL UP/DOWN, JOG).
PUSHER_AUTO_SEQUENCE_ENABLED = bool(_pusher.get("AUTO_SEQUENCE_ENABLED", False))
# Temporary: ignore the trolley limit switches (DI_TROLLEY_LIMIT) completely.
# trolley_hooked then means "pusher pin at top proximity" only, the
# limit-switch auto sequence never runs, and a parked "hooked" pin is not
# dropped back to idle by a limit switch. The pin is raised by an explicit
# command instead (web button, or the "pusher_up" sequence action at C1).
PUSHER_IGNORE_TROLLEY_LIMIT = bool(_pusher.get("IGNORE_TROLLEY_LIMIT", False))

# ── Horn ──────────────────────────────────────────────────────────────────────
# DO_AUTO: on for the duration of any autonomous motion (RUNNING or
# REVERSING). DO_ALARM: mirrors state.alarm_active, which nothing sets yet —
# wire up the actual alarm condition(s) later (sequence action, watchdog
# fault, etc.) and this output follows automatically.
_horn = _params.get("horn", {})
DO_HORN_AUTO  = _horn.get("DO_AUTO",  -1)
DO_HORN_ALARM = _horn.get("DO_ALARM", -1)

# ── Lidar safety fields (AUTO forward only) ───────────────────────────────────
# Three lidar outputs, read as DI (ON = object in that field, as the code
# sees it after DI_FLIPPED). OUTER caps speed at OUTER_SPEED, MIDDLE at
# MIDDLE_SPEED (both reached with DECEL_RATE, i.e. slowly), INNER brakes at
# once; after INNER clears the AGV waits RESUME_DELAY_S and continues.
# DO_AREA: the two outputs that select the lidar's field set (pattern set by
# the "set_lidar_area" sequence action, DEFAULT_AREA at start / reset).
_lidar = _params.get("lidar", {})
LIDAR_ENABLED        = bool(_lidar.get("ENABLED", False))
LIDAR_DI_OUTER       = _lidar.get("DI_OUTER",  -1)
LIDAR_DI_MIDDLE      = _lidar.get("DI_MIDDLE", -1)
LIDAR_DI_INNER       = _lidar.get("DI_INNER",  -1)
LIDAR_OUTER_SPEED    = float(_lidar.get("OUTER_SPEED",  0.5))
LIDAR_MIDDLE_SPEED   = float(_lidar.get("MIDDLE_SPEED", 0.2))
LIDAR_DECEL_RATE     = float(_lidar.get("DECEL_RATE", _params["speeds"]["DECEL_RATE"]))
LIDAR_RESUME_DELAY_S = float(_lidar.get("RESUME_DELAY_S", 2.0))
LIDAR_DO_AREA        = list(_lidar.get("DO_AREA", [-1, -1]))
LIDAR_DEFAULT_AREA   = [bool(v) for v in _lidar.get("DEFAULT_AREA", [False, False])]

# ── Homepost ──────────────────────────────────────────────────────────────────
# RFID_TAG: the homepost tag (keep in sync with the arrive_homepost sequence).
# DO_AT_HOME: on while state.at_homepost and the AGV is not RUNNING/REVERSING.
# ASSUME_AT_HOME_ON_BOOT: start the program with at_homepost=True (the AGV is
# normally parked at homepost when powered up).
_home = _params.get("homepost", {})
HOMEPOST_RFID_TAG      = _home.get("RFID_TAG", "0000")
DO_AT_HOME             = _home.get("DO_AT_HOME", -1)
ASSUME_AT_HOME_ON_BOOT = bool(_home.get("ASSUME_AT_HOME_ON_BOOT", False))


# ── Config validation (fail-fast at boot) ─────────────────────────────────────
class ConfigError(ValueError):
    """Raised at import time when a profile parameter is out of safe range."""


_MAX_SPEED_MPS = 5.0   # sanity ceiling for any commanded speed


def _validate():
    """Range-check critical parameters so the controller refuses to start on a
    bad profile rather than dividing by zero or running away mid-motion."""
    errors = []

    def check(cond, msg):
        if not cond:
            errors.append(msg)

    # Division-by-zero / runaway guards
    check(WHEEL_DIAMETER > 0, f"WHEEL_DIAMETER must be > 0 (got {WHEEL_DIAMETER})")
    check(GEAR_RATIO    > 0, f"GEAR_RATIO must be > 0 (got {GEAR_RATIO})")
    check(DT            > 0, f"DT must be > 0 (got {DT})")
    check(TI is None or TI > 0, f"TI must be > 0 (used as 1/TI) (got {TI})")
    check(V_RANGE       > 0, f"V_RANGE must be > 0 (got {V_RANGE})")
    check(DAC_RES       > 0, f"DAC_RES must be > 0 (got {DAC_RES})")
    check(RPM_PER_VOLT       != 0, f"RPM_PER_VOLT must be != 0 (got {RPM_PER_VOLT})")
    check(RPM_PER_VOLT_LEFT  != 0, f"RPM_PER_VOLT_LEFT must be != 0 (got {RPM_PER_VOLT_LEFT})")
    check(RPM_PER_VOLT_RIGHT != 0, f"RPM_PER_VOLT_RIGHT must be != 0 (got {RPM_PER_VOLT_RIGHT})")
    check(OUTPUT_CLAMP_RPM > 0, f"OUTPUT_CLAMP_RPM must be > 0 (got {OUTPUT_CLAMP_RPM})")
    check(OUTPUT_CLAMP_INTO_BAR_APPROACH > 0, f"OUTPUT_CLAMP_INTO_BAR_APPROACH must be > 0 (got {OUTPUT_CLAMP_INTO_BAR_APPROACH})")
    check(OUTPUT_CLAMP_AWAY_APPROACH     > 0, f"OUTPUT_CLAMP_AWAY_APPROACH must be > 0 (got {OUTPUT_CLAMP_AWAY_APPROACH})")
    check(N      > 0, f"N must be > 0 (got {N})")
    check(N_SLOW > 0, f"N_SLOW must be > 0 (got {N_SLOW})")
    check(N_APPROACH > 0, f"N_APPROACH must be > 0 (got {N_APPROACH})")
    check(N_REVERSE  > 0, f"N_REVERSE must be > 0 (got {N_REVERSE})")

    # Speeds: non-negative and within a sane ceiling
    for name, val in [
        ("MANUAL_TARGET_HIGH_SPEED", MANUAL_TARGET_HIGH_SPEED),
        ("MANUAL_TARGET_SLOW_SPEED", MANUAL_TARGET_SLOW_SPEED),
        ("AUTO_TARGET_HIGH_SPEED",   AUTO_TARGET_HIGH_SPEED),
        ("AUTO_TARGET_MAX_SPEED",    AUTO_TARGET_MAX_SPEED),
        ("AUTO_TARGET_SLOW_SPEED",   AUTO_TARGET_SLOW_SPEED),
        ("AUTO_TARGET_APPROACH_SPEED",   AUTO_TARGET_APPROACH_SPEED),
        ("AUTO_TARGET_REVERSE_SPEED",    AUTO_TARGET_REVERSE_SPEED),
    ]:
        check(0 <= val <= _MAX_SPEED_MPS,
              f"{name} must be in [0, {_MAX_SPEED_MPS}] m/s (got {val})")
    check(ACCEL_RATE > 0, f"ACCEL_RATE must be > 0 (got {ACCEL_RATE})")
    check(REVERSE_TIMEOUT_S > 0, f"REVERSE_TIMEOUT_S must be > 0 (got {REVERSE_TIMEOUT_S})")

    # IO mapping sanity
    check(NUM_DI >= 0, f"NUM_DI must be >= 0 (got {NUM_DI})")
    check(NUM_DO >= 0, f"NUM_DO must be >= 0 (got {NUM_DO})")
    check(NUM_AO >= 0, f"NUM_AO must be >= 0 (got {NUM_AO})")
    check(ERROR_SIGN_FORWARD in (-1.0, 1.0),
          f"pid_tuning.ERROR_SIGN_FORWARD must be -1 or 1 (got {ERROR_SIGN_FORWARD})")
    if LIDAR_ENABLED:
        for name, idx in [("lidar DI_OUTER", LIDAR_DI_OUTER), ("lidar DI_MIDDLE", LIDAR_DI_MIDDLE),
                          ("lidar DI_INNER", LIDAR_DI_INNER)]:
            check(-1 <= idx < NUM_DI, f"{name} must be -1 or in [0, {NUM_DI}) (got {idx})")
        check(LIDAR_OUTER_SPEED > 0 and LIDAR_MIDDLE_SPEED > 0,
              "lidar OUTER_SPEED / MIDDLE_SPEED must be > 0")
        check(LIDAR_DECEL_RATE > 0, f"lidar DECEL_RATE must be > 0 (got {LIDAR_DECEL_RATE})")
        check(LIDAR_RESUME_DELAY_S >= 0, "lidar RESUME_DELAY_S must be >= 0")
    check(len(LIDAR_DO_AREA) == 2 and len(LIDAR_DEFAULT_AREA) == 2,
          "lidar DO_AREA and DEFAULT_AREA must each have 2 entries")
    for idx in LIDAR_DO_AREA:
        check(-1 <= idx < NUM_DO, f"lidar DO_AREA channel must be -1 or in [0, {NUM_DO}) (got {idx})")
    if DI_REVERSE_MARKER >= 0:
        check(DI_REVERSE_MARKER < NUM_DI, f"DI_REVERSE_MARKER must be in [0, {NUM_DI}) (got {DI_REVERSE_MARKER})")
    if DI_CALL_START >= 0:
        check(DI_CALL_START < NUM_DI, f"DI_CALL_START must be in [0, {NUM_DI}) (got {DI_CALL_START})")
    if DO_AT_HOME >= 0:
        check(DO_AT_HOME < NUM_DO, f"homepost DO_AT_HOME must be in [0, {NUM_DO}) (got {DO_AT_HOME})")

    # Network ports
    for name, val in [("MODBUS_PORT", MODBUS_PORT), ("RFID_PORT", RFID_PORT)]:
        check(1 <= val <= 65535, f"{name} must be in 1..65535 (got {val})")

    # Modbus device/slave ID (standard 1..247 range)
    check(1 <= DEVICE_ID <= 247, f"DEVICE_ID must be in 1..247 (got {DEVICE_ID})")

    # Sensor limits
    check(SENSOR_MAX_MM      > 0, f"SENSOR_MAX_MM must be > 0 (got {SENSOR_MAX_MM})")
    check(SENSOR_MAX_STEP_MM > 0, f"SENSOR_MAX_STEP_MM must be > 0 (got {SENSOR_MAX_STEP_MM})")
    if REAR_SENSOR_ENABLED:
        check(REAR_SENSOR_MAX_MM      > 0, f"REAR_SENSOR_MAX_MM must be > 0 (got {REAR_SENSOR_MAX_MM})")
        check(REAR_SENSOR_MAX_STEP_MM > 0, f"REAR_SENSOR_MAX_STEP_MM must be > 0 (got {REAR_SENSOR_MAX_STEP_MM})")
        check(REAR_CAN_NODE_ID is not None, "REAR_CAN_NODE_ID must be set when REAR_SENSOR_COB_ID is defined")

    # Feedforward geometry (only matters when enabled)
    if FF_ENABLED:
        check(TRACK_WIDTH  > 0, f"TRACK_WIDTH_M must be > 0 (got {TRACK_WIDTH})")
        check(CURVE_RADIUS > 0, f"CURVE_RADIUS_M must be > 0 (got {CURVE_RADIUS})")
        check(0.0 < FF_ALPHA <= 1.0, f"FF_ALPHA must be in (0,1] (got {FF_ALPHA})")
        check(0.0 < FF_EXIT_ALPHA <= 1.0, f"FF_EXIT_ALPHA must be in (0,1] (got {FF_EXIT_ALPHA})")

    # Encoder / calibration (only matters when the encoder is enabled)
    if ENCODER_ENABLED:
        check(ENCODER_COUNTS_PER_REV > 0,
              f"encoder COUNTS_PER_REV must be > 0 (got {ENCODER_COUNTS_PER_REV})")
        check(ENCODER_TOTAL_RANGE   > 0,
              f"encoder TOTAL_RANGE must be > 0 (got {ENCODER_TOTAL_RANGE})")
        check(ENCODER_RADIUS_M      > 0,
              f"encoder WHEEL_DIAMETER_M must be > 0 (got {ENCODER_WHEEL_DIAMETER_M})")
        check(CAL_V_STEP > 0, f"calibration V_STEP must be > 0 (got {CAL_V_STEP})")
        check(0 < CAL_V_MAX <= _MAX_SPEED_MPS,
              f"calibration V_MAX must be in (0, {_MAX_SPEED_MPS}] (got {CAL_V_MAX})")
        check(CAL_DWELL_S > 0, f"calibration DWELL_S must be > 0 (got {CAL_DWELL_S})")

    # Pusher (only if this profile actually configures one — [-1,-1] means
    # "no pusher hardware", which pusher.py treats as a deliberate no-op)
    if any(ch >= 0 for ch in PUSHER_DO_UP) or any(ch >= 0 for ch in PUSHER_DO_DOWN):
        check(len(PUSHER_DO_UP) == 2 and len(PUSHER_DO_DOWN) == 2,
              "pusher DO_UP/DO_DOWN must each be a 2-element list")
        for name, idx in [("pusher DO_UP[0]", PUSHER_DO_UP[0]), ("pusher DO_UP[1]", PUSHER_DO_UP[1]),
                           ("pusher DO_DOWN[0]", PUSHER_DO_DOWN[0]), ("pusher DO_DOWN[1]", PUSHER_DO_DOWN[1])]:
            check(0 <= idx < NUM_DO, f"{name} must be in [0, {NUM_DO}) (got {idx})")
        for name, idx in [("DI_TROLLEY_LIMIT[0]", DI_TROLLEY_LIMIT[0]), ("DI_TROLLEY_LIMIT[1]", DI_TROLLEY_LIMIT[1]),
                           ("DI_PUSHER_PROX_TOP", DI_PUSHER_PROX_TOP), ("DI_PUSHER_PROX_BOTTOM", DI_PUSHER_PROX_BOTTOM)]:
            check(0 <= idx < NUM_DI, f"{name} must be in [0, {NUM_DI}) (got {idx})")
        check(PUSHER_HOOK_DELAY_S >= 0, f"pusher HOOK_DELAY_S must be >= 0 (got {PUSHER_HOOK_DELAY_S})")
        check(PUSHER_MOVE_TIMEOUT_S > 0, f"pusher MOVE_TIMEOUT_S must be > 0 (got {PUSHER_MOVE_TIMEOUT_S})")

    if errors:
        raise ConfigError(
            f"Invalid profile '{_param_path}':\n  - " + "\n  - ".join(errors))


_validate()


# ══════════════════════════════════════════════════════════════════════════════
#  LIVE-TUNABLE PARAMETERS  (params page → /api/params/tuning)
# ══════════════════════════════════════════════════════════════════════════════
# RAM only: the endpoint mutates these module globals in place. Nothing is ever
# written back to the profile JSON, so a restart restores the profile values.
#
# The endpoint is gated to idle ARMED / idle MANUAL. That matters: auto_mode is
# built fresh on every START (modes.py), so it re-reads all of these — including
# the PIDController constructor args — with no extra plumbing. manual_mode is
# the exception, and re-reads MANUAL_TARGET_* every cycle for the same reason.
#
# _validate() cannot be reused here (no arguments, validates the whole module,
# and it does not bound KP/TD/V_RED_COEF at all), so this table carries its own
# bounds. They are deliberately wider than any profile value — a guard against
# typos and runaway input, not a tuning opinion.
TUNABLE_SPECS = {   # key: (min, max)
    # Auto speeds and ramp rates
    "AUTO_TARGET_HIGH_SPEED":         (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_MAX_SPEED":          (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_SLOW_SPEED":         (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_APPROACH_SPEED":     (0.0,   _MAX_SPEED_MPS),
    "AUTO_TARGET_REVERSE_SPEED":      (0.0,   _MAX_SPEED_MPS),
    "ACCEL_RATE":                     (0.01,  10.0),
    "DECEL_RATE":                     (0.01,  10.0),
    "APPROACH_DECEL_RATE":            (0.01,  10.0),
    # Manual speeds
    "MANUAL_TARGET_HIGH_SPEED":       (0.0,   _MAX_SPEED_MPS),
    "MANUAL_TARGET_SLOW_SPEED":       (0.0,   _MAX_SPEED_MPS),
    "MANUAL_ACCEL_RATE":              (0.01,  10.0),
    "MANUAL_DECEL_RATE":              (0.01,  10.0),
    # PID gains — HIGH.  N >= 1: the derivative filter coefficient
    # alpha = dt / ((TD / N) + dt) degenerates as N -> 0.
    "KP":                             (0.0,   50.0),
    "TD":                             (0.0,   5.0),
    "N":                              (1.0,   100.0),
    "V_RED_COEF":                     (0.0,   50.0),
    # PID gains — SLOW
    "KP_SLOW":                        (0.0,   50.0),
    "TD_SLOW":                        (0.0,   5.0),
    "N_SLOW":                         (1.0,   100.0),
    "V_RED_COEF_SLOW":                (0.0,   50.0),
    # PID gains — APPROACH
    "KP_APPROACH":                    (0.0,   50.0),
    "TD_APPROACH":                    (0.0,   5.0),
    "N_APPROACH":                     (1.0,   100.0),
    "V_RED_COEF_APPROACH":            (0.0,   50.0),
    # PID gains — REVERSE (rear sensor, backing maneuver)
    "KP_REVERSE":                     (0.0,   50.0),
    "TD_REVERSE":                     (0.0,   5.0),
    "N_REVERSE":                      (1.0,   100.0),
    "V_RED_COEF_REVERSE":             (0.0,   50.0),
    # Output clamps
    "OUTPUT_CLAMP_RPM":               (1.0,   5000.0),
    "OUTPUT_CLAMP_INTO_BAR_APPROACH": (1.0,   5000.0),
    "OUTPUT_CLAMP_AWAY_APPROACH":     (1.0,   5000.0),
    # Curvature feedforward
    "FF_SCALE":                       (0.0,   5.0),
    "FF_ALPHA":                       (0.001, 1.0),
    "FF_EXIT_ALPHA":                  (0.001, 1.0),
}

# Snapshot for "reset to profile". Captured after _validate(), so it reflects the
# profile exactly, including every .get() fallback default.
TUNABLE_DEFAULTS = {k: globals()[k] for k in TUNABLE_SPECS}
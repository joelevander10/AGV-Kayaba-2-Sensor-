import asyncio


# ── Typed state domains ───────────────────────────────────────────────────────

class SystemState:
    """Owned by mode_manager and safety_watchdog."""
    def __init__(self):
        self.current_mode     = None   # None | "manual" | "armed" | "running" | "calibrate" | "emergency"
        self.emergency_active = False
        self.system_error     = False  # set by safety_watchdog on driver comms loss
        self.system_error_detail = None  # name of the stale driver, or None
        # DO_HORN_ALARM mirrors this. Nothing sets it yet — the actual alarm
        # condition(s) are still TBD; wire them up (a sequence action, a new
        # watchdog check, etc.) and the horn output follows automatically.
        self.alarm_active     = False


class KinematicState:
    """Owned by sequence_engine and mode_manager."""
    def __init__(self):
        self.speed_mode           = "SLOW"  # "HIGH" | "SLOW"
        self.sequence_stop        = False   # True = AGV should brake and wait
        self.pending_sequence     = None    # name of armed rfid_then_marker sequence
        self.web_manual_command   = None    # set by Flask remote; "forward"|"reverse"|"left"|"right"|"fwd_left"|"fwd_right"|"rvs_left"|"rvs_right"|None
        self.web_manual_expire    = 0.0     # epoch after which web_manual_command is treated as None
        self.nav_in_corner        = False   # True only while a NAV sequence holds SLOW; gates feedforward
        # Default is "right" (not "left") because the FRONT MGS1600 is mounted
        # such that its left_mm/right_mm reporting is physically mirrored —
        # the profile's fork-selection tags (e.g. AA/AE/BB) already account for
        # this by swapping their "side", so the idle/reset default must match.
        # (The rear sensor used by modes.reverse_mode has no equivalent of this
        # — REVERSE always follows its right_mm reading, unconditionally.)
        self.track_follow         = "right"  # "left" | "right" — which MGS1600 track the PID follows (fork selection)
        self.calibration_request  = False   # set by Flask to start the wheel-speed calibration ramp
        self.calibration_wheel    = "right" # "right" | "left" — which wheel the encoder is mounted on
        # Destination selector for RFID branch forks (e.g. "1"/"2"/"3" for
        # GOAL/Titik 1/2/3). Set from the HMI before START; gates any sequence
        # whose trigger carries a matching "requires_branch" (see
        # SequenceEngine._check_preconditions). Cleared only by an explicit
        # "clear_branch" sequence action (e.g. the merge-back RFID tag) — NOT
        # reset automatically on mode transitions, so a mid-route stop/resume
        # keeps the selection. Deliberately not typed/validated here; the
        # profile JSON's "requires_branch" strings are the source of truth.
        self.active_branch        = None
        # Reverse-drive request. Set True by SequenceEngine's "start_reverse"
        # action (tag "1") to command mode_manager into REVERSE mode (PID
        # tape-follow with error_sign flipped — see modes.reverse_mode).
        # Cleared by "stop_reverse" (fired by tag "C1") or by mode_manager on
        # emergency/manual-switch/RESET (via _reset_sequence_state).
        self.reverse_request      = False
        # Gates the tag-"1" reverse trigger so a single dead-end approach
        # doesn't re-fire once the AGV has already stopped at C1. Set True by
        # C1's sequence ("set_ignore_1"), cleared only by the "CR" tag
        # ("clear_ignore_1") — NOT reset on mode transitions, same persistence
        # model as active_branch.
        self.ignore_rfid_1        = False
        # Same for tag "2" (branch 2's reverse tag): set at the C1 / marker
        # stop ("set_ignore_2"), cleared at CE ("clear_ignore_2").
        self.ignore_rfid_2        = False
        # Homepost request. Set True by SequenceEngine's "go_home" action
        # (RFID tag "0") to command mode_manager to fully exit AUTO — cancels
        # the running task, idles the motors, and drops back to ARMED (same
        # destination as a manual RESET, just tag-triggered). Cleared by
        # mode_manager the instant it acts on it (also cleared by
        # _reset_sequence_state on emergency/manual-switch/RESET), so it's a
        # one-shot pulse, not a persistent gate like active_branch/ignore_rfid_1.
        self.homepost_request     = False
        # True while the AGV is parked at homepost. Set by the "go_home"
        # sequence action (arrived at RFID homepost tag), by reading the
        # homepost tag while not driving, or at boot if
        # config.ASSUME_AT_HOME_ON_BOOT. Cleared when AUTO launches from ARMED,
        # when any other tag is read, or on entering MANUAL. Drives
        # config.DO_AT_HOME and gates DI_CALL_START.
        self.at_homepost          = False
        # True for a run launched by DI_CALL_START. Gates the "stop_at_b0"
        # sequence ("requires_flag": "call_run"). Cleared by
        # _reset_sequence_state (any drop to ARMED/MANUAL/EMERGENCY), so the
        # run that continues after the B0 stop is a normal run.
        self.call_run             = False
        # True from the moment AUTO launches while at homepost until the first
        # OTHER tag is read. Gates arrive_homepost ("skip_if_flag") so the
        # homepost tag still being in reader range as the AGV pulls away
        # doesn't instantly stop it again.
        self.leaving_home         = False
        # Web button bypass — when True, web_btn_* override the physical DI signals
        # for START/RESET/MODE_SWITCH/EMERGENCY in mode_manager.
        self.web_button_mode      = False
        self.web_btn_start        = False   # momentary; mode_manager clears after detecting rising edge
        self.web_btn_reset        = False   # momentary; mode_manager clears after detecting rising edge
        self.web_btn_manual       = False   # level signal: True = MANUAL, False = AUTO
        self.web_btn_emergency    = False   # level signal: True = emergency active
        # Pusher pin lower request. Set True by the web dashboard's "full
        # travel" DOWN button (POST /api/pusher/move) to command pusher.py to
        # retract the pin; pusher.py clears it once DI_PUSHER_PROX_BOTTOM
        # confirms the pin is fully down (or aborts+clears it on a move
        # timeout, or on pusher_estop).
        self.pusher_down_request = False
        # Pusher pin raise request — same "full travel" pattern as
        # pusher_down_request above, but UP. Manual-only (the autonomous hook
        # sequence in pusher.py drives its own moving_up phase directly, not
        # through this flag). Cleared by pusher.py once DI_PUSHER_PROX_TOP
        # confirms the pin is fully up (or on timeout/estop).
        self.pusher_up_request  = False
        # Hold-to-run pusher jog — "up"|"down"|None. Set by the web dashboard's
        # press-and-hold buttons (POST /api/pusher/jog); the browser must keep
        # re-sending while held (same watchdog pattern as web_manual_command),
        # otherwise pusher_jog_expire lapses and pusher.py auto-stops. Overrides
        # the autonomous hook sequence and any full-travel request while active.
        self.pusher_jog_cmd     = None
        self.pusher_jog_expire  = 0.0   # epoch after which pusher_jog_cmd is treated as None
        # Pusher-specific emergency stop (separate from the AGV-wide
        # state.emergency_active) — set by the web dashboard's pusher E-STOP
        # button. While True, pusher.py de-energizes immediately and clears
        # any pending up/down/jog request so nothing resumes unexpectedly when
        # it's cleared; the operator must re-issue a fresh command afterward.
        self.pusher_estop        = False


class PerceptionState:
    """Written by hardware drivers; read by auto_mode and sequence_engine."""
    def __init__(self):
        self.latest_di     = None   # latest DI bits from Modbus
        self.latest_do     = {}     # {channel_no: bool} commanded DO state, updated by DOWriter
        self.latest_sensor = None   # last CAN frame dict from MGS1600 (front)
        self.can_last_rx   = 0.0    # epoch of last front-sensor CAN message received
        # Rear MGS1600 (dedicated CAN node) — used only by modes.reverse_mode.
        # None / 0.0 when config.REAR_SENSOR_ENABLED is False (no rear sensor
        # fitted on this profile).
        self.latest_rear_sensor = None   # last CAN frame dict from the rear MGS1600
        self.rear_can_last_rx   = 0.0    # epoch of last rear-sensor CAN message received
        # Motion telemetry — written by auto_mode each PID cycle
        self.left_rpm      = 0.0
        self.right_rpm     = 0.0
        self.pid_output    = 0.0
        self.target_speed  = 0.0    # current ramp target in m/s
        # Encoder telemetry — written by EncoderReader during calibration only
        self.encoder_count     = 0
        self.encoder_rpm       = 0.0   # encoder shaft rpm (signed)
        self.encoder_v         = 0.0   # AGV ground speed from encoder rim (m/s)
        self.encoder_last_rx   = 0.0   # epoch of last encoder frame (0.0 = never)
        self.encoder_connected = False
        # Trolley-hitch pusher (pusher.py) — see that file's docstring for the
        # full state machine. trolley_hooked is the interlock
        # core.sequence_engine's "wait_hooked" action polls before letting
        # stop_at_c1 resume driving.
        self.trolley_hooked = False
        self.pusher_status  = "idle"  # "idle"|"hook_wait"|"moving_up"|"hooked"|"moving_down"|"jog_up"|"jog_down"|"estopped"


# ── Shared state object ───────────────────────────────────────────────────────

class AMRState:
    """
    Holds all shared queues and state domains for the AMR.
    Must be instantiated inside an active asyncio event loop.

    All fields are also accessible as flat attributes via property shims
    so existing code continues to work without changes.
    """

    def __init__(self):
        # ── Queues for cross-task communication ───────────────────────────────
        self.di_queue     = asyncio.Queue()  # DI readings
        self.rfid_queue    = asyncio.Queue()  # RFID tag reads
        self.latest_rfid_tag: str | None = None  # last tag seen (hex string)
        # One-shot pulses from the MANUAL page's AUTO REVERSE panel
        # (POST /api/reverse). mode_manager consumes them: start only acts in
        # ARMED, stop only in REVERSING; stale pulses are dropped elsewhere.
        self.web_reverse_start = False
        self.web_reverse_stop  = False
        # Keyboard / keypad control (keyboard_control.py):
        #   kb_mode_override  None = follow the MANUAL/AUTO selector DI;
        #                     True/False = Q key forced MANUAL/AUTO (cleared
        #                     as soon as the physical selector is moved)
        #   kb_branch_request one-shot branch "1".."4" from keys 7/8/9/0
        #   kb_estop          E key emergency stop, latched until RESET
        #   keypad_devices    names of USB keypads currently read on the AGV PC
        self.kb_mode_override  = None
        self.kb_branch_request = None
        self.kb_estop          = False
        self.kb_reset_request  = False   # ENTER key: one RESET pulse
        # PID auto-tune (autotune.py), started from the PARAMS page.
        self.autotune_request  = False
        self.autotune_params   = {}
        self.autotune_status   = {"phase": "idle", "message": "", "cycles": 0,
                                  "elapsed_s": 0.0, "params": {}, "result": None}
        self.keypad_devices    = []
        # REVERSE speed override (m/s). Set by the "set_reverse_speed" action
        # (RFID CA: approach the stop marker at 0.1 m/s). None = use
        # config.AUTO_TARGET_REVERSE_SPEED. Cleared when a reverse starts and
        # by _reset_sequence_state.
        self.reverse_speed_override = None
        # REVERSE track selection ("left" | "right"), set by the
        # "set_reverse_track" action (RFID AE / BB while REVERSING). Reset to
        # "right" at the start of every reverse. Separate from track_follow so
        # the forward fork choice is untouched.
        self.reverse_track = "right"
        # DI10 stop marker is only obeyed after RFID BA was read in this
        # reverse ("arm_reverse_marker" action). Reset at every reverse start.
        self.reverse_marker_armed = False
        # 180° spin-turn (rotate.py): "left" | "right" request from the T/Y keys
        # or the manual page, consumed by mode_manager; status for the HMI.
        self.rotate_request = None
        # PID gain set override: "BRANCH" = SLOW branch gains (set by RFID
        # AA / AB / AC via set_speed "gains"); None = normal gains for the speed.
        self.gain_set = None
        self.rotate_status  = {"phase": "idle", "message": "", "angle": 0}
        # Lidar: zone currently seen ("inner"|"middle"|"outer"|None) and the
        # DO pattern selecting the lidar field set ([DO_AREA[0], DO_AREA[1]]).
        self.lidar_zone = None
        self.lidar_area = [False, False]
        # IO TEST MODE (web IO page). While io_test_mode is True, do_override
        # {channel: bool} wins over whatever the control code asked for in
        # do_setpoints — applied in the DO writer, so pusher/horn/motion keep
        # running normally underneath and take back over the instant a channel
        # is released. mode_manager force-exits test mode (clears everything)
        # whenever the AGV enters RUNNING/REVERSING/CALIBRATE.
        self.io_test_mode = False
        self.do_override  = {}
        self.sensor_queue = asyncio.Queue()  # CAN magnetic sensor readings (front)
        self.rear_sensor_queue = asyncio.Queue()  # CAN magnetic sensor readings (rear, REVERSE only)

        # ── Output setpoint tables (latest-wins, non-blocking) ────────────────
        # Producers (motion, auto_mode, watchdog) call set_ao/set_do; the AO/DO
        # writer tasks wake on the dirty Event and assert the *latest* values.
        # This bounds memory and prevents replay of stale commands after a
        # comms reconnect (a FIFO queue would replay the backlog).
        self.ao_setpoints = {}            # {channel_no: voltage}
        self.do_setpoints = {}            # {channel_no: bool}
        self.ao_dirty     = asyncio.Event()
        self.do_dirty     = asyncio.Event()

        # ── Wheel-speed calibration live status (read by Flask dashboard) ─────
        self.calibration_status = {
            "active":   False,
            "v_cmd":    0.0,
            "voltage":  0.0,
            "elapsed":  0.0,
            "wheel":    "right",
            "csv_path": None,
        }

        # ── Typed domains ──────────────────────────────────────────────────────
        self.system     = SystemState()
        self.kinematic  = KinematicState()
        self.perception = PerceptionState()

    # ── Output setpoint setters (synchronous: a group of calls with no await
    #    between them is applied atomically before a writer can wake) ──────────

    def set_ao(self, channel_no, voltage):
        self.ao_setpoints[channel_no] = voltage
        self.ao_dirty.set()

    def clear_do_overrides(self):
        """Drop every IO-test override and re-assert the program's own values.
        A channel the program never drove falls back to OFF."""
        for ch in list(self.do_override):
            self.do_setpoints.setdefault(ch, False)
        self.do_override.clear()
        self.do_dirty.set()

    def set_do(self, channel_no, active_state):
        self.do_setpoints[channel_no] = bool(active_state)
        self.do_dirty.set()

    # ── SystemState shims ─────────────────────────────────────────────────────

    @property
    def current_mode(self): return self.system.current_mode
    @current_mode.setter
    def current_mode(self, v): self.system.current_mode = v

    @property
    def emergency_active(self): return self.system.emergency_active
    @emergency_active.setter
    def emergency_active(self, v): self.system.emergency_active = v

    @property
    def system_error(self): return self.system.system_error
    @system_error.setter
    def system_error(self, v): self.system.system_error = v

    @property
    def system_error_detail(self): return self.system.system_error_detail
    @system_error_detail.setter
    def system_error_detail(self, v): self.system.system_error_detail = v

    @property
    def alarm_active(self): return self.system.alarm_active
    @alarm_active.setter
    def alarm_active(self, v): self.system.alarm_active = v

    # ── KinematicState shims ──────────────────────────────────────────────────

    @property
    def speed_mode(self): return self.kinematic.speed_mode
    @speed_mode.setter
    def speed_mode(self, v): self.kinematic.speed_mode = v

    @property
    def sequence_stop(self): return self.kinematic.sequence_stop
    @sequence_stop.setter
    def sequence_stop(self, v): self.kinematic.sequence_stop = v

    @property
    def pending_sequence(self): return self.kinematic.pending_sequence
    @pending_sequence.setter
    def pending_sequence(self, v): self.kinematic.pending_sequence = v

    @property
    def nav_in_corner(self): return self.kinematic.nav_in_corner
    @nav_in_corner.setter
    def nav_in_corner(self, v): self.kinematic.nav_in_corner = v

    @property
    def track_follow(self): return self.kinematic.track_follow
    @track_follow.setter
    def track_follow(self, v): self.kinematic.track_follow = v

    @property
    def calibration_request(self): return self.kinematic.calibration_request
    @calibration_request.setter
    def calibration_request(self, v): self.kinematic.calibration_request = v

    @property
    def calibration_wheel(self): return self.kinematic.calibration_wheel
    @calibration_wheel.setter
    def calibration_wheel(self, v): self.kinematic.calibration_wheel = v

    @property
    def active_branch(self): return self.kinematic.active_branch
    @active_branch.setter
    def active_branch(self, v): self.kinematic.active_branch = v

    @property
    def reverse_request(self): return self.kinematic.reverse_request
    @reverse_request.setter
    def reverse_request(self, v): self.kinematic.reverse_request = v

    @property
    def ignore_rfid_1(self): return self.kinematic.ignore_rfid_1
    @ignore_rfid_1.setter
    def ignore_rfid_1(self, v): self.kinematic.ignore_rfid_1 = v

    @property
    def ignore_rfid_2(self): return self.kinematic.ignore_rfid_2
    @ignore_rfid_2.setter
    def ignore_rfid_2(self, v): self.kinematic.ignore_rfid_2 = v

    @property
    def homepost_request(self): return self.kinematic.homepost_request
    @homepost_request.setter
    def homepost_request(self, v): self.kinematic.homepost_request = v

    @property
    def at_homepost(self): return self.kinematic.at_homepost
    @at_homepost.setter
    def at_homepost(self, v): self.kinematic.at_homepost = v

    @property
    def leaving_home(self): return self.kinematic.leaving_home
    @leaving_home.setter
    def leaving_home(self, v): self.kinematic.leaving_home = v

    @property
    def call_run(self): return self.kinematic.call_run
    @call_run.setter
    def call_run(self, v): self.kinematic.call_run = v

    @property
    def web_manual_command(self): return self.kinematic.web_manual_command
    @web_manual_command.setter
    def web_manual_command(self, v): self.kinematic.web_manual_command = v

    @property
    def web_manual_expire(self): return self.kinematic.web_manual_expire
    @web_manual_expire.setter
    def web_manual_expire(self, v): self.kinematic.web_manual_expire = v

    @property
    def web_button_mode(self): return self.kinematic.web_button_mode
    @web_button_mode.setter
    def web_button_mode(self, v): self.kinematic.web_button_mode = v

    @property
    def web_btn_start(self): return self.kinematic.web_btn_start
    @web_btn_start.setter
    def web_btn_start(self, v): self.kinematic.web_btn_start = v

    @property
    def web_btn_reset(self): return self.kinematic.web_btn_reset
    @web_btn_reset.setter
    def web_btn_reset(self, v): self.kinematic.web_btn_reset = v

    @property
    def web_btn_manual(self): return self.kinematic.web_btn_manual
    @web_btn_manual.setter
    def web_btn_manual(self, v): self.kinematic.web_btn_manual = v

    @property
    def web_btn_emergency(self): return self.kinematic.web_btn_emergency
    @web_btn_emergency.setter
    def web_btn_emergency(self, v): self.kinematic.web_btn_emergency = v

    @property
    def pusher_down_request(self): return self.kinematic.pusher_down_request
    @pusher_down_request.setter
    def pusher_down_request(self, v): self.kinematic.pusher_down_request = v

    @property
    def pusher_up_request(self): return self.kinematic.pusher_up_request
    @pusher_up_request.setter
    def pusher_up_request(self, v): self.kinematic.pusher_up_request = v

    @property
    def pusher_jog_cmd(self): return self.kinematic.pusher_jog_cmd
    @pusher_jog_cmd.setter
    def pusher_jog_cmd(self, v): self.kinematic.pusher_jog_cmd = v

    @property
    def pusher_jog_expire(self): return self.kinematic.pusher_jog_expire
    @pusher_jog_expire.setter
    def pusher_jog_expire(self, v): self.kinematic.pusher_jog_expire = v

    @property
    def pusher_estop(self): return self.kinematic.pusher_estop
    @pusher_estop.setter
    def pusher_estop(self, v): self.kinematic.pusher_estop = v

    # ── PerceptionState shims ─────────────────────────────────────────────────

    @property
    def latest_di(self): return self.perception.latest_di
    @latest_di.setter
    def latest_di(self, v): self.perception.latest_di = v

    @property
    def latest_do(self): return self.perception.latest_do
    @latest_do.setter
    def latest_do(self, v): self.perception.latest_do = v

    @property
    def latest_sensor(self): return self.perception.latest_sensor
    @latest_sensor.setter
    def latest_sensor(self, v): self.perception.latest_sensor = v

    @property
    def can_last_rx(self): return self.perception.can_last_rx
    @can_last_rx.setter
    def can_last_rx(self, v): self.perception.can_last_rx = v

    @property
    def latest_rear_sensor(self): return self.perception.latest_rear_sensor
    @latest_rear_sensor.setter
    def latest_rear_sensor(self, v): self.perception.latest_rear_sensor = v

    @property
    def rear_can_last_rx(self): return self.perception.rear_can_last_rx
    @rear_can_last_rx.setter
    def rear_can_last_rx(self, v): self.perception.rear_can_last_rx = v

    @property
    def left_rpm(self): return self.perception.left_rpm
    @left_rpm.setter
    def left_rpm(self, v): self.perception.left_rpm = v

    @property
    def right_rpm(self): return self.perception.right_rpm
    @right_rpm.setter
    def right_rpm(self, v): self.perception.right_rpm = v

    @property
    def pid_output(self): return self.perception.pid_output
    @pid_output.setter
    def pid_output(self, v): self.perception.pid_output = v

    @property
    def target_speed(self): return self.perception.target_speed
    @target_speed.setter
    def target_speed(self, v): self.perception.target_speed = v

    @property
    def encoder_count(self): return self.perception.encoder_count
    @encoder_count.setter
    def encoder_count(self, v): self.perception.encoder_count = v

    @property
    def encoder_rpm(self): return self.perception.encoder_rpm
    @encoder_rpm.setter
    def encoder_rpm(self, v): self.perception.encoder_rpm = v

    @property
    def encoder_v(self): return self.perception.encoder_v
    @encoder_v.setter
    def encoder_v(self, v): self.perception.encoder_v = v

    @property
    def encoder_last_rx(self): return self.perception.encoder_last_rx
    @encoder_last_rx.setter
    def encoder_last_rx(self, v): self.perception.encoder_last_rx = v

    @property
    def encoder_connected(self): return self.perception.encoder_connected
    @encoder_connected.setter
    def encoder_connected(self, v): self.perception.encoder_connected = v

    @property
    def trolley_hooked(self): return self.perception.trolley_hooked
    @trolley_hooked.setter
    def trolley_hooked(self, v): self.perception.trolley_hooked = v

    @property
    def pusher_status(self): return self.perception.pusher_status
    @pusher_status.setter
    def pusher_status(self, v): self.perception.pusher_status = v

    # ── PLCState shims ────────────────────────────────────────────────────────
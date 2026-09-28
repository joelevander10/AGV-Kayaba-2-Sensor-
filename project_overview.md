# AGV KIM2A Controller — System Overview

> Orientation doc for coding agents (and humans). Describes architecture and the
> implementation patterns you must respect when editing. It is **not** a line-by-line
> code reference — read the cited files for exact logic. Verified against the codebase
> on 2026-06-05.

---

## 1. What this is

Controller for a **differential-drive, magnetic-tape-following AGV**. It follows a
**stadium-shaped loop, counter-clockwise** (every corner is a left turn). It drives two
motors via analog-voltage speed commands + digital direction/brake coils, follows tape
with a PID steering loop, reacts to RFID tags and tape/proximity markers to run 
**sequences** (speed zones, timed stops, PLC trolley-load handshakes), and exposes a
Flask web HMI.

- **Language/runtime:** Python 3 (asyncio), single event loop + one Flask daemon thread.
- **Entry point:** `main.py`
- **Run:** `AGV_ID=agv1_kim python3 main.py`
- **Active profile:** `profiles/agv1_kim.json` (all tunables live here — see §9).
- **Deployment:** runs as systemd unit `agv-controller.service` (HMI can restart it).

> ⚠️ Only `agv1_kim` is in use. Ignore `agv2_kim.json`.

---

## 2. Architecture at a glance

One asyncio event loop runs all hardware + control tasks concurrently. Tasks never call
each other directly — they communicate through a single shared **`AMRState`** object
(`state.py`) passed by reference. Flask runs in a separate daemon thread and touches the
same `AMRState` (GIL-safe for the simple reads/single-attribute writes it does).

### Task graph (all spawned in `main.run()`)

| Task | Source | Role |
|---|---|---|
| `AOWriter.run` | `drivers/modbus_ao.py` | Writes analog speed setpoints (DAC) over Modbus TCP. **Always on.** |
| `DOWriter.run` | `drivers/modbus_do.py` | Writes digital direction/brake coils over Modbus TCP. Gated by `DIO_ENABLED`. |
| `DIReader.run` | `drivers/modbus_di.py` | Polls discrete inputs (buttons, selector, e-stop, prox). Gated by `DIO_ENABLED`. **Watched.** |
| `CANReader.run` | `drivers/can_mgs1600.py` | MGS1600 magnetic guide sensor over CAN/slcan. Gated by `CAN_ENABLED`. **Watched.** |
| `RFIDReader.run` | `drivers/rfid_tcp.py` | RFID tag reader over raw TCP. Gated by `NAV_ENABLED`. **Watched.** |
| `SLMPDriver.run` | `drivers/slmp_plc.py` | Mitsubishi FX5U PLC link (SLMP/MC). Gated by `SLMP_ENABLED`. (REMOVED) |
| `safety_watchdog` | `safety_watchdog.py` | Monitors the "watched" sensor drivers; faults on stale data. |
| `rfid_processor` | `rfid_processor.py` | Thin dispatcher: RFID queue → `engine.on_rfid_tag()`. |
| `mode_manager` | `modes.py` | Central FSM. Spawns `manual_mode` / `auto_mode` / `calibrate_mode` as subtasks. |

`EncoderReader.run` (`drivers/can_encoder.py`) is spawned only transiently by
`calibrate_mode`, never as a core task.

### Key architectural rules (do not violate)

1. **Never block the event loop.** All synchronous hardware I/O (pymodbus sync, pymcprotocol, python-can) goes through `loop.run_in_executor(...)`.
2. **All motion goes through `motion.py`**, which writes to `AMRState` setpoint tables — never write DO/AO directly except in `main.shutdown()`.
3. **Output is latest-wins, not a queue** (see §4). Producers call `state.set_ao()/set_do()`.
4. **Queues are drained keep-latest** each cycle (`while not q.empty()`) so stale frames never accumulate.
5. **`mode_manager` owns all mode transitions.** `auto_mode`/`manual_mode` are passive subtasks that get cancelled on transition. They do **not** change mode.
6. **`mode_manager` and `safety_watchdog` are the only tasks that halt motion unilaterally.**
7. **All sequence behavior is data**, defined in profile JSON — adding/removing sequences needs zero Python (see §7).

---

## 3. Shared state — `state.py` / `AMRState`

`AMRState` groups fields into typed sub-domains but exposes **flat property shims** so
code can write `state.speed_mode` directly. When adding a field, add it to the right
sub-domain **and** add a property shim pair.

| Domain | Owner | Notable fields |
|---|---|---|
| `SystemState` | mode_manager, watchdog | `current_mode` (`None`/`manual`/`armed`/`running`/`calibrate`/`emergency`), `emergency_active`, `system_error` |
| `KinematicState` | sequence_engine, mode_manager | `speed_mode` (`HIGH`/`SLOW`/`APPROACH`), `sequence_stop`, `pending_sequence`, `nav_in_corner` (gates feedforward), `web_manual_command`/`web_manual_expire`, `calibration_request`/`calibration_wheel`, `web_button_mode` + `web_btn_start`/`web_btn_reset`/`web_btn_manual`/`web_btn_emergency` (HMI button bypass — see §5) |
| `PerceptionState` | hardware drivers + auto_mode | `latest_di`, `latest_do`, `latest_sensor`, `can_last_rx`, motion telemetry (`left_rpm`/`right_rpm`/`pid_output`/`target_speed`), encoder telemetry |
| `PLCState` | SLMPDriver | `plc_inputs` (M0–M4), `plc_sequence_request` (int → one-hot, `None`=clear), `plc_sequence_pulse_expire`, `plc_sequence_complete` (list[bool]) |
| `TrolleyState` | SLMPDriver | `y_bits`/`x_bits` (read), `top_m_bits`/`bot_m_bits` (read), `write_bits` (set by Flask, written each cycle) |

### Cross-task channels (in `AMRState.__init__`)

- **Input queues:** `di_queue`, `rfid_queue`, `sensor_queue` (asyncio.Queue, drained keep-latest).
- **Output setpoint tables:** `ao_setpoints` / `do_setpoints` dicts + `ao_dirty` / `do_dirty` `asyncio.Event`s. `set_ao/set_do` update the dict and set the Event; the writer task wakes, snapshots the dict, and asserts the latest value per channel. This bounds memory and **prevents stale-command replay after a reconnect** (a FIFO queue would replay the backlog). A group of `set_*` calls with no `await` between them is applied atomically.

---

## 4. Motion & kinematics — `motion.py`

- **Per-wheel motor calibration** (left/right differ ~6%): `rpm = RPM_PER_VOLT[side]*V + RPM_VOLT_OFFSET[side]`. `rpm_to_voltage()` is side-aware and clamps to `[0, 5] V`; `rpm <= 0 → 0 V` (true stop, no creep from the affine offset).
- `mps_to_rpm(v)`: `motor_rpm = (v*60 / WHEEL_CIRCUMFERENCE) * GEAR_RATIO`.
- `_drive(state, left_fwd, left_v, right_fwd, right_v)` is the single primitive; all helpers (`set_forward`, `set_reverse`, `set_left/right`, `set_forward_left/right`, `set_reverse_left/right`, `set_brake`, `idle`) call it. Channel indices come from `config.MOTOR_CHANNELS` — **never hardcode DO/AO channels.**
- `set_brake` energizes brake coils + 0 V; `idle` releases brakes + 0 V (used on emergency so the AGV can be pushed by hand).

---

## 5. The mode FSM — `modes.py: mode_manager`

States: `None → armed` at boot (gives Modbus writers time to connect), then driven by
DI signals each loop:

- `DI_MODE_SWITCH`: HIGH = MANUAL, LOW = AUTO. Live switching either way.
- `DI_EMERGENCY`: NO contact, **True = triggered**. Any→`emergency` (idle, not brake). Recovery on `DI_RESET` rising edge → manual or armed depending on selector.
- `DI_START` rising edge in `armed` + **tape present** → `running` (spawns `auto_mode`).
- `DI_RESET` rising edge in `running` → back to `armed`.
- `state.calibration_request` in `armed` → `calibrate` (spawns `calibrate_mode`); RESET or cleared request → back to `armed`.
- `state.system_error` (watchdog) → cancel active task, brake, hold until cleared.

`_reset_sequence_state()` clears speed_mode→HIGH, sequence_stop, nav_in_corner,
pending_sequence, calibration_request, and disarms the sequence engine on transitions.

> **There is no reverse driving mode.** `auto_mode` is forward-only. (Old docs mention a
> `reverse` state / `reverse_auto_request` — that has been removed.)

#### Web-button bypass

When `state.web_button_mode` is True (toggled from the `/manual` HMI), `mode_manager`
reads **start / reset / man-auto-switch** from `web_btn_start`/`web_btn_reset`/
`web_btn_manual` instead of the physical DI bits — software mirrors of the four panel
buttons. `web_btn_start`/`web_btn_reset` are momentary: `mode_manager` clears them after
detecting their rising edge so each press fires once.

> **Emergency is never bypassed.** `emergency_safe` always ANDs the physical
> `DI_EMERGENCY` with the web flag: `emergency_safe = not phys_emergency and not
> web_btn_emergency`. Either the physical e-stop **or** the web emergency button halts the
> AGV, in bypass mode or not.

### `auto_mode` (forward tape-following PID loop)

Per cycle (`modes.py:auto_mode`):
1. Guard order: emergency → sequence_stop (hard brake + hold) → resume.
2. Pick `target_speed` and PID gains from `speed_mode` (`HIGH`/`SLOW`/`APPROACH`). Gains only swap when the mode actually changes.
3. Drain `sensor_queue` keep-latest. On tape loss or `sensor_failure`, brake + reset PID; re-arm on reacquire.
4. Fire sequence-engine marker hooks: `engine.on_marker("left"/"right")` from the CAN frame; `engine.on_marker("prox")` on the **rising edge** of `DI_PROX` (only when SEQ+SLMP enabled).
5. **Accel/decel ramp** toward `target_speed`: up at `ACCEL_RATE`; down at `APPROACH_DECEL_RATE` when `speed_mode == "APPROACH"` (gentle crawl into a station), else `DECEL_RATE` (corner/normal slowdown).
6. Sensor sanity: reject `|left_mm| > SENSOR_MAX_MM`; slew-clamp jumps `> SENSOR_MAX_STEP_MM`.
7. **Curvature feedforward** (corners only — see §6) → low-pass filtered.
8. `pid.compute(pv, base_rpm, error_sign=-1.0, feedforward=...)` → per-wheel rpm → per-wheel voltage → `state.set_ao(0/1, v)`.
9. CAN-timeout guard (`time - can_last_rx > CAN_TIMEOUT` → brake).
10. `RunRecorder` (`debugging/plotter.py`) logs per-cycle telemetry for PNG/CSV plots.

### `manual_mode` (pendant + web jog)

Drains `di_queue`; maps button combos (fwd/rev × left/right, singles, idle) to motion
helpers. `state.web_manual_command` (set by the `/manual` HMI page) **takes priority**
over physical buttons and auto-expires after `web_manual_expire` (400 ms safety).

---

## 6. PID controller — `core/pid.py: PIDController`

Velocity-form steering controller. PV is the lateral tape offset `left_mm`; output is a
per-wheel rpm differential.

- **P:** `kp * e`, where `e = error_sign * pv` (`error_sign=-1.0` forward).
- **I:** with deadband — only accumulates when `|e| < TI_DEADBAND`; clamped to `±TI_MAX`; term is `kp * (1/TI) * integral`.
- **D:** filtered derivative, `alpha = dt / (td/n + dt)`, on PV (not error).
- **dt is measured** with `perf_counter` (clamped to `[0.2, 5]×DT`) — not nominal `DT` — because the loop only runs on fresh frames.
- **Output clamp** `±OUTPUT_CLAMP_RPM` applies to the PID steering term only; **feedforward is added after the clamp** (it's a known-good geometric command).
- **Speed reduction:** slows both wheels when off-track / moving fast laterally; low-pass filtered (`SR_ALPHA`), capped at `SR_CAP * base_rpm`. Scaled by `V_RED_COEF` (per speed mode).
- Wheel outputs: `left = base - speed_reduction + output_total`; `right = base - speed_reduction - output_total`.
- `update_gains(kp, td, n, v_red_coef)` swaps gains live **without** resetting integrator/filters (smooth). `reset()` zeros everything.

Three gain sets exist in config: HIGH (`KP`/`TD`/`N`/`V_RED_COEF`), SLOW (`*_SLOW`), and
APPROACH (`*_APPROACH`). Convention: **KP scales roughly linearly with target speed.**

### Curvature feedforward (`config.py` feedforward block, applied in `auto_mode`)

In a sustained turn a pure-feedback loop settles with a standing cross-track error.
Feeding the geometric differential forward removes that lag so corners can be taken
faster:

```
output_ff = FF_SCALE * FF_DIRECTION_SIGN * 0.5 * base_rpm * (TRACK_WIDTH / CURVE_RADIUS)
```

- Applied **only while `state.nav_in_corner`** is True — i.e. a NAV sequence is holding `SLOW`. A SEQ approach using SLOW does **not** get feedforward (the engine clears `nav_in_corner` for SEQ).
- `FF_DIRECTION_SIGN = -1.0` for left turns (the stadium is CCW → all left).
- Magnitude is **speed-proportional via `base_rpm`** — it auto-scales when you change corner speed, so `FF_SCALE` usually need not change with speed.
- Low-pass filtered by `FF_ALPHA` to smooth corner entry/exit.

---

## 7. Sequence engine — `core/sequence_engine.py: SequenceEngine`

Declarative runner. Sequences live in profile `"sequences"`. **Add/remove/modify a
sequence = edit JSON + restart, zero Python.** To add a new *action type*, add one
`async def _act_*` handler and register it in `__init__` (or call `register_action()`).

### Trigger types
- `rfid` — fires immediately on matching tag.
- `marker` — fires when a matching tape marker (`left`/`right`) is detected.
- `rfid_then_marker` — RFID **arms** the sequence (sets `speed_mode` to `approach_speed`, default `APPROACH`; records in `_armed`); the matching marker then runs the action list, **skipping** the `set_speed`+`wait_marker` approach steps that already happened. `marker_side: "prox"` uses `DI_PROX` instead of the CAN tape frame.

### Action types
`set_speed`, `wait_marker`, `stop_agv` (sets `sequence_stop=True` → auto_mode hard-brakes), 
`resume` (clears stop, optional speed), `sequence_stop` (timed stop then auto-resume),
`wait_seconds`, `plc_request` (one-hot `plc_sequence_request` + pulse window),
`wait_plc_complete` (Phase A: wait complete flag LOW = PLC ack; Phase B: wait HIGH = done,
bounded by `done_timeout` → raises `SequencePLCFault` = **hold stopped until operator
RESET**, never auto-resume).

### Gating & lifecycle
- **Category** `"navigation"` vs `"sequence"` (explicit, or inferred: pure-`set_speed` ⇒ navigation). NAV gated by `NAV_ENABLED`, SEQ by `SEQ_ENABLED`.
- **Preconditions:** `requires_mode` (usually `"running"`) + per-name `cooldown_s` + **one sequence at a time** (`_active_sequence` guard).
- `cancel_active()` / `cancel_armed()` are called by `mode_manager` on every transition so stale armed/running state never survives a mode change.
- A NAV sequence leaves `nav_in_corner` set (a speed zone persists); a SEQ sequence clears it on completion.

See the per-station `trolley_load_*` entries in the profile for the canonical
`rfid_then_marker` → approach → stop → PLC handshake → resume pattern.

---

## 8. Drivers — `drivers/` (+ `drivers/base.py`)

Two ABCs:
- **`SensorDriver`** — `run(state)` loops forever, calls `self._record_rx()` on each successful read; `get_health()` feeds the watchdog. (DI, CAN, RFID, Encoder.)
- **`ActuatorDriver`** — output-only `run(state)`, **not** watchdog-monitored (a stuck output shows as "not moving"). (DO, AO, SLMP.)

Each driver also exposes a module-level function shim (`di_reader`, etc.); `io_hardware.py`
re-exports them for backwards compat. **New code should import the classes from
`drivers/*.py`.**

| Driver | Transport | Notes |
|---|---|---|
| `DIReader` | Modbus TCP discrete inputs | Applies `DI_FLIPPED` at read time so all downstream code sees logical values. Poll `DI_POLL_INTERVAL` (0.01 s). |
| `DOWriter` | Modbus TCP coils | Wakes on `do_dirty`; snapshots `do_setpoints`; re-asserts on reconnect. |
| `AOWriter` | Modbus TCP registers | `dac = int(v / V_RANGE * DAC_RES)`. Wakes on `ao_dirty`. |
| `CANReader` | slcan / CANable2 (auto-discovered by USB VID/PID), 500 kbit/s | MGS1600 frame: COB-ID `SENSOR_COB_ID`, `<hh` left/right mm + flag byte. |
| `RFIDReader` | raw TCP socket | Sends `RFID_INIT_CMD` on connect; parses hex stream split on `CF`; tag = `packet[26:30]`. 2 s recv timeout still pings the watchdog. |
| `SLMPDriver` | pymcprotocol Type3E binary | (REMOVED) |
| `EncoderReader` | slcan / CANopen, `ENCODER_BITRATE` | Calibration only. **Shares the one CANable adapter** with `CANReader` — calibration stops the MGS1600 driver first, then restores it. |

> The CANable adapter is a single shared resource. Only one CAN driver may own it at a
> time — this is why `DriverManager` and `calibrate_mode` hand it back and forth.

---

## 9. Configuration & profiles — `config.py` + `profiles/agv1_kim.json` 

`config.py` loads `profiles/$AGV_ID.json` at import (default `agv1_kim`, falls back to
legacy `parameters.json`). It flattens JSON into module-level constants, applies defaults
via `.get()`, and **`_validate()` fail-fast range-checks** critical params at import
(refuses to boot a bad profile rather than dividing by zero / running away).

Profile sections: `networking`, `io_mapping` (incl. `DI_PROX`, `DI_FLIPPED`), 
`motor_channels`, `hardware` (per-wheel cal), `kinematics`, `speeds` (incl. `DECEL_RATE`, 
`APPROACH_DECEL_RATE`), `pid_tuning` (HIGH/SLOW/APPROACH gain sets + shared + speed-reduction 
coeffs), `can_sensor` (flags + sanity limits), `rfid`, `encoder`, `calibration`, 
`feedforward`, `sequences`, `features` (flags), `watchdog`, `timing`.

### Feature flags & live toggling — `main.py: DriverManager`

Flags: `DIO_ENABLED`, `CAN_ENABLED`, `NAV_ENABLED`, `SEQ_ENABLED`, `SLMP_ENABLED`.
**Dependency hierarchy:** `NAV → SEQ → SLMP` (disabling a parent cascades to children;
enabling a child requires its parent). `CAN` is independent. `SEQ` has no driver of its
own — it's a pure gate the engine reads; `NAV` owns the RFID reader (SEQ also needs tags). 

`DriverManager` starts/stops these tasks **at runtime** from the Flask HMI
(`/api/params/features`), marshalling onto the event loop via
`run_coroutine_threadsafe`. Toggles are **not persisted** — on restart the controller
boots from the profile JSON. DIO and AO are fixed at boot; CAN runs under the manager so
calibration can release the adapter.

---
## 11. Wheel-speed calibration — `calibration.py` + `drivers/can_encoder.py`

Open-loop straight ramp to measure commanded-vs-actual ground speed. Launched as the
`calibrate` mode from `armed` (HMI `/api/calibration/start`). Drives both wheels straight,
stepping `CAL_V_START → CAL_V_MAX` by `CAL_V_STEP` every `CAL_DWELL_S`, while the CANopen
friction-wheel encoder measures actual speed (`v = omega * ENCODER_RADIUS_M`; multi-turn,
wraps at `ENCODER_TOTAL_RANGE`). Saves CSV + PNG to `analysis_plot/`. The `finally`/
`asyncio.shield` cleanup always idles motors, stops the encoder, and **restores the
MGS1600 CAN driver**. Any mode change / RESET / web-stop / emergency cancels it.

---

## 12. Web HMI — `app/app.py` (Flask daemon thread)

Serves on `http://<LOCAL_IP>:5000`. Shares `AMRState`, `SequenceEngine`, and
`DriverManager` (set once via `run_server`). Pages: `/` (dashboard), `/manual`, `/io`,
`/params`, `/trolley`, `/errors`. Notable APIs:

- `GET /api/state` — full snapshot (AGV summary, PLC writes/reads, sensor + wheel velocities, sequence engine status).
- `POST /api/manual/command` — web jog (manual mode only; 400 ms expiry).
- `POST /api/web_button/mode` — `{"enable": bool}` toggles `web_button_mode` (clears all `web_btn_*` on disable).
- `POST /api/web_button/press` — `{"button": "start"|"reset"|"manual"|"emergency", "value": bool}` mirrors a panel button (403 unless bypass is active). See §5. 
- `POST /api/sequence` — (REMOVED)
- `POST /api/params/features` — live feature toggles via `DriverManager` (blocked while `running`/`emergency`).
- `GET /api/calibration/status`, `POST /api/calibration/start|stop`.
- `GET /api/trolley/state`, `POST /api/trolley/write` (REMOVED)
- `GET /api/io`, `GET /api/params`, `GET /api/errors`, `GET /api/wifi`.
- `POST /api/restart` — restarts the systemd service (needs passwordless sudoers rule; blocked while running/emergency).

---

## 13. Safety, logging, shutdown

- **`safety_watchdog`** polls each watched `SensorDriver`'s `get_health()` every 50 ms; if any exceeds its timeout it sets `system_error` and immediately zeros AO 0/1. `mode_manager` then brakes and holds until recovery. Watched: DI (`WATCHDOG_DI_TIMEOUT_S`), CAN, RFID.
- **`logger.py`**: `setup_logging()` once from `main`. Non-blocking `QueueHandler` + background `QueueListener` (file + console + in-memory ring) so disk stalls can't jitter the control loop. File/console at INFO; an in-memory deque keeps WARNING+ for the `/errors` page (`get_error_log()`). Third-party noise (werkzeug/pymodbus/can) silenced.
- **`main.shutdown()`** (on SIGINT/SIGTERM → CancelledError): opens fresh Modbus clients and zeros all AO registers and DO coils **directly**, bypassing the setpoint tables.

---

## 14. Editing cheatsheet

| To change… | Edit |
|---|---|
| Speeds, accel/decel, PID gains, feedforward, sensor limits | `profiles/agv1_kim.json` (validated at boot) |
| Add/remove/modify a sequence or speed zone | `profiles/agv1_kim.json` `"sequences"` (no Python) |
| Add a new sequence **action type** | new `_act_*` in `core/sequence_engine.py` + register it |
| Steering control law | `core/pid.py` and the feedforward block in `modes.py:auto_mode` |
| Mode transitions / FSM | `modes.py:mode_manager` |
| Motor wiring / channels | `motor_channels` in profile (never hardcode) |
| Add a hardware driver | subclass `SensorDriver`/`ActuatorDriver` in `drivers/`, wire into `main.run()` (and `DriverManager._SPEC` if toggleable) |
| PLC bits / handshake | (REMOVED) |
| HMI endpoints/pages | `app/app.py` + `app/templates/` |

**Debugging scripts** live in `debugging/` (e.g. `check_connections.py`, `read_dio.py`,
`motor_test.py` — *moves the AGV*, `slmp_tester10.py`, encoder readers). Most are
standalone and may read hardware directly / not honor `DI_FLIPPED`.

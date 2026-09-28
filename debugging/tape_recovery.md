# Lost-Tape Recovery Rotation — Implementation Plan

## Context

Today the AGV's only response to losing the magnetic tape is to **brake and hold**
([modes.py:125-135](../modes.py#L125-L135)); it resumes only if the tape happens to
reappear under the sensor. In the recurring left-corner failure (error grows
−70/−80/−90 then loss, or the over-correction overshoot to +error), the AGV ends up
stopped just *off* the line and needs a human to nudge it back. This feature makes the
AGV **actively re-acquire** the tape by rotating in place toward where the tape was
last seen, bounded to 5 s, before falling back to the current hold behavior.

The controller already knows the last lateral tape position (`last_valid_pv =
sensor["left_mm"]`, **positive = tape to the AGV's left**), and in-place rotation
primitives already exist — so this is a localized change, no new mode/FSM work.

## Decisions (confirmed)

- **Rotate toward the last-seen tape side** (tape last on the left → spin left). This
  sweeps the sensor back over the tape; it is the correct direction for our
  drift-to-outside failure mode.
- **On 5 s timeout: brake + auto-resume** — stop rotating, hold, and resume normally
  if the tape reappears (i.e. fall back to today's behavior). No operator RESET
  required. One search attempt per loss episode (no re-spinning).
- **Suppress rotation during an active sequence / APPROACH** (station load points,
  PLC handshakes) — there, just brake-and-hold to avoid spinning into trolley
  equipment.

## Behavior spec

On a tape-loss frame (`not sensor["tape_detected"]`):

1. **Skip the search and just brake-hold (current behavior) if any:**
   recovery disabled; a search was already attempted this loss episode
   (`recovery_attempted`); `state.speed_mode == "APPROACH"` or a sequence is active
   (suppress-near-stations); or `abs(last_valid_pv) < RECOVERY_MIN_OFFSET_MM`
   (no confident direction — e.g. tape lost while centered / sensor glitch).
2. **Otherwise run one rotation search** (`_tape_recovery_search`):
   - direction = **left** if `last_valid_pv > 0` else **right**.
   - command `motion.set_left/​set_right(state, mps_to_rpm(RECOVERY_ROTATE_SPEED_MPS))`
     (0.2 m/s per wheel → ~50°/s; ~250° max over 5 s).
   - each cycle: drain `sensor_queue` keep-latest; if a frame has `tape_detected`,
     stop and return **reacquired**; abort and return if `state.emergency_active`
     or `state.sequence_stop`; `await asyncio.sleep(config.DT)`.
   - return **timeout** after `RECOVERY_TIMEOUT_S`.
3. **After the search:**
   - reacquired → `motion.set_brake`, `pid.reset()`, `current_target_speed = 0`,
     `ff_filtered = 0`, `recovery_attempted = False`, reuse the existing
     "TAPE REACQUIRED — resuming" path (ramp forward from 0).
   - timeout/abort → `motion.set_brake`, `recovery_attempted = True`,
     `tape_was_lost = True` (today's hold/auto-resume fallback).
4. Reset `recovery_attempted = False` whenever the tape is reacquired.

## Files to modify

- **[modes.py](../modes.py)** — main change.
  - In `auto_mode`, capture `last_valid_pv` (the side) *before* it is nulled, add a
    `recovery_attempted` flag near the other loop-local state, and replace the
    brake-and-hold body of the tape-loss branch with the logic above.
  - Add a local `async def _tape_recovery_search(state, spin_left: bool) -> str`
    returning `"reacquired" | "timeout" | "aborted"`. Reuse `motion.set_left/
    set_right`, `motion.mps_to_rpm`, the keep-latest `sensor_queue` drain pattern,
    and `motion.set_brake`. Detect "sequence active" via the engine's existing
    status accessor (e.g. `engine.status()` / `engine._active_sequence`), passed into
    `auto_mode` already.
  - Safety: the loop is cooperative (`await` each cycle) so `mode_manager` cancels it
    on emergency/mode-change/system_error exactly as it cancels `auto_mode` today;
    the explicit `emergency_active`/`sequence_stop` checks just make it prompt. Ensure
    motors are stopped on every exit path (reacquired/timeout/abort) and rely on
    `mode_manager`'s cancel→idle/brake for the CancelledError path.
  - Only trigger on genuine tape-not-detected with a healthy sensor — do **not** spin
    on a sensor hardware failure (`FLAG_SENSOR_FAIL`); if that path exists separately,
    keep it brake-only.

- **[config.py](../config.py)** — load + validate a new `recovery` section, mirroring the
  `calibration` loader pattern ([config.py:222-225](../config.py#L222-L225),
  [config.py:300-303](../config.py#L300-L303)):
  - `RECOVERY_ENABLED` (bool), `RECOVERY_ROTATE_SPEED_MPS` (0.2),
    `RECOVERY_TIMEOUT_S` (5.0), `RECOVERY_MIN_OFFSET_MM` (~5.0),
    `RECOVERY_SUPPRESS_DURING_SEQUENCE` (bool).
  - `_validate`: `RECOVERY_TIMEOUT_S > 0`, `0 < RECOVERY_ROTATE_SPEED_MPS <= _MAX_SPEED_MPS`.

- **[profiles/agv1_kim.json](../profiles/agv1_kim.json)** — add a `"recovery"` block:
  ```json
  "recovery": {
    "_comment": "Lost-tape recovery: rotate in place toward last-seen tape side to re-acquire.",
    "ENABLED": 1,
    "ROTATE_SPEED_MPS": 0.2,
    "TIMEOUT_S": 5.0,
    "MIN_OFFSET_MM": 5.0,
    "SUPPRESS_DURING_SEQUENCE": 1
  }
  ```

- **[motion.py](../motion.py)** — no change; `set_left`/`set_right` already do the
  counter-rotation with positive AO + reversed direction coil.

## Risks / notes

- **Wrong-segment re-acquire:** after a large spin the AGV could acquire a different
  tape segment or face the wrong heading. Mitigated by toward-last-seen direction,
  the 5 s bound, and suppress-near-stations; residual risk on tight/parallel track
  sections. Acceptable for a first version; could later cap accepted re-acquire to a
  smaller sweep.
- **It moves the AGV unexpectedly to an operator** — log clearly at WARNING on entry
  ("LOST TAPE — searching <dir>") and on outcome.

## Verification

1. **Bench/logic:** temporarily force `last_valid_pv` positive then negative and
   confirm `_tape_recovery_search` chooses left then right (log the direction).
2. **On track, E-stop in hand:** run AUTO; induce a loss by briefly lifting the
   sensor off the tape on a straight. Confirm: spins **toward** the last-seen side,
   stops the instant tape is re-detected, resumes forward and re-centers.
3. **Timeout path:** induce a loss with the tape removed; confirm it spins ≤5 s,
   then brakes and holds, and auto-resumes when the tape is placed back under it.
4. **Suppression:** induce a loss during an APPROACH/active sequence; confirm it
   brake-holds with **no** rotation.
5. **Emergency during search:** hit E-stop mid-rotation; confirm immediate halt
   (idle) via `mode_manager` cancel.
6. Confirm a bad `recovery` profile (e.g. `TIMEOUT_S: 0`) is rejected at boot by
   `_validate`.

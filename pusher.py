"""
pusher.py — Trolley-hitch pusher pin controller
================================================
Independent background task. Watches the trolley limit switches and the
pusher-pin proximity sensors, drives the pusher actuator DOs, and arbitrates
between the autonomous hook sequence and manual web-dashboard control. Runs
unconditionally alongside the other core tasks (see main.py) but is a no-op
if this profile has no pusher hardware configured (config.PUSHER_DO_UP/DOWN
both [-1,-1]).

The AUTONOMOUS hook sequence is gated by config.PUSHER_AUTO_SEQUENCE_ENABLED
— an explicit profile flag ("pusher.AUTO_SEQUENCE_ENABLED"), default False
(see _auto_sequence_ready()). While it's off, DO8-11 only ever move in
response to an explicit button command (FULL UP/DOWN or JOG from the web
dashboard); they never turn on by themselves. This is deliberately NOT
inferred from whether DI_TROLLEY_LIMIT/DI_PUSHER_PROX_TOP have channel
numbers assigned in the profile — a channel can be numbered before the
physical sensor is actually wired, and an unwired/floating or misread limit
switch can false-positive "caught" with nothing attached (observed during
bring-up: the limit switches read "caught" within one poll cycle of boot,
no trolley present, firing moving_up on a hardware false-positive). Flip
AUTO_SEQUENCE_ENABLED to true only once the limit switches AND top
proximity sensor are physically installed and verified against real
hardware.

Hardware
--------
DI_TROLLEY_LIMIT[0], DI_TROLLEY_LIMIT[1] — normally-closed limit switches:
    True  = switch un-pressed -> trolley NOT yet caught by the hitch.
    False = switch pressed    -> trolley mechanically caught.
DI_PUSHER_PROX_TOP / DI_PUSHER_PROX_BOTTOM — inductive proximity sensors
    reading the pusher pin's own position (top/bottom of travel).
PUSHER_DO_UP / PUSHER_DO_DOWN — each a 2-element list; both channels of a
    pair are driven together, matching the "2 DO per direction" wiring.

state.trolley_hooked — physical truth, computed every cycle
-------------------------------------------------------------
`state.trolley_hooked = caught and at_top` is (re)computed at the TOP of
every loop iteration, unconditionally — before e-stop/emergency/jog/phase
logic — because it reflects the hitch's actual mechanical state, not
"whether the motor is currently being told to do something". This is the
interlock core.sequence_engine's "wait_hooked" action polls to release the
stop_at_c1 "AGV ga boleh lanjut" hold — see that action's docstring for the
fail-open-on-timeout caveat. It stays correct however the pin got to the top
(autonomous hook sequence, a manual full-travel UP, or jog).

Control sources, highest priority first
----------------------------------------
1. state.pusher_estop — pusher-only e-stop (web dashboard button), separate
   from the AGV-wide state.emergency_active. De-energizes immediately,
   clears any pending up/down/jog request (so nothing resumes on its own the
   instant it's cleared — operator must re-issue), and reports
   pusher_status="estopped". Does NOT touch trolley_hooked (still live/real).
2. state.emergency_active / state.system_error — AGV-wide stop conditions
   (same convention as auto_mode/reverse_mode). De-energizes but leaves the
   in-progress phase alone (paused, not aborted) so it can resume once clear.
3. state.pusher_jog_cmd ("up"|"down") — hold-to-run web control: drives
   continuously while set, stops the instant it's cleared/expires. Still
   respects top/bottom proximity as a hard stop even while held (added
   defensively — not explicitly requested, but driving into the mechanical
   end of travel indefinitely seemed like the wrong default). A fresh jog
   command clears any pending full-travel request so an old command can't
   resume by surprise right after a manual jog.
   Watchdog: the browser must keep re-sending while the button is held
   (POST /api/pusher/jog every ~150-200ms is plenty against the 400ms
   expiry); pusher_jog_expire lapsing (dropped connection, closed tab) is
   treated exactly like releasing the button.
4. state.pusher_up_request / state.pusher_down_request — "full travel" web
   buttons: one press latches the request, pusher.py drives to the
   respective proximity sensor and clears the flag on arrival (or on
   PUSHER_MOVE_TIMEOUT_S / estop). Both are manual-only (see note below) —
   neither is part of the autonomous sequence.
5. Autonomous hook sequence (below) — the only thing that runs with no
   command from anyone.

Autonomous sequence (idle -> hook_wait -> moving_up -> hooked)
------------------------------------------------------------------------------
Not gated by any RFID tag; the reverse maneuver that brings the AGV into
range of the limit switches is handled elsewhere (modes.reverse_mode + the
profile's "stop_at_c1" sequence).
1. idle       — watching DI_TROLLEY_LIMIT.
2. hook_wait  — the instant BOTH limit switches read False (trolley caught),
                start a PUSHER_HOOK_DELAY_S settle timer. Back to idle if
                either switch re-reads True before the timer elapses.
3. moving_up  — once the settle timer elapses (switches still both False),
                drive the pusher UP. Aborted back to idle (de-energized) if
                a limit switch re-asserts mid-lift (trolley slipped — this
                abort is specific to the AUTONOMOUS path; a manual UP request
                is allowed to run to the top even without a trolley caught,
                e.g. for bench-testing the actuator), or on
                PUSHER_MOVE_TIMEOUT_S with no top-proximity hit (jam guard —
                added defensively, not explicitly requested).
4. hooked     — reached the instant DI_PUSHER_PROX_TOP reads True and the
                limit switches are still both False. Pusher de-energized.
                Falls back to idle if a limit switch re-asserts while parked
                here (trolley worked loose) — trolley_hooked follows
                automatically since it's recomputed every cycle.

Pusher DOWN (state.pusher_down_request) has no autonomous trigger — it is
web-dashboard-only (full-travel button, or jog). Nothing here decides *when*
to lower the pin on its own.
"""

import asyncio
import logging
import time

import config

logger = logging.getLogger(__name__)

_POLL_S       = 0.05
_JOG_STATUSES = ("jog_up", "jog_down")


def _configured():
    return any(ch >= 0 for ch in config.PUSHER_DO_UP) or any(ch >= 0 for ch in config.PUSHER_DO_DOWN)


def _limits_caught(di):
    """True once BOTH trolley limit switches read False (mechanically caught)."""
    l0, l1 = config.DI_TROLLEY_LIMIT
    if l0 < 0 or l1 < 0 or di is None or len(di) <= max(l0, l1):
        return False
    return not di[l0] and not di[l1]


def _prox(di, idx):
    if idx < 0 or di is None or len(di) <= idx:
        return False
    return bool(di[idx])


def _auto_sequence_ready():
    """The AUTONOMOUS hook sequence (idle -> hook_wait -> moving_up on its
    own, no button) is gated by config.PUSHER_AUTO_SEQUENCE_ENABLED — an
    explicit, operator-set flag (profile "pusher.AUTO_SEQUENCE_ENABLED"),
    defaulting to False. This is NOT inferred from whether DI_TROLLEY_LIMIT /
    DI_PUSHER_PROX_TOP have channel numbers assigned: a channel can be
    numbered in the profile before the physical sensor is actually wired, and
    an unwired/floating or misread limit switch can false-positive "caught"
    with no trolley present — which is exactly what happened during bring-up
    here (limit switches read "caught" within one poll cycle of boot, no
    trolley attached, firing moving_up on a hardware false-positive). Until
    the operator flips the flag on (after physically verifying the limit
    switches AND top proximity sensor), DO8-11 only ever move in response to
    an explicit button command (FULL UP/DOWN, JOG) — those still work either
    way, since a manual UP with no proximity just runs to
    PUSHER_MOVE_TIMEOUT_S and stops, which is expected given the button was
    pressed on purpose. The channel numbers are still range-checked (see
    config._validate) so a bad index can't slip through once enabled."""
    l0, l1 = config.DI_TROLLEY_LIMIT
    return (config.PUSHER_AUTO_SEQUENCE_ENABLED
            and l0 >= 0 and l1 >= 0 and config.DI_PUSHER_PROX_TOP >= 0)


def _drive(state, up: bool, down: bool):
    for ch in config.PUSHER_DO_UP:
        if ch >= 0:
            state.set_do(ch, up)
    for ch in config.PUSHER_DO_DOWN:
        if ch >= 0:
            state.set_do(ch, down)


async def pusher_controller(state):
    if not _configured():
        logger.info("[Pusher] No pusher hardware configured in this profile — task idle.")
        return

    logger.info("[Pusher] Started (limit=%s, prox_top=%s, prox_bottom=%s)",
                config.DI_TROLLEY_LIMIT, config.DI_PUSHER_PROX_TOP, config.DI_PUSHER_PROX_BOTTOM)
    if not _auto_sequence_ready():
        logger.warning("[Pusher] AUTO_SEQUENCE_ENABLED is off (or DI_TROLLEY_LIMIT/"
                        "DI_PUSHER_PROX_TOP unset) — autonomous hook sequence DISABLED. "
                        "DO8-11 only respond to explicit button commands (FULL UP/DOWN, JOG). "
                        "Set profile \"pusher.AUTO_SEQUENCE_ENABLED\": true once the limit "
                        "switches and top proximity sensor are physically wired and verified.")

    # Unconditional safety default: DO8-11 must never be energized without an
    # explicit command. This forces both DO_UP and DO_DOWN channels off the
    # instant this task starts — not dependent on the loop first reaching the
    # "idle" branch below — so a coil left ON from a previous run (Modbus
    # slaves retain coil state across a master restart; this process coming
    # back up does NOT reset the physical relay by itself) gets cleared
    # immediately rather than waiting on connection/scheduling timing.
    _drive(state, up=False, down=False)

    phase          = "idle"   # idle | hook_wait | moving_up | hooked | moving_down
    hook_deadline  = None
    move_deadline  = None
    moving_up_auto = False    # True only when moving_up was entered via the autonomous
                               # hook sequence — gates the "abort on limit re-assert" guard

    while True:
        di     = state.latest_di
        caught = _limits_caught(di)
        at_top = _prox(di, config.DI_PUSHER_PROX_TOP)
        at_bot = _prox(di, config.DI_PUSHER_PROX_BOTTOM)
        now    = time.monotonic()

        # Physical truth, always live — see module docstring.
        state.trolley_hooked = caught and at_top

        # Jog watchdog: browser stopped refreshing (button released, dropped
        # connection, closed tab) — same pattern as web_manual_expire.
        if state.pusher_jog_cmd is not None and time.time() > state.pusher_jog_expire:
            state.pusher_jog_cmd = None

        # A jog phase never survives to the next non-jog cycle unhandled —
        # land back in idle so the normal phase machine below picks up
        # cleanly from current sensor state.
        if phase in _JOG_STATUSES:
            phase = "idle"

        # ── 1. Pusher e-stop — highest priority ────────────────────────────
        if state.pusher_estop:
            _drive(state, up=False, down=False)
            if state.pusher_up_request or state.pusher_down_request or state.pusher_jog_cmd:
                state.pusher_up_request   = False
                state.pusher_down_request = False
                state.pusher_jog_cmd      = None
                logger.warning("[Pusher] E-STOP — cleared pending pusher command(s)")
            phase = "idle"
            state.pusher_status = "estopped"
            await asyncio.sleep(_POLL_S)
            continue

        # ── 2. AGV-wide emergency / driver fault ───────────────────────────
        if state.emergency_active or state.system_error:
            _drive(state, up=False, down=False)
            state.pusher_status = phase   # paused, not aborted — resumes once clear
            await asyncio.sleep(_POLL_S)
            continue

        # ── 3. Manual jog (hold-to-run) — overrides everything below ──────
        if state.pusher_jog_cmd in ("up", "down"):
            # A fresh jog command supersedes any latched full-travel request
            # so releasing the jog button doesn't silently resume an old one.
            # Also updates the local `phase` (not just pusher_status) so that
            # when jog ends, the _JOG_STATUSES reset below lands cleanly in
            # "idle" instead of falling back into a stale moving_up/moving_down
            # phase and resuming its old direction on its own.
            state.pusher_up_request   = False
            state.pusher_down_request = False
            if state.pusher_jog_cmd == "up":
                _drive(state, up=not at_top, down=False)
                phase = "jog_up"
            else:
                _drive(state, up=False, down=not at_bot)
                phase = "jog_down"
            state.pusher_status = phase
            await asyncio.sleep(_POLL_S)
            continue

        # ── 4. Manual full-travel requests (web dashboard) ─────────────────
        if state.pusher_up_request and phase in ("idle", "hook_wait", "hooked"):
            phase          = "moving_up"
            moving_up_auto = False
            move_deadline  = now + config.PUSHER_MOVE_TIMEOUT_S
            logger.info("[Pusher] Manual UP requested")

        if state.pusher_down_request and phase in ("idle", "hook_wait", "hooked"):
            if phase == "hook_wait":
                logger.info("[Pusher] DOWN requested — cancelling pending auto UP")
            phase         = "moving_down"
            move_deadline = now + config.PUSHER_MOVE_TIMEOUT_S

        # ── 5. Autonomous hook sequence / phase machine ─────────────────────
        if phase == "idle":
            _drive(state, up=False, down=False)
            if caught and _auto_sequence_ready():
                phase         = "hook_wait"
                hook_deadline = now + config.PUSHER_HOOK_DELAY_S
                logger.info("[Pusher] Trolley limit switches OFF — %.1fs settle before pusher UP",
                            config.PUSHER_HOOK_DELAY_S)

        elif phase == "hook_wait":
            if not caught:
                logger.warning("[Pusher] Limit switch re-asserted during settle — back to idle")
                phase = "idle"
            elif now >= hook_deadline:
                phase          = "moving_up"
                moving_up_auto = True
                move_deadline  = now + config.PUSHER_MOVE_TIMEOUT_S
                logger.info("[Pusher] Settle elapsed — pusher UP")

        elif phase == "moving_up":
            if at_top:
                _drive(state, up=False, down=False)
                state.pusher_up_request = False
                logger.info("[Pusher] Top proximity reached — trolley_hooked=%s", state.trolley_hooked)
                phase = "hooked" if state.trolley_hooked else "idle"
            elif now >= move_deadline:
                logger.error("[Pusher] UP move timed out after %.0fs (no top proximity) — aborting",
                             config.PUSHER_MOVE_TIMEOUT_S)
                _drive(state, up=False, down=False)
                state.pusher_up_request = False
                phase = "idle"
            elif moving_up_auto and not caught:
                logger.warning("[Pusher] Limit switch re-asserted mid-lift — aborting auto UP")
                _drive(state, up=False, down=False)
                phase = "idle"
            else:
                _drive(state, up=True, down=False)

        elif phase == "hooked":
            _drive(state, up=False, down=False)
            if not caught:
                logger.warning("[Pusher] Limit switch re-asserted while hooked — un-hooking")
                phase = "idle"

        elif phase == "moving_down":
            if at_bot:
                _drive(state, up=False, down=False)
                state.pusher_down_request = False
                logger.info("[Pusher] Bottom proximity reached — DOWN complete")
                phase = "idle"
            elif now >= move_deadline:
                logger.error("[Pusher] DOWN move timed out after %.0fs (no bottom proximity) — aborting",
                             config.PUSHER_MOVE_TIMEOUT_S)
                _drive(state, up=False, down=False)
                state.pusher_down_request = False
                phase = "idle"
            else:
                _drive(state, up=False, down=True)

        state.pusher_status = phase
        await asyncio.sleep(_POLL_S)

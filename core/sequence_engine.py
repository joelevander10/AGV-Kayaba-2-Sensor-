"""
Sequence Engine — declarative, config-driven sequence runner.

Sequences are defined in the profile JSON under "sequences". Each sequence has:
  - a trigger  (rfid | marker | rfid_then_marker)
  - an actions list (ordered steps executed sequentially)
  - a cooldown  (seconds to ignore the same trigger after completion)
  - requires_mode   (mode that must be active for the trigger to fire)
  - requires_branch (optional — must equal state.active_branch for the trigger
                      to fire; used to gate destination fork tags, e.g. AA/AB/AC
                      only reacting when the AGV was dispatched to that branch.
                      Cleared via the "clear_branch" action on a merge tag.)

Adding a new sequence:  edit the profile JSON — zero Python changes needed.
Adding a new action type: add one method + register it in __init__.

Trigger types
-------------
rfid              : fires immediately when the matching RFID tag is read
marker            : fires when the matching tape marker is detected by auto_mode
rfid_then_marker  : RFID arms the approach (slows down), marker executes the rest
"""

import asyncio
import logging
import time

import config

logger = logging.getLogger(__name__)


class SequenceTimeout(Exception):
    pass



class SequenceEngine:

    def __init__(self, state, sequence_defs):
        """
        Args:
            state:          AMRState — shared state object
            sequence_defs:  list of sequence dicts loaded from config.SEQUENCES
        """
        self._state     = state
        self._sequences = sequence_defs

        # name → epoch at which the cooldown expires (0.0 = never triggered yet)
        self._cooldowns: dict = {}

        # rfid_then_marker: name → seq_def for sequences that have been RFID-armed
        # and are waiting for their marker
        self._armed: dict = {}

        # name of currently executing sequence (one at a time)
        self._active_sequence: str | None = None
        self._current_seq: dict | None = None

        # asyncio Task for the running sequence — cancelled on mode transitions
        self._active_task: asyncio.Task | None = None

        # Most recent notable sequence event (timeout / completion / PLC fault),
        # surfaced on the dashboard. None until the first event.
        # {name, kind, message, ts}
        self._last_event: dict | None = None

        # ── Built-in action registry ──────────────────────────────────────────
        self._actions: dict = {
            "set_speed":         self._act_set_speed,
            "set_track":         self._act_set_track,
            "wait_marker":       self._act_wait_marker,
            "stop_agv":          self._act_stop_agv,
            "resume":            self._act_resume,
            "sequence_stop":     self._act_sequence_stop,
            "wait_seconds":      self._act_wait_seconds,
            "clear_branch":      self._act_clear_branch,
            "start_reverse":     self._act_start_reverse,
            "stop_reverse":      self._act_stop_reverse,
            "set_ignore_1":      self._act_set_ignore_1,
            "clear_ignore_1":    self._act_clear_ignore_1,
            "go_home":           self._act_go_home,
            "exit_auto":         self._act_exit_auto,
            "wait_hooked":       self._act_wait_hooked,
            "pusher_up":         self._act_pusher_up,
            "pusher_down":       self._act_pusher_down,
            "wait_pusher_bottom": self._act_wait_pusher_bottom,
            "set_reverse_speed": self._act_set_reverse_speed,
            "set_lidar_area":    self._act_set_lidar_area,
        }

    # ── Plugin point ─────────────────────────────────────────────────────────

    def register_action(self, name: str, handler):
        """Register a custom action handler so new action types can be added
        without modifying this file.

        handler signature:  async def handler(params: dict) -> None
        """
        self._actions[name] = handler

    # ── Feature category gating (navigation vs sequence) ─────────────────────

    @staticmethod
    def _category(seq: dict) -> str:
        """Classify a sequence as 'navigation' or 'sequence'.

        Explicit "category" in the JSON wins. Otherwise infer: a sequence whose
        actions are purely speed changes (corner slow-downs) is navigation;
        anything that stops the AGV, waits, or talks to the PLC is sequence."""
        cat = seq.get("category")
        if cat in ("navigation", "sequence"):
            return cat
        actions = seq.get("actions", [])
        if actions and all(a.get("type") == "set_speed" for a in actions):
            return "navigation"
        return "sequence"

    def _category_enabled(self, seq: dict) -> bool:
        """True if the feature flag for this sequence's category is on."""
        if self._category(seq) == "navigation":
            return config.NAV_ENABLED
        return config.SEQ_ENABLED

    # ── Public trigger interface (called by rfid_processor and auto_mode) ────

    async def on_rfid_tag(self, tag_hex: str):
        """Called by rfid_processor whenever a tag is read."""
        now = time.time()
        for seq in self._sequences:
            trig = seq["trigger"]

            if trig["type"] == "rfid" and trig["rfid_tag"] == tag_hex:
                if not self._category_enabled(seq):
                    continue
                if not self._check_preconditions(seq, now):
                    continue
                self._active_task = asyncio.create_task(self._run_sequence(seq))

            elif trig["type"] == "rfid_then_marker" and trig["rfid_tag"] == tag_hex:
                if not self._category_enabled(seq):
                    continue
                if not self._check_preconditions(seq, now):
                    continue
                # Arm: slow down for approach, record armed state. The approach
                # speed is configurable per trigger ("approach_speed"); e.g. SLOW
                # for a curved approach (with feedforward), APPROACH for a precise
                # 0.1 m/s station crawl. Defaults to APPROACH.
                approach_speed = trig.get("approach_speed", "APPROACH")
                self._armed[seq["name"]] = seq
                self._state.speed_mode       = approach_speed
                self._state.pending_sequence = seq["name"]
                # A SEQ approach arm is not a corner — clear feedforward so a
                # station approach on a straight doesn't get the corner bias.
                self._state.nav_in_corner = False
                logger.info("[SEQ] Armed '%s' — approach at %s for marker", seq["name"], approach_speed)

                # Watchdog: if the marker never shows up, don't sit armed at
                # APPROACH crawl indefinitely — fault out and resume HIGH.
                approach_timeout = trig.get("approach_timeout")
                if approach_timeout:
                    asyncio.create_task(
                        self._approach_timeout_watch(seq["name"], approach_timeout))

    async def on_reverse_marker(self):
        """Called by reverse_mode on a rising edge of DI_REVERSE_MARKER.
        Fires every sequence whose trigger has "reverse_marker": true — the
        same sequence as its RFID tag, so marker and tag share one cooldown
        and whichever arrives first wins (the other is ignored while the
        sequence runs and during its cooldown)."""
        now = time.time()
        for seq in self._sequences:
            if not seq["trigger"].get("reverse_marker"):
                continue
            if not self._category_enabled(seq):
                continue
            if not self._check_preconditions(seq, now):
                continue
            logger.info("[SEQ] Reverse marker — launching '%s'", seq["name"])
            self._active_task = asyncio.create_task(self._run_sequence(seq))

    async def on_marker(self, side: str):
        """Called by auto_mode whenever a left/right marker is detected in the
        sensor frame.  Fires both armed rfid_then_marker sequences and pure
        marker-triggered sequences."""
        now = time.time()

        # Check armed rfid_then_marker sequences first
        for name, seq in list(self._armed.items()):
            trig = seq["trigger"]
            if trig.get("marker_side") == side:
                if not self._category_enabled(seq):
                    continue
                del self._armed[name]
                self._state.pending_sequence = None
                logger.info("[SEQ] Marker '%s' detected — launching '%s'", side, name)
                # skip set_speed + wait_marker steps (already done during approach)
                self._active_task = asyncio.create_task(self._run_sequence(seq, skip_approach=True))

        # Check pure marker-triggered sequences
        for seq in self._sequences:
            trig = seq["trigger"]
            if trig["type"] == "marker" and trig.get("marker_side") == side:
                if not self._category_enabled(seq):
                    continue
                if not self._check_preconditions(seq, now):
                    continue
                self._active_task = asyncio.create_task(self._run_sequence(seq))

    async def cancel_active(self):
        """Cancel the currently running sequence task, if any.
        Called by mode_manager on emergency, manual switch, or stop."""
        if self._active_task and not self._active_task.done():
            self._active_task.cancel()
            try:
                await self._active_task
            except (asyncio.CancelledError, Exception):
                pass
        self._active_task = None

    async def _approach_timeout_watch(self, name: str, timeout: float):
        """If `name` is still armed `timeout` seconds after its RFID arm-trigger
        (i.e. the marker never arrived), disarm it, log a fault, and resume
        SLOW (the AGV's normal running speed) so it doesn't sit crawling at
        APPROACH speed forever."""
        await asyncio.sleep(timeout)
        if name in self._armed:
            del self._armed[name]
            self._state.pending_sequence = None
            self._state.speed_mode       = "SLOW"
            logger.error("[SEQ] '%s' approach timed out after %.0fs — no marker, "
                         "resuming SLOW", name, timeout)
            self._record_event(name, "timeout",
                               f"approach timed out after {timeout:.0f}s — no marker, resumed SLOW")

    def cancel_armed(self):
        """Disarm all pending rfid_then_marker sequences.
        Called by mode_manager on any mode transition (emergency, manual switch,
        reset) so stale armed states don't survive the mode change."""
        if self._armed:
            logger.info("[SEQ] Disarming %d pending sequence(s)", len(self._armed))
        self._armed.clear()
        self._state.pending_sequence = None

    # ── Status (for dashboard) ────────────────────────────────────────────────

    def _record_event(self, name: str, kind: str, message: str) -> None:
        """Stash the most recent notable sequence outcome for the dashboard.
        kind is one of 'completed' | 'timeout' | 'fault'."""
        self._last_event = {
            "name": name, "kind": kind, "message": message, "ts": time.time(),
        }

    def status(self) -> dict:
        """Returns a snapshot for the Flask /api/state response."""
        now = time.time()
        last_event = None
        if self._last_event is not None:
            ev = self._last_event
            last_event = {
                "name":    ev["name"],
                "kind":    ev["kind"],
                "message": ev["message"],
                "age_s":   round(now - ev["ts"], 1),
            }
        return {
            "active":    self._active_sequence,
            "armed":     list(self._armed.keys()) or None,
            "cooldowns": {
                name: round(exp - now, 1)
                for name, exp in self._cooldowns.items()
                if exp > now
            },
            "last_event": last_event,
        }

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _check_preconditions(self, seq: dict, now: float) -> bool:
        """Returns True if the sequence may fire (mode + cooldown checks)."""
        required_mode = seq.get("requires_mode")
        if required_mode and self._state.current_mode != required_mode:
            logger.debug("[SEQ] '%s' ignored — mode is '%s', need '%s'",
                         seq["name"], self._state.current_mode, required_mode)
            return False

        # Destination gate: a fork tag (e.g. AA/AB/AC) only fires the sequence
        # for the branch the AGV was actually dispatched to. Sequences with no
        # "requires_branch" key (the vast majority — speed zones, checkpoints,
        # trolley loads) are ungated and always pass this check.
        required_branch = seq.get("requires_branch")
        if required_branch is not None and self._state.active_branch != required_branch:
            logger.debug("[SEQ] '%s' ignored — active_branch is '%s', need '%s'",
                         seq["name"], self._state.active_branch, required_branch)
            return False

        # Generic "must NOT be set" gate — e.g. tag "1"'s reverse trigger is
        # skipped once ignore_rfid_1 is set (by the C1 stop), so backing out
        # of a dead-end doesn't re-arm itself on every pass over that tag.
        skip_flag = seq.get("skip_if_flag")
        if skip_flag and getattr(self._state, skip_flag, False):
            logger.debug("[SEQ] '%s' ignored — %s is set", seq["name"], skip_flag)
            return False

        # Generic "MUST be set" gate — the opposite of skip_if_flag. e.g. the
        # B0 stop only fires on runs launched by DI_CALL_START (call_run).
        need_flag = seq.get("requires_flag")
        if need_flag and not getattr(self._state, need_flag, False):
            logger.debug("[SEQ] '%s' ignored — %s is not set", seq["name"], need_flag)
            return False

        cooldown_exp = self._cooldowns.get(seq["name"], 0.0)
        if now < cooldown_exp:
            logger.debug("[SEQ] '%s' ignored — cooldown %.1fs remaining",
                         seq["name"], cooldown_exp - now)
            return False

        if self._active_sequence is not None:
            logger.debug("[SEQ] '%s' ignored — '%s' already running",
                         seq["name"], self._active_sequence)
            return False

        return True

    async def _run_sequence(self, seq: dict, skip_approach: bool = False):
        """Execute the action list for a sequence.

        skip_approach=True skips the wait_marker step (the marker that triggered
        this sequence already satisfied it). set_speed steps still run, so a
        sequence can switch speed zones after the marker.
        """
        name = seq["name"]
        logger.info("[SEQ] Running '%s'", name)
        self._active_sequence = name
        self._current_seq = seq          # available to action handlers

        try:
            for action in seq["actions"]:
                atype = action["type"]

                if skip_approach and atype == "wait_marker":
                    continue

                handler = self._actions.get(atype)
                if handler is None:
                    logger.warning("[SEQ] '%s' — unknown action type '%s', skipping",
                                   name, atype)
                    continue

                await handler(action)

            logger.info("[SEQ] Completed '%s'", name)
            self._record_event(name, "completed", "completed")

        except asyncio.CancelledError:
            logger.info("[SEQ] Cancelled '%s'", name)
            self._state.sequence_stop = False
            self._state.nav_in_corner = False  # cancel always clears (mode reset follows)
            raise

        except SequenceTimeout as exc:
            logger.warning("[SEQ] '%s' timed out (%s) — resuming", name, exc)
            self._state.sequence_stop = False
            self._state.speed_mode    = "SLOW"
            self._record_event(name, "timeout", f"timed out ({exc}) — resumed SLOW")

        finally:
            self._active_sequence = None
            self._current_seq     = None
            # Only clear nav_in_corner for SEQ sequences on completion.
            # NAV sequences leave it set (they established a speed zone that
            # persists until the next speed change or a reset).
            if self._category(seq) != "navigation":
                self._state.nav_in_corner = False
            self._cooldowns[name] = time.time() + seq.get("cooldown_s", 0.0)

    # ── Built-in action handlers ──────────────────────────────────────────────
    # Each handler receives the full action dict (params) from the JSON.

    async def _act_set_speed(self, p: dict):
        self._state.speed_mode = p["speed"]
        # Only a NAV sequence setting SLOW means "we are in a corner" — seq
        # approaches may also use SLOW but must not get curvature feedforward.
        if self._category(self._current_seq) == "navigation":
            self._state.nav_in_corner = (p["speed"] == "SLOW")
        else:
            self._state.nav_in_corner = False
        logger.info("[SEQ] Speed → %s", p["speed"])

    async def _act_set_track(self, p: dict):
        # Fork selection: switch which MGS1600 track the steering PID follows.
        # The sensor always reports both tracks (superimposed on a single line),
        # so this is purely a host-side choice — no command is sent to the sensor.
        # Front (auto_mode) only — REVERSE has no equivalent fork selection, it
        # always follows the rear sensor's right_mm reading unconditionally.
        side = str(p.get("side", "")).lower()
        if side not in ("left", "right"):
            logger.warning("[SEQ] set_track: invalid side %r — ignored", p.get("side"))
            return
        self._state.track_follow = side
        logger.info("[SEQ] Track follow → %s", side.upper())

    async def _act_clear_branch(self, p: dict):
        # Merge-back action: releases the destination selection so the next
        # dispatch requires an explicit pick from the HMI again. Fires
        # unconditionally (no requires_branch on the merge tag itself) so it
        # works no matter which of the 3 branches was actually taken.
        prev = self._state.active_branch
        self._state.active_branch = None
        logger.info("[SEQ] active_branch cleared (was '%s')", prev)

    async def _act_start_reverse(self, p: dict):
        # Commands mode_manager into REVERSE (PID tape-follow, direction
        # flipped). mode_manager polls reverse_request and does the actual
        # mode transition — this action only raises the flag.
        self._state.reverse_request = True
        logger.info("[SEQ] Reverse requested")

    async def _act_stop_reverse(self, p: dict):
        # Ends the reverse maneuver (fired by the C1 tag). Harmless no-op if
        # the AGV wasn't reversing — mode_manager only acts on this flag while
        # current_mode == "reversing".
        self._state.reverse_request = False
        logger.info("[SEQ] Reverse stop requested")

    async def _act_set_ignore_1(self, p: dict):
        self._state.ignore_rfid_1 = True
        logger.info("[SEQ] ignore_rfid_1 SET — tag '1' will be ignored until CR")

    async def _act_clear_ignore_1(self, p: dict):
        self._state.ignore_rfid_1 = False
        logger.info("[SEQ] ignore_rfid_1 CLEARED — tag '1' re-armed")

    async def _act_go_home(self, p: dict):
        # One-shot: raises the flag, mode_manager does the actual mode exit
        # (cancel AUTO, idle, drop to ARMED) and clears the flag right after.
        self._state.homepost_request = True
        self._state.at_homepost      = True
        logger.info("[SEQ] Homepost reached — requesting exit from AUTO")

    async def _act_exit_auto(self, p: dict):
        # Same exit path as go_home (stop, drop to ARMED, wait for START) but
        # NOT at homepost — used by the B0 stop. Does not touch at_homepost.
        self._state.homepost_request = True
        logger.info("[SEQ] Stop point reached — requesting exit from AUTO")

    async def _act_wait_marker(self, p: dict):
        side     = p["side"]
        deadline = time.time() + p.get("timeout", 15.0)
        logger.info("[SEQ] Waiting for %s marker (timeout %.0fs)", side, p.get("timeout", 15.0))
        while time.time() < deadline:
            if self._state.current_mode != "running":
                raise asyncio.CancelledError()
            if side == "prox":
                # Magnetic proximity sensor on DI_PROX (not the CAN tape frame).
                di = self._state.latest_di
                if config.DI_PROX >= 0 and di and len(di) > config.DI_PROX \
                        and di[config.DI_PROX]:
                    return
            else:
                sen = self._state.latest_sensor
                if sen and sen.get(f"{side}_marker"):
                    return
            await asyncio.sleep(0.02)
        raise SequenceTimeout(f"wait_marker side={side}")

    async def _act_set_lidar_area(self, p: dict):
        """Select the lidar field set: {"area": [bool, bool]} drives
        lidar.DO_AREA[0] / [1] (DO3 / DO7 on agv2). Stays until the next
        set_lidar_area or a reset (back to lidar.DEFAULT_AREA)."""
        area = p.get("area")
        if not (isinstance(area, list) and len(area) == 2):
            logger.warning("[SEQ] set_lidar_area: 'area' must be [bool, bool] — ignored")
            return
        self._state.lidar_area = [bool(area[0]), bool(area[1])]
        logger.info("[SEQ] Lidar area → %s", self._state.lidar_area)

    async def _act_set_reverse_speed(self, p: dict):
        """Override the REVERSE target speed (m/s), e.g. RFID CA slows the
        approach to the stop marker. reverse_mode ramps to it with DECEL_RATE."""
        speed = float(p["speed"])
        if speed <= 0:
            logger.warning("[SEQ] set_reverse_speed: speed must be > 0 (got %s) — ignored", speed)
            return
        self._state.reverse_speed_override = speed
        logger.info("[SEQ] Reverse speed → %.2f m/s", speed)

    async def _act_pusher_down(self, p: dict):
        """Command the pusher to full-travel DOWN — same path as the web FULL
        DOWN button (pusher.py drives until DI_PUSHER_PROX_BOTTOM or
        MOVE_TIMEOUT_S). Doesn't wait; follow it with wait_pusher_bottom.
        Skipped if the pusher e-stop is latched."""
        if self._state.pusher_estop:
            logger.warning("[SEQ] pusher_down skipped — pusher E-STOP is active")
            return
        self._state.pusher_up_request   = False
        self._state.pusher_down_request = True
        logger.info("[SEQ] Pusher DOWN requested")

    async def _act_wait_pusher_bottom(self, p: dict):
        """Block until the pusher's bottom proximity (DI_PUSHER_PROX_BOTTOM)
        reads ON. No "timeout" (or 0) = wait as long as it takes, AGV stays
        braked; RESET / emergency / MANUAL cancel it. With a timeout > 0 the
        generic SequenceTimeout handler resumes anyway (fail-open)."""
        idx      = config.DI_PUSHER_PROX_BOTTOM
        timeout  = float(p.get("timeout") or 0)
        deadline = time.time() + timeout if timeout > 0 else None
        logger.info("[SEQ] Waiting for pusher bottom proximity DI%s%s", idx,
                    "" if deadline is None else f" (timeout {timeout:.0f}s)")
        last_log = time.time()
        while deadline is None or time.time() < deadline:
            if self._state.current_mode not in ("running", "reversing"):
                raise asyncio.CancelledError()
            di = self._state.latest_di
            if idx >= 0 and di is not None and len(di) > idx and di[idx]:
                logger.info("[SEQ] Pusher at bottom")
                return
            if time.time() - last_log >= 10.0:
                last_log = time.time()
                logger.info("[SEQ] Still waiting for pusher bottom (pusher=%s)",
                            self._state.pusher_status)
            await asyncio.sleep(0.05)
        raise SequenceTimeout("wait_pusher_bottom")

    async def _act_pusher_up(self, p: dict):
        """Command the pusher to full-travel UP — same path as the web FULL UP
        button (pusher.py drives until DI_PUSHER_PROX_TOP or MOVE_TIMEOUT_S).
        Doesn't wait; follow it with wait_hooked. Skipped if the pusher
        e-stop is latched."""
        if self._state.pusher_estop:
            logger.warning("[SEQ] pusher_up skipped — pusher E-STOP is active")
            return
        self._state.pusher_down_request = False
        self._state.pusher_up_request   = True
        logger.info("[SEQ] Pusher UP requested")

    async def _act_wait_hooked(self, p: dict):
        """Block until pusher.py confirms the trolley hitch is caught AND the
        pusher pin has reached the top proximity sensor (state.trolley_hooked)
        — the "AGV ga boleh lanjut [sampai trolley kekait]" interlock. Meant
        to sit between "stop_reverse" and "resume" in stop_at_c1: sequence_stop
        is still True at that point (stop_agv already ran), so the AGV stays
        braked for the whole wait regardless of which mode_manager state it's
        nominally in.

        No "timeout" (or 0) = wait as long as it takes: the AGV stays braked
        until trolley_hooked is True. RESET / emergency / MANUAL still abort
        it (the mode leaves running/reversing and the sequence is cancelled).

        With a "timeout" > 0 the old fail-open behaviour applies: on timeout
        _run_sequence's SequenceTimeout handler clears sequence_stop and the
        AGV drives off even though hooking was never confirmed."""
        timeout  = float(p.get("timeout") or 0)
        deadline = time.time() + timeout if timeout > 0 else None
        if deadline is None:
            logger.info("[SEQ] Waiting for trolley hooked (trolley limit caught + pusher top) — no timeout")
        else:
            logger.info("[SEQ] Waiting for trolley hooked confirmation (timeout %.0fs)", timeout)
        last_log = time.time()
        while deadline is None or time.time() < deadline:
            # "reversing" too: at C1 this starts while mode_manager is still
            # switching REVERSING → RUNNING (stop_reverse just ran).
            if self._state.current_mode not in ("running", "reversing"):
                raise asyncio.CancelledError()
            if self._state.trolley_hooked:
                logger.info("[SEQ] Trolley hooked confirmed")
                return
            if time.time() - last_log >= 10.0:
                last_log = time.time()
                logger.info("[SEQ] Still waiting for trolley hooked (pusher=%s)",
                            self._state.pusher_status)
            await asyncio.sleep(0.05)
        # Fail-open exits REVERSE first: at C1 stop_reverse comes AFTER this
        # step, so without this the generic timeout handler (which clears
        # sequence_stop) would let the AGV resume backing up past C1.
        if self._state.reverse_request:
            self._state.reverse_request = False
            logger.warning("[SEQ] wait_hooked timeout while reversing — ending REVERSE (forward next)")
        raise SequenceTimeout("wait_hooked")

    async def _act_stop_agv(self, p: dict):
        logger.info("[SEQ] Stopping AGV")
        self._state.sequence_stop = True # This will be cleared by resume or mode change

    async def _act_resume(self, p: dict):
        self._state.sequence_stop = False
        speed = p.get("speed")
        if speed is not None:
            self._state.speed_mode = speed
        logger.info("[SEQ] Resuming — speed=%s", speed or "(unchanged)")

    async def _act_sequence_stop(self, p: dict):
        """Timed stop: stop the AGV for `duration` seconds then resume.
        Optional `resume_speed` ("HIGH"/"SLOW") forces speed_mode on resume —
        without it, the AGV resumes at whatever speed_mode was active before
        the stop (unchanged)."""
        duration     = p["duration"]
        resume_speed = p.get("resume_speed")
        logger.info("[SEQ] Timed stop %.1fs", duration)
        self._state.sequence_stop = True
        await asyncio.sleep(duration)
        if resume_speed is not None:
            self._state.speed_mode = resume_speed
        self._state.sequence_stop = False
        logger.info("[SEQ] Timed stop ended%s",
                    f" — resumed {resume_speed}" if resume_speed else "")

    async def _act_wait_seconds(self, p: dict):
        await asyncio.sleep(p["duration"])
"""
keypad_evdev.py — USB keypad / keyboard plugged into the AGV PC
================================================================
Reads key presses straight from Linux (/dev/input/event*, python-evdev) and
hands them to keyboard_control.py — the same key map as the web MANUAL page.

* Hot-plug: devices are re-scanned every 2 s, so the dongle can be plugged in
  or replugged while the controller runs.
* Movement keys (WASD) are hold-to-drive: while held the command is refreshed
  every 100 ms; on release, unplug or read error the AGV stops (and the 400 ms
  watchdog in manual_mode stops it anyway if this task ever stalls).
* GRAB (profile keypad.GRAB): the keypad's keys go only to the AGV, not to the
  AGV PC's desktop or terminal.
* Permissions: the user running the controller must be in group "input"
  (build_release.sh does this for the service user "agv").
"""
import asyncio
import logging

import config
import keyboard_control as kc

logger = logging.getLogger(__name__)

try:
    import evdev
    from evdev import ecodes
except ImportError:          # not installed (or not Linux)
    evdev = None
    ecodes = None

SCAN_INTERVAL_S = 2.0
REFRESH_S = 0.1


def _code_map():
    """evdev key code -> our key name (top-row digits AND numpad digits)."""
    m = {}
    for letter in "wasdzxrqetykl":
        m[getattr(ecodes, f"KEY_{letter.upper()}")] = letter
    for d in "0123456789":
        m[getattr(ecodes, f"KEY_{d}")] = d
        m[getattr(ecodes, f"KEY_KP{d}")] = d
    # Arrow keys / remote D-pad (e.g. the ring around OK on an air-mouse
    # remote) drive the same as WASD.
    m[ecodes.KEY_UP] = "w"
    m[ecodes.KEY_DOWN] = "s"
    m[ecodes.KEY_LEFT] = "a"
    m[ecodes.KEY_RIGHT] = "d"
    m[ecodes.KEY_ENTER] = "enter"       # Enter key (and OK on many remotes) = RESET
    m[ecodes.KEY_KPENTER] = "enter"
    return m


def _is_keyboard(dev):
    keys = dev.capabilities().get(ecodes.EV_KEY, [])
    return ecodes.KEY_W in keys or ecodes.KEY_KP1 in keys or ecodes.KEY_UP in keys


def _name_ok(name):
    if config.KEYPAD_NAME_CONTAINS and config.KEYPAD_NAME_CONTAINS.lower() not in name.lower():
        return False
    return not any(x and x.lower() in name.lower() for x in config.KEYPAD_EXCLUDE_CONTAINS)


async def keypad_reader(state):
    if not config.KEYPAD_ENABLED:
        logger.info("[Keypad] disabled in profile (keypad.ENABLED)")
        return
    if evdev is None:
        logger.warning("[Keypad] python-evdev not installed — USB keypad on the AGV PC is "
                       "not available (pip install evdev)")
        return

    codes = _code_map()
    devices = {}            # path -> (InputDevice, task)
    held = {}               # path -> set of movement keys held on that device
    sent_cmd = [None]       # last movement command this task sent

    def _publish():
        allheld = set().union(*held.values()) if held else set()
        cmd = kc.move_command(allheld)
        if cmd is not None:
            if kc.apply_move(state, cmd):
                sent_cmd[0] = cmd
        elif sent_cmd[0] is not None:
            kc.apply_move(state, None)      # release -> stop at once
            sent_cmd[0] = None

    async def _refresh():
        while True:
            if any(held.values()):
                _publish()
            await asyncio.sleep(REFRESH_S)

    async def _read(path, dev):
        held[path] = set()
        try:
            async for ev in dev.async_read_loop():
                if ev.type != ecodes.EV_KEY:
                    continue
                key = codes.get(ev.code)
                if key is None:
                    continue
                if key in kc.MOVE_KEYS:
                    if ev.value == 1:
                        held[path].add(key)
                    elif ev.value == 0:
                        held[path].discard(key)
                    _publish()
                elif ev.value == 1:                 # press (ignore auto-repeat)
                    ok, msg = kc.handle_key(state, key, source="keypad")
                    if not ok:
                        logger.warning("[Keypad] %s ignored — %s", key.upper(), msg)
        except (OSError, asyncio.CancelledError) as e:
            if not isinstance(e, asyncio.CancelledError):
                logger.warning("[Keypad] '%s' disconnected (%s)", dev.name, e)
        finally:
            held.pop(path, None)
            _publish()                              # stop if it was driving
            devices.pop(path, None)
            state.keypad_devices = [d.name for d, _ in devices.values()]
            try:
                dev.close()
            except Exception:
                pass

    refresher = asyncio.create_task(_refresh())
    logger.info("[Keypad] watching for USB keypads (name filter: %r)", config.KEYPAD_NAME_CONTAINS)
    try:
        while True:
            for path in evdev.list_devices():
                if path in devices:
                    continue
                try:
                    dev = evdev.InputDevice(path)
                except OSError:
                    continue                        # no permission / vanished
                if not _is_keyboard(dev) or not _name_ok(dev.name):
                    dev.close()
                    continue
                if config.KEYPAD_GRAB:
                    try:
                        dev.grab()
                    except OSError as e:
                        logger.warning("[Keypad] could not grab '%s' (%s)", dev.name, e)
                devices[path] = (dev, asyncio.create_task(_read(path, dev)))
                state.keypad_devices = [d.name for d, _ in devices.values()]
                logger.info("[Keypad] connected: '%s' (%s)", dev.name, path)
            await asyncio.sleep(SCAN_INTERVAL_S)
    finally:
        refresher.cancel()
        for _, (dev, task) in list(devices.items()):
            task.cancel()

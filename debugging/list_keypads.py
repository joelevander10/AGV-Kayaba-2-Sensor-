#!/usr/bin/env python3
"""List the keyboards/keypads Linux sees, to pick keypad.NAME_CONTAINS.
Run on the AGV PC:  python3 debugging/list_keypads.py
(If nothing shows: sudo usermod -aG input $USER, then log out/in.)"""
import evdev
from evdev import ecodes
found = False
for path in evdev.list_devices():
    d = evdev.InputDevice(path)
    keys = d.capabilities().get(ecodes.EV_KEY, [])
    if ecodes.KEY_W in keys or ecodes.KEY_KP1 in keys or ecodes.KEY_UP in keys:
        found = True
        print(f"{path:20s}  {d.name!r}   phys={d.phys}")
print("" if found else "No keyboard found (check the dongle, or permissions: group 'input').")

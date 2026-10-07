"""
engineer_auth.py — engineer password for the web HMI's protected functions
===========================================================================
The password itself is never stored: only a salted hash (werkzeug
scrypt/pbkdf2) in config.ENGINEER_AUTH_FILE, next to the program
(/opt/agv2/engineer_auth.json in a release, beside config.py in the repo —
git-ignored). In a release the file is root-owned, readable only by the
service account.

Set or change it:
    release : sudo AGV_ID=agv2_kim /opt/agv2/main.bin --set-engineer-password
    source  : AGV_ID=agv2_kim python3 main.py --set-engineer-password
"""

import getpass
import json
import os
import time

from werkzeug.security import check_password_hash, generate_password_hash

import config

MIN_LENGTH = 6


def is_set() -> bool:
    return _load_hash() is not None


def _load_hash():
    try:
        with open(config.ENGINEER_AUTH_FILE, "r") as f:
            return json.load(f).get("hash")
    except (OSError, ValueError):
        return None


def verify(password: str) -> bool:
    h = _load_hash()
    if not h or not isinstance(password, str):
        return False
    try:
        return check_password_hash(h, password)
    except Exception:
        return False


def set_password_interactive() -> int:
    """Prompt twice on the terminal and store the hash. Returns an exit code."""
    path = config.ENGINEER_AUTH_FILE
    print(f"Engineer password file: {path}")
    pw1 = getpass.getpass("New engineer password: ")
    if len(pw1) < MIN_LENGTH:
        print(f"Too short — use at least {MIN_LENGTH} characters. Nothing changed.")
        return 1
    pw2 = getpass.getpass("Repeat password: ")
    if pw1 != pw2:
        print("Passwords do not match. Nothing changed.")
        return 1

    data = {"hash": generate_password_hash(pw1),
            "set_at": time.strftime("%Y-%m-%d %H:%M:%S")}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f)
    os.chmod(tmp, 0o640)
    # In a release (run with sudo): root owns it, the "agv" service group reads it.
    # (os.geteuid does not exist on Windows — skip the ownership step there.)
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        try:
            import grp
            os.chown(tmp, 0, grp.getgrnam("agv").gr_gid)
        except KeyError:
            pass
    os.replace(tmp, path)
    print("Engineer password saved. Restart is not needed.")
    return 0

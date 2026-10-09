#!/usr/bin/env python3
"""
rfid_settings.py — read / change the UHF reader's stored settings
(CP-203910 / R121MU-910 "UHF Senior Reader", CF frame protocol).

Stop the controller first (the reader accepts one connection):
    sudo systemctl stop agv-controller

    python3 debugging/rfid_settings.py read
    python3 debugging/rfid_settings.py set interval=0 buzzer=0 q=1
    python3 debugging/rfid_settings.py restore <25-byte hex printed by 'read'>

    sudo systemctl start agv-controller

'set' changes only the fields you name; every other byte is written back
exactly as read. The settings before the change are saved to
~/rfid_params_backup.txt (first time only) so 'restore' can undo it.
"""
import binascii
import os
import socket
import sys
import time

READER = ("192.168.3.200", 2022)
CMD_SET, CMD_GET = 0x0071, 0x0072

# 25-byte parameter block, in order.
FIELDS = [
    ("address", 1), ("protocol", 1), ("work_mode", 1), ("interface", 1),
    ("baud", 1), ("wiegand", 1), ("antenna", 1), ("region", 1),
    ("start_freq_mhz", 2), ("start_freq_khz", 2), ("step_khz", 2),
    ("channels", 1), ("power", 1), ("inv_area", 1), ("q", 1), ("session", 1),
    ("acs_addr", 1), ("acs_len", 1), ("filter", 1), ("trigger", 1),
    ("buzzer", 1), ("interval", 1),
]
EDITABLE = {"interval", "buzzer", "q", "filter", "power", "session"}
NOTES = {
    "work_mode": "0=answer 1=active(auto-send) 2=trigger",
    "power":     "dBm (33 = max)",
    "q":         "Gen2 Q: 0-1 fastest for one tag at a time",
    "session":   "Gen2 session S0..S3",
    "filter":    "same-tag filter time",
    "buzzer":    "0 = buzzer off",
    "interval":  "pause between scans in active mode (0 = continuous)",
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def frame(cmd: int, data: bytes = b"") -> bytes:
    f = bytes([0xCF, 0xFF, cmd >> 8, cmd & 0xFF, len(data)]) + data
    c = crc16(f)
    return f + bytes([c >> 8, c & 0xFF])


def request(cmd: int, data: bytes = b"") -> bytes:
    with socket.create_connection(READER, timeout=3) as s:
        s.sendall(frame(cmd, data))
        buf, t_end = b"", time.time() + 3
        while time.time() < t_end:
            try:
                chunk = s.recv(1024)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            i = buf.find(b"\xCF")
            while i >= 0 and len(buf) - i >= 5:
                total = 7 + buf[i + 4]
                if len(buf) - i < total:
                    break
                f = buf[i:i + total]
                if crc16(f[:-2]) == (f[-2] << 8 | f[-1]) and (f[2] << 8 | f[3]) == cmd:
                    return f
                i = buf.find(b"\xCF", i + 1)
    raise SystemExit(f"No answer to command 0x{cmd:04X} — is the controller stopped?")


def decode(block: bytes) -> dict:
    out, i = {}, 0
    for name, n in FIELDS:
        out[name] = int.from_bytes(block[i:i + n], "big")
        i += n
    return out


def encode(vals: dict) -> bytes:
    return b"".join(vals[name].to_bytes(n, "big") for name, n in FIELDS)


def read_block() -> bytes:
    f = request(CMD_GET)
    if f[5] != 0:
        raise SystemExit(f"Reader returned status 0x{f[5]:02X}")
    block = f[6:-2]
    if len(block) != 25:
        raise SystemExit(f"Unexpected parameter length {len(block)}: {f.hex().upper()}")
    return block


def show(block: bytes):
    for k, v in decode(block).items():
        print(f"  {k:15s} {v:5d}   {NOTES.get(k, '')}")
    print("  raw:", block.hex().upper())


def write_block(block: bytes):
    f = request(CMD_SET, block)
    if f[5] != 0:
        raise SystemExit(f"Reader refused the change (status 0x{f[5]:02X}) — nothing written.")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("read", "set", "restore"):
        print(__doc__)
        return
    old = read_block()
    if sys.argv[1] == "read":
        show(old)
        return

    backup = os.path.expanduser("~/rfid_params_backup.txt")
    if not os.path.exists(backup):
        with open(backup, "w") as fh:
            fh.write(old.hex().upper() + "\n")
        print(f"Original settings saved to {backup}")

    if sys.argv[1] == "restore":
        new = bytes.fromhex(sys.argv[2])
        if len(new) != 25:
            raise SystemExit("restore needs the 25-byte hex (50 characters)")
    else:
        vals = decode(old)
        for arg in sys.argv[2:]:
            k, _, v = arg.partition("=")
            if k not in EDITABLE:
                raise SystemExit(f"'{k}' can't be changed here (allowed: {', '.join(sorted(EDITABLE))})")
            vals[k] = int(v)
        new = encode(vals)

    print("Before:"); show(old)
    write_block(new)
    after = read_block()
    print("After:"); show(after)
    print("OK" if after == new else "WARNING: reader settings differ from what was written")


if __name__ == "__main__":
    main()

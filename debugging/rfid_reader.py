import asyncio
import binascii
import socket
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import config
from drivers.rfid_tcp import parse_frames, tag_from_frame


async def main():
    loop = asyncio.get_event_loop()
    print(f"Connecting to RFID reader at {config.RFID_IP}:{config.RFID_PORT}...")
    print("Press Ctrl+C to stop.\n")

    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(False)
        try:
            await loop.sock_connect(sock, (config.RFID_IP, config.RFID_PORT))
            await loop.sock_sendall(sock, config.RFID_INIT_CMD)
            print("Connected. Waiting for tags...\n")
            buf = bytearray()

            while True:
                data = await loop.sock_recv(sock, 1024)
                if not data:
                    print("Connection closed by reader.")
                    break

                buf.extend(data)
                for frame in parse_frames(buf):
                    tag = tag_from_frame(frame)
                    raw = binascii.hexlify(frame).decode().upper()
                    if tag is None:
                        print(f"Frame (no tag): {raw}")
                    else:
                        print(f"{time.strftime('%H:%M:%S')}.{int(time.time()*1000)%1000:03d}  "
                              f"Tag: {tag}  (dec={int(tag, 16)})  raw={raw}")

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}. Retrying in 2s...")
        finally:
            sock.close()

        await asyncio.sleep(2)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

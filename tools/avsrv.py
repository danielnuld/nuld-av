"""Serves a PS2V stream (tools/mkpsv.py) to nuld-av's network source (nav_net_open) on port 8091: by netring.irx's
fast path (UDP, tools/fastsrv.py) and, for a PS2 whose Ethernet driver can't be hooked, by TCP as fast as it takes it.

usage: python tools/avsrv.py <stream.psm>
"""
import socket
import sys
import threading
import time
from pathlib import Path

import fastsrv


def serve(conn, addr, psm):
    t0, sent = time.time(), 0
    try:
        conn.recv(64)  # the request line ("play\n")
        with open(psm, "rb") as f:
            while block := f.read(64 << 10):
                conn.sendall(block)
                sent += len(block)
    except OSError as e:
        print(f"{addr[0]}: {e}")
    finally:
        conn.close()
        dt = time.time() - t0
        print(f"{addr[0]}: TCP: sent {sent >> 20} MB in {dt:.0f} s = {sent / 1024 / max(dt, 0.001):.0f} KB/s", flush=True)


def main():
    psm = Path(sys.argv[1])
    fastsrv.serve(8091, lambda what: open(psm, "rb").read)
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 8091))
    s.listen()
    print(f"serving {psm} on 8091 (UDP fast path and TCP)", flush=True)
    while True:
        conn, addr = s.accept()
        print(f"{addr[0]}: connected by TCP", flush=True)
        threading.Thread(target=serve, args=(conn, addr, psm), daemon=True).start()


if __name__ == "__main__":
    main()

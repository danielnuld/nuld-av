"""The PC's screen, live, to nuld-av on a PS2 (nav_net_open + nav_play_live), on UDP port 8091.

When the PS2 asks, ffmpeg starts capturing the screen (gdigrab, Windows), scales it to 1280x720 and encodes MPEG-2
with I and P pictures; tools/psvpack repacks each picture as it leaves the encoder (through WSL) and it goes to the
PS2 at once by netring.irx's fast path (tools/fastsrv.py). When the PS2 leaves, both stop.

usage: python tools/livesrv.py [--rate 12M] [--fps 30] [--title "a window's title"]
       (run from the project folder: it calls ./tools/psvpack through WSL; ffmpeg in PATH)
"""
import argparse
import collections
import struct
import subprocess
import threading
import time

import fastsrv


class Live:
    """ffmpeg -> psvpack -> whole pictures, handed to fastsrv picture by picture."""

    def __init__(self, a):
        grab = ["-f", "gdigrab", "-framerate", str(a.fps), "-draw_mouse", "1"]
        grab += ["-i", f"title={a.title}"] if a.title else ["-i", "desktop"]
        # no B pictures (nothing to reorder), one whole picture every two seconds, each written out as it is coded.
        # The buffer the rate control counts on is small: no picture much larger than its share, so none waits
        self.ffmpeg = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-fflags", "nobuffer", *grab,
             "-vf", "scale=1280:720:flags=bilinear:out_color_matrix=bt709:out_range=tv,format=yuv420p",
             "-c:v", "mpeg2video", "-bf", "0", "-g", str(a.fps * 2), "-dc", "10", "-b:v", a.rate, "-maxrate", a.rate,
             "-bufsize", str(int(float(a.rate.rstrip("M")) * 1e6 / 4)), "-flags", "+low_delay", "-flush_packets", "1",
             "-f", "mpeg2video", "-"], stdout=subprocess.PIPE, bufsize=0)
        self.pack = subprocess.Popen(["wsl", "./tools/psvpack", "-", "-", str(a.fps), "1"], stdin=self.ffmpeg.stdout,
                                     stdout=subprocess.PIPE, bufsize=0)
        self.units, self.cur, self.at, self.ended = collections.deque(), b"", 0, False
        self.lock = threading.Lock()
        self.made = self.most = 0
        self.t0 = time.time()
        threading.Thread(target=self.reader, daemon=True).start()

    def exactly(self, n):
        b = b""
        while len(b) < n:
            d = self.pack.stdout.read(n - len(b))
            if not d:
                raise EOFError
            b += d
        return b

    def reader(self):  # psvpack's output cut into pictures: header (the first time) + 'V' chunk + 'P' chunk
        try:
            unit = self.exactly(64)
            while True:
                head = self.exactly(8)
                unit += head + self.exactly(struct.unpack_from("<I", head, 4)[0])
                if head[:1] == b"P":
                    with self.lock:
                        self.units.append(unit)
                        self.made += 1
                        self.most = max(self.most, len(self.units))
                    unit = b""
        except (EOFError, OSError, ValueError):
            pass
        self.ended = True

    def read(self, n):
        with self.lock:
            d = b""
            while len(d) < n:
                if self.at >= len(self.cur):
                    if not self.units:
                        break
                    self.cur, self.at = self.units.popleft(), 0
                k = self.cur[self.at:self.at + n - len(d)]
                self.at += len(k)
                d += k
            # short only when nothing more is waiting: the PS2 hands what it has to its player at once on a short
            # datagram, and one such for each of a row of small pictures is more than it can do
            if not d:
                return b"" if self.ended else None
            return d

    def close(self):
        dt = time.time() - self.t0
        print(f"live: {self.made} pictures in {dt:.0f} s ({self.made / max(dt, 1e-3):.1f}/s), at most {self.most} waiting "
              "for the PS2 to take them", flush=True)
        for p in (self.ffmpeg, self.pack):
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], capture_output=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", default="12M", help="video bit rate (default 12M; the PS2's network takes ~20M)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--title", help="one window instead of the desktop")
    a = ap.parse_args()
    subprocess.run(["wsl", "true"], capture_output=True)  # WSL up before the first picture needs it (seconds, cold)
    fastsrv.serve(8091, lambda what: Live(a), "live")
    print("waiting for the PS2 on UDP 8091", flush=True)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()

"""The PC's screen, live, to nuld-av on a PS2 (nav_net_open + nav_play_live), on UDP port 8091.

When the PS2 asks, ffmpeg starts capturing the screen (gdigrab, Windows), scales it to 1280x720 and encodes MPEG-2
with I and P pictures; tools/psvpack repacks each picture as it leaves the encoder and it goes to the PS2 at once by
netring.irx's fast path (tools/fastsrv.py). When the PS2 leaves, both stop. With --audio, what Windows plays goes
along as MP2 ('A' chunks, for nav_live_audio): `pip install soundcard numpy`.

usage: python tools/livesrv.py [--rate 12M] [--fps 30] [--title "a window's title"] [--audio]
       (tools/psvpack.exe if it is there: tools/build_psvpack.bat; else ./tools/psvpack through WSL, run from the
       project folder. ffmpeg in PATH)
"""
import argparse
import collections
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import fastsrv


def audio_feed(pipe):
    """Windows' sound to ffmpeg, 10 ms at a time, until ffmpeg goes away. soundcard pads silence with zeros (WASAPI
    loopback itself sends nothing while nothing plays), so ffmpeg's audio input never starves."""
    import numpy as np
    import soundcard as sc
    try:
        mic = sc.get_microphone(sc.default_speaker().name, include_loopback=True)
        with mic.recorder(samplerate=48000, channels=2, blocksize=480) as r:
            while True:
                pipe.write((np.clip(r.record(numframes=480), -1, 1) * 32767).astype("<i2").tobytes())
    except (OSError, ValueError):  # ffmpeg left (its stdin closed)
        pass
    except Exception as e:
        print(f"live: audio capture stopped ({type(e).__name__}: {e})", flush=True)


class Live:
    """ffmpeg -> psvpack -> whole pictures (and MP2 frames), handed to fastsrv as they come.
    a: fps, rate ("12M"), title, grab ("gdigrab" / "ddagrab"), region ("X,Y,WxH"), audio (bool), audio_rate ("192k")."""

    def __init__(self, a):
        pre = ""
        if getattr(a, "grab", "gdigrab") == "ddagrab":  # Desktop Duplication: the frames come from the GPU
            grab, pre = ["-f", "lavfi", "-i", f"ddagrab=framerate={a.fps}:draw_mouse=1"], "hwdownload,format=bgra,"
        else:
            grab = ["-f", "gdigrab", "-framerate", str(a.fps), "-draw_mouse", "1"]
            if getattr(a, "region", None):  # one monitor, or a game's window beside something else
                x, y, size = a.region.split(",")
                grab += ["-offset_x", x, "-offset_y", y, "-video_size", size]
            grab += ["-i", f"title={a.title}"] if getattr(a, "title", None) else ["-i", "desktop"]
        audio, aout = getattr(a, "audio", False), []
        if audio:
            # MP2, a frame per datagram to a local port (read below). Not in one output with the video: ffmpeg's
            # muxers hold a stream back for the other, and the pictures left in bursts paced by the audio frames
            self.asock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.asock.bind(("127.0.0.1", 0))
            arate = getattr(a, "audio_rate", "192k")
            frame = int(arate.rstrip("kK")) * 1000 * 1152 // 48000 // 8
            grab += ["-f", "s16le", "-ar", "48000", "-ac", "2", "-i", "pipe:0"]
            aout = ["-map", "1:a", "-c:a", "mp2", "-b:a", arate, "-f", "mp2",
                    f"udp://127.0.0.1:{self.asock.getsockname()[1]}?pkt_size={frame}"]
        # no B pictures (nothing to reorder), each picture written out as it is coded. A whole picture only every ten
        # seconds: it is ~100 KB, 40 ms and more on the PS2's network, and nothing is lost on the way that it would
        # have to repair (cuts get their own). The rate control's buffer is small: no picture much over its share
        self.ffmpeg = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-fflags", "nobuffer", *grab, "-map", "0:v",
             "-vf", pre + "scale=1280:720:flags=bilinear:out_color_matrix=bt709:out_range=tv,format=yuv420p",
             "-c:v", "mpeg2video", "-bf", "0", "-g", str(a.fps * 10), "-dc", "10", "-b:v", a.rate, "-maxrate", a.rate,
             "-bufsize", str(int(float(a.rate.rstrip("M")) * 1e6 / 4)), "-flags", "+low_delay", "-flush_packets", "1",
             "-f", "mpeg2video", "-", *aout], stdin=subprocess.PIPE if audio else None, stdout=subprocess.PIPE, bufsize=0)
        # the Windows build if it is there (tools/build_psvpack.bat): through WSL's pipes a picture took ~30 ms more
        exe = Path(__file__).with_name("psvpack.exe")
        pack = [str(exe)] if exe.exists() else ["wsl", "./tools/psvpack"]
        self.pack = subprocess.Popen([*pack, "-", "-", str(a.fps), "1"], stdin=self.ffmpeg.stdout,
                                     stdout=subprocess.PIPE, bufsize=0)
        self.units, self.cur, self.at, self.ended = collections.deque(), b"", 0, False
        self.lock = threading.Lock()
        self.made = self.most = 0
        self.t0 = time.time()
        threading.Thread(target=self.reader, daemon=True).start()
        if audio:
            threading.Thread(target=audio_feed, args=(self.ffmpeg.stdin,), daemon=True).start()
            threading.Thread(target=self.audio_reader, daemon=True).start()

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

    def audio_reader(self):  # an MP2 frame -> an 'A' chunk and padding to 16 bytes, once the stream has begun
        try:
            while not self.ended:
                d = self.asock.recv(4096)
                if self.made:
                    pad = -len(d) % 16
                    with self.lock:
                        self.units.append(struct.pack("<B3xI", ord("A"), len(d)) + d +
                                          struct.pack("<B3xI", ord("P"), pad) + bytes(pad))
        except OSError:
            pass

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
        self.ended = True
        for p in (self.ffmpeg, self.pack):
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], capture_output=True)
        if hasattr(self, "asock"):
            self.asock.close()


def add_args(ap):
    ap.add_argument("--rate", default="12M", help="video bit rate (default 12M; the PS2's network takes ~20M)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--title", help="one window instead of the desktop")
    ap.add_argument("--region", help="X,Y,WxH of the desktop (physical pixels) instead of all of it")
    ap.add_argument("--grab", default="gdigrab", choices=["gdigrab", "ddagrab"], help="how the screen is captured")
    ap.add_argument("--audio", action="store_true", help="what Windows plays, as MP2 (pip install soundcard numpy)")
    ap.add_argument("--audio-rate", default="192k")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_args(ap)
    a = ap.parse_args()
    if not Path(__file__).with_name("psvpack.exe").exists():
        subprocess.run(["wsl", "true"], capture_output=True)  # WSL up before the first picture needs it (seconds, cold)
    fastsrv.serve(8091, lambda what: Live(a), "live")
    print("waiting for the PS2 on UDP 8091", flush=True)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()

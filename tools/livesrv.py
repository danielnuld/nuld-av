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


def audio_feed(live):
    """Windows' sound to the audio ffmpeg, 10 ms at a time, for as long as the stream lives. soundcard pads silence
    with zeros (WASAPI loopback itself sends nothing while nothing plays), so ffmpeg's input never starves."""
    import warnings

    import numpy as np
    import soundcard as sc
    warnings.filterwarnings("ignore", category=sc.SoundcardRuntimeWarning)  # "data discontinuity": every hiccup
    try:
        mic = sc.get_microphone(sc.default_speaker().name, include_loopback=True)
        with mic.recorder(samplerate=48000, channels=2, blocksize=480) as r:
            while not live.closed:
                pcm = (np.clip(r.record(numframes=480), -1, 1) * 32767).astype("<i2").tobytes()
                try:
                    live.affmpeg.stdin.write(pcm)
                except (OSError, ValueError):  # closed
                    break
    except Exception as e:
        print(f"live: audio capture stopped ({type(e).__name__}: {e})", flush=True)


# Pictures made and not yet taken by the PS2 before they are given up: its network or its decoder is not keeping up,
# and everything behind them (the sound too) would come out that much late, later and later. They are dropped, the
# capture starts again (its first picture is a whole one: nothing after it needs the dropped ones). The bit rate
# stays: on the console these were hiccups of the network at a tenth of what it carries, and 3/4 less each time
# left the picture at 3 Mbit/s for nothing. 6 pictures: 200 ms
BEHIND = 6


class Live:
    """ffmpeg -> psvpack -> whole pictures (and MP2 frames), handed to fastsrv as they come.
    a: fps, rate ("12M"), title, grab ("gdigrab" / "ddagrab"), encoder ("mpeg2video" / "mpeg2_qsv"), region
    ("X,Y,WxH"), audio (bool), audio_rate ("192k")."""

    def __init__(self, a):
        self.a, self.audio = a, getattr(a, "audio", False)
        if self.audio:
            # An ffmpeg of its own for the sound, MP2, a frame per datagram to this local port (audio_reader). Not the
            # video's: ffmpeg keeps its inputs in step, so each hiccup of the capture under a game held the sound
            # back too, and each restart of the capture cut it.
            # 48000 Hz declared as 4000 and set right by asetrate: ffmpeg cuts raw audio into packets of about a tenth
            # of a second, and the MP2 frames left four at a time every 90 ms (measured; 20-30 ms apart like this).
            # The PS2 holds ~100 ms of sound: it ran dry and then had too much, twice a second
            self.asock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.asock.bind(("127.0.0.1", 0))
            arate = getattr(a, "audio_rate", "192k")
            frame = int(arate.rstrip("kK")) * 1000 * 1152 // 48000 // 8
            self.affmpeg = subprocess.Popen(
                ["ffmpeg", "-loglevel", "error", "-f", "s16le", "-ar", "4000", "-ac", "2", "-i", "pipe:0",
                 "-af", "asetrate=48000", "-c:a", "mp2", "-b:a", arate, "-flush_packets", "1", "-f", "mp2",
                 f"udp://127.0.0.1:{self.asock.getsockname()[1]}?pkt_size={frame}"],
                stdin=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        self.units, self.cur, self.at, self.ended, self.closed = collections.deque(), b"", 0, False, False
        self.lock = threading.Lock()
        self.made = self.most = self.starts = self.vwait = self.flushed = 0
        self.t0 = self.last = time.time()
        self.ffmpeg = self.pack = None
        self.vgap, self.agap, self.alast = [], [], 0  # s between pictures / sound frames since the last line (watch)
        threading.Thread(target=self.reader, daemon=True).start()
        threading.Thread(target=self.watch, daemon=True).start()
        if self.audio:
            threading.Thread(target=self.audio_reader, daemon=True).start()
            threading.Thread(target=audio_feed, args=(self,), daemon=True).start()

    def spawn(self):
        a, pre = self.a, ""
        if getattr(a, "grab", "gdigrab") == "ddagrab":  # Desktop Duplication: the frames come from the GPU
            grab, pre = ["-f", "lavfi", "-i", f"ddagrab=framerate={a.fps}:draw_mouse=1"], "hwdownload,format=bgra,"
        else:
            grab = ["-f", "gdigrab", "-framerate", str(a.fps), "-draw_mouse", "1"]
            if getattr(a, "region", None):  # one monitor, or a game's window beside something else
                x, y, size = a.region.split(",")
                grab += ["-offset_x", x, "-offset_y", y, "-video_size", size]
            grab += ["-i", f"title={a.title}"] if getattr(a, "title", None) else ["-i", "desktop"]
        # no B pictures (nothing to reorder), each picture written out as it is coded. A whole picture only every ten
        # seconds: it is ~100 KB, 40 ms and more on the PS2's network, and nothing is lost on the way that it would
        # have to repair (cuts get their own). The rate control's buffer is small: no picture much over its share
        bits = int(float(a.rate.rstrip("M")) * 1e6)
        rate = ["-b:v", str(bits), "-maxrate", str(bits), "-bufsize", str(bits // 4)]
        if getattr(a, "encoder", "mpeg2video") == "mpeg2_qsv":
            # Intel's hardware encoder: the CPU is the game's. async_depth 1: it hands a picture back before it takes
            # the next (4, its default, are 100 ms more)
            fmt, enc = "nv12", ["-c:v", "mpeg2_qsv", "-async_depth", "1", "-bf", "0", "-g", str(a.fps * 10), *rate]
        else:
            fmt = "yuv420p"
            enc = ["-c:v", "mpeg2video", "-bf", "0", "-g", str(a.fps * 10), "-dc", "10", *rate, "-flags", "+low_delay"]
        src = [*grab, "-map", "0:v",
               "-vf", pre + f"scale=1280:720:flags=bilinear:out_color_matrix=bt709:out_range=tv,format={fmt}"]
        if fmt == "nv12" and pre:
            # ddagrab + Quick Sync: captured, scaled and coded on the GPU, the picture never comes down to memory
            # (hwdownload and swscale on a whole desktop were the CPU's, and the CPU is the game's)
            src = ["-init_hw_device", "d3d11va=dx", "-init_hw_device", "qsv=qs@dx", "-filter_hw_device", "dx",
                   "-filter_complex", f"ddagrab=framerate={a.fps}:draw_mouse=1,hwmap=derive_device=qsv,"
                   "vpp_qsv=w=1280:h=720:format=nv12:out_color_matrix=bt709:out_range=tv"]
        self.ffmpeg = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-fflags", "nobuffer", *src,
             *enc, "-flush_packets", "1", "-f", "mpeg2video", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        # the Windows build if it is there (tools/build_psvpack.bat): through WSL's pipes a picture took ~30 ms more
        exe = Path(__file__).with_name("psvpack.exe")
        pack = [str(exe)] if exe.exists() else ["wsl", "./tools/psvpack"]
        self.pack = subprocess.Popen([*pack, "-", "-", str(a.fps), "1"], stdin=self.ffmpeg.stdout,
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        self.last = time.time()
        self.starts += 1

    def kill(self):
        for p in (self.ffmpeg, self.pack):
            if p:
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], capture_output=True)

    def exactly(self, n):
        b = b""
        while len(b) < n:
            d = self.pack.stdout.read(n - len(b))
            if not d:
                raise EOFError
            b += d
        return b

    def reader(self):
        """psvpack's output cut into pictures: header (the stream's first time) + 'V' chunk + 'P' chunk. When the
        capture ends or stops giving pictures (watch), it is started again and the stream goes on with its first
        picture, a whole one: the screen can't be captured while Windows shows a UAC prompt or a game takes it for
        itself (gdigrab: "Failed to capture image (error 5)"), and ffmpeg kept running on its audio input with the
        video dead, so the PS2 stayed on the last picture (console, 2026-11-15, Fortnite starting)."""
        while not self.closed:
            self.spawn()
            late = False
            try:
                unit = self.exactly(64)  # each psvpack's own header: the PS2 gets the first one only
                if self.made:
                    unit = b""
                while True:
                    head = self.exactly(8)
                    unit += head + self.exactly(struct.unpack_from("<I", head, 4)[0])
                    if head[:1] == b"P":
                        with self.lock:
                            self.units.append((1, unit))
                            self.made += 1
                            self.vwait += 1
                            self.most = max(self.most, self.vwait)
                            late = self.vwait > BEHIND
                            if late:  # see BEHIND
                                self.units.clear()
                                self.vwait, self.flushed = 0, self.flushed + 1
                        self.vgap.append(time.time() - self.last)
                        self.last = time.time()
                        unit = b""
                        if late:
                            print(f"live: the PS2 was {BEHIND} pictures behind: they are dropped", flush=True)
                            raise EOFError
            except (EOFError, OSError, ValueError):
                pass
            self.kill()
            if not self.closed and not late:
                print(f"live: the capture stopped after {self.made} pictures; starting it again", flush=True)
                time.sleep(0.5)
        self.ended = True

    def watch(self):  # a capture alive and silent for 2 s is as dead as one that left
        n = 0
        while not self.closed:
            time.sleep(0.5)
            if time.time() - self.last > 2:
                self.last = time.time()
                self.kill()
            n += 1
            if n % 10 == 0 and self.vgap:  # every 5 s: how evenly the PC makes them (a game takes its share of it)
                v, a, self.vgap, self.agap = sorted(self.vgap), sorted(self.agap), [], []
                print(f"live: {len(v)} pictures in 5 s, {sum(g > 0.05 for g in v)} of them over 50 ms after the one "
                      f"before (longest {v[-1] * 1000:.0f} ms); sound frames at most {a[-1] * 1000 if a else 0:.0f} ms apart",
                      flush=True)

    def audio_reader(self):  # an MP2 frame -> an 'A' chunk and padding to 16 bytes, once the stream has begun
        try:
            while not self.closed:
                d = self.asock.recv(4096)
                now = time.time()
                self.agap.append(now - (self.alast or now))
                self.alast = now
                if self.made:
                    pad = -len(d) % 16
                    with self.lock:
                        self.units.append((0, struct.pack("<B3xI", ord("A"), len(d)) + d +
                                           struct.pack("<B3xI", ord("P"), pad) + bytes(pad)))
        except OSError:
            pass

    def read(self, n):
        with self.lock:
            d = b""
            while len(d) < n:
                if self.at >= len(self.cur):
                    if not self.units:
                        break
                    video, self.cur = self.units.popleft()
                    self.at, self.vwait = 0, self.vwait - video
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
              f"for the PS2 to take them, the capture started {self.starts} times, {self.flushed} of them to catch up",
              flush=True)
        self.closed = self.ended = True
        self.kill()
        if self.audio:
            self.affmpeg.kill()
            self.asock.close()


def add_args(ap):
    ap.add_argument("--rate", default="12M", help="video bit rate (default 12M; the PS2's network takes ~20M)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--title", help="one window instead of the desktop")
    ap.add_argument("--region", help="X,Y,WxH of the desktop (physical pixels) instead of all of it")
    ap.add_argument("--grab", default="gdigrab", choices=["gdigrab", "ddagrab"], help="how the screen is captured")
    ap.add_argument("--encoder", default="mpeg2video", choices=["mpeg2video", "mpeg2_qsv"],
                    help="mpeg2_qsv: Intel Quick Sync, when a game needs the CPU")
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

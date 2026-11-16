"""A video as nuld-av plays it (PS2V; src/psvdec.c decodes it): <out>.psm for tools/avsrv.py, a
64-byte header "psv <pictures> <fps num> <fps den> <w> <h>", then chunks {u8 'V' or 'A', 3 x 0, u32 bytes} + payload,
one 'V' per picture with the audio (MP2 48 kHz stereo 256 kbit/s) half a second ahead of it.

The video is MPEG-2 from ffmpeg (1280x720, I/P, BT.709) repacked by tools/psvpack (C, built in WSL: gcc -O2 -o
tools/psvpack tools/psvpack.c). The rate control is ours, for the one decoder there is: the video is cut in chunks of 60
pictures, each coded at a constant quantiser, and the quantisers are the lowest that the PS2 can be fed: its network
takes NET bytes a second and its buffer holds BUF, so a hard scene may run well over the line's rate for seconds on
what the easy ones before it left in the buffer. ffmpeg's own rate control (capped -q:v, or two passes) knows nothing
of that buffer and starved exactly the hard scenes: 34 dB where a constant quantiser at the same size gave 38
(2026-11-15, tools/darkcheck.py).

usage: python tools/mkpsv.py <video> [out, default stream] [seconds] [start second]
       (run from the project folder: it calls ./tools/psvpack through WSL)
"""
import os
import shutil
import struct
import tempfile
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

AUDIO_BPS = 256000 // 8
# what the PS2's network is counted on for, KB/s. 1400: TCP through lwIP (1500-1590 measured while decoding). 2200:
# netring.irx's fast path (2026-11-15: 2500 without a datagram lost, 2900-3700 with some sent again; receive only)
NET_KBS = int(os.environ.get("PSV_NET", 2500))
# the PS2: what its network delivers while it decodes (1500-1590 KB/s measured; the stream of 2026-11-15 that asked
# 1550 for 25 s ran its buffer dry and stalled) less a margin and the audio, in bytes of MPEG-2 (PS2V is ~6% larger);
# the player's video buffer (src/nav.c VBUF, 8 MB) less a reserve; what it holds when it starts (its prebuffer, 2 MB)
NET = int((NET_KBS * 1024 - AUDIO_BPS) / 1.06)
BUF = int(6.5 * (1 << 20) / 1.06)
START = int(1.5 * (1 << 20) / 1.06)
FLOOR = 512 << 10
CHUNK = 60  # pictures: one coded whole, then 59 predicted (a cut inside gets its own: ffmpeg's scene detection)
LADDER = [2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 17, 20, 24, 31]
# -dc 10: a whole picture's block averages in steps of 2 instead of 8 (gradients); SATD motion search a hair better
ENC = ["-c:v", "mpeg2video", "-g", str(CHUNK), "-bf", "0", "-mbd", "rd", "-trellis", "2", "-dc", "10", "-cmp", "2", "-subcmp", "2"]


def run(cmd):
    subprocess.run(cmd, check=True, stderr=subprocess.DEVNULL)


def allocate(sizes, dur):
    """sizes[i][q index] -> bytes (filled in on demand by the caller's encode); returns the q index per chunk.
    The buffer, chunk by chunk: it gains NET * dur (never past BUF) and loses the chunk. Where it would fall under FLOOR,
    the chunks since it was last full that have the lowest quantiser get the next one up: the hard stretch ends at
    one quantiser throughout, as low as the buffer and the line allow."""
    def first_dry(q):  # (the first chunk the buffer runs dry at or None, the last one before it that saw it full)
        occ, full = START, -1
        for i in range(len(sizes)):
            occ += NET * dur
            if occ >= BUF:
                occ, full = BUF, i
            occ -= sizes[i](q[i])
            if occ < FLOOR:
                return i, full
        return None, full

    q = [0] * len(sizes)
    while True:
        bad, full = first_dry(q)
        if bad is None:
            break
        window = range(full, bad + 1) if full >= 0 else range(bad + 1)
        low = min(q[i] for i in window)
        if low == len(LADDER) - 1:
            break  # nothing left to give: the stretch plays at the coarsest quantiser and may stall
        up = [i for i in window if q[i] == low]
        for i in up:
            q[i] += 1
        sizes.prepare(up, q)
    # the steps up were taken by whole stretches: give back, hardest chunks first, what still fits
    # (every lower quantiser of a chunk was coded on the way up)
    for i in sorted(range(len(q)), key=lambda i: -q[i]):
        while q[i] > 0:
            q[i] -= 1
            if first_dry(q)[0] is not None:
                q[i] += 1
                break
    return q


class Chunks:
    """The chunks' encodes at each quantiser, made when asked for, several at a time."""

    def __init__(self, inter, tmp, n, frame_dur):
        self.inter, self.tmp, self.n, self.frame_dur, self.done = inter, tmp, n, frame_dur, {}

    def path(self, i, qi):
        return self.tmp / f"c{i:04d}_q{LADDER[qi]}.m2v"

    def encode(self, i, qi):
        if (i, qi) not in self.done:
            at = max(0.0, (i * CHUNK - 0.5) * self.frame_dur)  # the first picture at or after: picture i * CHUNK
            run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{at:.6f}", "-i", str(self.inter), "-frames:v", str(CHUNK),
                 "-pix_fmt", "yuv420p", *ENC, "-q:v", str(LADDER[qi]), "-qmin", "1", "-an", "-f", "mpeg2video", str(self.path(i, qi))])
            self.done[(i, qi)] = self.path(i, qi).stat().st_size

    def prepare(self, idx, q):
        with ThreadPoolExecutor(os.cpu_count() or 4) as ex:
            list(ex.map(lambda i: self.encode(i, q[i]), idx))

    def __len__(self):
        return self.n

    def __getitem__(self, i):
        def size(qi):
            self.encode(i, qi)
            return self.done[(i, qi)]
        return size


def pack_mux(m2v, mp2, psm, num, den):
    """MPEG-2 video + MP2 audio -> the .psm stream; the inputs are deleted."""
    psv = m2v.with_suffix(".psv")
    exe = Path(__file__).with_name("psvpack.exe")  # the Windows build (tools/build_psvpack.bat), else WSL's
    if exe.exists():
        subprocess.run([str(exe), str(m2v), str(psv), str(num), str(den)], check=True)
    else:
        wsl = lambda p: subprocess.run(["wsl", "wslpath", "-a", str(p).replace("\\", "/")], capture_output=True,
                                       text=True, check=True).stdout.strip()
        subprocess.run(["wsl", "./tools/psvpack", wsl(m2v), wsl(psv), str(num), str(den)], check=True)
    v, a = psv.read_bytes(), mp2.read_bytes()
    assert v[:4] == b"PSV1"
    w, h, pics = struct.unpack_from("<HHI", v, 4)
    with open(psm, "wb") as f:
        f.write(f"psv {pics} {num} {den} {w} {h}\n".encode().ljust(64, b"\0"))
        at, sent = 64, 0
        for k in range(pics):
            upto = min(len(a), int((k * den / num + 0.5) * AUDIO_BPS))
            if upto > sent:
                f.write(struct.pack("<B3xI", ord("A"), upto - sent) + a[sent:upto])
                sent = upto
            ipu, side = struct.unpack_from("<II", v, at)
            n = 16 + side + ipu
            f.write(struct.pack("<B3xI", ord("V"), n) + v[at:at + n])
            at += n
        if sent < len(a):
            f.write(struct.pack("<B3xI", ord("A"), len(a) - sent) + a[sent:])
    for t in (m2v, mp2, psv):
        t.unlink()
    secs = pics * den / num
    print(f"{psm}: {psm.stat().st_size >> 20} MB, {pics} pictures at {num}/{den}, {psm.stat().st_size * 8 / secs / 1e6:.1f} Mbit/s on average")
    return pics


def main():
    src = sys.argv[1]
    out = Path(sys.argv[2] if len(sys.argv) > 2 else "stream")
    cut = ["-t", sys.argv[3]] if len(sys.argv) > 3 else []
    start = ["-ss", sys.argv[4]] if len(sys.argv) > 4 else []
    # the intermediate and the chunks on the system's temporary folder: ~1 GB a minute of video is written and read
    # back, and on a slow disk (this project lives on an SD card) that took 15 minutes for 90 s of video instead of 2
    tmp = Path(tempfile.mkdtemp(prefix="mkpsv_"))
    inter, mp2, m2v, psm = tmp / "inter.mkv", tmp / "audio.mp2", tmp / "video.m2v", out.with_suffix(".psm")
    rate = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=r_frame_rate",
                           "-of", "csv=p=0", src], capture_output=True, text=True, check=True).stdout.strip().strip(",").split("/")
    num, den = int(rate[0]), int(rate[1] if len(rate) > 1 else 1)
    if num / den > 30.5:  # 60 fps sources: every other picture
        num, den = 30000, 1001
    # the pictures once, as the PS2 will show them (1280x720, BT.709), lossless and every one a key: the chunks seek in it
    print("scaling...", flush=True)
    run(["ffmpeg", "-y", "-loglevel", "error", *start, *cut, "-i", src,
         "-map", "0:v:0", "-vf", f"fps={num}/{den},scale=1280:720:flags=lanczos:out_color_matrix=bt709:out_range=tv,setsar=1",
         "-pix_fmt", "yuv420p", "-c:v", "utvideo", str(inter),
         "-map", "0:a:0", "-c:a", "mp2", "-b:a", "256k", "-ar", "48000", "-ac", "2", "-f", "mp2", str(mp2)])
    frames = int(subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-count_packets", "-show_entries",
                                 "stream=nb_read_packets", "-of", "csv=p=0", str(inter)], capture_output=True, text=True,
                                check=True).stdout.strip().strip(","))
    n = (frames + CHUNK - 1) // CHUNK
    chunks = Chunks(inter, tmp, n, den / num)
    print(f"{frames} pictures in {n} chunks; coding...", flush=True)
    chunks.prepare(range(n), [0] * n)
    q = allocate(chunks, CHUNK * den / num)
    with open(m2v, "wb") as f:
        for i in range(n):
            f.write(chunks.path(i, q[i]).read_bytes())
    hist = {LADDER[k]: q.count(k) for k in sorted(set(q))}
    print("quantiser -> chunks:", hist, flush=True)
    if os.environ.get("PSV_KEEP"):  # the MPEG-2 kept, for tools/darkcheck.py
        shutil.copy(m2v, out.with_suffix(".keep.m2v"))
    pics = pack_mux(m2v, mp2, psm, num, den)
    if pics != frames:
        print(f"WARNING: {pics} pictures coded of {frames}")
    shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()

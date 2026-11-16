"""How much of the dark areas' detail an encode keeps: the source (scaled as tools/mkpsv.py does) against the MPEG-2
stream, luma only, per 16x16 block. Dark: the source block's mean under 64. Prints PSNR over all blocks and over the
dark ones, and the detail kept (the encoded block's standard deviation over the source's, blocks with some texture).

usage: python tools/darkcheck.py <source> <start s> <seconds> <encode.m2v> [...more encodes]
"""
import subprocess
import sys

import numpy as np

W, H = 1280, 720
VF = "scale=1280:720:flags=lanczos:out_color_matrix=bt709:out_range=tv,setsar=1,fps=30000/1001"


def luma(args, n=None):
    raw = subprocess.run(["ffmpeg", "-loglevel", "error", *args, "-pix_fmt", "gray", "-f", "rawvideo", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, dtype=np.uint8).reshape(-1, H, W)[:n]


def blocks(a):  # frames x H x W -> frames x 45 x 80 x 256
    f = a.shape[0]
    return a.reshape(f, H // 16, 16, W // 16, 16).transpose(0, 1, 3, 2, 4).reshape(f, H // 16, W // 16, 256).astype(np.float64)


def main():
    src, ss, t = sys.argv[1:4]
    step = "select='not(mod(n,6))'"  # every 6th picture
    ref = blocks(luma(["-ss", ss, "-t", t, "-i", src, "-vf", f"{VF},{step}", "-fps_mode", "passthrough"]))
    dark = ref.mean(axis=3) < 64
    tex = ref.std(axis=3) > 2.0
    print(f"{ref.shape[0]} pictures, {100 * dark.mean():.0f}% of blocks dark, {100 * (dark & tex).mean():.0f}% dark with texture")
    for enc in sys.argv[4:]:
        e = blocks(luma(["-i", enc, "-vf", step, "-fps_mode", "passthrough"], ref.shape[0]))
        r = ref[:e.shape[0]]
        mse = ((e - r) ** 2).mean(axis=3)
        d, x = dark[:e.shape[0]], tex[:e.shape[0]]
        psnr = lambda m: 10 * np.log10(255 * 255 / m.mean())
        keep = lambda m: np.median(e.std(axis=3)[m] / r.std(axis=3)[m])
        import os
        print(f"{os.path.basename(enc):28s} {os.path.getsize(enc) * 8 / float(t) / 1e6:5.2f} Mbit/s  PSNR all {psnr(mse):.1f}  dark {psnr(mse[d]):.1f} dB"
              f"  detail kept: dark {100 * keep(d & x):.0f}%  bright {100 * keep(~d & x):.0f}%")


if __name__ == "__main__":
    main()

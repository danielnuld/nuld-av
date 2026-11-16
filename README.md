# nuld-av

720p video with sound on an unmodified PlayStation 2: 1280x720, up to 30 pictures a second, 32-bit colour, from a
stream of around 15 Mbit/s received over the network. Measured on a SCPH-75001 over component: a five-and-a-half
minute video played with every picture shown on time.

How it gets there:

- **PS2V**, the video format. The pictures are MPEG-2 (the PS2's IPU decodes nothing else at speed), repacked on a PC
  so the console never asks the IPU for a header field: per picture, a list of macroblock records read from memory
  (type, quantiser, motion vector already decoded) and a stream with only the coefficients. The IPU decodes block
  after block without a pause; a 720p picture takes ~13 ms to decode where ps2sdk's libmpeg takes ~23.
- **Rate control for the one decoder there is.** `tools/mkpsv.py` cuts the video in two-second chunks and gives each
  the lowest quantiser the console can be fed, simulating its network and its 8 MB buffer.
- **The network's fast path.** `netring.irx` hooks the Ethernet driver (ps2sdk's smap-modular-netman) and takes the
  stream's UDP datagrams straight into its buffer, around lwIP: ~2.6 MB/s where TCP gave 1.5.
- **The display.** One 32-bit 720p frame fits the GS's memory, not two: each 64-line band is redrawn right behind the
  TV's beam. YCbCr becomes RGB on the GS (BT.709), with CLUT passes.

## Using it

```c
#include <nuldav.h>

nav_source src;
if (nav_net_open(ip, 8091, 2600, &src) == 0) {   // or any source of your own: one function that gives bytes
    nav_play(&src, NULL, NULL);                  // returns when the video ends
    nav_net_close();
}
```

`include/nuldav.h` says what must be loaded first. `example/main.c` is a whole program: modules, network, play.

## Building

ps2dev with ps2sdk and its ports (libmad). From Windows with ps2dev in WSL: `wsl bash /mnt/d/ps2/nuld-av/build.sh`.
It makes `libnuldav.a`, `iop/netring/netring.irx` and `example/nuldav.elf`.

## Making and serving a video (on a PC)

```
gcc -O2 -o tools/psvpack tools/psvpack.c          (once, in WSL)
python tools/mkpsv.py video.mp4 out               (ffmpeg needed; about two minutes per minute of video)
python tools/avsrv.py out.psm
```

`tools/darkcheck.py` measures what an encode keeps of the source (PSNR, dark areas apart).

## Not there yet

- Only 1280x720 streams, I and P pictures, the default quantiser matrices.
- Sources other than the network: write a `nav_source` (a file read into a buffer is a dozen lines).
- Seeking, pausing.
- Scenes that need more than the network carries (night scenes full of lights) lose detail. Ideas, cheapest first:
  light denoising of just those scenes, B pictures, a faster decoder (frames kept as planes) to raise the rate.

## Licence

MIT (see `LICENSE`): use it in any PS2 project.

Two files keep their origin's licence: `src/psvdec.c` and `src/mc.s` derive from ps2sdk's libmpeg, Copyright (c)
2006-2007 Eugene Plotnikov, Academic Free License 2.0.

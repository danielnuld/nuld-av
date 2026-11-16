// nuld-av: 720p video with sound on an unmodified PlayStation 2.
//
// The stream is PS2V (tools/mkpsv.py makes one from any video): MPEG-2 pictures repacked so the PS2's IPU decodes
// block after block with nothing asked of it in between, plus MP2 sound. The player shows 1280x720 at up to 30
// pictures a second in 32-bit colour and keeps picture and sound together.
//
// What the program must have done before nav_play:
//   - loaded libsd.irx and audsrv.irx (sound);
//   - for nav_net_open: ps2dev9, netman, smap-modular-netman, ps2ip-nm and this library's netring.irx, with the
//     network up (an address set, by DHCP or by hand).
// nav_play takes the GS (it sets 720p and leaves the screen black: set your own mode again afterwards), the IPU, DMA
// channels 3, 4 and 9, the scratchpad and audsrv for as long as it runs.
#ifndef NULDAV_H
#define NULDAV_H

#ifdef __cplusplus
extern "C" {
#endif

// Where the stream's bytes come from. pull gives the next piece: > 0 bytes at *data, valid until the next call;
// 0 nothing yet (it is called again shortly); < 0 the end. It runs on a thread of the player's and may block.
typedef struct nav_source {
	int (*pull)(void *user, const unsigned char **data);
	void *user;
} nav_source;

typedef struct nav_stats {
	int pictures; // in the stream
	int decoded, shown;
	int dropped; // not shown: they came too late for their time
	int late;    // shown after their time
} nav_stats;

// Plays the stream to its end (or nav_stop). Blocks; the calling thread decodes. log, if given, gets a line every
// five seconds and at the start and end. 0, or: -1 not a PS2V stream, -2 out of memory, -3 not 1280x720,
// -4 the GS's memory, -5 the IPU refused a picture.
int nav_play(const nav_source *src, void (*log)(const char *line), nav_stats *out);

// The same for a live stream (a PC's screen: tools/livesrv.py): nothing is buffered and there is no clock, every
// picture is shown as soon as it is decoded and an older one still waiting is dropped for it. No sound: the
// stream's 'A' chunks are ignored and audsrv is left alone, for the program to play its own.
int nav_play_live(const nav_source *src, void (*log)(const char *line), nav_stats *out);

// Ends a nav_play under way, from another thread or an interrupt handler.
void nav_stop(void);

// A stream from tools/avsrv.py on a PC: ip in network order (a.b.c.d = a | b << 8 | c << 16 | d << 24). It comes by
// netring.irx's fast path, UDP taken at the Ethernet driver, at up to kbs KB/s (2600 is what a SCPH-75001 took
// without trouble), or by TCP if the driver can't be hooked. 0 and src filled in, or negative.
int nav_net_open(unsigned int ip, int port, int kbs, nav_source *src);
void nav_net_close(void);
unsigned int nav_net_gaps(void); // datagrams that had to be sent again so far (0 over TCP)

#ifdef __cplusplus
}
#endif

#endif

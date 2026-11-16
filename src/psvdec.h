// PS2V decoder (psvdec.c). One stream at a time; the IPU, DMA channels 3, 4 and 9 and the scratchpad are its own
// between psv_open and psv_close.
#ifndef PSVDEC_H
#define PSVDEC_H

// a picture's header in the stream (tools/psvpack.c), followed by `side` bytes of records and `ipu` bytes for the IPU
typedef struct {
	unsigned int ipu, side;
	unsigned short nrec;
	unsigned char type, flags;
	unsigned int nbdec;
} psv_pic;

// frames used in turn: a picture handed out stays whole while the next PSV_FRAMES - 1 are decoded
#define PSV_FRAMES 7

int psv_open(int w, int h); // 1, or 0 without memory (PSV_FRAMES frames of w x h x 1.5 bytes)
// decodes one picture; the frame as macroblocks row after row, each 16x16 Y, 8x8 Cb, 8x8 Cr (384 bytes), written
// behind the cache. ipu must be 16-byte aligned and flushed from the cache. NULL: the IPU found an error
void *psv_picture(const psv_pic *h, const unsigned char *side, const unsigned char *ipu);
void psv_close(void);

// EE cycles since the caller last cleared them: 0 waiting for the IPU, 1 prediction, 2 skipped macroblocks,
// 3 waiting for the reference gather, 4 cache invalidation, 5 set up
extern unsigned int psv_prof[6];

#endif

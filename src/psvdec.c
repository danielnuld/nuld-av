// PS2V decoder: a picture from macroblock records read from RAM and an IPU stream that holds only block layers
// (tools/psvpack.c has the format). While the IPU decodes macroblock n (BDEC), the EE predicts and writes n - 1 with
// the MMI routines of mc.s. Frame pictures, I and P, 4:2:0.
//
// Derived from ps2sdk's libmpeg (Copyright (c) 2006-2007 Eugene Plotnikov, Academic Free License 2.0): the
// scratchpad layout, the reference gather and mc.s are its own; the bitstream parsing through the IPU is gone (the
// EE asked it for every header field: 5-8 ms a 720p picture with the IPU idle, measured on a SCPH-75001).
#include <kernel.h>
#include <malloc.h>
#include <string.h>
#include <tamtypes.h>
#include <ee_regs.h>
#include <dma_tags.h>

#include "psvdec.h"

typedef struct { u8 y[16][16], cb[8][8], cr[8][8]; } mb_t; // a frame is these, row after row (mc.s counts on 384)

// mc.s reads these by offset
typedef struct motion {
	u8 *src;       // 0: the four macroblocks gathered around the reference block
	s16 *dst_y;    // 4
	s16 *dst_c;    // 8
	int x, y, h;   // 12 16 20: the luma block's place in them
	int interlace; // 24
	int field;     // 28
	void (*luma)(void);   // 32
	void (*chroma)(void); // 36
	int cx, cy;    // 40 44: the chroma block's place (_psv_do_mc2)
} motion_t;
typedef struct motions {
	u8 *dst_y;    // 0: the macroblock in the frame
	u8 *dst_c;    // 4
	u8 *src;      // 8: what put_block packs (the IPU's blocks, or the prediction)
	u8 *spr_blk;  // 12: the IPU's six blocks of s16
	u8 *spr_res;  // 16: the prediction as s16
	u8 *spr_mc;   // 20: the gathered reference macroblocks
	int stride;   // 24
	int n;        // 28
	void (*block_op)(struct motions *); // 32
	motion_t m[2];                      // 36
} motions_t;

extern void _psv_put_block_fr(motions_t *);
extern void _psv_add_block_frfr(motions_t *);
extern void _psv_do_mc2(motion_t *);
extern void _psv_put_luma(void), _psv_put_luma_X(void), _psv_put_luma_Y(void), _psv_put_luma_XY(void);
extern void _psv_put_chroma(void), _psv_put_chroma_X(void), _psv_put_chroma_Y(void), _psv_put_chroma_XY(void);
static void (*const luma_op[4])(void) = {_psv_put_luma, _psv_put_luma_X, _psv_put_luma_Y, _psv_put_luma_XY};
static void (*const chroma_op[4])(void) = {_psv_put_chroma, _psv_put_chroma_X, _psv_put_chroma_Y, _psv_put_chroma_XY};

#define UNCACHED(p) ((__typeof__(p))((u32)(p) | 0x20000000))
static inline u32 cyc(void) { u32 c; __asm__ volatile("mfc0 %0, $9" : "=r"(c)); return c; }

u32 psv_prof[6];
static motions_t mc[2];
static mb_t *frame[PSV_FRAMES];
static void *arena;
static struct { u8 x, y; } *mbxy;
static int mbw, mbh, cur_frame;
static qword_t ref_pack[4] __attribute__((aligned(64)));

// the default MPEG-2 matrices, in the order the stream gives them. ponytail: a stream with its own would need them
// carried in the file (psvpack refuses those today)
static const u32 qm_intra[16] __attribute__((aligned(16))) = {
	0x13101008, 0x16161310, 0x16161616, 0x1B1A181A, 0x1A1A1B1B, 0x1B1B1A1A, 0x1D1D1D1B, 0x1D222222,
	0x1B1B1D1D, 0x20201D1D, 0x26252222, 0x22232325, 0x28262623, 0x30302828, 0x38382E2E, 0x5345453A};
static const u32 qm_inter[16] __attribute__((aligned(16))) = {
	0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010,
	0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010, 0x10101010};

// both IPU channels stopped with the DMAC held, then the IPU reset
static void ipu_reset(void)
{
	DI();
	u32 e = *R_EE_D_ENABLER;
	*R_EE_D_ENABLEW = e | 0x10000;
	__asm__ volatile("sync.l");
	*R_EE_D4_CHCR &= ~0x100, *R_EE_D3_CHCR &= ~0x100, *R_EE_D4_QWC = 0;
	*R_EE_D_ENABLEW = e & ~0x10000;
	EI();
	*R_EE_IPU_CTRL = 1u << 30;
	while (*R_EE_IPU_CTRL & 0x80000000) {}
}

static void set_matrix(const u32 *m, u32 cmd)
{
	*R_EE_IPU_CMD = 0; // BCLR
	while (*R_EE_IPU_CTRL & 0x80000000) {}
	for (int i = 0; i < 4; i++)
		__asm__ volatile("lq $2, 0(%0) \n sq $2, 0(%1)" : : "r"(m + i * 4), "r"((void *)A_EE_IPU_in_FIFO) : "$2", "memory");
	*R_EE_IPU_CMD = cmd;
	while (*R_EE_IPU_CTRL & 0x80000000) {}
}

int psv_open(int w, int h)
{
	static int aw, ah;
	if (arena && (w != aw || h != ah)) free(arena), arena = NULL;
	mbw = (w + 15) / 16, mbh = (h + 15) / 16, aw = w, ah = h;
	// a row and a macroblock more per frame: the gather reads the block to the right and the one under
	u32 size = (mbw * (mbh + 1) + 1) * sizeof(mb_t);
	if (!arena) arena = memalign(64, size * PSV_FRAMES + mbw * mbh * 2); // kept after psv_close: see nav.c
	if (!arena) return 0;
	for (int i = 0; i < PSV_FRAMES; i++) frame[i] = (mb_t *)((u8 *)arena + i * size);
	mbxy = (void *)((u8 *)arena + PSV_FRAMES * size);
	for (int i = 0; i < mbw * mbh; i++) mbxy[i].x = i % mbw, mbxy[i].y = i / mbw;
	for (int i = 0; i < 2; i++) { // libmpeg's two scratchpad sets: blocks 0x300, prediction 0x300, reference 0x1800
		u8 *s = (u8 *)0x70000000 + i * 0x1E00;
		mc[i].spr_blk = s, mc[i].spr_res = s + 0x300, mc[i].spr_mc = s + 0x600;
	}
	cur_frame = 0;
	ipu_reset();
	set_matrix(qm_intra, 0x50000000);
	set_matrix(qm_inter, 0x58000000);
	return 1;
}

void psv_close(void)
{
	if (!arena) return;
	ipu_reset();
}

static int wait_bdec(void)
{
	u32 t = cyc();
	for (int spin = 0; *R_EE_D3_QWC;)
		if ((*R_EE_IPU_CTRL & 0x4000) || ++spin > 2000000) return 0; // the IPU's error flag, or stuck
	psv_prof[0] += cyc() - t;
	return 1;
}

static void predict(motions_t *m)
{
	u32 t = cyc();
	if (m->m[0].luma) _psv_do_mc2(&m->m[0]);
	m->block_op(m);
	psv_prof[1] += cyc() - t;
}

void *psv_picture(const psv_pic *h, const u8 *side, const u8 *ipu)
{
	mb_t *fwd = frame[cur_frame], *cur;
	int mba = 0, k = 0, pending = 0, prev = 0, mvx = 0, mvy = 0;
	u32 qs = 0;
	cur_frame = cur_frame + 1 >= PSV_FRAMES ? 0 : cur_frame + 1;
	cur = frame[cur_frame];

	// the input: the channel stopped (the last picture's padding may be left in it), the FIFO emptied, this
	// picture's stream started
	DI();
	u32 e = *R_EE_D_ENABLER;
	*R_EE_D_ENABLEW = e | 0x10000;
	__asm__ volatile("sync.l");
	*R_EE_D4_CHCR &= ~0x100, *R_EE_D4_QWC = 0;
	*R_EE_D_ENABLEW = e & ~0x10000;
	EI();
	// the reference is read through the cache for the skipped macroblocks, and it was written behind it: no line
	// of it may be left in the cache from the last time this buffer was a reference
	if (h->type != 1) {
		u32 t = cyc();
		InvalidDCache(fwd, (u8 *)(fwd + mbw * mbh) - 1);
		psv_prof[4] += cyc() - t;
	}
	*R_EE_IPU_CMD = 0; // BCLR
	while (*R_EE_IPU_CTRL & 0x80000000) {}
	*R_EE_IPU_CTRL = (u32)(h->flags & 3) << 16 | (u32)(h->flags >> 3 & 1) << 20 | (u32)(h->flags >> 4 & 1) << 21 |
	                 (u32)(h->flags >> 2 & 1) << 22 | (u32)h->type << 24;
	*R_EE_D4_MADR = (u32)ipu & 0x0fffffff, *R_EE_D4_QWC = h->ipu >> 4, *R_EE_D4_CHCR = 0x101;

	for (int i = 0; i < h->nrec; i++) {
		u32 fl = *side++, skip = fl >> 5, dcr = 1;
		motions_t *m = &mc[k];
		// the record
		if (skip == 7) skip = side[0] | side[1] << 8, side += 2;
		if (fl & 16) qs = *side++;
		if (skip || (fl & 1) || !(fl & 12)) mvx = mvy = 0;
		if (fl & 1) dcr = fl >> 2 & 1;
		else if ((fl & 12) == 8) {
			u32 v = *side++;
			mvx += (int)((v & 15) ^ 8) - 8, mvy += (int)((v >> 4) ^ 8) - 8;
		} else if ((fl & 12) == 12) {
			mvx = (s16)(side[0] | side[1] << 8), mvy = (s16)(side[2] | side[3] << 8);
			side += 4;
		}
		// skipped: the macroblock as it was, read through the cache (uncached reads took ~5 ms a 720p picture) and
		// written behind it like the rest of the frame
		if (skip) {
			u32 t = cyc();
			for (; skip; skip--, mba++) {
				const u8 *s = (const u8 *)(fwd + mba);
				u8 *d = UNCACHED((u8 *)(cur + mba));
				for (int o = 0; o < 384; o += 64)
					__asm__ volatile("lq $8, 0(%1) \n lq $9, 16(%1) \n lq $10, 32(%1) \n lq $11, 48(%1) \n"
					                 "sq $8, 0(%0) \n sq $9, 16(%0) \n sq $10, 32(%0) \n sq $11, 48(%0)"
					                 : : "r"(d + o), "r"(s + o) : "$8", "$9", "$10", "$11", "memory");
			}
			psv_prof[2] += cyc() - t;
		}
		if (pending && !wait_bdec()) return NULL;
		u32 t = cyc();
		m->dst_y = UNCACHED((u8 *)(cur + mba)), m->dst_c = m->dst_y + 256, m->stride = mbw * sizeof(mb_t);
		pending = fl & 3;
		if (pending) {
			*R_EE_D3_MADR = ((u32)m->spr_blk & 0x0fffffff) | 0x80000000, *R_EE_D3_QWC = 0x30, *R_EE_D3_CHCR = 0x100;
			for (int spin = 0; *R_EE_IPU_CTRL & 0x80000000;)
				if (++spin > 2000000) return NULL;
			*R_EE_IPU_CMD = 0x20000000 | (fl & 1) << 27 | dcr << 26 | qs << 16 | 1; // BDEC; FB 1: the '1' before the bits
		}
		if (fl & 1) m->m[0].luma = NULL, m->src = m->spr_blk, m->block_op = _psv_put_block_fr;
		else {
			motion_t *r = &m->m[0];
			int sx = (mbxy[mba].x << 4) + (mvx >> 1), sy = (mbxy[mba].y << 4) + (mvy >> 1);
			int bx = sx >> 4, by = sy >> 4, cx = mvx / 2, cy = mvy / 2;
			bx = bx < 0 ? 0 : bx >= mbw ? mbw - 1 : bx, by = by < 0 ? 0 : by >= mbh ? mbh - 1 : by;
			u32 from = (u32)(fwd + bx + by * mbw);
			r->dst_y = (s16 *)m->spr_res, r->dst_c = (s16 *)(m->spr_res + 512);
			r->x = sx & 15, r->y = sy & 15, r->h = 16, r->interlace = 0, r->field = 0;
			r->luma = luma_op[(mvx & 1) | (mvy & 1) << 1];
			// the chroma vector is the luma one halved towards zero (ISO 13818-2 7.6.3.7). libmpeg halves the luma
			// position instead (towards minus infinity): half a pel off for negative odd vectors, and the colour
			// drifted from the encoder's until the next I picture (Cb 59 dB -> 32 dB over 60 pictures)
			r->cx = (r->x >> 1) + (cx >> 1) - (mvx >> 2), r->cy = (r->y >> 1) + (cy >> 1) - (mvy >> 2);
			r->chroma = chroma_op[(cx & 1) | (cy & 1) << 1];
			// the reference block's macroblock, the one to its right and the two under them, to the scratchpad
			u32 tw = cyc();
			while (*R_EE_D9_CHCR & 0x100) {}
			psv_prof[3] += cyc() - tw;
			*R_EE_D9_QWC = 0, *R_EE_D9_SADR = (u32)m->spr_mc & 0x0fffffff, *R_EE_D9_TADR = (u32)ref_pack;
			qword_t *q = UNCACHED(&ref_pack[0]);
			DMATAG_REF(q, 0x30, from, 0, 0, 0); q++;
			DMATAG_REF(q, 0x30, from + mbw * sizeof(mb_t), 0, 0, 0); q++;
			DMATAG_REFE(q, 0, 0, 0, 0, 0);
			r->src = m->spr_mc;
			__asm__ volatile("sync.l");
			*R_EE_D9_CHCR = 0x105;
			if (fl & 2) m->block_op = _psv_add_block_frfr;
			else m->src = m->spr_res, m->block_op = _psv_put_block_fr;
		}
		psv_prof[5] += cyc() - t;
		if (prev) predict(&mc[!k]);
		prev = 1, k ^= 1, mba++;
	}
	if (pending && !wait_bdec()) return NULL;
	while (*R_EE_D9_CHCR & 0x100) {} // the last macroblock's reference
	if (prev) predict(&mc[!k]);
	return cur;
}

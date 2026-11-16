// psvpack: an MPEG-2 video elementary stream (I/P frame pictures, progressive, frame_pred_frame_dct)
// repacked for the PS2 as PS2V: per picture, a side stream of macroblock records the EE reads from RAM (type,
// quantiser, the motion vector already decoded) and an IPU stream with nothing but each coded macroblock's block
// layer (coded_block_pattern + coefficients), so the IPU decodes macroblock after macroblock (BDEC) and the EE never
// asks it for a header field. The picture is bit-identical to the MPEG-2 one.
//
// The IPU stream: per coded macroblock a '1' bit and then its bits. After a BDEC the IPU looks at the next 8 bits
// and, if they are zero, goes looking for a start code; an intra macroblock can begin with 8 zero bits, the '1'
// (skipped by BDEC's FB field) keeps that from ever happening.
//
// File: 64-byte header {"PSV1", u16 w, h, u32 pictures, u32 fps num, den}; per picture a 16-byte header
// {u32 ipu bytes, u32 side bytes, u16 records, u8 type (1 I, 2 P), u8 flags (idp | qst << 2 | alt scan << 3 |
// intra vlc << 4), u32 BDECs}, the side stream and the IPU stream, each padded to 16 bytes.
// Record, 1 to 8 bytes: a byte {bit 0 intra, bit 1 pattern (either: the IPU decodes it), bits 2-3 mode, bit 4 a
// quantiser byte follows, bits 5-7 macroblocks skipped before this one (7: a u16 follows with the count)}, then the
// count, the quantiser scale code and the vector, each if present. Mode, intra: bit 2 is the IPU's DC reset.
// Mode, not intra, the vector in half pels: 0 none (zero), 1 the last record's, 2 the last record's plus a byte
// {dx, dy: 4 bits each, signed}, 3 {s16 x, s16 y}. "The last record's" is zero after an intra record, a mode 0 one,
// skipped macroblocks and at the picture's start. The quantiser holds from record to record.
//
// Streaming ("-" for both: stdin to stdout, behind a live encoder): each picture goes out the moment its last
// macroblock is read, already as the player's stream (what tools/mkpsv.py writes to a .psm): once a 64-byte header
// "psv 999999999 <fps num> <den> <w> <h>", then per picture a chunk {'V', 3 x 0, u32 bytes} + the picture and an
// empty chunk {'P', 7 x 0}, which keeps every picture's end on a multiple of 16 bytes of the stream (the PS2's
// network path hands over what it has there).
//
// usage: psvpack in.m2v out.psv [fps num den]
//        psvpack - - [fps num den]
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32 // cl /O2 tools/psvpack.c (tools/build_psvpack.bat): through WSL's pipes a live picture took ~30 ms more
#include <fcntl.h>
#include <io.h>
#define read _read
typedef int ssize_t;
#else
#include <unistd.h>
#endif

static uint8_t *B;
static size_t N, bcap;
static uint64_t pos; // bits
static int streaming;
static jmp_buf starved; // streaming: the unit being read is not all here yet

static void die(const char *what)
{
	// streaming: a code that makes no sense within reach of the data's end was read with the zeros peek() pads with
	if (streaming && (pos >> 3) + 4 >= N) longjmp(starved, 1);
	fprintf(stderr, "psvpack: %s at byte %llu bit %d\n", what, (unsigned long long)(pos >> 3), (int)(pos & 7));
	exit(1);
}
static unsigned peek(int n) // n <= 24
{
	size_t by = pos >> 3;
	// streaming: out of data when the first byte asked for is missing. The up to three after it read as zero, as
	// they do at a file's end: a picture's last code (an end of block, '10') has nothing after it yet, and zeros
	// can't complete a code that isn't whole (every code ends in a 1 or is followed by one that does)
	if (streaming && by >= N) longjmp(starved, 1);
	uint32_t v = 0;
	for (int i = 0; i < 4; i++) v = v << 8 | (by + i < N ? B[by + i] : 0);
	return (uint32_t)(v << (pos & 7)) >> (32 - n);
}
static unsigned get(int n) { unsigned v = n ? peek(n) : 0; pos += n; return v; }

static int fill(void) // streaming: more from stdin; 0 at its end
{
	if (N + 65536 > bcap) B = realloc(B, bcap = bcap ? bcap * 2 : 1 << 20);
	ssize_t r = read(0, B + N, 65536);
	if (r <= 0) return 0;
	N += r;
	return 1;
}

static uint8_t *W; // the picture's IPU stream
static uint64_t wpos;
static void put(unsigned v, int n)
{
	for (int i = n - 1; i >= 0; i--, wpos++)
		if (v >> i & 1) W[wpos >> 3] |= 0x80 >> (wpos & 7);
}

// ---- the variable length codes (ISO 13818-2 annex B) ----
static int mbai(void) // B-1
{
	for (int inc = 0;;) {
		unsigned v = peek(11);
		if (v == 8) { pos += 11, inc += 33; continue; }
		if (v >= 1024) { pos += 1; return inc + 1; }
		if (v >> 8 >= 2) { pos += 3; return inc + (v >> 8 == 3 ? 2 : 3); }
		if (v >> 7 >= 2) { pos += 4; return inc + (v >> 7 == 3 ? 4 : 5); }
		if (v >> 6 >= 2) { pos += 5; return inc + (v >> 6 == 3 ? 6 : 7); }
		if (v >> 4 >= 6) { pos += 7; return inc + (v >> 4 == 7 ? 8 : 9); }
		if (v >> 3 >= 6) { pos += 8; return inc + 21 - (v >> 3); }
		if (v >> 1 >= 18) { pos += 10; return inc + 39 - (v >> 1); }
		if (v >= 24) { pos += 11; return inc + 57 - v; }
		die("macroblock_address_increment");
	}
}
enum { INTRA = 1, PATTERN = 2, MOTION = 4, DCR = 8, QUANT = 16 };
static int mbtype(int pict) // B-2, B-3
{
	unsigned v = peek(6);
	if (pict == 1) {
		if (v >> 5) { pos += 1; return INTRA; }
		if (v >> 4 == 1) { pos += 2; return INTRA | QUANT; }
	} else {
		if (v >> 5) { pos += 1; return MOTION | PATTERN; }
		if (v >> 4 == 1) { pos += 2; return PATTERN; }
		if (v >> 3 == 1) { pos += 3; return MOTION; }
		if (v >> 1 == 3) { pos += 5; return INTRA; }
		if (v >> 1 == 2) { pos += 5; return MOTION | PATTERN | QUANT; }
		if (v >> 1 == 1) { pos += 5; return PATTERN | QUANT; }
		if (v == 1) { pos += 6; return INTRA | QUANT; }
	}
	die("macroblock_type");
	return 0;
}
static int mcode(void) // B-10
{
	unsigned w = peek(10);
	int m, l;
	if (w >> 9) { pos += 1; return 0; }
	if (w >> 8) m = 1, l = 2;
	else if (w >> 7) m = 2, l = 3;
	else if (w >> 6) m = 3, l = 4;
	else if (w >> 4 == 3) m = 4, l = 6;
	else if (w >> 3 >= 3 && w >> 3 <= 5) m = 10 - (w >> 3), l = 7;
	else if (w >> 1 >= 9 && w >> 1 <= 11) m = 19 - (w >> 1), l = 9;
	else if (w >= 12 && w <= 17) m = 28 - w, l = 10;
	else { die("motion_code"); return 0; }
	pos += l;
	return get(1) ? -m : m;
}
static int mvdec(int pred, int fcode) // 7.6.3.1
{
	int r = fcode - 1, c = mcode(), d = 0;
	if (c) {
		int res = r ? get(r) : 0;
		d = ((abs(c) - 1) << r) + res + 1;
		if (c < 0) d = -d;
	}
	int v = pred + d, lim = 16 << r;
	if (v < -lim) v += 2 * lim;
	else if (v >= lim) v -= 2 * lim;
	return v;
}
static const struct { uint16_t code; uint8_t len, cbp; } cbp_tab[] = { // B-9
	{7, 3, 60}, {13, 4, 4}, {12, 4, 8}, {11, 4, 16}, {10, 4, 32}, {19, 5, 12}, {18, 5, 48}, {17, 5, 20}, {16, 5, 40},
	{15, 5, 28}, {14, 5, 44}, {13, 5, 52}, {12, 5, 56}, {11, 5, 1}, {10, 5, 61}, {9, 5, 2}, {8, 5, 62}, {15, 6, 24},
	{14, 6, 36}, {13, 6, 3}, {12, 6, 63}, {23, 7, 5}, {22, 7, 9}, {21, 7, 17}, {20, 7, 33}, {19, 7, 6}, {18, 7, 10},
	{17, 7, 18}, {16, 7, 34}, {31, 8, 7}, {30, 8, 11}, {29, 8, 19}, {28, 8, 35}, {27, 8, 13}, {26, 8, 49}, {25, 8, 21},
	{24, 8, 41}, {23, 8, 14}, {22, 8, 50}, {21, 8, 22}, {20, 8, 42}, {19, 8, 15}, {18, 8, 51}, {17, 8, 23}, {16, 8, 43},
	{15, 8, 25}, {14, 8, 37}, {13, 8, 26}, {12, 8, 38}, {11, 8, 29}, {10, 8, 45}, {9, 8, 53}, {8, 8, 57}, {7, 8, 30},
	{6, 8, 46}, {5, 8, 54}, {4, 8, 58}, {7, 9, 31}, {6, 9, 47}, {5, 9, 55}, {4, 9, 59}, {3, 9, 27}, {2, 9, 39},
};
static int cbp(void)
{
	unsigned v = peek(9);
	for (unsigned i = 0; i < sizeof cbp_tab / sizeof cbp_tab[0]; i++)
		if (v >> (9 - cbp_tab[i].len) == cbp_tab[i].code) { pos += cbp_tab[i].len; return cbp_tab[i].cbp; }
	die("coded_block_pattern");
	return 0;
}
static void coefs(int first) // B-14, only the lengths: to the end of block. first: a non-intra block's first coefficient
{
	for (;; first = 0) {
		unsigned v = peek(17);
		int z = 0, l;
		while (z < 17 && !(v >> (16 - z) & 1)) z++;
		switch (z) {
		case 0:
			if (first) l = 2;
			else if (v >> 15 & 1) l = 3;
			else { pos += 2; return; } // end of block
			break;
		case 1: l = v >> 14 & 1 ? 4 : 5; break;
		case 2: l = v >> 12 & 3 ? 6 : 9; break;
		case 3: l = 7; break;
		case 4: l = 8; break;
		case 5: l = 24; break; // escape: 6 + run 6 + level 12
		case 6: l = 11; break;
		case 7: l = 13; break;
		case 8: l = 14; break;
		case 9: l = 15; break;
		case 10: l = 16; break;
		case 11: l = 17; break;
		default: die("DCT coefficient"); return;
		}
		pos += l;
	}
}
// B-15, the other table for intra blocks (intra_vlc_format 1; hardware encoders use it): {code, bits} before the
// sign, the standard's values as libavcodec/mpeg12data.c lists them. Then its escape ('000001' + 18 bits) and its
// end of block ('0110'). Only the lengths are needed: len15[the next 16 bits] = the code's bits, 0x80 | 4 for the
// end of block, 0 for none
static const uint16_t b15[][2] = {
	{0x02, 2}, {0x06, 3}, {0x07, 4}, {0x1c, 5}, {0x1d, 5}, {0x05, 6}, {0x04, 6}, {0x7b, 7},
	{0x7c, 7}, {0x23, 8}, {0x22, 8}, {0xfa, 8}, {0xfb, 8}, {0xfe, 8}, {0xff, 8}, {0x1f, 14},
	{0x1e, 14}, {0x1d, 14}, {0x1c, 14}, {0x1b, 14}, {0x1a, 14}, {0x19, 14}, {0x18, 14}, {0x17, 14},
	{0x16, 14}, {0x15, 14}, {0x14, 14}, {0x13, 14}, {0x12, 14}, {0x11, 14}, {0x10, 14}, {0x18, 15},
	{0x17, 15}, {0x16, 15}, {0x15, 15}, {0x14, 15}, {0x13, 15}, {0x12, 15}, {0x11, 15}, {0x10, 15},
	{0x02, 3}, {0x06, 5}, {0x79, 7}, {0x27, 8}, {0x20, 8}, {0x16, 13}, {0x15, 13}, {0x1f, 15},
	{0x1e, 15}, {0x1d, 15}, {0x1c, 15}, {0x1b, 15}, {0x1a, 15}, {0x19, 15}, {0x13, 16}, {0x12, 16},
	{0x11, 16}, {0x10, 16}, {0x05, 5}, {0x07, 7}, {0xfc, 8}, {0x0c, 10}, {0x14, 13}, {0x07, 5},
	{0x26, 8}, {0x1c, 12}, {0x13, 13}, {0x06, 6}, {0xfd, 8}, {0x12, 12}, {0x07, 6}, {0x04, 9},
	{0x12, 13}, {0x06, 7}, {0x1e, 12}, {0x14, 16}, {0x04, 7}, {0x15, 12}, {0x05, 7}, {0x11, 12},
	{0x78, 7}, {0x11, 13}, {0x7a, 7}, {0x10, 13}, {0x21, 8}, {0x1a, 16}, {0x25, 8}, {0x19, 16},
	{0x24, 8}, {0x18, 16}, {0x05, 9}, {0x17, 16}, {0x07, 9}, {0x16, 16}, {0x0d, 10}, {0x15, 16},
	{0x1f, 12}, {0x1a, 12}, {0x19, 12}, {0x17, 12}, {0x16, 12}, {0x1f, 13}, {0x1e, 13}, {0x1d, 13},
	{0x1c, 13}, {0x1b, 13}, {0x1f, 16}, {0x1e, 16}, {0x1d, 16}, {0x1c, 16}, {0x1b, 16}, {0x01, 6},
};
static uint8_t len15[65536];
static void b15_init(void)
{
	for (unsigned i = 0; i < sizeof b15 / sizeof b15[0] + 2; i++) {
		unsigned code = i < 111 ? b15[i][0] : i == 111 ? 1 : 6, bits = i < 111 ? b15[i][1] : i == 111 ? 6 : 4;
		unsigned len = i < 111 ? bits + 1 : i == 111 ? 24 : 0x80 | 4;
		for (unsigned v = code << (16 - bits); v < (code + 1) << (16 - bits); v++) len15[v] = len;
	}
}
static void coefs15(void)
{
	for (;;) {
		unsigned l = len15[peek(16)];
		if (!l) { die("DCT coefficient (B-15)"); return; }
		if (l & 0x80) { pos += 4; return; }
		pos += l;
	}
}

static void dc(int chroma) // B-12, B-13 and the differential
{
	unsigned v = peek(10);
	int ones = 0, size;
	while (ones < 10 && (v >> (9 - ones) & 1)) ones++;
	if (!chroma) {
		if (ones == 0) pos += 2, size = v >> 8 & 1 ? 2 : 1;
		else if (ones == 1) pos += 3, size = v >> 7 & 1 ? 3 : 0;
		else if (ones == 2) pos += 3, size = 4;
		else if (ones >= 9) pos += 9, size = 11;
		else pos += ones + 1, size = ones + 2;
	} else {
		if (ones == 0) pos += 2, size = v >> 8 & 1 ? 1 : 0;
		else if (ones == 1) pos += 2, size = 2;
		else if (ones >= 10) pos += 10, size = 11;
		else pos += ones + 1, size = ones + 1;
	}
	pos += size;
}

// the next start code at or after pos: its value, pos after it and *at where it starts; -1 if there is none (yet)
static int find_start(size_t *at)
{
	for (size_t i = (pos + 7) >> 3; i + 4 <= N; i++)
		if (!B[i] && !B[i + 1] && B[i + 2] == 1) { *at = i, pos = (uint64_t)(i + 4) << 3; return B[i + 3]; }
	return -1;
}

enum { CAP = 4 << 20 };
static FILE *o;
static uint8_t *side;
static uint32_t fps_num = 30000, fps_den = 1001, wmax = CAP;
static uint64_t out_side, out_ipu, all_bdec, all_mb;
static int pics;
// what a unit read only in part must leave as it was (streaming)
static struct state {
	int w, h, mbw, mbh, pict, fcode[2], flags, open;
	uint32_t ns, nrec, nbdec;
	int last, total, sent_qs, rp[2]; // last coded macroblock's address; macroblocks seen; what the reader holds
	uint64_t wpos;
} S, saved;
static size_t unit_at;

static void emit(void) // the picture is whole: out it goes
{
	put(1, 1);
	while (wpos & 7) put(1, 1);
	uint32_t ni = wpos >> 3;
	while (ni & 15) W[ni++] = 0xFF;
	memset(W + ni, 0xFF, 16), ni += 16; // the IPU reads ahead of the last macroblock
	while (S.ns & 15) side[S.ns++] = 0;
	uint8_t ph[16];
	uint16_t r16 = S.nrec;
	memcpy(ph, &ni, 4), memcpy(ph + 4, &S.ns, 4), memcpy(ph + 8, &r16, 2);
	ph[10] = S.pict, ph[11] = S.flags;
	memcpy(ph + 12, &S.nbdec, 4);
	if (streaming) {
		uint8_t ck[8] = {'V'}, pad[8] = {'P'};
		uint32_t n = 16 + S.ns + ni;
		if (!pics) {
			char hd[64] = {0};
			snprintf(hd, sizeof hd, "psv 999999999 %u %u %d %d\n", fps_num, fps_den, S.w, S.h);
			fwrite(hd, 1, 64, o);
		}
		memcpy(ck + 4, &n, 4);
		fwrite(ck, 1, 8, o);
		fwrite(ph, 1, 16, o), fwrite(side, 1, S.ns, o), fwrite(W, 1, ni, o);
		fwrite(pad, 1, 8, o);
		fflush(o);
	} else
		fwrite(ph, 1, 16, o), fwrite(side, 1, S.ns, o), fwrite(W, 1, ni, o);
	out_side += S.ns, out_ipu += ni, all_bdec += S.nbdec, all_mb += S.total, pics++, S.open = 0;
	wmax = ni + 64;
}

static void unit(int code)
{
	if (code == 0xB3) { // sequence header
		S.w = get(12), S.h = get(12);
		get(8), get(18), get(1), get(10), get(1);
		if (get(1)) die("custom intra matrix: not handled yet");
		if (get(1)) die("custom non-intra matrix: not handled yet");
		S.mbw = (S.w + 15) / 16, S.mbh = (S.h + 15) / 16;
	} else if (code == 0xB5) { // extensions
		int id = get(4);
		if (id == 1) {
			get(8);
			if (!get(1)) die("not a progressive sequence");
		} else if (id == 3) die("quant matrix extension: not handled yet");
		else if (id == 8) {
			int f0 = get(4), f1 = get(4);
			get(8);
			int idp = get(2), ps = get(2);
			get(1);
			int fpfd = get(1), cmv = get(1), qst = get(1), ivf = get(1), as = get(1);
			if (ps != 3 || !fpfd || cmv) die("only frame pictures with frame_pred_frame_dct and no concealment vectors");
			S.fcode[0] = f0, S.fcode[1] = f1, S.flags = idp | qst << 2 | as << 3 | ivf << 4;
		}
	} else if (code == 0x00) { // picture header
		get(10);
		int pict = get(3);
		if (pict != 1 && pict != 2) die("only I and P pictures");
		memset(W, 0, wmax);
		wpos = 0;
		S.pict = pict, S.ns = S.nrec = S.nbdec = 0, S.last = -1, S.total = 0, S.open = 1, S.sent_qs = -1, S.rp[0] = S.rp[1] = 0;
	} else if (code >= 1 && code <= 0xAF && S.open) { // a slice
		int qs = get(5);
		if (get(1)) { get(8); while (get(1)) get(8); }
		int mba = (code - 1) * S.mbw + mbai() - 1, pmv[2] = {0, 0}, prev_intra = 0;
		for (;;) {
			if (mba >= S.mbw * S.mbh) die("macroblock address past the picture");
			int t = mbtype(S.pict), mv[2] = {0, 0};
			if (t & QUANT) qs = get(5);
			if (t & MOTION) {
				mv[0] = pmv[0] = mvdec(pmv[0], S.fcode[0]);
				mv[1] = pmv[1] = mvdec(pmv[1], S.fcode[1]);
			}
			uint64_t from = pos;
			if (t & INTRA) {
				pmv[0] = pmv[1] = 0;
				for (int b = 0; b < 6; b++) {
					dc(b >= 4);
					if (S.flags & 16) coefs15();
					else coefs(0);
				}
			} else if (t & PATTERN) {
				for (int c = cbp(); c; c &= c - 1) coefs(1);
			}
			if (S.pict == 2 && !(t & (MOTION | INTRA))) pmv[0] = pmv[1] = 0;
			if (S.ns + 8 > CAP || (wpos >> 3) + 4096 > CAP) die("picture too large");
			uint16_t skip = mba - S.last - 1;
			uint8_t *r = side + S.ns++;
			*r = (t & (INTRA | PATTERN)) | (skip < 7 ? skip : 7) << 5;
			if (skip >= 7) memcpy(side + S.ns, &skip, 2), S.ns += 2;
			if ((t & (INTRA | PATTERN)) && qs != S.sent_qs) *r |= 16, side[S.ns++] = S.sent_qs = qs;
			if (skip) S.rp[0] = S.rp[1] = 0;
			if (t & INTRA) {
				if (!prev_intra) *r |= 4;
				S.rp[0] = S.rp[1] = 0;
			} else if (!(t & MOTION)) S.rp[0] = S.rp[1] = 0; // mode 0
			else {
				int dx = mv[0] - S.rp[0], dy = mv[1] - S.rp[1];
				if (!dx && !dy) *r |= 1 << 2;
				else if (dx >= -8 && dx < 8 && dy >= -8 && dy < 8) *r |= 2 << 2, side[S.ns++] = (dx & 15) | (dy & 15) << 4;
				else {
					int16_t m[2] = {mv[0], mv[1]};
					*r |= 3 << 2, memcpy(side + S.ns, m, 4), S.ns += 4;
				}
				S.rp[0] = mv[0], S.rp[1] = mv[1];
			}
			S.nrec++;
			if (t & (INTRA | PATTERN)) {
				put(1, 1);
				uint64_t save = pos;
				for (pos = from; pos < save;) {
					int k = save - pos > 24 ? 24 : (int)(save - pos);
					put(get(k), k);
				}
				S.nbdec++;
			}
			S.total += mba - S.last, S.last = mba, prev_intra = t & INTRA;
			if (S.total == S.mbw * S.mbh) { emit(); return; } // the picture's last macroblock: nothing after it yet
			if (streaming && (pos >> 3) + 3 > N) longjmp(starved, 1); // the 23 bits below must be the stream's own
			if (!peek(23)) break; // the slice ends
			int inc = mbai();
			if (inc > 1) pmv[0] = pmv[1] = 0, prev_intra = 0;
			mba += inc;
		}
	}
}

int main(int argc, char **argv)
{
	if (argc < 3) { fprintf(stderr, "usage: psvpack in.m2v out.psv [fps num den]   (- - : stdin to stdout, streaming)\n"); return 1; }
	streaming = !strcmp(argv[1], "-");
	if (argc > 4) fps_num = atoi(argv[3]), fps_den = atoi(argv[4]);
	uint8_t head[64] = "PSV1";
	if (streaming) {
#ifdef _WIN32
		_setmode(0, _O_BINARY), _setmode(1, _O_BINARY);
#endif
		o = stdout;
	} else {
		FILE *f = fopen(argv[1], "rb");
		if (!f) { perror(argv[1]); return 1; }
		fseek(f, 0, SEEK_END);
		N = ftell(f);
		fseek(f, 0, SEEK_SET);
		B = malloc(N + 8);
		if (fread(B, 1, N, f) != N) return 1;
		fclose(f);
		memset(B + N, 0, 8);
		o = fopen(argv[2], "wb");
		if (!o) { perror(argv[2]); return 1; }
		fwrite(head, 1, 64, o);
	}
	W = malloc(CAP + 64), side = malloc(CAP);
	b15_init();
	for (;;) {
		if (streaming && (pos >> 3) > (4u << 20)) { // what was read, dropped
			size_t k = pos >> 3;
			memmove(B, B + k, N - k), N -= k, pos -= (uint64_t)k << 3;
		}
		int code = find_start(&unit_at);
		if (code < 0) {
			if (streaming && fill()) continue;
			break;
		}
		if (S.open && (code == 0 || code == 0xB3 || code == 0xB7 || code == 0xB8)) die("picture without all its macroblocks");
		saved = S, saved.wpos = wpos;
		if (streaming && setjmp(starved)) { // the unit was not all here: as it was, and again with more
			if (wpos > saved.wpos) {
				size_t a = saved.wpos >> 3, b = (wpos + 7) >> 3;
				W[a] &= (uint8_t)~(0xFF >> (saved.wpos & 7));
				if (b > a + 1) memset(W + a + 1, 0, b - a - 1);
			}
			S = saved, wpos = saved.wpos, pos = (uint64_t)unit_at << 3;
			if (!fill()) break;
			continue;
		}
		unit(code);
	}
	if (S.open) die("the last picture is not whole");
	if (streaming) return 0;
	uint16_t w16 = S.w, h16 = S.h;
	uint32_t p32 = pics;
	memcpy(head + 4, &w16, 2), memcpy(head + 6, &h16, 2), memcpy(head + 8, &p32, 4);
	memcpy(head + 12, &fps_num, 4), memcpy(head + 16, &fps_den, 4);
	fseek(o, 0, SEEK_SET);
	fwrite(head, 1, 64, o);
	fclose(o);
	printf("%dx%d, %d pictures: %zu bytes of MPEG-2 -> side %llu + IPU %llu = %llu bytes; %.0f%% of macroblocks need the IPU\n",
	       S.w, S.h, pics, N, (unsigned long long)out_side, (unsigned long long)out_ipu,
	       (unsigned long long)(out_side + out_ipu + 16ull * pics + 64), all_mb ? 100.0 * all_bdec / all_mb : 0.0);
	return 0;
}

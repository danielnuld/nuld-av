// nuld-av's network source (include/nuldav.h): netring.irx receives on the IOP and DMAs into a ring here; this side
// only reads the ring and tells the IOP now and then how far it got. iop/netring/netring.c has the rest.
#include <kernel.h>
#include <sifrpc.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "netring.h"
#include "nuldav.h"

static SifRpcClientData_t cd;
static u32 buf[16] __attribute__((aligned(64)));
static u8 ring[512 << 10] __attribute__((aligned(64)));
static nr_status_t st_mem[4] __attribute__((aligned(64))); // a cache line of its own: read behind the cache
#define ST ((volatile nr_status_t *)((u32)st_mem | 0x20000000))
static u32 rd, acked;
static int open_;

static int call(int fn, int len)
{
	return SifCallRpc(&cd, fn, 0, buf, len, buf, 16, NULL, NULL) < 0 ? -100 : (int)buf[0];
}

static int pull(void *user, const unsigned char **data)
{
	(void)user;
	if (rd - acked >= 64 << 10) buf[0] = acked = rd, call(NR_ACK, 4); // what the last calls handed out is consumed
	u32 w = ST->write;
	if (w == rd) return ST->done ? -1 : 0;
	u32 o = rd % sizeof ring, n = w - rd;
	if (n > sizeof ring - o) n = sizeof ring - o;
	if (n > 16 << 10) n = 16 << 10;
	InvalidDCache(ring + (o & ~63), ring + ((o + n + 63) & ~63) - 1); // the IOP DMAed it behind the cache
	*data = ring + o, rd += n;
	return n;
}

// the stream started with one of two requests (the second, if any, when the IOP refuses the first)
static int start(unsigned int ip, int port, const char *req, const char *or_req, nav_source *src)
{
	if (!cd.server) {
		for (int i = 0; i < 1000 && !cd.server; i++) {
			if (SifBindRpc(&cd, NETRING_RPC, 0) < 0) return -1;
			if (!cd.server) usleep(1000);
		}
		if (!cd.server) return -1; // netring.irx is not loaded
	}
	if (open_) nav_net_close();
	nr_start_t s;
	memset(&s, 0, sizeof s);
	s.ip = ip, s.port = (port >> 8 & 255) | (port & 255) << 8;
	s.ring = (u32)ring & 0x1fffffff, s.size = sizeof ring, s.status = (u32)st_mem & 0x1fffffff;
	ST->write = 0, ST->done = 0, ST->err = 0, rd = acked = 0;
	snprintf(s.req, sizeof s.req, "%s", req);
	memcpy(buf, &s, sizeof s);
	int r = call(NR_START, sizeof s);
	if (r < 0 && or_req) {
		snprintf(s.req, sizeof s.req, "%s", or_req);
		memcpy(buf, &s, sizeof s);
		r = call(NR_START, sizeof s);
	}
	if (r < 0) return -2; // nobody answers there
	open_ = 1, src->pull = pull, src->user = NULL;
	return 0;
}

int nav_net_open(unsigned int ip, int port, int kbs, nav_source *src)
{
	// the fast path; plain TCP if the module can't hook the Ethernet driver (smap.irx instead of the modular one).
	// The number at the end tells this request from the last one's repeats: the IOP says it again until the stream
	// starts, and the server must not take those for a new stream
	char req[44];
	u32 once;
	__asm__ volatile("mfc0 %0, $9" : "=r"(once));
	snprintf(req, sizeof req, "fast %d play %u\n", kbs, once);
	return start(ip, port, req, "play\n", src);
}

int nav_net_tcp(unsigned int ip, int port, const char *req, nav_source *src)
{
	return start(ip, port, req, NULL, src);
}

void nav_net_close(void)
{
	if (open_) call(NR_STOP, 0), open_ = 0;
}

unsigned int nav_net_gaps(void) { return ST->err; }

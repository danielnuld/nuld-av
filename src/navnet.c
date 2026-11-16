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

int nav_net_open(unsigned int ip, int port, int kbs, nav_source *src)
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
	// the fast path; plain TCP if the module can't hook the Ethernet driver (smap.irx instead of the modular one)
	snprintf(s.req, sizeof s.req, "fast %d play\n", kbs);
	memcpy(buf, &s, sizeof s);
	int r = call(NR_START, sizeof s);
	if (r < 0) {
		strcpy(s.req, "play\n");
		memcpy(buf, &s, sizeof s);
		r = call(NR_START, sizeof s);
	}
	if (r < 0) return -2; // nobody answers there
	open_ = 1, src->pull = pull, src->user = NULL;
	return 0;
}

void nav_net_close(void)
{
	if (open_) call(NR_STOP, 0), open_ = 0;
}

unsigned int nav_net_gaps(void) { return ST->err; }

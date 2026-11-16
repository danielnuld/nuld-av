// netring: a TCP stream received on the IOP (its own lwIP, ps2ip-nm.irx) and pushed into a ring in EE memory by SIF
// DMA in 32 KB chunks. The EE runs no TCP and makes no call per packet: it reads the write position the IOP DMAs
// after each chunk and gives back what it consumed now and then (NR_ACK). Shared by iop/netring/netring.c and the EE.
#ifndef NETRING_H
#define NETRING_H

#define NETRING_RPC 0x4e52494e // "NRIN"
#define NETRING_CHUNK (32 << 10)

enum { NR_START = 1, NR_ACK, NR_STOP };

// NR_START's argument, 64 bytes. Addresses are EE physical, 64-byte aligned; size is a multiple of NETRING_CHUNK.
// ip / port in network order (sockaddr_in's); req is sent to the server once connected (netsrv.py: "KB/s\n")
typedef struct {
	unsigned int ip, port, ring, size, status;
	char req[44];
} nr_start_t;

// what the IOP DMAs to `status` after each chunk (16 bytes): bytes written so far (ring offset = write % size), and
// done once the server closed or recv failed (err: the recv result then)
typedef struct {
	unsigned int write, done;
	int err;
	unsigned int pad;
} nr_status_t;

#endif

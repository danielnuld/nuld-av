// nuld-av's example: plays what tools/avsrv.py serves on the PC named in config.ini. The modules a program must load
// and the network set-up are here; the playing is three calls (nav_net_open, nav_play, nav_net_close).
//
// On a USB stick: this ELF anywhere, and a folder ipubench/ with config.ini:
//   [red]   ip = dhcp            (or ip / mascara / puerta for a fixed address)
//   [link]  servidor = 192.168.1.10
// What the player says is written to ipubench/results.txt.
#include <debug.h>
#define NEWLIB_PORT_AWARE
#include <fileXio_rpc.h>
#include <iopcontrol.h>
#include <kernel.h>
#include <loadfile.h>
#include <netman.h>
#include <ps2ips.h>
#include <sbv_patches.h>
#include <sifrpc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "ini.h"
#include "nuldav.h"

static const char *dir = "mass0:/ipubench/"; // or host:ipubench/ next to the ELF (PCSX2 without a stick)
static char path[64];
static const char *in_dir(const char *name) { snprintf(path, sizeof path, "%s%s", dir, name); return path; }
#define IRX(m) extern unsigned char m##_irx[]; extern unsigned int size_##m##_irx
IRX(iomanX); IRX(fileXio); IRX(bdm); IRX(bdmfs_fatfs); IRX(usbd_mini); IRX(usbmass_bd_mini);
IRX(libsd); IRX(audsrv); IRX(ps2dev9); IRX(netman); IRX(smap); IRX(ps2ip_nm); IRX(ps2ips); IRX(netring);

static int load(unsigned char *irx, unsigned int size)
{
	int r = 0;
	return SifExecModuleBuffer(irx, size, 0, NULL, &r) >= 0 && r != 1;
}

static void logline(const char *line)
{
	// closed after every line: on a FAT stick the file's size is only written on close
	FILE *f = fopen(in_dir("results.txt"), "a");
	if (f) fputs(line, f), fclose(f);
}

static u32 aton(const char *s)
{
	unsigned a = 0, b = 0, c = 0, d = 0;
	sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d);
	return a | b << 8 | c << 16 | d << 24;
}

static int wait_for(int (*ok)(void))
{
	for (int t = 0; t < 10 && !ok(); t++) sleep(1);
	return ok();
}
static int link_up(void) { return NetManIoctl(NETMAN_NETIF_IOCTL_GET_LINK_STATUS, NULL, 0, NULL, 0) == NETMAN_NETIF_ETH_LINK_STATE_UP; }
static int dhcp_bound(void)
{
	t_ip_info ip;
	return ps2ip_getconfig("sm0", &ip) >= 0 && ip.dhcp_status == DHCP_STATE_BOUND;
}

int main(void)
{
	static ini cfg;
	char line[160];
	while (!SifIopReset("", 0)) {}
	while (!SifIopSync()) {}
	SifInitRpc(0);
	sbv_patch_enable_lmb();
	sbv_patch_disable_prefix_check();
	init_scr();
	scr_printf("nuld-av example\n");
	// the stick (for config.ini and the log), sound, then the network: the Ethernet driver in its modular build
	// (smap-modular-netman.irx), which netring.irx hooks for the fast path
	int ok = load(iomanX_irx, size_iomanX_irx) && load(fileXio_irx, size_fileXio_irx);
	if (ok) fileXioInit();
	ok = ok && load(bdm_irx, size_bdm_irx) && load(bdmfs_fatfs_irx, size_bdmfs_fatfs_irx) && load(usbd_mini_irx, size_usbd_mini_irx) &&
	     load(usbmass_bd_mini_irx, size_usbmass_bd_mini_irx) && load(libsd_irx, size_libsd_irx) && load(audsrv_irx, size_audsrv_irx) &&
	     load(ps2dev9_irx, size_ps2dev9_irx) && load(netman_irx, size_netman_irx) && load(smap_irx, size_smap_irx) &&
	     load(ps2ip_nm_irx, size_ps2ip_nm_irx) && load(ps2ips_irx, size_ps2ips_irx) && load(netring_irx, size_netring_irx);
	if (!ok) { scr_printf("a module did not load\n"); SleepThread(); }
	for (int i = 0; i < 10 && !ini_load(&cfg, in_dir("config.ini")); i++) { // the stick mounts a while after
		if (i == 3) dir = "host:ipubench/";
		else sleep(1);
	}
	const char *srv = ini_get(&cfg, "link", "servidor", ""), *ip = ini_get(&cfg, "red", "ip", "dhcp");
	if (!*srv) { scr_printf("no ipubench/config.ini with [link] servidor\n"); SleepThread(); }
	remove(in_dir("results.txt"));

	NetManInit();
	int dhcp = !strcasecmp(ip, "dhcp");
	t_ip_info info;
	if (ps2ip_init() < 0 || ps2ip_getconfig("sm0", &info) < 0) { scr_printf("no network stack\n"); SleepThread(); }
	info.dhcp_enabled = dhcp;
	if (!dhcp) {
		u32 a = aton(ip), k = aton(ini_get(&cfg, "red", "mascara", "255.255.255.0")), g = aton(ini_get(&cfg, "red", "puerta", ""));
		memcpy(&info.ipaddr, &a, 4), memcpy(&info.netmask, &k, 4), memcpy(&info.gw, &g, 4);
	}
	ps2ip_setconfig(&info);
	if (!wait_for(link_up) || (dhcp && !wait_for(dhcp_bound))) { scr_printf("no link or no address\n"); SleepThread(); }

	nav_source src;
	nav_stats st = {0};
	int r = nav_net_open(aton(srv), 8091, 2600, &src);
	if (r < 0) snprintf(line, sizeof line, "no stream from %s:8091 (tools/avsrv.py): %d\n", srv, r);
	else {
		// [link] vivo = 1: the PC's screen from tools/livesrv.py, shown as it comes; else a video from tools/avsrv.py
		r = (atoi(ini_get(&cfg, "link", "vivo", "0")) ? nav_play_live : nav_play)(&src, logline, &st);
		snprintf(line, sizeof line, "nav_play %d: %d of %d pictures shown, %d dropped, %d late; %u datagrams sent again\n", r,
		         st.shown, st.pictures, st.dropped, st.late, nav_net_gaps());
		nav_net_close();
	}
	logline(line);
	init_scr(); // the player left 720p and a black screen
	scr_printf("%s", line);
	SleepThread();
	return 0;
}

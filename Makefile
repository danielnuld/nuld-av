# nuld-av: libnuldav.a (the player), iop/netring/netring.irx (the network receiver) and example/nuldav.elf.
# From Windows: wsl bash /mnt/d/ps2/nuld-av/build.sh   (tools/psvpack: gcc -O2 -o tools/psvpack tools/psvpack.c)
EE_LIB = libnuldav.a
EE_OBJS = src/nav.o src/navnet.o src/psvdec.o src/mc.o
EE_INCS += -Iinclude -Isrc -I$(PS2SDK)/ports/include # libmad: ps2sdk ports
EE_CFLAGS += -O2 -Wall

EX_IRX = iomanX fileXio bdm bdmfs_fatfs usbd_mini usbmass_bd_mini libsd audsrv ps2dev9 netman smap ps2ip_nm ps2ips netring
EX_OBJS = example/main.o example/ini.o $(EX_IRX:%=example/%_irx.o)
EX_LIBS = -L. -lnuldav -L$(PS2SDK)/ports/lib -lmad -laudsrv -lnetman -lps2ips -lfileXio -lpatches -ldraw -lgraph -ldma -ldebug -lc

all: $(EE_LIB) example/nuldav.elf

example/nuldav.elf: $(EX_OBJS) $(EE_LIB)
	$(EE_CC) -T$(EE_LINKFILE) -O2 -o $@ $(EX_OBJS) $(EE_LDFLAGS) $(EX_LIBS)
	$(EE_STRIP) --strip-all $@

example/%.o: example/%.c
	$(EE_CC) $(EE_CFLAGS) $(EE_INCS) -Iexample -c $< -o $@

src/nav.o: include/nuldav.h src/psvdec.h
src/navnet.o: include/nuldav.h include/netring.h
src/psvdec.o: src/psvdec.h

# the modules the example embeds
example/%_irx.c:
	$(PS2SDK)/bin/bin2c $(PS2SDK)/iop/irx/$*.irx $@ $*_irx
example/ps2ip_nm_irx.c: # the file has a dash, a C name can't
	$(PS2SDK)/bin/bin2c $(PS2SDK)/iop/irx/ps2ip-nm.irx $@ ps2ip_nm_irx
example/smap_irx.c: # the modular build: netring.irx hooks its receive path
	$(PS2SDK)/bin/bin2c $(PS2SDK)/iop/irx/smap-modular-netman.irx $@ smap_irx
example/netring_irx.c: iop/netring/netring.irx
	$(PS2SDK)/bin/bin2c $< $@ netring_irx
iop/netring/netring.irx: iop/netring/netring.c include/netring.h
	$(MAKE) -C iop/netring

clean:
	rm -f $(EE_LIB) src/*.o example/*.o example/*_irx.c example/*.elf
	$(MAKE) -C iop/netring clean

include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal

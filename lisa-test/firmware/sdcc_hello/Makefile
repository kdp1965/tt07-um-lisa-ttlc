# Builds sdcc_hello.ihx with the SDCC LISA port and copies it next to the
# other demo images (../sdcc_hello.ihx), where web/build_assets.py picks it up.
SDCC ?= ../../../../lisa-tools/sdcc-lisa/build/bin/sdcc
SIM  ?= ../../../../lisa-tools/lisa_sim/lisa_sim

all: ../sdcc_hello.ihx

../sdcc_hello.ihx: sdcc_hello.ihx
	cp $< $@

sdcc_hello.ihx: sdcc_hello.c $(wildcard ../../../../lisa-tools/sdcc-lisa/build/device/lib/build/lisa/*.rel)
	$(SDCC) -mlisa -o $@ $<

# quick check on the simulator: the banner, then the answer to 's'
sim: sdcc_hello.ihx
	printf 'step 200000\nuart ?\nstep 400000\nuart s\nstep 200000\nquit\n' | $(SIM) $< > /dev/null; \
	echo "(use the socat pty bridge or lisa_sim -r to see the output)"

clean:
	rm -f *.ihx *.asm *.lst *.rel *.sym *.map *.lk *.cdb *.noi

.PHONY: all sim clean

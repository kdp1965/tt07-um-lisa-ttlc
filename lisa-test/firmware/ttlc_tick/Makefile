# ttlc_tick: LISA program that clocks the TTLC (see ttlc_tick.S).
# Needs the lisa-tools toolchain next to this repository (or set LISA_TOOLS).
LISA_TOOLS ?= ../../../../lisa-tools
AS = $(LISA_TOOLS)/lisa_as/lisa_as
LD = $(LISA_TOOLS)/lisa_ld/lisa_ld
ASFLAGS = -w16 -7

PROG = ttlc_tick

all: $(PROG)

# the linker's output is the image LISA Commander loads: one hex opcode per line
$(PROG): $(PROG).rel isr_test.rel
	$(LD) -T lisa.ld -M -o $@ -t isr_test.rel $<

$(PROG).rel: $(PROG).S
	$(AS) $(ASFLAGS) -o $@ $<

isr_test.rel: isr_test.S
	$(AS) $(ASFLAGS) -o $@ $<

clean:
	rm -f $(PROG) $(PROG).rel $(PROG).lst $(PROG).map isr_test.rel

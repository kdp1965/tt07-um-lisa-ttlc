'''
ttlc_sim.py - RP2040 emulation of the TTLC's external shift-register I/O.

The TTLC (MC14500B logic controller inside tt_um_lisa) talks to 48 outputs
and 48 inputs through 74HC595 / 74HC166 shift-register chains: three serial
data lines in each direction, a shift clock and a latch.  With debug register
0x1b routing those signals to uo_out, they land on RP2040 pins of the TT06/TT07
demo board, so two PIO state machines can play the shift registers:

   uo_out[0..2] = GPIO 5, 6, 7   serial data out (outputs 0-15, 16-31, 32-47)
   uo_out[5]    = GPIO 14        latch (idles high; low pulse before and after a scan)
   uo_out[6]    = GPIO 15        shift clock
   ui_in[4..6]  = GPIO 17,18,19  serial data in (inputs 0-15, 16-31, 32-47)

A scan is 16 shift clocks; bit 15 of each segment goes first and data is
sampled on the rising edge.  The "capture" machine samples the three data-out
lines on every rising edge and pushes 24 bits twice per scan.

The TT07 silicon's input shifter (shift_reg_io.v) shifts 17 times per scan:
once per rising edge and once more after the last falling edge.  What it
keeps is the last 16 samples, so by default the "feed" machine presents bit
15 after the FIRST falling edge (the sample taken at the first rising edge is
the one that falls off the end) and leaves bit 0 on the pins for that 17th
sample - the same thing an extra D flip-flop in front of a 74HC165 chain would
do.  start(compensate=False) uses the plain 74HC165 timing instead, for RTL
with the 16-sample scan (an FPGA build with the fix); it only matters when
the emulator is started, not while it runs.  DMA ring
buffers keep both FIFOs served without the CPU: the outputs always sit in
`out_ring`, and whatever `set_inputs()` wrote to `in_ring` is replayed on
every scan.  The web app sends this file to RAM next to lisa_flash.py.
'''
import rp2
import uctypes
from machine import Pin, Timer

_LATCH = 14
_SCLK = 15
_DOUT = 5          # GPIO 5, 6, 7
_DIN = 17          # GPIO 17, 18, 19
_SM_FEED = 4       # PIO1 SM0 and SM1
_SM_CAP = 5
_DREQ_TX = 8       # PIO1 TX0
_DREQ_RX = 13      # PIO1 RX1


@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, autopush=True, push_thresh=24)
def _capture():
    wrap_target()
    wait(0, gpio, 14)          # start-of-scan latch pulse
    wait(1, gpio, 14)
    set(x, 15)
    label("bit")
    wait(1, gpio, 15)          # rising edge: the data lines are stable
    in_(pins, 3)
    wait(0, gpio, 15)
    jmp(x_dec, "bit")
    wait(0, gpio, 14)          # end-of-scan latch pulse
    wait(1, gpio, 14)
    wrap()


@rp2.asm_pio(out_init=(rp2.PIO.OUT_LOW,) * 3, out_shiftdir=rp2.PIO.SHIFT_RIGHT,
             autopull=True, pull_thresh=24)
def _feed():
    # for the TT07 silicon: each bit goes out AFTER the falling edge (see below)
    wrap_target()
    wait(0, gpio, 14)
    wait(1, gpio, 14)
    set(x, 15)
    label("bit")
    wait(1, gpio, 15)
    wait(0, gpio, 15)
    out(pins, 3)
    jmp(x_dec, "bit")
    wait(0, gpio, 14)
    wait(1, gpio, 14)
    wrap()


@rp2.asm_pio(out_init=(rp2.PIO.OUT_LOW,) * 3, out_shiftdir=rp2.PIO.SHIFT_RIGHT,
             autopull=True, pull_thresh=24)
def _feed_fixed():
    # for RTL with the 16-sample scan: each bit goes out BEFORE its rising edge, like a 74HC165 chain
    wrap_target()
    wait(0, gpio, 14)
    wait(1, gpio, 14)
    set(x, 15)
    label("bit")
    out(pins, 3)
    wait(1, gpio, 15)
    wait(0, gpio, 15)
    jmp(x_dec, "bit")
    wait(0, gpio, 14)
    wait(1, gpio, 14)
    wrap()


def _aligned(nbytes):
    # DMA ring buffers must be naturally aligned
    buf = bytearray(2 * nbytes)
    off = (-uctypes.addressof(buf)) % nbytes
    return buf, uctypes.addressof(buf) + off, memoryview(buf)[off:off + nbytes]


class TtlcSim:
    def __init__(self):
        self.running = False
        self.compensate = True      # play the 17-sample scan of the TT07 silicon (False: fixed RTL)
        self.inputs = 0
        self.timer = None
        self.tick_state = 0
        self.images = (bytes(8), bytes(8))

    def start(self, compensate=None):
        if self.running:
            return                          # a different `compensate` takes effect at the next start
        if compensate is not None:
            self.compensate = compensate
        self.in_buf, self.in_addr, self.in_ring = _aligned(8)
        self.out_buf, self.out_addr, self.out_ring = _aligned(8)
        self.set_inputs(0)
        for sm in (_SM_FEED, _SM_CAP):      # PIO1 SM0/SM1 and its instruction memory are ours
            rp2.StateMachine(sm).active(0)
        pio = rp2.PIO(1)
        for prog in (_feed, _feed_fixed, _capture):
            try:
                pio.remove_program(prog)    # also forgets where the program was loaded
            except Exception:
                pass
        pio.remove_program()                # and anything left by an earlier load of this file
        self.feed = rp2.StateMachine(_SM_FEED, _feed if self.compensate else _feed_fixed, out_base=Pin(_DIN))
        self.cap = rp2.StateMachine(_SM_CAP, _capture, in_base=Pin(_DOUT))
        self.dma_in = rp2.DMA()
        self.dma_out = rp2.DMA()
        self.dma_in.config(read=self.in_addr, write=self.feed, count=0x7fffffff,
                           ctrl=self.dma_in.pack_ctrl(size=2, inc_read=True, inc_write=False,
                                                      ring_size=3, ring_sel=False, treq_sel=_DREQ_TX),
                           trigger=True)
        self.dma_out.config(read=self.cap, write=self.out_addr, count=0x7fffffff,
                            ctrl=self.dma_out.pack_ctrl(size=2, inc_read=False, inc_write=True,
                                                        ring_size=3, ring_sel=True, treq_sel=_DREQ_RX),
                            trigger=True)
        self.feed.active(1)
        self.cap.active(1)
        self.running = True
        print('@sim=1')
        print('@compensate=%d' % self.compensate)

    def tick(self, period_ms):
        '''Toggle input 47 every period_ms (0 stops it): a clock for timing in PLC programs.'''
        if self.timer:
            self.timer.deinit()
            self.timer = None
        self.tick_state = 0
        if period_ms > 0:
            self.timer = Timer(-1)
            self.timer.init(period=int(period_ms), mode=Timer.PERIODIC, callback=self._tick)
        self._load_ring()
        print('@tick=%d' % period_ms)

    def _tick(self, t):
        # soft timer callback: no allocation, just copy the precomputed image
        self.tick_state ^= 1
        self.in_ring[:] = self.images[self.tick_state]


    def stop(self):
        if not self.running:
            return
        if self.timer:
            self.timer.deinit()
            self.timer = None
        self.feed.active(0)
        self.cap.active(0)
        for d in (self.dma_in, self.dma_out):
            # RP2040-E13: a DREQ-paced channel that is aborted while waiting for
            # a DREQ restarts on the next one - disable it first, then abort
            d.ctrl = d.pack_ctrl(enable=False)
            d.active(0)
            d.close()
        rp2.PIO(1).remove_program()
        for g in range(_DIN, _DIN + 3):     # give ui_in[4..6] back to the SDK (low)
            Pin(g, Pin.OUT, value=0)
        self.running = False
        print('@sim=0')

    @staticmethod
    def _image(value):
        # the two 24-bit words the feed machine shifts out for a 48-bit input vector
        w = [0, 0]
        for k in range(16):                 # scan order: bit 15 first
            b = 15 - k
            s = ((value >> b) & 1) | (((value >> (16 + b)) & 1) << 1) | (((value >> (32 + b)) & 1) << 2)
            w[k >> 3] |= s << (3 * (k & 7))
        return bytes(((w[i] >> (8 * j)) & 0xff) for i in range(2) for j in range(4))

    def _load_ring(self):
        self.images = (self._image(self.inputs & ~(1 << 47)), self._image(self.inputs | (1 << 47)))
        self.in_ring[:] = self.images[self.tick_state if self.timer else (self.inputs >> 47) & 1]

    def set_inputs(self, value):
        '''value: 48-bit int, bit i = TTLC input i (address 48 + i); bit 47 is the tick's when it runs.'''
        self.inputs = value
        self._load_ring()
        print('@inputs=%012x' % value)

    def outputs(self):
        '''The 48 outputs as last shifted out by the TTLC (bit i = output i).'''
        value = 0
        for i in range(2):
            v = 0
            for j in range(4):
                v |= self.out_ring[4 * i + j] << (8 * j)
            for k in range(8):
                s = (v >> (21 - 3 * k)) & 7
                b = 15 - (8 * i + k)
                value |= (s & 1) << b | ((s >> 1) & 1) << (16 + b) | ((s >> 2) & 1) << (32 + b)
        print('@outputs=%012x' % value)
        return value


# A previous load of this file (an earlier connect) may have left its
# emulator running; it lives in the same namespace, so stop it first.
try:
    ttlc.stop()
except Exception:
    pass
ttlc = TtlcSim()
print('@ttlcsim=1')

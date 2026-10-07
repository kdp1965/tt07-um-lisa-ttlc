'''
lisa_spiram.py - the RP2040 as LISA's SPI RAM on CS1 (uio[4]), on a TT07
demo board running the MicroPython build from mbell_micropython
(branch lisa_spi_ram: rp2.enable_sim_spi_ram and friends).

    >>> import lisa_spiram
    >>> lisa_spiram.setup()        # select tt_um_lisa, debugger up, emulator on, CS1 configured
    >>> lisa_spiram.pattern()      # write / read the RAM through the debugger's QSPI port
    >>> lisa_spiram.cache(True)    # the data cache onto the RAM (False: back to the 128 bytes)

The debug registers (debug_regs.v): 0x15 data cache chip select, 0x16
debugger chip select, 0x17 {addr_16b, is_flash, quad_mode} per CS,
0x1c io mux (uio[4] = CE1), 0x1d cache control, 0x1e SPI mode / CE
delay / clock divider, 0x10/0x11 the debugger's QSPI address, 0x20 a
16-bit word there (the address steps by 2).
'''
import time
import machine
import rp2
from machine import Pin
from ttboard.mode import RPMode

_uart = None


def _tt():
    import __main__
    try:
        return __main__.tt
    except AttributeError:
        from ttboard.demoboard import DemoBoard
        return DemoBoard.get()


def _select():
    tt = _tt()
    sh = tt.shuttle
    if tt.mode != RPMode.ASIC_RP_CONTROL:
        tt.mode = RPMode.ASIC_RP_CONTROL
    sh.tt_um_lisa.enable()
    tt.ui_in[7] = 0
    tt.reset_project(True)
    time.sleep_ms(10)
    tt.reset_project(False)


def _debugger():
    global _uart
    _uart = machine.UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13), rxbuf=1024)
    for _ in range(5):
        _uart.write(b'\n')
        time.sleep_ms(60)
    _uart.read()
    _uart.write(b'v')
    time.sleep_ms(100)
    v = _uart.read()
    if not v or b'lisav' not in v:
        raise RuntimeError('no debugger: %r' % v)
    return v.strip()


def rreg(a):
    _uart.read()
    _uart.write(b'r%02x\n' % a)
    t0 = time.ticks_ms()
    buf = b''
    while time.ticks_diff(time.ticks_ms(), t0) < 200:
        c = _uart.read()
        if c:
            buf += c
            if b'\n' in buf:
                break
    s = buf.strip()
    return int(s[-4:], 16)


def wreg(a, v):
    _uart.write(b'w%02x%04x\n' % (a, v & 0xffff))
    time.sleep_us(300)


def setaddr(a):
    wreg(0x11, a >> 16)
    wreg(0x10, a & 0xffff)


def setup(clkdiv=1, cedelay=127, mode=1):
    _select()
    print('debugger:', _debugger())
    print('sim ram:', rp2.enable_sim_spi_ram(), len(rp2.sim_spi_ram()))
    regs = [
        (0x1c, 0x0003),                            # uio[4] = CE1
        (0x1b, 0x5455),                            # uo_out: LISA port B
        (0x17, 0x0024),                            # CE1: SPI, 16-bit addresses, not flash; CE0: SPI, flash, 24-bit
        (0x1e, 0x0800 | (mode << 12) | (cedelay << 4) | clkdiv),  # CS0 SPI mode 1; CS1 mode (1: shift on the falling edge); CE delay; SCLK = clk / (2 (div + 1))
        (0x16, 0x0002),                            # the debugger's QSPI port on CS1
        (0x1d, 0x0007),                            # data cache off for now
    ]
    for a, v in regs:
        wreg(a, v)
    for a, v in regs:
        r = rreg(a)
        if r != v:
            raise RuntimeError('reg %02x reads %04x, wrote %04x' % (a, r, v))
    print('registers ok')


def pattern(n=64, base=0):
    '''n words at `base` through the debugger: written, checked on the RP2040
    side, read back.  Returns the number of mismatches.'''
    m = rp2.sim_spi_ram()
    for i in range(2 * n):
        m[base + i] = 0xee
    setaddr(base)
    words = [((i * 0x1234 + 0x5678) ^ (i << 8)) & 0xffff for i in range(n)]
    for w in words:
        wreg(0x20, w)
    _uart.flush()                  # the commands are still going out
    time.sleep_ms(2)
    bad_w = 0
    for i, w in enumerate(words):
        got = m[base + 2 * i] | (m[base + 2 * i + 1] << 8)
        if got != w:
            if bad_w < 8:
                print('write %04x: ram has %04x, wanted %04x' % (base + 2 * i, got, w))
            bad_w += 1
    setaddr(base)
    bad_r = 0
    for i, w in enumerate(words):
        got = rreg(0x20)
        if got != w:
            if bad_r < 8:
                print('read %04x: got %04x, wanted %04x' % (base + 2 * i, got, w))
            bad_r += 1
    print('pattern: %d words, %d write errors, %d read errors, report %r' % (n, bad_w, bad_r, rp2.report_sim_spi_ram()))
    return bad_w + bad_r


def cache(on):
    '''The data cache (lisa2 port) on CS1 - or back to the 128-byte RAM.'''
    v = rreg(0x15)
    wreg(0x15, (v & 0xfffc) | (2 if on else 1))
    wreg(0x1d, 0x0013 if on else 0x0007)       # invalidate, cache on (map 3 = 32K) / cache off
    time.sleep_ms(1)
    print('0x15=%04x 0x1d=%04x' % (rreg(0x15), rreg(0x1d)))


def flash():
    '''The debugger's QSPI port back on the flash (CS0).'''
    wreg(0x16, 0x0001)

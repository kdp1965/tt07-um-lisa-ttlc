'''
lisa_flash.py - board-side helpers for the LISA Commander web app.

The web app sends this file to the RP2040's RAM over the raw REPL each time
it connects (nothing is stored on the filesystem).  It provides:

  lisa_select()    enable tt_um_lisa and pulse its reset
  lisa_release()   deselect it, so the ASIC stops driving the uio pins
  flash            a LisaFlash: programs the SPI flash on the QSPI Pmod
                   straight from the RP2040, without going through LISA

Lines printed as "@key=value" are results for the web app to parse.

Pin use on the TT06/TT07 demo board (RP2040 GPIO):

  uio[0] = 21  flash CS          uio[4] = 25  SD2 (flash /WP)
  uio[1] = 22  SD0 / MOSI        uio[5] = 26  SD3 (flash /HOLD)
  uio[2] = 23  SD1 / MISO        uio[6] = 27  RAM A CS
  uio[3] = 24  SCK               uio[7] = 28  RAM B CS

Those SPI pins are not on an RP2040 hardware SPI, so this uses SoftSPI,
which still moves about 2 Mbit/s.
'''
import gc
import time
import binascii
import machine
from machine import Pin
from ttboard.mode import RPMode

_UIO = (21, 22, 23, 24, 25, 26, 27, 28)
_SECTOR = 4096
_PAGE = 256


def lisa_select():
    '''Enable tt_um_lisa, then reset it with its debug UART line idle.'''
    sh = tt.shuttle
    if not sh.has('tt_um_lisa'):
        raise RuntimeError('tt_um_lisa is not on shuttle %s' % sh.run)
    if tt.mode != RPMode.ASIC_RP_CONTROL:
        tt.mode = RPMode.ASIC_RP_CONTROL
    if sh.tt_um_lisa.enable() is False:
        raise RuntimeError('the SDK refused to enable tt_um_lisa')

    # ui_in[7] low at reset selects autobaud.  Bring the debug RX line
    # (ui_in[3]) to UART idle before reset is released, so the autobaud
    # logic starts from a clean line.
    tt.ui_in[7] = 0
    machine.UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))
    tt.reset_project(True)
    time.sleep_ms(10)
    tt.reset_project(False)

    print('@shuttle=%s' % sh.run)
    print('@clock=%d' % (tt.auto_clocking_freq if tt.is_auto_clocking else 0))


def lisa_release():
    '''Select design 0 (the chip ROM), which leaves every uio pin undriven.'''
    tt.shuttle.tt_um_chip_rom.enable()


class LisaFlash:
    def __init__(self):
        self.spi = None

    def open(self):
        '''Take the flash pins.  Call lisa_release() first.'''
        self.cs = Pin(_UIO[0], Pin.OUT, value=1)
        # keep both PSRAMs deselected, and /WP and /HOLD inactive
        for i in (4, 5, 6, 7):
            Pin(_UIO[i], Pin.OUT, value=1)
        self._to_spi_mode()
        self.spi = machine.SoftSPI(baudrate=4000000, polarity=0, phase=0,
                                   sck=Pin(_UIO[3]), mosi=Pin(_UIO[1]),
                                   miso=Pin(_UIO[2]))
        # wake from power-down, then software reset: back to power-on state
        self._cmd(b'\xab')
        self._cmd(b'\x66')
        self._cmd(b'\x99')
        time.sleep_ms(1)

    def close(self):
        '''Give every uio pin back as an input.'''
        self.spi = None
        for g in _UIO:
            Pin(g, Pin.IN, pull=None)

    def _to_spi_mode(self):
        # The flash may have been left in QPI or QSPI continuous-read mode
        # (TinyQV does that), where it ignores plain SPI commands.  0xFF
        # clocked with all four data lines high ends both: 2 clocks is the
        # QPI "exit" opcode, 8 clocks fills the continuous-read mode bits.
        sck = Pin(_UIO[3], Pin.OUT, value=0)
        Pin(_UIO[1], Pin.OUT, value=1)
        Pin(_UIO[2], Pin.OUT, value=1)
        for clocks in (2, 8):
            self.cs(0)
            for _ in range(clocks):
                sck(1)
                sck(0)
            self.cs(1)
        Pin(_UIO[2], Pin.IN, pull=None)

    def _cmd(self, tx, nread=0):
        self.cs(0)
        try:
            self.spi.write(tx)
            if nread:
                return self.spi.read(nread)
        finally:
            self.cs(1)

    def _addr_cmd(self, opcode, addr):
        return bytes((opcode, (addr >> 16) & 0xff, (addr >> 8) & 0xff, addr & 0xff))

    def jedec_id(self):
        return self._cmd(b'\x9f', 3)

    def wait_ready(self, timeout_ms=5000):
        t0 = time.ticks_ms()
        while self._cmd(b'\x05', 1)[0] & 1:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise RuntimeError('timed out waiting for the flash')

    def read(self, addr, n):
        return self._cmd(self._addr_cmd(0x03, addr), n)

    def erase_sector(self, addr):
        self._cmd(b'\x06')
        self._cmd(self._addr_cmd(0x20, addr))
        self.wait_ready()

    def program(self, addr, data):
        mv = memoryview(data)
        off = 0
        while off < len(data):
            a = addr + off
            n = min(_PAGE - (a % _PAGE), len(data) - off)
            self._cmd(b'\x06')
            self._cmd(self._addr_cmd(0x02, a) + mv[off:off + n])
            self.wait_ready()
            off += n

    # ---- entry points used by the web app ------------------------------
    # Each one releases the pins again if it fails, and keeps allocations
    # small: the TT SDK leaves little free (and fragmented) heap.

    def probe(self):
        '''Report the flash JEDEC ID.'''
        lisa_release()
        self.open()
        try:
            print('@id=' + binascii.hexlify(self.jedec_id()).decode())
        finally:
            self.close()

    def begin(self, addr, length):
        '''Deselect LISA, take the pins and erase the sectors to be written.'''
        gc.collect()
        lisa_release()
        self.open()
        try:
            jid = self.jedec_id()
            print('@id=' + binascii.hexlify(jid).decode())
            if jid in (b'\x00\x00\x00', b'\xff\xff\xff'):
                raise RuntimeError('no SPI flash found on uio[0..3]')
            first = addr - (addr % _SECTOR)
            n = 0
            for a in range(first, addr + length, _SECTOR):
                self.erase_sector(a)
                n += 1
            print('@erased=%d' % n)
        except Exception:
            self.close()
            raise

    def write(self, addr, data):
        '''Program one chunk and read it back.'''
        try:
            self.program(addr, data)
            if self.read(addr, len(data)) != data:
                raise RuntimeError('verify failed in chunk at 0x%06x' % addr)
        except Exception:
            self.close()
            raise
        print('@ok=%d' % (addr + len(data)))

    def end(self):
        self.close()
        print('@done=1')

    def readout(self, addr, length):
        '''Deselect LISA and print flash contents as lines of hex.'''
        gc.collect()
        lisa_release()
        self.open()
        try:
            for a in range(addr, addr + length, 256):
                n = min(256, addr + length - a)
                print('@data=' + binascii.hexlify(self.read(a, n)).decode())
        finally:
            self.close()


flash = LisaFlash()
print('@helper=1')

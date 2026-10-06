'''
uartPass.py - USB serial <-> LISA debug UART pass-through.

Runs on the RP2040 of a Tiny Tapeout TT06/TT07 demo board (MicroPython +
TT SDK) and bridges the USB serial port (the REPL connection) to the pins
the LISA debug controller listens on:

    ui_in[3]  = GPIO12 = UART0 TX  ->  LISA debug RX
    uo_out[4] = GPIO13 = UART0 RX  <-  LISA debug TX

Usage, with the LISA project already enabled:

    >>> import uartPass            # the first import starts the pass-through
    >>> uartPass.passthrough()     # ...and this starts it again later

Send Ctrl-C (0x03) to stop it and get the REPL back.

Sideband: a NUL byte (0x00) from the USB side starts a line of Python that is
run on the board when its newline arrives (instead of being forwarded), with
its output sent back between two NUL bytes.  LISA traffic never contains NUL,
and the pass-through keeps flowing meanwhile.  The web app uses this to drive
the TTLC I/O emulator while the console stays attached to LISA.

The LISA Commander web app (lisa-test/web) installs this file on the board
and keeps it up to date; lisa_pydb.py --init uses it too.
'''
import sys
import select
import machine

VERSION = '2.1'


def passthrough(baudrate=115200):
    # Create the UART on every start: the TT SDK retunes the RP2040 clocks
    # when a project is enabled, which leaves an older UART at the wrong baud.
    uart = machine.UART(0, baudrate=baudrate, tx=machine.Pin(12),
                        rx=machine.Pin(13), rxbuf=1024)

    # The .buffer streams carry raw bytes: no CR/LF translation either way,
    # and bytes that are not valid UTF-8 pass through instead of raising.
    usb_in = sys.stdin.buffer
    usb_out = sys.stdout.buffer

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    cmd = None

    while True:
        # USB serial -> LISA (or a sideband command line)
        if poll.poll(0):
            c = usb_in.read(1)
            if cmd is not None:
                if c == b'\n':
                    _sideband(bytes(cmd), usb_out)
                    cmd = None
                else:
                    cmd += c
            elif c == b'\x00':
                cmd = bytearray()
            else:
                uart.write(c)

        # LISA -> USB serial
        n = uart.any()
        if n:
            usb_out.write(uart.read(n))


def _sideband(src, out):
    import __main__
    out.write(b'\x00')
    try:
        exec(src.decode(), __main__.__dict__)
    except Exception as e:
        print('@error=' + repr(e))
    out.write(b'\x00')


try:
    passthrough()
except KeyboardInterrupt:
    print("Passthrough terminated.")
except Exception as e:
    print("Error:", e)

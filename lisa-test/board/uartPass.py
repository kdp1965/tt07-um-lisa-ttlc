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

The LISA Commander web app (lisa-test/web) installs this file on the board
and keeps it up to date; lisa_pydb.py --init uses it too.
'''
import sys
import select
import machine

VERSION = '2.0'


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

    while True:
        # USB serial -> LISA
        if poll.poll(0):
            uart.write(usb_in.read(1))

        # LISA -> USB serial
        n = uart.any()
        if n:
            usb_out.write(uart.read(n))


try:
    passthrough()
except KeyboardInterrupt:
    print("Passthrough terminated.")
except Exception as e:
    print("Error:", e)

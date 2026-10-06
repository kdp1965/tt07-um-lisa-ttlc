# LISA Commander — browser-based programmer, console and debugger

A Web Serial app for testing the **LISA** microcontroller (`tt_um_lisa`, TT06 and
TT07) on a Tiny Tapeout demo board, in the spirit of the
[TinyQV programmer](https://program.tinyqv.com/): connect the board, click
**Program**, and talk to the running program through a UART console.

Live: **https://kdp1965.github.io/tt07-um-lisa-ttlc/** (Chrome or Edge).

Everything lives in this directory:

| | |
|---|---|
| `web/index.html` | the app — a single page, no build step, no external scripts |
| `web/assets.js` | generated: the board-side Python and the demo firmware, embedded |
| `web/build_assets.py` | regenerates `assets.js` after editing `board/` or `firmware/` |
| `board/uartPass.py` | USB ↔ LISA-debug-UART pass-through, installed on the board's filesystem |
| `board/lisa_flash.py` | project select/reset and direct SPI-flash programming; sent to the board's RAM on connect |
| `firmware/bringup_tt07`, `bringup_tt06` | the bring-up demo (prints a banner on `?`), one hex opcode per line |
| `test/` | hardware regression test of the app's protocol code (Node + a small serial bridge) |

## Using it

You need Chrome or Edge, a TT06/TT07 demo board running the TT MicroPython SDK
(2.0.x), and the QSPI Pmod on the bidirectional Pmod connector — LISA runs from
the SPI flash on `uio[0..3]` (CS, MOSI, MISO, SCK). The PSRAMs and the Pmod's
`SD2/SD3` lines are not used.

Open the page (GitHub Pages, `python3 -m http.server` in `web/`, or simply
double-click `index.html` — Chrome treats local files as a secure context), close
any `mpremote`/terminal session on the port, then:

1. **Connect board** and pick the board's serial port. The app enters the raw
   REPL, checks the SDK and shuttle, installs or updates `/uartPass.py` on the
   board if needed, probes the flash (JEDEC ID), enables `tt_um_lisa`, pulses its
   reset, starts the pass-through, locks the debug UART's autobaud, checks the
   debugger version (`lisav1.2`) and writes the setup registers. About 3 s.
2. **Program** the selected image. Two methods, chosen with the radio buttons:
   * **Direct** — the RP2040 drives the flash's SPI pins itself (SoftSPI, ~2 Mbit/s),
     while LISA is deselected so the ASIC is not driving those pins. The flash is first
     forced back into plain SPI mode (TinyQV leaves it in QSPI continuous-read mode),
     then sectors are erased, pages programmed and read back. Afterwards LISA is
     re-selected, reset and set up again. A few hundred ms for the demo, plus ~2 s
     for the re-initialization.
   * **Via LISA** — the debug UART: halt, `w21 0006`/`w21 01d8` (write-enable, 64 KB
     block erase), then one `w20xxxx` write and `r22` status poll per opcode. The
     flash sees exactly what LISA's own QSPI controller sends. Measured at
     3.8 ms per opcode in a browser-equivalent setup: ~3 s for the 799-opcode demo,
     a couple of minutes for a full 32K-opcode image.
3. With **Run after programming** ticked, the core starts at PC 0 and gets the UART
   (`l`). Click into the console and press `?` for the banner (`s` prints a
   counter line, unformatted: the firmware's `printf` does not do `%d` yet).

The **LISA core** panel has Run / Halt / Resume / Step / Reset, the registers
(PC, SP, RA, IX, ACC, flags, cond, and the opcode at PC read from the flash) and
a dump of the 128-byte data RAM with SP marked. **Verify** reads the flash back
(over SPI or through LISA, per the selected method) and compares it with the
image. Step and the RAM dump work around two properties of the TT07 silicon,
see below.

### The UART console

The console is the pass-through to LISA's debug UART, so it is either talking to
the **debugger** (type `v`, `r02⏎`, `w020000⏎` … yourself) or, after `l`, to the
**LISA program**. The pill above the console shows who owns it. Run/Resume hand
the UART to the program; Halt and the other debugger actions take it back with
`+++` (300 ms guard either side) and hand it back afterwards when the program is
still running. Enter sends CR; Ctrl-C is never sent (it would stop the
pass-through on the RP2040); the bring-up firmware drops characters that arrive
back to back, so paste sparingly.

### TT07 silicon notes (what the app does about them)

Found by reading `lisa_dbg.v` / `lisa_core.v` / `tt_um_lisa.v` and confirmed on a
TT07 board:

* **Debugger data reads return `RAM[IX]`.** With the data cache off, the cache's
  `d_ready` is a constant 1, so the core answers a register-6 read (`dbg_ready =
  d_ready_r`) in the access's first clock — before the pipelined address register
  and the RAM have seen the debugger's address. What the debug controller latches
  is the RAM output for the *idle* address, which while halted is IX
  (`d_addr = stop ? ix : …`). The app therefore reads RAM by saving IX, writing
  the wanted address to IX (register 5), reading register 6, and restoring IX.
  Writes through registers 7/6 land correctly. (The same RTL with the cache
  *enabled* produces a proper multi-cycle ready, which is presumably why this
  looked fine on TT06 with an SRAM.)
* **The step bit does nothing.** Register 0 bit 2 pulses the core's `stop` low for
  a single clock (`cont` is cleared by `cont_q` on the next edge), and the TT07
  pipeline's pre-decode state checks `stop` again and returns to fetch, so no
  instruction executes and PC does not move. The app's Step instead decodes the
  opcode at PC, puts hardware breakpoints (registers 0x08–0x0d) on every address
  it can continue at — fall-through, branch target, RA for `ret`/`rc`/`rz`/`reti`,
  IX for `call ix`/`jmp ix` — resumes, waits for the halt, and clears them. `rets`
  (returns to IA, not readable) only gets the fall-through breakpoint.
* **Reading register 0xf advances PC.** Any access to the "current opcode"
  register increments PC afterwards (`dbg_inc`: it is meant for loading code),
  which looks like "stepping" if you poll it. The app never touches 0xf; the
  opcode shown is read from the flash at `2 × PC`.

### Flash base (Advanced)

Default 0. A 64 KB-aligned base lets a LISA image coexist with other flash
contents (TinyQV firmware at 0, say). It is written to debug registers 0x10–0x12
and used by both programming methods and by Verify.

### Setup registers

Written on every (re-)initialization and read back (these were spread over
`loadFlash()`/`setFlashBase()`/`noCache()` in `lisa_pydb.py`, but a program loaded
directly needs them too):

| reg | value | |
|---|---|---|
| 0x11 / 0x12 / 0x10 | flash base | debug QSPI address MSB, LISA1 base, debug address LSB |
| 0x1c | 0x0000 | uio mux: plain SPI pins |
| 0x1b | 0x5455 | uo_out mux: LISA PortB |
| 0x17 | 0x0004 | CE0 is a single-SPI flash with 24-bit addresses |
| 0x1e | 0x08d3 | SPI mode 1, SCLK divider 3, 13 clocks between CE activations |
| 0x1d | 0x0007 | data cache off: the 128-byte RAM is the whole data space (no SRAM needed) |

### Firmware formats

* one 4-digit hex opcode per line (what `lisa_ld` and `lisa_pydb.py` use),
* `@addr` plus 2-digit hex bytes (the cocotb `firmware.hex` format; `@addr` is a byte address),
* a raw `.bin`.

Opcodes are stored little-endian at byte address `2 × PC`, which is what the
debugger's `w20` writes and what LISA's instruction fetch reads.

## Board side

`uartPass.py` is what makes the USB serial port behave as LISA's UART. It is
installed to `/uartPass.py` if missing or different from the copy in `board/`,
and is also what `lisa_pydb.py --init` imports. Note that it uses **UART0** on
GPIO12/13 (`ui_in[3]`/`uo_out[4]`): on the RP2040 those pins are UART0, and
`machine.UART(1, …)` on them raises `ValueError: bad TX pin`. It re-creates the
UART each time it starts (the SDK changes the RP2040 clocks when a project is
enabled), and passes bytes through unmodified in both directions.

`lisa_flash.py` is never stored on the board: the app sends it, stripped of
comments and docstrings, over the raw REPL (raw-paste mode) on every connect.
With the TT SDK loaded the RP2040 has only ~65 KB of fragmented heap, which is
why the transfers use 512-byte chunks and why the app soft-reboots the board
(through the friendly REPL, so `main.py` runs) if the SDK's `tt` object is gone
or a `MemoryError` shows up.

After editing `board/*` or `firmware/*`:

```sh
python3 web/build_assets.py
```

## Manual use (mpremote / lisa_pydb.py)

```sh
mpremote cp board/uartPass.py :/uartPass.py
mpremote repl
>>> tt.shuttle.tt_um_lisa.enable()
>>> import uartPass          # now the REPL *is* LISA's debug UART; Ctrl-C ends it
v                            # -> lisav1.2
```

`lisa_pydb.py --init --flash --run bringup_tt07` (in the `lisa-tools` repo) does the
same thing from the command line.

## Hardware test

`test/hw_test.mjs` runs the app's real protocol code (the `lisa-core` script block)
under Node against a board, through `test/bridge.py` (pyserial, TCP 5555):

```sh
python3 test/bridge.py /dev/cu.usbmodemXXXX &
node test/hw_test.mjs            # connect, direct, vialisa, verify, debug, disconnect
```

It programs an altered demo directly (banner ends in `LISA*`), the original via
LISA (`LISA!`), verifies both ways, and exercises halt, a dozen single steps
checked against the image's decode, RAM read-back, reset/resume, the `+++`
hand-back and reconnecting. It overwrites the first sectors of the flash.

## Publishing

`.github/workflows/pages.yaml` deploys `lisa-test/web` as this repository's
GitHub Pages site, **https://kdp1965.github.io/tt07-um-lisa-ttlc/**, on every
push to `main` that touches `lisa-test/` (or by hand from the Actions tab). It
regenerates `assets.js` first and warns if the committed copy was stale.

A Pages deployment replaces the whole site, so the GDS viewer job in `gds.yaml`
now only runs when that workflow is started manually — doing so would put the
viewer back in place of the app until the next `pages` run.

## Troubleshooting

* **"could not enter the MicroPython raw REPL"** — something else holds the port
  (mpremote, a terminal), or the board is wedged: press its reset button.
* **"LISA's debug UART does not answer"** — the autobaud needs `ui_in[7]` low at
  reset and the debug RX on `ui_in[3]`; the app sets both. Try *Re-init LISA*.
* **"no SPI flash answers on uio[0..3]"** — the QSPI Pmod is missing or on the
  wrong connector.
* **The board stops responding to USB** — MicroPython hit a fatal error (an
  uncaught `MemoryError` in the raw REPL does this). Reset the board.

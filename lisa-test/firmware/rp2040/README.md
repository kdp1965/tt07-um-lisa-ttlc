# RP2040 firmware with the SPI RAM emulation

`micropython-lisa_spi_ram.uf2` is Ken's MicroPython build for the TT07 demo
board's RP2040: MicroPython v1.23.0 (board `RPI_PICO`) from
https://github.com/kdp1965/micropython branch `lisa_spi_ram` (Michael Bell's
fork with the Tiny Tapeout additions), commit `b26f01cda`, built 2026-10-06.
It adds `rp2.enable_sim_spi_ram()`, `rp2.sim_spi_ram()` and
`rp2.report_sim_spi_ram()`: the RP2040 emulates a 32K SPI RAM on LISA's CS1
(`uio[4]`, PIO + DMA) so programs built with `sdcc -mlisa --tt07-cache` can
run with the data cache on.

LISA Commander embeds it (`web/build_assets.py`) and offers to flash it when
the "Data cache" image is chosen on a board whose MicroPython lacks it: the
board is restarted into its bootloader (`machine.bootloader()`), the file is
written to the `RPI-RP2` drive, and the board comes back with the TT SDK and
the files on its flash intact.  By hand: `mpremote exec "import machine;
machine.bootloader()"` then `cp micropython-lisa_spi_ram.uf2 /Volumes/RPI-RP2/`.

Before flashing, the app reads the RP2040's whole flash back (MicroPython
and its filesystem, 2 MB on the demo board) and saves it as a `.uf2` of the
same shape as the stock Tiny Tapeout image - a download, and a copy in the
browser - so the Board panel's "Restore…" can put it back; "Back up RP2040"
does that alone, and "Restore…" takes any `.uf2` too.

Rebuilding (macOS, see the notes in `lisa-tools`'s memory):

```sh
cd ~/projects/lisa/mbell_micropython
export PATH=/opt/homebrew/bin:/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin:/usr/bin:/bin
export CMAKE_POLICY_VERSION_MINIMUM=3.5
make -C mpy-cross CFLAGS_EXTRA="-Wno-error=unterminated-string-initialization -Wno-error=gnu-folding-constant"
make -C ports/rp2 BOARD=RPI_PICO
cp ports/rp2/build-RPI_PICO/firmware.uf2 <here>/micropython-lisa_spi_ram.uf2
python3 ../web/build_assets.py
```

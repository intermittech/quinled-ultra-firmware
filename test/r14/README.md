# Boundary, timing and target regressions (test/r14)

The round-14 review's harnesses (GPT-6 Astra's QA run), kept as regressions: they run the production sources as they
are, the `.cmd` files with MSVC AddressSanitizer, the Python ones on the compiled ELF in `tools/rv32emu.py`. Run them
from the firmware root; `build/` is scratch. Every one passes; the three that found bugs (a longer frame through the
Ultra-Digital receiver's capture, its version blink, an ABORT between ANNOUNCE halves) guard their fixes.

| Command | What it checks |
|---|---|
| `cmd /d /c test\r14\build_boundaries.cmd` | Frame / checksum / length limits, every board-id combination of the update's app side, an ABORT between ANNOUNCE halves (the half dropped), counter wraps and saturation, every half-word cut in five record stores. |
| `cmd /d /c test\r14\build_timing.cmd` | 1,080 sample-exact delay-line cases and 2,160 two-receiver telemetry windows (argument `rc`: the telemetry only). |
| `cmd /d /c test\r14\build_bl_matrix.cmd` | The three release images against the three bootloader hardware ids, a partial transfer and its retry (needs the release below). |
| `python test/r14/size_xpack.py` | The size gate's XW=0 budgets on the release ELFs (the WCH XW rows need `tools/size_gate.py`). |
| `python test/r14/test_ud_emulated.py --case good` | BOARD=3: decoding, CRCs, the pages at every rate, truncations, single-bit corruptions, header flags, clock wraps. |
| `python test/r14/test_ud_emulated.py --case length` | Longer captures whose first 320 bits hold a valid slot and header: refused (no answer, no planned cut). |
| `python test/r14/test_ud_emulated.py --case ring` | The same through the compiled ring capture and the SEND decision: only an exact 320-bit POLL is answered. |
| `python test/r14/test_ud_main.py --release` | IDENTIFY 5 through the release main loop: the status LED shows the version. |
| `python test/r14/test_ud_main.py --release --case length` | Longer IDENTIFY captures through the release main loop: refused. |
| `python test/r14/test_ud_main.py --release --case hardware` | Behind another board's bootloader the delay line and its DMA stay off. |
| `python test/r14/test_ud_main.py --release --case wrap` | The release main loop across the ms / tick wraps, the notice's expiry. |
| `python test/r14/test_ud_uart.py` | BOARD=3's UART: the preamble across a timer reload, the payload, the last stop bit, an interrupted burst. |
| `python test/r14/test_ud_poll_cycles.py` | The release poll / SRAM ISR's cycle estimates on idle, pixel and POLL samples (`--sweep`: every word and sample offset). |
| `python test/r14/test_flash_target.py` | The compiled effects store: power cuts, torn magic half-words, the sequence's wrap, clear and retry. |
| `python test/r14/bootloader_sender.py --esphome <repo> --image <release.bin>` | The bootloader suite with ESPHome's sender and a real release image (ASan; the dependency read-only). |

The release they use (XW=0, the pinned xPack GCC 15.2):

```bash
RISCV_GCC=<xPack riscv-none-elf-gcc.exe> XW=0 RX=1 UD=1 bash tools/build_release.sh 0.6.11-rc.14
```

The BOARD=3 build `test_ud_emulated.py` (`--elf` for another) and the standalone `test_ud_main.py` use, with the
functions as written:

```bash
XW=0 LTO=0 EXTRA_CFLAGS="-DBOARD=3 -fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/r14-ud-api bash tools/build_target.sh
```

`test_flash_target.py` uses `build/r14-measure-b1/fw.elf` (`XW=0 MEASURE=1 OUT=build/r14-measure-b1`), or the ELF
given as its first argument.

Emulated MMIO and NOR cut models don't replace the bench: they model preemption and cuts explicitly, not analog
voltages or the flash controller's behaviour on silicon.

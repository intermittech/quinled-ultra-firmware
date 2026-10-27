# Firmware model and pin-role contract.

The round-14 review's harnesses, kept as regressions: they run the production sources as they
are, the `.cmd` files with MSVC AddressSanitizer, the Python ones on the compiled ELF in `tools/rv32emu.py`. Run them
from the firmware root; `build/` is scratch. These application harnesses contain release-specific fixtures; the
[Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M)
capture and timing cases require the matching application build. Current bootloader
verification is under `bootloader/test/run_abi3.py`; no blanket current pass is claimed for this directory.

| Command | What it checks |
|---|---|
| `cmd /d /c test\r14\build_boundaries.cmd` | Frame / checksum / length limits, every board-id combination of the update's app side, an ABORT between ANNOUNCE halves (the half dropped), counter wraps and saturation, every half-word cut in five record stores. |
| `cmd /d /c test\r14\build_timing.cmd` | 1,080 sample-exact delay-line cases and 2,160 two-receiver telemetry windows (argument `rc`: the telemetry only). |
| `python test/r14/size_xpack.py` | The size gate's XW=0 budgets on the release ELFs (the WCH XW rows need `tools/size_gate.py`). |
| `python test/r14/test_ur_digital_m_emulated.py --case good` | BOARD=3: decoding, CRCs, the pages at every rate, truncations, single-bit corruptions, header flags, clock wraps. |
| `python test/r14/test_ur_digital_m_emulated.py --case length` | Longer captures whose first 320 bits hold a valid slot and header: refused (no answer, no planned cut). |
| `python test/r14/test_ur_digital_m_emulated.py --case ring` | The same through the compiled ring capture and the SEND decision: only an exact 320-bit POLL is answered. |
| `python test/r14/test_ur_digital_m_main.py --release` | IDENTIFY 5 through the release main loop: the status LED shows the version. |
| `python test/r14/test_ur_digital_m_main.py --release --case length` | Longer IDENTIFY captures through the release main loop: refused. |
| `python test/r14/test_ur_digital_m_main.py --release --case hardware` | Behind another board's bootloader the delay line and its DMA stay off. |
| `python test/r14/test_ur_digital_m_main.py --release --case wrap` | The release main loop across the ms / tick wraps, the notice's expiry. |
| `python test/r14/test_ur_digital_m_uart.py` | BOARD=3's UART: the preamble across a timer reload, the payload, the last stop bit, an interrupted burst. |
| `python test/r14/test_ur_digital_m_poll_cycles.py` | The release poll / SRAM ISR's cycle estimates on idle, pixel and POLL samples (`--sweep`: every word and sample offset). |
| `python test/r14/test_flash_target.py` | The compiled effects store: power cuts, torn magic half-words, the sequence's wrap, clear and retry. |

The release they use (XW=0, the pinned xPack GCC 15.2):

```bash
RISCV_GCC=<xPack riscv-none-elf-gcc.exe> XW=0 UR_ANALOG_M=1 UR_DIGITAL_M=1 bash tools/build_release.sh 0.7.1
```

The BOARD=3 build `test_ur_digital_m_emulated.py` (`--elf` for another) and the standalone `test_ur_digital_m_main.py` use, with the
functions as written:

```bash
XW=0 LTO=0 BOARD=3 EXTRA_CFLAGS="-fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/r14-ur_digital_m-api bash tools/build_target.sh
```

`test_flash_target.py` uses `build/r14-measure-b1/fw.elf` (`XW=0 MEASURE=1 OUT=build/r14-measure-b1`), or the ELF
given as its first argument.

Emulated MMIO and NOR cut models don't replace the bench: they model preemption and cuts explicitly, not analog
voltages or the flash controller's behaviour on silicon.

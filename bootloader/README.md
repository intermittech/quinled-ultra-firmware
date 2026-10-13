# QuinLED dig2analog-ultra-5ch bootloader

**Status: phase 1 built and host-tested, not run on hardware.** It updates the board's firmware over the data line
from ESPHome (the `dig2analog_ultra` hub), with no back-channel and no case opening. Design and decisions:
[`../../research/firmware-update.md`](../../research/firmware-update.md). Formats (frames, descriptor, status
records, hand-over, flash map): [`../src/core/ultra_update.h`](../src/core/ultra_update.h), shared with the app and
the ESPHome component.

## What it does

**Every reset starts here, at HSI 8 MHz.**

- **Fast path:** a confirmed app with a valid descriptor is started at once. That costs ~0.1 ms (788 cycles measured
  with `measure_bl.py`); no clock or peripheral is touched. A Diff-Power switch-on is as fast as without a bootloader.
- Everything else runs in **full mode** (144 MHz, interrupts off), which always ends in a software reset:

| Action | When | What |
|---|---|---|
| VERIFY | first boot after an update | body CRC with the CRC unit, after a reset (zero-wait copy question); bad: descriptor erased, recovery |
| TRIAL | unconfirmed app (after an update, until it saw normal Ultra frames + a PING) | listen <= 150 ms; the first normal Ultra frame ends it; an ANNOUNCE starts a session; then boot once |
| SESSION | the app asked (`UPD_REQ_ENTER` in the hand-over block) | listen up to 3 s for the ANNOUNCE, receive, commit |
| RECOVERY | no valid app, or 3 watchdog resets in a row | dark, red feedback channel blinks 1 Hz at 10 %; the gate takes the board's Ultra slot so the boards behind it keep working; any session can install an app |

**A session**

- The ANNOUNCE (two halves of the 64-byte descriptor) is checked: hardware id, board revision, size, load address,
  minimum bootloader version, version policy (newer; same or older only with FORCE). A reject touches nothing.
- **Accept:** the descriptor sector is erased first, then the body area. The scene log and the settings are never
  touched.
- **DATA chunks** (32 bytes, 8 per page) collect in a 48-page RAM pool. A full pool drops its least complete page, so
  merging across passes keeps working.
- **Programming:** a complete page is programmed (fast page program) at the next page boundary, while the host
  pauses; one per pause.
- **END:** the descriptor is written last (32 half-words), then the board resets into VERIFY.
- **Interrupted:** a missing page or ABORT leaves recovery.

**Receiver**

- SPI1 samples DIN at 36 MHz into a 4 KB DMA ring.
- `bl_rx.c` follows the level runs of the samples: no timer and no interrupt, and frame ends come from the samples
  themselves.
- It keeps the bit count and the last 384 bits: a whole UPD frame, or any Ultra frame's trailer.

**Gate**

- TIM2 counts DIN edges on ETR and drives FWD_EN on CH2 (the 2-timer app's mechanism).
- During sessions it stays open. In recovery it blocks the first 256 bits after a normal Ultra frame.

## Files

| File | |
|---|---|
| `bl_core.c/.h` | portable logic: boot decision, session, pool, erase plan, commit, verify (host-tested) |
| `bl_rx.c/.h` | streaming decoder (host-tested) |
| `bl_hw.c/.h` | CH32V203 registers: 144 MHz, SPI sampler, TIM2 gate, flash at HCLK / 2, CRC unit, IWDG, PVD, feedback PWM, UART |
| `bl_main.c` | reset flow and main loop; `upd_bl_info_t` at 0x08000010 |
| `bl_start.S`, `bl.ld` | startup (no vector table) and the 8 KB link; RAM after the 32-byte hand-over block |
| `build_bl.sh` | `OUT=<dir> BL_HW_REV=<n> bash bootloader/build_bl.sh` |
| `measure_bl.py` | runs the compiled functions in `tools/rv32emu.py` |
| `test/` | `build_bl_test.cmd`: `bl_tests.exe` (C) and `sender_tests.exe` (the ESPHome sender, C++) |

## Size and timing (compiled, xPack GCC 15.2, `-Os -fno-optimize-crc`)

| | |
|---|---|
| Flash | 6,520 of 8,192 bytes. `-fno-optimize-crc`: GCC 15 would turn the bitwise CRCs into 1.75 KB of tables |
| RAM | 16,960 B buffers (`.noinit`: ring 4 KB, session + 48-page pool 12.7 KB) + 1.5 KB stack, of 20,448 |
| Fast boot path | ~0.1 ms at 8 MHz (descriptor CRC by the CRC unit, status records, hand-over) |
| Decoder | 36.7 cycles per 16-sample word on average, 39 inside a frame: 61 % of the 144 MHz CPU (budget 64) |
| Per UPD frame | 30-44 µs of session work at 144 MHz |

**Clock choice.**

- 96 MHz with 24 MHz sampling would avoid the HCLK / 2 switch, but the decoder needed 79 % of the CPU inside frames.
- 144 MHz / 36 MHz reuses the app's sampling rate and thresholds, with margin.
- Flash operations then run at HCLK / 2, and the decoder resyncs after each one; the host's page pause covers it.

## Tests (`bootloader\test\build_bl_test.cmd`, MSVC; 441 + 50 checks, 53 with `BL_TEST_IMAGE`)

**Flash model**

- Erased = 0xE339E339.
- A second program without an erase is a violation.
- The bootloader, scene log and settings areas must never be touched.
- Power cuts at any operation leave the area half done.

**`bl_tests.exe`**

- **Erase plan:** contiguous, inside 0x2000-0xF6FF, descriptor sector first.
- **Descriptor checks:** every field.
- **Boot decision table:** including watchdog loop, reset flags, request survival.
- **Clean update**, 30.5 KB, 3 passes: 4.2 s, verified, trial, confirmed by PING, then the fast path.
- **Rejects:** same / older version, other hardware or revision, newer bootloader needed, too big. Flash
  byte-identical, back in the app with the result. FORCE installs.
- **Interruptions:** host gone after the erase, ABORT, a board powering up mid-session (catches an interleaved
  ANNOUNCE).
- **Power cut at every flash operation** (61 of 61 for a 6 KB image, every 7th of 163 for 30.5 KB):
  - it never runs a broken image;
  - a retry always installs.
- **Trial windows and status area:** strikes, records stop 4 short of full, CONFIRMED still fits; watchdog loop;
  recovery gate decisions.
- **Bad body under a committed descriptor:** caught by VERIFY.
- **VDD low (PVD):** nothing written, done later.
- **Frame loss, 20 runs each, + 1 % bit errors:**

  | Frame loss | Updated | In recovery (retry) |
  |---|---|---|
  | 0.1 % | 20 | 0 |
  | 1 % | 20 | 0 |
  | 5 % | 16 | 4 |
  | 10 % | 2 | 18 |

  The 5 % result is close to the statistical limit of 3 passes (17 / 20 with an unlimited pool).
- **Streaming decoder:** 240 / 240 frames exact (384, 12,864, 40, 320 bits; HSI ±2.2 %, ±60 ns jitter, 20 µs RMT
  hiccups). Also: resync mid-frame, too-long pulse.
- **End to end at sample level:** waveform → decoder → session → installed.
- **`BL_TEST_IMAGE`:** a real release image from `tools/release/mkimage.py` installs (Python and C formats agree).

**`sender_tests.exe`** (the ESPHome component's `update_session.h`; runs when `build_bl_test.cmd` finds
quinled-ultra-esphome: `ULTRA_ESPHOME`, else next to the firmware repository, else next to the hardware project)

- 30.5 KB: 4.4 s on the wire at the hub's worst-case bit time.
- Full slot (54,784 B): 7.2 s.
- 1 % loss + a 50-board lead-in + a forced downgrade.
- ABORT, PING.
- `fw_version.h` semver ordering.
- The real release image (40.8 KB): 5.6 s.
- The ESPHome copy of `ultra_update.h` must be identical to the master.

## Bench checks (need hardware)

| # | Check | If it fails |
|---|---|---|
| 1 | Fast page program (FTPG, 64 words + PGSTRT), fast page erase, 4 KB sector erase at HCLK / 2; BSY / WRBSY / EOP; times (2 / 16 / 16 ms) | sequence per WCH's library; times feed `UPD_PAGE_PAUSE` (host) |
| 2 | Erased flash reads 0xE339E339 | status checks accept 0xFFFF too |
| 3 | Standard half-word programming of single records into a page that already holds records (status area, descriptor) | records as whole fast pages (needs a page per record) |
| 4 | After programming the zero-wait area, do CPU reads see the new data before a reset? | the design doesn't depend on it (VERIFY runs after a reset) |
| 5 | Upper 160 KB (staging, option B): ESIG_FLACAP, erase / program / verify, retention | option A only (as built) |
| 6 | CPU and DMA during a page program at HCLK / 2: does the decoder resync in the host's 2.7 ms pause? | longer page pause |
| 7 | SPI1 RX-only clock keeps running with DMA (as the app) | full-duplex master with a dummy TX DMA |
| 8 | TIM2 ETR edge counter + CH2 toggle at 256 edges on FWD_EN (recovery gate) | gate stays open in recovery (downstream boards shift by one slot) |
| 9 | Bootloader → app jump: relocated `mtvec`, HPE, IWDG, the app's SystemInit from reset clocks | soft reset with BOOT_ONCE before the jump |
| 10 | App → bootloader by software reset with the hand-over block intact; IWDG state after a software reset | longer IWDG timeout in the bootloader |
| 11 | Power-on to first PWM with a confirmed app: no more than ~0.1 ms more than without a bootloader (Diff-Power) | none needed |
| 12 | PVD level at 2.9 V on the board's 3.3 V rail | another PLS level |
| 13 | WCH-LinkUtility: factory hex leaves the status area erased (no gap filling); WRPR0 write protection of sectors 0-1 | hex with explicit 0xFFFF? (then check 3 again) |
| 14 | Relay power-cut rig: 200+ cuts during sessions and trial boots, then a session must always succeed | — |
| 15 | Chain of 3-10 boards: skip / reject / recovery-gate behaviour, frame loss counters (UART `exit` lines) | — |

The UART (TP5, 115200 8N1) prints the action, the app version and, at the exit, request / result / UPD frames /
pages programmed.

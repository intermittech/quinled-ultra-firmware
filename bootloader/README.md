# QuinLED Dig2Analog-Ultra-5ch bootloader

**Status: hardened and host-tested, not run on hardware.** It updates the board's firmware over the data line
from ESPHome (the `dig2analog_ultra` hub), with no back-channel and no case opening. How to use it: the wiki page
*Updating over the data line*. Formats (frames, descriptor, status records, hand-over, flash map):
[`../src/core/ultra_update.h`](../src/core/ultra_update.h), shared with the app and the ESPHome component.

**Board:** QuinLED Dig2Analog-Ultra-5ch rev 0.4, hardware id 1, board revision 3 (`UPD_REV_D2A_R04`, the default of
`build_bl.sh`). Its bootloader accepts only images for that hardware id whose revision mask includes bit 3; images
for another board or revision are rejected before anything is erased. The QuinLED Diff-Power Ultra-Analog receiver (hardware id 2,
no hardware yet) gets its own bootloader build.

## What it does

**Every reset starts here, at HSI 8 MHz.**

- **Fast path:** start the IWDG before any app code, validate the descriptor/status and check the option bytes at
  HSI 8 MHz. A confirmed app then starts with a 2.7–6.4 s watchdog window (nominal 4 s). A hang before the app starts
  its own watchdog therefore still reaches recovery. DIN low adds no qualification delay (the compiled path: below, "Fast boot path").
- **Power-on escape:** DIN continuously high for at least 20 ms at startup opens a 3 s update-session window.
  A low sample before the threshold cancels it; software/watchdog resets do not sample for an escape. The window
  survives the first provisioning reset. A fresh app still passes VERIFY before any timeout or SAME-skip jump.
- **First provisioning:** before any app runs, protect bootloader sectors 0–1. Read all eight option halfwords
  twice at HSI, validate byte/inverse pairs, preserve RDPR=0xA5, USER, DATA and WRPR1–3, and clear only WRPR0 bits
  0–1. Reset, then check loaded WPR, RDPR, and the CRC-16 of **all eight** values. Already protected boards are not
  rewritten. This also protects an otherwise blank board. Low VDD, untrusted options, read protection, failed writes
  or a failed read-back keep the board in dark recovery; they cannot time out into an unprotected app. Reboot after
  correcting low VDD; an interrupted option erase/rewrite may require SWD/jig erase and reprovisioning.
- Everything else runs in **full mode** (144 MHz, interrupts off), which always ends in a software reset:

| Action | When | What |
|---|---|---|
| VERIFY | first boot after an update | body CRC with the CRC unit, after a reset (zero-wait copy question); bad: descriptor erased, recovery |
| TRIAL | unconfirmed app (after an update, until it confirms itself: a PING after normal Ultra frames, or 60 s with 100 frames decoded and no watchdog reset) | listen <= 150 ms; the first normal Ultra frame ends it; an ANNOUNCE starts a session; then boot once |
| SESSION | the app asked (`UPD_REQ_ENTER`) or the power-on DIN escape | listen up to 3 s for the ANNOUNCE, receive, commit |
| RECOVERY | no valid app, a protection failure, or 3 watchdog resets in a row | dark, red feedback channel blinks 1 Hz at 10 %; the gate takes the board's Ultra slot. With a valid, verified, confirmed app and protection OK, retry after 60 s without a session. Invalid/unconfirmed apps stay in recovery. A running session uses its own timeout. |

The retry and a SAME/rejected skip back to a confirmed app clear the watchdog-loop counter. Three further
watchdog resets re-enter recovery. PINGs and normal frames cannot keep the 60 s listening window alive forever.

**A session**

- The ANNOUNCE (two halves of the 64-byte descriptor) is checked: hardware id, board revision, size, load address,
  minimum bootloader version, version policy (newer; same or older only with FORCE). A reject touches nothing.
- **Accept:** revoke the descriptor magic with **one halfword program to zero**, then erase its sector first,
  followed by the body area. If revocation fails, no erase starts. This prevents a partial first-sector erase from
  leaving a confirmed descriptor over a damaged body. The scene log and settings are never touched. The only
  permitted repeat program is this magic revocation; its physical behavior must pass the silicon bench.
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

- TIM3 CH1 captures DIN's falling edges on PA6 (the pin the sampler reads too); its trigger output clocks TIM2
  (ITR2), so TIM2 counts DIN edges, and TIM2 CH2 drives FWD_EN on PA1. The app counts with TIM4 the same way.
- During sessions it stays open. In recovery it blocks the first 256 bits after a normal Ultra frame.

**Pins (rev 0.4, the app's pin map):** PA6 DIN (SPI1_MISO + TIM3_CH1), PA1 FWD_EN (TIM2_CH2), PA9 PWM_R (TIM1_CH2,
the 10 % feedback blink), PB15 D9, PA2 USART2_TX (TP5). PA0 / PA4 analog inputs; every other PWM pin stays an
input, and the board's pull-downs keep the gate drivers off.

## Files

| File | |
|---|---|
| `bl_core.c/.h` | portable logic: boot decision, session, pool, erase plan, commit, verify (host-tested) |
| `bl_rx.c/.h` | streaming decoder (host-tested) |
| `bl_hw.c/.h` | CH32V203 registers: 144 MHz, SPI sampler, TIM3 → TIM2 gate, flash at HCLK / 2, CRC unit, IWDG, PVD, feedback PWM, UART |
| `bl_main.c` | reset flow and main loop; `upd_bl_info_t` at 0x08000010 |
| `bl_start.S`, `bl.ld` | startup (no vector table) and the 8 KB link; RAM after the 32-byte hand-over block |
| `build_bl.sh` | `OUT=<dir> BL_HW_REV=<n> bash bootloader/build_bl.sh` (the build id from `BL_BUILD_ID`) |
| `BL_BUILD_ID` | the build id in the info block, one per bootloader code (now 20261016), and the md5 a release must build; a new id when, and only when, the code changes |
| `measure_bl.py` | runs the compiled functions in `tools/rv32emu.py` |
| `test/` | `build_bl_test.cmd`: `bl_tests.exe` (C) and `sender_tests.exe` (the ESPHome sender, C++) |

## Size and timing (compiled, xPack GCC 15.2, `-Os -fno-optimize-crc`)

| | |
|---|---|
| Flash | 7,496 of 8,192 bytes; 696 free (build 20261016). Bootloader version 2, the 57,088-byte app slot. Exact md5: `BL_BUILD_ID`. `-fno-optimize-crc` avoids GCC 15's 1.75 KB CRC tables. |
| RAM | 16,960 B buffers (`.noinit`: ring 4 KB, session + 48-page pool 12.7 KB) + 1.5 KB stack, of 20,448 |
| Fast boot path | HSI 8 MHz; early IWDG + descriptor/status + protection checks. Low DIN: no 20 ms wait; compiled `bl_boot` 1,082 estimated cycles / 0.135 ms (excludes startup/jump). Full latency needs the bench. |
| Decoder | 36.7 cycles per 16-sample word on average, 39 inside a frame: 61 % of the 144 MHz CPU (budget 64) |
| Per UPD frame | 30-44 µs of session work at 144 MHz |

**Clock choice.**

- 96 MHz with 24 MHz sampling would avoid the HCLK / 2 switch, but the decoder needed 79 % of the CPU inside frames.
- 144 MHz / 36 MHz reuses the app's sampling rate and thresholds, with margin.
- Flash operations then run at HCLK / 2, and the decoder resyncs after each one; the host's page pause covers it.

## Tests (`bootloader\test\build_bl_test.cmd`, MSVC)

**Flash model**

- Erased = 0xE339E339.
- A second program without an erase is a violation, except a zero halfword at the descriptor magic (revocation).
- The bootloader, scene log and settings areas must never be touched.
- Power cuts at any operation leave the area half done. The partial-erase case flips a single body bit
  towards the chip's erased `0xE339` readout while preserving every descriptor/status byte except the prior revoke;
  it does not replace the sector with random data.

**`bl_tests.exe`**

- **Erase plan:** contiguous, inside 0x2000-0xFFFF (bootloader 2; 0x2000-0xF6FF in the lower layout), descriptor
  sector first.
- **Descriptor checks:** every field.
- **Boot decision table:** including watchdog loop, reset flags, request survival.
- **Clean update**, 30.5 KB, 3 passes: 4.2 s, verified, trial, confirmed by PING, then the fast path.
- **Rejects:** same / older version, other hardware or revision, newer bootloader needed, too big. Flash
  byte-identical, back in the app with the result. FORCE installs.
- **Interruptions:** host gone after the erase, ABORT, a board powering up mid-session (catches an interleaved
  ANNOUNCE).
- **Power cut at every flash operation** (62 of 62 for a 6 KB image, every 7th of 164 for 30.5 KB):
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
- Full slot of the lower layout (54,784 B): 7.2 s (bootloader 2's slot is 57,088 B).
- 1 % loss + a 50-board lead-in + a forced downgrade.
- ABORT, PING.
- `fw_version.h` semver ordering.
- The real release image (40.8 KB): 5.6 s.
- The ESPHome copy of `ultra_update.h` must be identical to the master.

The last run: **736 checks** (upper records), **737** (lower records), **15,572** sender checks, all green.

**Hardening regressions** (`test/test_bl_r11.c`, included in both flash-layout builds)

- The actual shared `bl_boot` entry: an app hangs before its watchdog init, three resets reach recovery, a 60 s
  retry clears the counter, and a persistent hang returns to recovery. SAME retry, wraparound, invalid/unconfirmed
  descriptors, normal/PING traffic and an active session at the timeout boundary are covered.
- Partial first-sector erase with the old CRC and CONFIRMED record retained, refused revoke (no erase), clean retry;
  the cut-at-every-operation tests include revocation.
- DIN low, 1 µs, 19.999 ms, 20 ms, 25 ms; power vs software/watchdog reset, timer wrap, the 3 s window, update through
  the escape, and a fresh bad body which must still fail VERIFY.
- First WRPR provisioning before the app, preserving all other option values, once-only behavior, no descriptor,
  low VDD, bad snapshot, RDPR protected, write failure, full-option read-back mismatch, and an option-erase cut.
- `test/negative_r11.py <old-commit> <outdir>` (MSVC developer prompt): compile the old core with only a test entry
  adapter and require assertion failures for each of a, b, c, d and D1 (the controls: the core of `8db58cdf`, without the
  hardening).
- `python bootloader/test/test_boot_hw.py <bl.elf>`: the compiled main reaches the app with IWDG started; real option
  read/write helpers check clock, double reads, inverses, all EOP/WRP failure positions, relocking, and the DIN timer.

The hand-over's `reserved` word carries a pending option CRC or a tagged protection report
(`src/core/bl_protection.h`); the checksum formula and the wire formats are `ultra_update.h`'s. The app consumes protection events once; the jig still checks loaded WPR and
RDPR. In blocked recovery, the bootloader UART prints the protection result (and recovery feedback blinks).

## Bench checks (need hardware)

| # | Check | If it fails |
|---|---|---|
| 1 | Fast page program (FTPG, 64 words + PGSTRT), fast page erase, 4 KB sector erase at HCLK / 2; BSY / WRBSY / EOP; times (2 / 16 / 16 ms). An operation that ends without EOP is an error, as WRPRTERR: an update stops at the first failed erase, before a later sector goes (`test/test_r6_boot_eop.py` runs the compiled helpers and session on the emulator) | sequence per WCH's library; times feed `UPD_PAGE_PAUSE` (host); a no-EOP end on silicon: the bench plan |
| 2 | Erased flash reads 0xE339E339 | status checks accept 0xFFFF too |
| 3 | Standard half-word programming of single records into a page that already holds records; additionally reprogram the existing descriptor magic halfword to zero, cut its following sector erase at many points, and prove it cannot revive a valid descriptor over a changed body | records as whole fast pages (needs a page per record) |
| 4 | After programming the zero-wait area, do CPU reads see the new data before a reset? | the design doesn't depend on it (VERIFY runs after a reset) |
| 5 | Upper 160 KB (staging, option B): ESIG_FLACAP, erase / program / verify, retention | option A only (as built) |
| 6 | CPU and DMA during a page program at HCLK / 2: does the decoder resync in the host's 2.7 ms pause? | longer page pause |
| 7 | SPI1 RX-only clock keeps running with DMA (as the app) | as the app's fallback (firmware README, the SPI1 bench row) |
| 8 | TIM3 CH1 capture → TIM2 (ITR2) edge counter + TIM2 CH2 toggle at 256 edges on FWD_EN (recovery gate) | gate stays open in recovery (downstream boards shift by one slot) |
| 9 | Bootloader → app jump: relocated `mtvec`, HPE, IWDG, the app's SystemInit from reset clocks | soft reset with BOOT_ONCE before the jump |
| 10 | App → bootloader by software reset with the hand-over block intact; IWDG state after a software reset | longer IWDG timeout in the bootloader |
| 11 | Measure confirmed low-DIN power-on to PWM; hold DIN high >=20 ms, enter the 3 s window, install, and verify a 19 ms pulse does not enter. Also hang the app at its first instruction: three IWDG resets, 60 s recovery, retry. | fix qualification/watchdog timing before factory release |
| 12 | PVD level at 2.9 V on the board's 3.3 V rail | another PLS level |
| 13 | Factory hex leaves status gaps erased; bootloader first boot writes WRPR0 bits 0-1, preserves all other options, resets, reads all options back, and the jig sees WPR protected / RDPR A5 before any app output. Cut option erase/rewrite and test low VDD / failure recovery. | reprovision through SWD; do not ship until the option/reset behavior is proven |
| 14 | Relay power-cut rig: 200+ cuts during sessions and trial boots, then a session must always succeed | — |
| 15 | Chain of 3-10 boards: skip / reject / recovery-gate behaviour, frame loss counters (UART `exit` lines) | — |

The UART (TP5, 115200 8N1) prints the action, the app version and, at the exit, request / result / UPD frames /
pages programmed.

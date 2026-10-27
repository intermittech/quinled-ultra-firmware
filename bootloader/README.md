# ABI-3 declared-profile bootloader

**Not deployable: physical qualification remains incomplete.**

The receiver uses the pinned QuinLED Ultra ABI-3 contract in
`src/core/ultra_update.h`. Each protected QDB2 identity supplies the exact target,
revision, variant, layout and flash deadlines; the descriptor cannot change them.
The host verifies its signed manifest and complete image before transmission.
The bootloader independently checks the QDU6 descriptor before revocation.

All three V203 boards use the host-paced declared profile. They do not implement
bootloader STATUS replies. The CH32V407 Diff-Power Ultra Receiver Digital-Medium
(16 KiB bootloader, layout 4) uses the status profile instead: see below. Installed-app confirmation requires its full identity,
UID, version and FW_UPDATE page with the requested answer-window INDEX.
There is one application slot: interrupted updates do not provide rollback.
After revocation, recovery must accept a fresh complete transfer; an incomplete
image must never run.

The bootloader uses minimum-unit cursor erase with erased readback, bounded
multipass page staging, repeat checking, full-body CRC before metadata, and
magic-last commit. Flash waits run from SRAM under shared physical deadlines.
Recovery (no usable app, no update running) shows a slow status-LED blink, 1 s on / 1 s off from the main loop's
ms time base, outputs dark; an update session keeps the LED off. Housekeeping never blinks. The decoder detects
800/600/400/200 kHz frame periods; session profile changes are rejected.

| Board | WCH / xPack flash | Free of 8,192 B |
|---|---:|---:|
| [Dig2Analog-Plus-5ch](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Dig2Analog-Plus-5ch) | 8,048 / 8,048 B | 144 B |
| [Diff-Power Ultra Receiver Analog-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Analog-M) | 8,032 / 8,032 B | 160 B |
| [Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M) | 8,144 / 8,144 B | 48 B |

Every V203 bootloader forwards update traffic to the boards behind it for its whole stay, on all three
boards: TIM2 CH2 drives FWD_EN (PA1) as a forced output, held LOW from reset and forced HIGH at the first
350 us of LOW without an edge (DIN LOW), never inside a frame. A forced timer output needs no CPU, so it
stays put through every erase, program and status write, recovery, foreign sessions and FAILED waits.
Recovery after ordinary Ultra frames keeps the slot plan (the board's 256 bits held, the rest forwarded),
re-armed after 200 us of LOW without an edge after a frame (past the hub's release); every later return to full
forwarding (after a frame the slot plan held) waits for 350 us of LOW without an edge, DIN LOW, too. Nothing
writes the gate while DIN is HIGH. The CH32V407 bootloader does the same with TIM2 CH4 on FWD_EN (PA3).
The entry blackout (reset to open, the line LOW) is 0.94 ms on the model at 8 MHz boot plus 144 MHz init;
the declared trusted bound is **5 ms** (PLL lock and startup copy included; bench item to confirm).

The final jump enters the app at its link alias (the descriptor's entry is
physical; the app's startup computes gp, sp and its RAM PC-relatively).

Static SRAM is 17,464 B; 1,448 B remains after the unchanged 1,536 B stack and
32 B handover. LTO is enabled. These are bootloader-only measurements; application
integration has a separate size gate.

Build from the firmware root:

```text
WCH_GCC_DIR=<WCH GCC 15> RISCV_GCC=<xPack riscv-none-elf-gcc> python bootloader/test/run_abi3.py --build --full-slot
```

The six identity checks, 576 compiled pure descriptor checks and twelve
bootloader matrix jobs pass. The pinned C99 sender and actual ESPHome updater
drive each board at all four profiles, including complete 57,088-byte bodies.
The production main-loop path includes DMA capture, decoding and modeled flash;
sender tests lose first-pass chunks and exercise refused/retimed transmissions.
Every flash mutation is cut at three fractions, restarted, and recovered by
a fresh transfer; abort, stale/conflicting frames, faults and deadlines are tested.
`--build` builds the six bootloaders into `build/bl-abi3/current/bl<board>-<wch|xpack>/`
(`--folder` takes another set); the logs and `results.json` go to `--evidence`
(default `build/bl-abi3/matrix`).

Applications call the [QBA1 validation table](../docs/boot-api.md), with checked
version and pointers. Missing/incompatible services refuse updates. Confirmation
requires an idle session-zero PING and finishes within the advertised dispatch
and programming bounds. Compiled chain tests cover full UID, immutable target,
FW_UPDATE and nonzero window INDEX; modeled handover preserves the black latch
and masks interrupts through reset. The explicit analog post-update output ACK
waits for confirmation or the ten-second idle bound.

## CH32V407 Digital-Medium: the status profile

The V407 bootloader (12,312 B WCH / 12,416 B xPack of 16,384 B) follows the
status profile of the pinned update protocol: only the node an UPD_SELECT names
(its UID and immutable target) acts on the session and answers its UPD_STATUS
polls. It locks to the host's clock from the update frames' trailer times (at
least 1 s, +-200 ppm), answers at that rate on the answer UART, and runs each
flash operation (one 4 KiB erase unit, the first being the metadata sector's,
one 256-byte block, or the verification with the metadata) only after a poll was
answered BUSY with its deadline and that answer's window has closed. COMMITTED
holds until the session's PING acknowledges it. Its flash erase and program code
lives in the zero-wait flash; the app runs only with RAM_CODE_MOD = 0.

```text
python bootloader/test/test_bl_abi3_v407.py <bl.elf>                  transactions, cuts, faults, options
python bootloader/test/test_bl_abi3_v407.py <bl.elf> --app <fw.elf>   the boot path through the app's startup
python bootloader/test/test_bl_status_v407.py <bl.elf> <out> --proto <protocol checkout>   the C99 sender loop
python bootloader/test/test_bl_wire_v407.py <bl.elf> <out>            the main loop on the wire: t0, the answer
```

Physical flash latency, protection, output safety, forwarding, clock margins and
stack watermark still require bench qualification. Packaging remains disabled.

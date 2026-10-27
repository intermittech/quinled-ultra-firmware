# QBA1 boot validation services

The applications reuse the protected bootloader's QDU6 descriptor and update
frame validators. The app still owns session assembly, conflicting repeats,
version policy, update handover, confirmation admission and physical readback.
CRC8/16 routines used by normal protocol traffic remain local. Flash writes and
status scanning stay with their owning image; the API exports no flash writer.

| Board | WCH app / free of 57,088 B | Boot / free of 8,192 B | App SRAM / free after reservations |
|---|---:|---:|---:|
| [Dig2Analog-Plus-5ch](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Dig2Analog-Plus-5ch) | 49,388 / 7,700 | 8,048 / 144 | 15,780 / 2,540 |
| [Diff-Power Ultra Receiver Analog-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Analog-M) | 55,360 / 1,728 | 8,032 / 160 | 15,812 / 2,508 |
| [Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M) | 56,572 / 516 | 8,144 / 48 | 16,196 / 2,124 |

The unchanged stack reservation is 2,048 B plus 112 B handover/breadcrumb space.
Receiver SRAM floors remain 1,024/1,536 B. Boot SRAM is 17,464 B, leaving 1,448 B
after its 1,536 B stack and 32 B handover. WCH and xPack boot images are identical.
xPack receiver flash is measured only under the existing build contract.
The 812 B free-flash reporting target is met on the Dig2Analog-Plus-5ch and the Analog-Medium; the Digital-Medium keeps
516 B (296 B short of it, above its 512 B floor).
Version blink, Fire, Lava Lamp and calibration remain present.

## ABI and admission

The MCU-owned `src/mcu/ch32v20x/target.json` defines the slot, physical/link
aliases and table placement. `tools/gen_targets.py` generates its C and linker
layout. The table is at physical `0x08000030`, after the 32-byte QDB2 identity.

| Byte offset | Little-endian field |
|---|---|
| 0 | u32 magic `0x31414251` (QBA1) |
| 4 | u16 total byte length, currently 20 |
| 6 | u16 table version, currently 1 |
| 8 | u32 physical descriptor validator address |
| 12 | u32 physical frame validator address |
| 16 | u32 reserved, zero |

Version 1 apps accept appended version 1 entries: length must be at least 20,
word aligned and contained in the boot slot. Before each call they check magic,
version and length, then the selected entry is checked for halfword alignment
and range `[0x08000050,0x08002000)`. Missing/incompatible metadata or an invalid
pointer refuses update processing, reports rejection and prevents confirmation
or handover. No fallback jumps into older bootloaders. Provisioning must pair
these apps with a QBA1 bootloader; an application update cannot add the table to
an already protected bootloader. These bounds detect compatibility errors, not
malicious replacement of trusted boot flash.

Both entries use RV32 ILP32: `a0` points to caller-owned read-only bytes and
returns signed int. Descriptor input is 64 bytes; zero means valid, otherwise
an `UPD_ERROR_*` value. Frame input is the contiguous 48-byte block/header;
zero means valid, -1 invalid. The checked app stub reserves -2 for unavailable
or unsafe entry metadata. Boot descriptor checks use its immutable target.
Neither function keeps mutable static state, allocates, accesses peripherals,
changes interrupt state, or relies on boot RAM initialization.

The compiled export paths use no gp-relative data and never change gp. Calls
therefore preserve the application's gp without a switching stub. App ISRs
remain enabled. The proof rejects gp/CSR/MMIO access and non-stack writes,
tests foreign gp values, nested calls and a real app poll ISR at every visited
instruction address. Current direct callee stack maxima are 32/16 B; checked
stub plus helper costs are at most 4,008/1,513 modeled cycles. Stack analysis
accounts for the otherwise invisible 32 B cross-image callee.

## Confirmation and handover

Only a canonical session-zero PING received while idle can request confirmation
after startup checks. The first request has a 100 ms admission deadline;
repeats cannot extend it. Ordinary headers and ANNOUNCE cancel pending authority.
Physical halfword readback follows BSY completion. A single-halfword flash
operation has a 19.9 ms SRAM deadline including setup, reserving two 50 us ticks
for cleanup within the declared 20 ms operation limit; stuck BSY resets in SRAM.
The host's 120 ms quiet interval covers admission plus programming. The model
tests expiry, missing readback and stuck programming on all three apps.

Analog handover forces outputs low and closes forwarding with interrupts masked
through reset. Digital handover/confirmation first transmits black for the full
configured pixel capacity and completes the latch, including an empty local
pixel configuration. The chain decoder retains the entire 40-byte update block.
Analog post-update ACK plus version waits for confirmation or ten seconds; it
does not occupy the initial dark confirmation interval. Fault indication wins.

## Qualification and extension contract

The boot API's qualification covered per-fix sizes and hashes, the twelve-job
boot matrix, 1,155 cross-image API checks and paired ISR comparisons, the
release size/SRAM gates, app actions, chain UID/INDEX replies, the host
suites, 400 native cases, 200 short capture cases, 200 calibration cases and
12 flash-window cases, all on the same images (boards 1/2/3, WCH apps, both boot
compilers). `test/test_boot_api_app.py` and `test/test_boot_api_chain.py` take
such an image folder and a JSON output path.
The enabled-sense analog ISR comparison adds at most three modeled cycles;
the existing budgets are unchanged. Compiled chain cases test four profiles,
full UID and immutable target/body identity, INDEX 1/2, refused selected-session
confirmation and masked handover in 19.95–20.55 ms. They are receiver-level wire
simulations; physical multi-receiver confirmation still needs bench testing.

A new board supplies its board registry facts, immutable target/geometry,
descriptor and MCU pin binding, then passes the same size, SRAM, identity,
transaction and app timing gates. It must not add board-number branches to the
API. A new MCU additionally supplies its own layout, linker placement/assertions,
instruction alias mapping, callable ABI, gp/reentrancy/ISR proof, flash execution
rules and bounded SRAM write/reset/readback implementation. The CH32V203 layout
is not a portable global constant.

The local CH32FV2x/V3x RM V2.5 sections 32.1–32.3 (printed pp. 597–598) document
code flash and write protection; section 1.3 (p. 16) confirms zero and physical
`0x08000000` aliases. The CH32V203 datasheet V3.0 sections 1/2.5.2 identifies the
zero-wait code area. The existing protected boot slot remains executable without
mode/clock changes. Pure calls run outside erase/program intervals; writers and
their interrupt paths execute from SRAM. Emulator register-save and flash timing
are modeled inputs. Flash latency/protection, clock margins, output safety,
forwarding, power cuts and stack watermark remain physical qualification work.
Release packaging stays off until that bench qualification is done.

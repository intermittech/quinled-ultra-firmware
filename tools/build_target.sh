#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Build the firmware for the CH32V203 (QingKe V4, RV32IMAC; built as rv32imc, the atomics are not used).
# Compiler: $RISCV_GCC, else xPack riscv-none-elf-gcc (%LOCALAPPDATA%\xPacks\...), else PlatformIO's
# ESP32 RISC-V GCC 8.4 (compile check only).
# Output: build/target/fw.elf, .bin, .hex (flash with WCH-LinkUtility / wlink), .map, .lst and a size report.
# LDSCRIPT=<file>: another linker script; vendor/wch/Link_app.ld links the app behind the bootloader (0x00002100,
# tools/build_release.sh). Default: vendor/wch/Link.ld, a standalone image at 0x00000000.
# Link-time optimisation (-flto, one partition) by default: ~2.5 KB smaller. LTO=0: without (as up to 0.6.0).
# MEASURE=1: a measurement build for tools/measure.py / flash_stores.py (core/lto.h: MEASURED functions out of line).
set -e
FW="$(cd "$(dirname "$0")/.." && pwd)"
XPACK="$(ls -d "$LOCALAPPDATA"/xPacks/xpack-riscv-none-elf-gcc-*/bin 2>/dev/null | sort -V | tail -1)"
if [ -n "$RISCV_GCC" ]; then GCC="$RISCV_GCC"
elif [ -n "$XPACK" ]; then GCC="$XPACK/riscv-none-elf-gcc"
else GCC="$HOME/.platformio/packages/toolchain-riscv32-esp/bin/riscv32-esp-elf-gcc"
fi
PFX="${GCC%gcc}"
# -mtune=size (0.6.4, fable-flash): the size cost model for every file and the LTO link (-292 B on the 0.6.4 tree; a
# tune on some files only stops LTO's inlining across them). Measured alone: +1..+4 cycles on 8 render rows (the render
# cache then took 230-2,000 off each), the SET slot -48, the flicker render -151; the rest the same.
if "$GCC" --version | head -1 | grep -q " 8\."; then ARCH="-march=rv32imc -mabi=ilp32"; LTO="${LTO:-0}"; else ARCH="-march=rv32imc_zicsr_zifencei -mabi=ilp32 -mtune=size"; fi
LDARCH="-march=rv32imc -mabi=ilp32"   # selects the rv32imc multilib (libgcc)
# STRICT_ALIGN=1 (0.6.6): -mstrict-align, no load / store at a misaligned address. -mtune=size lets GCC merge byte
# accesses into halfword / word ones at odd offsets (0.6.5: 17 in the release app); the QingKe V4 manual lists the
# misaligned-access exceptions (mcause 4 / 6) without saying the core handles them: BENCH. Cost: README "New in 0.6.6".
if [ "${STRICT_ALIGN:-0}" = 1 ]; then ARCH="$ARCH -mstrict-align"; fi
OUT="${OUT:-$FW/build/target}"
mkdir -p "$OUT"
# the Diff-Power receiver (BOARD=2): its own set (0.6.6): no MON_EXTRA monitoring (D5 / M5 / M3 / D6: the console
# only; the return channel carries its values), so 0.6.4's layout's texts (ZL), the records still upper; no test jig
# (--no-jig: it has no test pad); RC_PAGES_024=<0 / 1>: the three pages only the 0.2.4 proposal defines (OUTPUTS, ACKS,
# ERRORS), 0 = UNSUPPORTED until its review
RECEIVER=0
case " $EXTRA_CFLAGS " in *" -DBOARD=2 "*) RECEIVER=1; EXTRA_CFLAGS="$EXTRA_CFLAGS -DMON_EXTRA=0 -DRC_PAGES_024=${RC_PAGES_024:-0}" ;; esac
# STACK_SIZE=<bytes> (0.6.6): the stack, 2048 by default. 1536 is ready (static worst case 832 B with the ISRs nested,
# tools/stack.py: main 720 + poll 64 + a slew 48; the QingKe V4B nests two levels) and waits for the bench: the painted
# `stack=` high-water mark of the 30-day soak (test/ROUND4_BENCH_PLAN.md). The receiver needs at least 1792 B (its main
# loop 928 B: 1,040 B nested). The linker scripts put the reset breadcrumbs right below it (Link_sections.ld).
SS="${STACK_SIZE:-2048}"
if [ "$RECEIVER" = 1 ] && [ "$SS" -lt 1792 ]; then SS=1792; fi
STACK_LD=""
if [ "$SS" != 2048 ]; then STACK_LD="-Wl,--defsym=QL_STACK_SIZE=$SS"; fi
# the console's text, compressed (core/cz.h: tools/gen_console_text.py writes $OUT/gen/cz_text.h; Python 3, stdlib)
"${PYTHON:-python}" "$FW/tools/gen_console_text.py" "$OUT/gen" $([ "${PERF:-0}" != 0 ] && echo --perf) \
    $([ "${UPPER_RECORDS:-1}" != 0 ] && [ "$RECEIVER" = 0 ] && echo --upper) $(echo " $EXTRA_CFLAGS " | grep -q -- " -DBOARD=2 " && echo --no-jig)
echo "compiler: $("$GCC" --version | head -1)"
# -msmall-data-limit=400: every variable up to 400 bytes goes to .sbss / .sdata, in reach of gp (one instruction
# less per access); the big buffers (ul, din_ring / din_copy, tail_copy, uart_tx, the PWM tables) stay outside.
# Small constants (.srodata) stay in flash (Link_sections.ld).
if [ "${MEASURE:-0}" = 1 ]; then EXTRA_CFLAGS="$EXTRA_CFLAGS -DMEASURE_BUILD"; fi
# UPPER_RECORDS (0.6.5, src/core/ultra_update.h): 1 (default) = the records in the upper CodeFlash, the code area to
# 0x10000; 0 = 0.6.4's layout (the bench fallback): the *_lower.ld linker scripts.
UPPER_RECORDS="${UPPER_RECORDS:-1}"
EXTRA_CFLAGS="$EXTRA_CFLAGS -DUPPER_RECORDS=$UPPER_RECORDS"
# PERF=1 / PERF=2 (0.6.5): the console's `perf` (core/perf.h; 1 the mcycle counter, 2 SysTick), a bench build only:
# bigger and slower than the release (build_release.sh sets PERF=0); flash it standalone (the default Link.ld)
if [ "${PERF:-0}" != 0 ]; then EXTRA_CFLAGS="$EXTRA_CFLAGS -DPERF_BUILD=$PERF"; fi
# 0.6.4 (review round 4): -fno-jump-tables (gpt-opt: branch dispatch is smaller on RV32IMC than flash-resident address
# tables, and the measured render / dispatch paths are faster; the bootloader has its own flags), -fno-ipa-sra
# (fable-flash: with LTO the scalar replacement of aggregate parameters made bigger clones than the calls it saved).
# 0.6.5: -fno-shrink-wrap-separate (-24 B on the upper-flash tree, no measured row slower; 0 B in 0.6.2's sweep);
# -fno-ipa-modref (-84 B; Quindor's choice: 6 rows +1..+10 cycles, 10 rows faster).
# Measured and left out (fable-flash): per-file tunes (they stop LTO's inlining), -fipa-pta, -fmerge-all-constants,
# -fshort-enums, the inliner's parameters: 0 B or worse. 0.6.5 re-sweep: -fno-ipa-modref -84 B but 6 rows +1..+10
# cycles (taken: Quindor); -fno-ipa-icf / -cp-clone / -ra / -reference-addressable / -stack-alignment, -fno-tree-vrp 0 B;
# -fno-ipa-cp, -fno-inline-small-functions, -fno-tree-loop-distribute-patterns, -fno-ipa-reference / -pure-const /
# -bit-cp / -vrp, -fno-tree-pre, small-data limits 200 / 800 / 2048: larger.
# 0.6.6 (review round 5): -malign-data=natural for every file (the cold ones had it; explicit DMA / copy alignment
# stays; two duplicate CRC tables merge: -56 B, 16 B RAM), -fno-partial-inlining (-84 B); no measured row slower.
# Then (a 270-option sweep, each with measure.py): --param max-inline-insns-auto=5, max-hoist-depth=0 and
# max-grow-copy-bb-insns=4 (alone +16 / +64 / +36 B: LTO's choices interact; the set is what pays), no row slower.
OPT="-Os -g -fno-partial-inlining -malign-data=natural --param=max-inline-insns-auto=5 --param=max-hoist-depth=0 --param=max-grow-copy-bb-insns=4 -fno-jump-tables -fno-ipa-sra -fno-shrink-wrap-separate -fno-ipa-modref -ffunction-sections -fdata-sections -msmall-data-limit=400 -Wall -Wextra -Wno-unused-parameter"
CFLAGS="$ARCH $OPT -fno-common -DCH32V20x_D6 -DUPD_CRC_ULTRA -I$FW/vendor/wch -I$FW/src/hal -I$FW/src -I$OUT/gen $EXTRA_CFLAGS"
SRC="$FW/src/app/main.c $FW/src/hal/hw.c"
for f in "$FW"/src/core/*.c; do SRC="$SRC $f"; done
# GCC 15 turns bitwise CRC loops into 256-entry tables (-foptimize-crc). The firmware update's CRCs (upd_app.c) run
# once per update frame: built without them (1.8 KB). The CRC-8 / CRC-16 of ultra_protocol.h use nibble tables, which it
# leaves alone. Older compilers don't know the option.
NOCRC=""
# cold files (the console, records, ...): -msave-restore, and -malign-data=natural (GCC otherwise aligns every
# string literal and byte array to 4 bytes: ~1.5 bytes of padding per string)
COLD="${COLD_CFLAGS--msave-restore}"   # (-malign-data=natural: every file since 0.6.6, OPT above)
if echo "int x;" | "$GCC" -fno-optimize-crc -x c -c - -o "$OUT/nocrc_probe.o" 2>/dev/null; then NOCRC="-fno-optimize-crc"; fi
rm -f "$OUT/nocrc_probe.o"
"$GCC" $CFLAGS -x assembler-with-cpp -c "$FW/vendor/wch/startup_ch32v20x_D6.S" -o "$OUT/startup.o"
"$GCC" $CFLAGS -x assembler-with-cpp -c "$FW/src/hal/isr.S" -o "$OUT/isr.o"
OBJS="$OUT/startup.o $OUT/isr.o"
# LTO: each file's own flags below (-O2, -msave-restore, -fno-optimize-crc) stay with its functions; the link
# compiles the whole program once more with $OPT (warnings included). div64.c stays outside: it defines libgcc's
# __udivdi3 / __divdi3, which the compiler calls only after LTO has run. The asm (startup, isr.S) is never LTO.
LTOFLAGS=""; LDLTO=""
if [ "${LTO:-1}" = 1 ]; then LTOFLAGS="-flto"; LDLTO="$ARCH $OPT -flto -flto-partition=none"; fi
for s in $SRC; do
  o="$OUT/$(basename "${s%.c}").o"
  f=""
  case "$s" in */ws_decode.c) f="-O2" ;; esac          # the per-frame bit decoder: -15 % cycles for 64 bytes
  case "$s" in */upd_app.c) f="$NOCRC" ;; esac
  case "$s" in */cli.c|*/config.c|*/store.c|*/scene.c|*/calstore.c|*/upd_app.c|*/ack.c|*/climit.c|*/mcutemp.c) f="$f $COLD" ;; esac
  case "$s" in */pwrev.c|*/fwver.c|*/health.c|*/vdip.c) f="$f $COLD" ;; esac   # also cold: every ms at most / boot
  case "$s" in */pagepair.c|*/vsense.c) f="$f $COLD" ;; esac   # 0.6.5: -56 B, no row slower (boot / flash ops; per ms)
  # (flashrec.c not cold: +244 B, LTO inlines the stores into their callers; 0.6.5 re-measured: dark / ocled / energy /
  # sstart / smooth / gamma +8..+216 B, presets / ultra_update / rxpower 0)
  # (energy.c not since 0.6.3: LTO doesn't inline across -msave-restore, and its small calls into main / hw.c cost more)
  case "$s" in */div64.c) ;; *) f="$f $LTOFLAGS" ;; esac
  "$GCC" $CFLAGS $f -c "$s" -o "$o"
  OBJS="$OBJS $o"
done
LDSCRIPT="${LDSCRIPT:-$FW/vendor/wch/Link.ld}"
if [ "$UPPER_RECORDS" = 0 ]; then
  case "$LDSCRIPT" in */Link.ld|*/Link_app.ld) LDSCRIPT="${LDSCRIPT%.ld}_lower.ld" ;; esac
fi
"$GCC" $LDARCH $LDLTO $STACK_LD -nostartfiles -Wl,--gc-sections -Wl,-Map="$OUT/fw.map" -L"$FW/vendor/wch" -T "$LDSCRIPT" \
  $OBJS -o "$OUT/fw.elf" -lgcc
"${PFX}objcopy" -O binary "$OUT/fw.elf" "$OUT/fw.bin"
"${PFX}objcopy" -O ihex "$OUT/fw.elf" "$OUT/fw.hex"
"${PFX}objdump" -d -S "$OUT/fw.elf" > "$OUT/fw.lst"
"${PFX}size" -A "$OUT/fw.elf" | grep -E "^\.(init|vector|text|data|bss|stack)|Total"
"${PFX}size" "$OUT/fw.elf"

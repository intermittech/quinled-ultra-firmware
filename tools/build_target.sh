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
# XW (0.6.7, Quindor 2026-10-14; review round 5, the XW study): WCH's XW extension of the QingKe V4B, 16-bit byte /
# halfword loads and stores (c.lbu, c.lhu, c.sb, c.sh and their sp forms), with WCH's GCC 15.2 (MounRiver Studio
# 2.5.0's "RISC-V Embedded GCC15", in WCH_GCC_DIR, pinned by tools/toolchain/check_wch_gcc.sh): ~1.5 KB smaller.
# XW=0: the compiler above (xPack GCC 15.2), byte-identical to 0.6.6's builds: the comparison and the fallback. The
# bootloader stays on xPack GCC (bootloader/build_bl.sh). tools/rv32emu.py runs XW; no objdump disassembles it.
XW="${XW:-1}"
if [ "$XW" = 1 ]; then
  if [ -z "$WCH_GCC_DIR" ]; then
    echo "XW=1 (the default) needs WCH GCC 15.2: WCH_GCC_DIR=<its folder> (tools/toolchain/README.md); XW=0 builds without" >&2
    exit 2
  fi
  bash "$FW/tools/toolchain/check_wch_gcc.sh" "$WCH_GCC_DIR" || exit 2
  GCC="$WCH_GCC_DIR/bin/riscv32-wch-elf-gcc"
fi
G="${GCC%.exe}"; PFX="${G%gcc}"      # (a RISCV_GCC ending in .exe, as the release workflow sets it)
# -mtune=size (0.6.4, fable-flash): the size cost model for every file and the LTO link (-292 B on the 0.6.4 tree; a
# tune on some files only stops LTO's inlining across them). Measured alone: +1..+4 cycles on 8 render rows (the render
# cache then took 230-2,000 off each), the SET slot -48, the flicker render -151; the rest the same.
if "$GCC" --version | head -1 | grep -q " 8\."; then ARCH="-march=rv32imc -mabi=ilp32"; LTO="${LTO:-0}"; else ARCH="-march=rv32imc_zicsr_zifencei -mabi=ilp32 -mtune=size"; fi
LDARCH="-march=rv32imc -mabi=ilp32"   # selects the rv32imc multilib (libgcc)
if [ "$XW" = 1 ]; then
  ARCH="-march=rv32imc_zicsr_zifencei_xw -mabi=ilp32 -mtune=size"
  LDARCH="-march=rv32imc_xw -mabi=ilp32"   # (WCH's rv32imc_xw multilib: its libgcc with XW too)
fi
# STRICT_ALIGN=1 (0.6.6): -mstrict-align, no load / store at a misaligned address. -mtune=size lets GCC merge byte
# accesses into halfword / word ones at odd offsets (0.6.5: 17 in the release app); the QingKe V4 manual lists the
# misaligned-access exceptions (mcause 4 / 6) without saying the core handles them: BENCH. Cost: README "New in 0.6.6".
if [ "${STRICT_ALIGN:-0}" = 1 ]; then ARCH="$ARCH -mstrict-align"; fi
OUT="${OUT:-$FW/build/target}"
mkdir -p "$OUT"
# the Diff-Power receiver (BOARD=2): its own set (0.6.6): no MON_EXTRA monitoring (D5 / M5 / M3 / D6: the console
# only; the return channel carries its values), so 0.6.4's layout's texts (ZL), the records still upper; the test jig
# on TP6 (0.6.7: the console's RX, hal/hw.c hw_jig_pad_low). (0.6.9: RC_PAGES_024 gone: OUTPUTS, ACKS and ERRORS are
# official since protocol 0.2.4, always built in)
# The Diff-Power Ultra-Digital receiver (BOARD=3, docs/ultradigital-firmware.md): the receiver's set too, and its own app
# (src/app/main_ud.c: no light engine) with the delay line (src/hal/hw_dline.c, src/core/dline.c)
RECEIVER=0
UD=0
case " $EXTRA_CFLAGS " in
  *" -DBOARD=2 "*) RECEIVER=1; EXTRA_CFLAGS="$EXTRA_CFLAGS -DMON_EXTRA=0" ;;
  *" -DBOARD=3 "*) RECEIVER=1; UD=1; EXTRA_CFLAGS="$EXTRA_CFLAGS -DMON_EXTRA=0" ;;
esac
# STACK_SIZE=<bytes> (0.6.6): the stack, 2048 by default. 1536 is ready (static worst case 928 B with the ISRs nested,
# tools/stack.py, 0.6.7: main 800 + poll 80 + a slew 48; the QingKe V4B nests two levels) and waits for the bench: the
# painted `stack=` high-water mark of the 30-day soak (test/ROUND4_BENCH_PLAN.md). The receiver needs at least 1792 B
# (its main loop 864 B: 992 B nested). The linker scripts put the reset breadcrumbs right below it (Link_sections.ld).
SS="${STACK_SIZE:-2048}"
if [ "$RECEIVER" = 1 ] && [ "$SS" -lt 1792 ]; then SS=1792; fi
STACK_LD=""
if [ "$SS" != 2048 ]; then STACK_LD="-Wl,--defsym=QL_STACK_SIZE=$SS"; fi
# CONSOLE_FULL=1 (0.6.7, core/config.h): the bench's console with its commands; 0 (the release): it only sends. A PERF
# build has it (its `perf`).
CONSOLE_FULL="${CONSOLE_FULL:-0}"
if [ "${PERF:-0}" != 0 ]; then CONSOLE_FULL=1; fi
# (0.6.9) CONSOLE_FULL=1 builds are standalone (no bootloader), flashed over SWD: behind the bootloader the full
# console no longer fits the slot
case " $EXTRA_CFLAGS ${LDSCRIPT:-} " in
  *" -DUPD_APP=1 "*|*Link_app*)
    if [ "$CONSOLE_FULL" = 1 ]; then
      echo "error: CONSOLE_FULL=1 builds are standalone (no bootloader), flashed over SWD: no UPD_APP / Link_app*.ld" >&2
      exit 1
    fi ;;
esac
EXTRA_CFLAGS="$EXTRA_CFLAGS -DCONSOLE_FULL=$CONSOLE_FULL"
# the console's text, compressed (core/cz.h: tools/gen_console_text.py writes $OUT/gen/cz_text.h; Python 3, stdlib)
"${PYTHON:-python}" "$FW/tools/gen_console_text.py" "$OUT/gen" $([ "${PERF:-0}" != 0 ] && echo --perf) \
    $([ "${UPPER_RECORDS:-1}" != 0 ] && [ "$RECEIVER" = 0 ] && echo --upper) $([ "$CONSOLE_FULL" = 1 ] && echo --full)
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
# 0.6.7 (XW; speed for flash, within: the poll / slew / gate ISRs and the fast trip <= +5 cycles, the decoders <= +5 %,
# the worst case < 95 % CPU): --param gcse-unrestricted-cost=0, -fno-tree-sra, max-grow-copy-bb-insns 4 -> 3: -412 B
# (the receiver -492 B); the poll ISR's first edges / ring copies / frame ends +5 cycles, the decoders +1.7 .. +2.2 %,
# the worst case 90.8 -> 91.4 %; the gate ISR 3 instructions shorter, the SRAM trip path the same; the stack +16 B
# (main, the poll ISR). Left out (alone on this tree; with the set): -fno-if-conversion -156 B; -136 B (the fast trip
# +6, the first edges +7 cycles), -fno-caller-saves -132 B; -212 B (the clock sync +6.5 %), -fno-tree-forwprop -104 B
# (the SET +14 %), -fno-move-loop-invariants -168 B (the ring copies +216 cycles), uninlined-function-insns=0 / 1 -172 B
# / with gcse + sra -100 B (the frame end +31 / the first edges +21 cycles), max-grow-copy-bb-insns=0 -56 B (the capture
# decode +5.5 %), -fno-if-conversion2 (with gcse + sra +44 B), gcse-unrestricted-cost=1 (= 0), 2 (= 3).
# Round 9 (code size, the XW=0 build; both compilers, 4 images): -Oz = -Os to the byte (GCC 15, globally and on the
# cold files alone); crossjumping is at its most (-Os sets min-crossjump-insns 1): 2 / 3 / 4 / 5 / 6 / 8 +84 / +144 /
# +156 / +156 / +184 / +216 B, -fno-crossjumping +384 B; max-tail-merge-comparisons 20 / 50, max-tail-merge-iterations
# 1 / 4, max-crossjump-edges 1000: 0 B; -fno-reorder-blocks +112 B; -freorder-blocks-algorithm=simple -12 B (only
# ws_decode.c's -O2 changes: the WS2915 decode +0.9 %, the Ultra capture +1.8 %, the WS2805 / header rows faster: out).
OPT="-Os -g -fno-partial-inlining -malign-data=natural --param=max-inline-insns-auto=5 --param=max-hoist-depth=0 --param=max-grow-copy-bb-insns=3 --param=gcse-unrestricted-cost=0 -fno-tree-sra -fno-jump-tables -fno-ipa-sra -fno-shrink-wrap-separate -fno-ipa-modref -ffunction-sections -fdata-sections -msmall-data-limit=400 -Wall -Wextra -Wno-unused-parameter"
CFLAGS="$ARCH $OPT -fno-common -DCH32V20x_D6 -DUPD_CRC_ULTRA -I$FW/vendor/wch -I$FW/src/hal -I$FW/src -I$OUT/gen $EXTRA_CFLAGS"
# (round 11, B2) the build's firmware-update hardware id (board.h BOARD_HW_ID, from BOARD) as the ELF's absolute
# symbol fw_hw_id: no byte in the image; tools/release/mkimage.py takes the descriptor's hw_id from it
HW_ID="$(printf '#include "board.h"\nFW_HW_ID=BOARD_HW_ID\n' | "$GCC" $CFLAGS -E -P -x c - 2>/dev/null |
         sed -n 's/^FW_HW_ID=\([0-9][0-9]*\)u\{0,1\}$/\1/p')"
if [ -z "$HW_ID" ]; then echo "error: board.h's BOARD_HW_ID didn't resolve" >&2; exit 1; fi
if [ "$UD" = 1 ]; then
  # BOARD=3: its app, the hal with the delay line, and only the core files it uses (dline.c once it is there).
  # flashrec.c keeps only BOARD 3's lifetime/local-effect record descriptors. Unused calibration/settings functions
  # are discarded at link time; scene.c also holds the write budgets (energy.c).
  SRC="$FW/src/app/main_ud.c $FW/src/hal/hw.c $FW/src/hal/hw_cold.c $FW/src/hal/hw_dline.c $FW/src/hal/mem.c"
  for f in ack calstore config div64 energy flashrec fwver mcutemp pagepair pwrev rc scene store upd_app vdip vsense ws_decode ud/wsenc ud/udfx ud/udchain; do
    SRC="$SRC $FW/src/core/$f.c"
  done
  if [ -f "$FW/src/core/dline.c" ]; then SRC="$SRC $FW/src/core/dline.c"; fi
  # the local pixel effects' engine (core/fx.h, docs/ultradigital-effects.md): src/core/fx*.c
  CFLAGS="$CFLAGS -I$FW/src/core"        # (udfx.h includes "fx.h")
  SRC="$SRC $(ls "$FW"/src/core/fx*.c | tr '\n' ' ')"
else
  # BOARD=1 / 2: the light engine's app and every core file but the Ultra-Digital receiver's delay line logic
  SRC="$FW/src/app/main.c $FW/src/hal/hw.c $FW/src/hal/hw_cold.c $FW/src/hal/mem.c"
  # (nor the local pixel effects' engine, core/fx*.c: core/ud/ is not globbed)
  for f in "$FW"/src/core/*.c; do case "$f" in */dline.c|*/fx*.c) ;; *) SRC="$SRC $f" ;; esac; done
fi
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
# compiles the whole program once more with $OPT (warnings included). div64.c and mem.c stay outside: they define
# helpers the compiler calls after LTO. mem.c uses -O2 for the word-copy/fill loops and disables builtin / loop
# recognition to prevent recursion into itself. The asm (startup, isr.S) is never LTO.
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
  case "$s" in */hw_cold.c) f="$f $COLD" ;; esac   # (round 9) hw.c's boot / flash half; the ISRs and hot helpers stay in hw.c
  case "$s" in */calhist.c) f="$f $COLD" ;; esac   # (round 11, O1-Q1) the calibration history: app -56 B XW=0 / -64 B XW
  # (flashrec.c not cold: +244 B, LTO inlines the stores into their callers; 0.6.5 re-measured: dark / ocled / energy /
  # sstart / smooth / gamma +8..+216 B, presets / ultra_update / rxpower 0)
  # (energy.c not since 0.6.3: LTO doesn't inline across -msave-restore, and its small calls into main / hw.c cost more)
  case "$s" in
    */mem.c) f="$f -O2 -fno-tree-loop-distribute-patterns -fno-builtin" ;;
    */div64.c) ;;
    *) f="$f $LTOFLAGS" ;;
  esac
  "$GCC" $CFLAGS $f -c "$s" -o "$o"
  OBJS="$OBJS $o"
done
LDSCRIPT="${LDSCRIPT:-$FW/vendor/wch/Link.ld}"
if [ "$UPPER_RECORDS" = 0 ]; then
  case "$LDSCRIPT" in */Link.ld|*/Link_app.ld) LDSCRIPT="${LDSCRIPT%.ld}_lower.ld" ;; esac
fi
# (round 9) the code's order: two links in the input order, without relaxation (every call 8 bytes) and with it,
# then the final one in the call-graph order computed from them (tools/link_order.py: more calls / tail jumps within
# the 2-byte c.jal / c.j's +-2 KB; the code itself the same). ORDER=0: the input order. The first links may overflow
# the slot (--noinhibit-exec: their maps and listings are all they give).
# Pack aligned inputs first (--sort-section=alignment sorts within each input statement: the order's one-section
# lines stay as they are); Link_sections.ld keeps startup and SRAM handler order with SORT_NONE.
link() { m="$1"; e="$2"; shift 2; "$GCC" $LDARCH $LDLTO $STACK_LD -nostartfiles -Wl,--gc-sections \
  -Wl,--sort-section=alignment -Wl,-Map="$OUT/$m" -L"$OUT" -L"$FW/vendor/wch" -T "$LDSCRIPT" $OBJS -o "$OUT/$e" -lgcc \
  -Wl,--defsym=fw_hw_id="$HW_ID" $LINK_EXTRA "$@"; }
cp "$FW/vendor/wch/text_order.ld" "$OUT/text_order.ld"
if [ "${ORDER:-1}" = 1 ]; then
  link fw.norelax.map fw.norelax.elf -Wl,--no-relax -Wl,--noinhibit-exec 2>/dev/null
  link fw.input.map fw.input.elf -Wl,--noinhibit-exec 2>/dev/null
  "${PFX}objdump" -d "$OUT/fw.norelax.elf" > "$OUT/fw.norelax.lst"
  "${PFX}objdump" -d "$OUT/fw.input.elf" > "$OUT/fw.input.lst"
  if ! "${PYTHON:-python}" "$FW/tools/link_order.py" "$OUT/fw.norelax.map" "$OUT/fw.norelax.lst" \
      "$OUT/fw.input.map" "$OUT/fw.input.lst" > "$OUT/text_order.new"; then
    link fw.map fw.elf                   # (the link's own error, when that is what stopped it)
    echo "error: tools/link_order.py failed (the code's order; ORDER=0 builds in the input order)" >&2
    exit 1
  fi
  mv "$OUT/text_order.new" "$OUT/text_order.ld"
fi
if ! link fw.map fw.elf; then
  if [ "$XW" != 1 ]; then                # (0.6.9) the XW=0 image may no longer fit the slot
    echo "error: the link failed with XW=0. If the region FLASH overflowed: this image needs WCH GCC's XW (XW=1," \
         "the default; tools/toolchain/README.md). The README's sizes say which images fit without it (0.6.9)." >&2
  fi
  exit 1
fi
"${PFX}objcopy" -O binary "$OUT/fw.elf" "$OUT/fw.bin"
"${PFX}objcopy" -O ihex "$OUT/fw.elf" "$OUT/fw.hex"
"${PFX}objdump" -d -S "$OUT/fw.elf" > "$OUT/fw.lst"
"${PFX}size" -A "$OUT/fw.elf" | grep -E "^\.(init|vector|text|data|bss|stack)|Total"
"${PFX}size" "$OUT/fw.elf"

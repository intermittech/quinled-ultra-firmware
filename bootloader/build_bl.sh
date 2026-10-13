#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Build the bootloader (8 KB at 0x08000000). Same compiler choice as tools/build_target.sh.
# Options (environment): OUT=<dir> (default build/bootloader), BL_HW_REV=<0..15> (board revision this bootloader
# is built for, default 3 = dig2analog-ultra-5ch rev 0.4), BL_BUILD_ID=<8 chars>, EXTRA_CFLAGS.
# Output: bl.elf / .bin / .hex / .map / .lst, size report; fails if it doesn't fit 8 KB or its RAM.
set -e
FW="$(cd "$(dirname "$0")/.." && pwd)"
BL="$FW/bootloader"
XPACK="$(ls -d "$LOCALAPPDATA"/xPacks/xpack-riscv-none-elf-gcc-*/bin 2>/dev/null | sort -V | tail -1)"
if [ -n "$RISCV_GCC" ]; then GCC="$RISCV_GCC"
elif [ -n "$XPACK" ]; then GCC="$XPACK/riscv-none-elf-gcc"
else echo "no RISC-V GCC (set RISCV_GCC)"; exit 1
fi
PFX="${GCC%gcc}"
OUT="${OUT:-$FW/build/bootloader}"
BL_HW_REV="${BL_HW_REV:-3}"
BL_BUILD_ID="${BL_BUILD_ID:-$(date +%Y%m%d)}"
# at most 8 characters, and only ones that stay one word in CFLAGS (a short id padded with spaces split it)
BL_BUILD_ID="$(printf '%s' "$BL_BUILD_ID" | tr -c 'A-Za-z0-9._-' '_' | cut -c1-8)"
mkdir -p "$OUT"
ARCH="-march=rv32imc_zicsr_zifencei -mabi=ilp32"
# -fno-optimize-crc: GCC 15 would turn the bitwise CRC loops into 1.75 KB of lookup tables (speed isn't needed here)
CFLAGS="$ARCH -Os -g -ffunction-sections -fdata-sections -fno-common -msmall-data-limit=8 -ffreestanding \
 -fno-optimize-crc -Wall -Wextra -Wno-unused-parameter -Wno-unterminated-string-initialization -DCH32V20x_D6 -DBL_HW_REV=$BL_HW_REV -DBL_BUILD_ID=\"$BL_BUILD_ID\" \
 -I$FW/vendor/wch -I$FW/src/hal -I$FW/src/core -I$BL $EXTRA_CFLAGS"
"$GCC" $CFLAGS -x assembler-with-cpp -c "$BL/bl_start.S" -o "$OUT/bl_start.o"
OBJS="$OUT/bl_start.o"
for s in bl_main bl_hw bl_core bl_rx; do
  "$GCC" $CFLAGS -c "$BL/$s.c" -o "$OUT/$s.o"
  OBJS="$OBJS $OUT/$s.o"
done
"$GCC" -march=rv32imc -mabi=ilp32 -nostartfiles -nostdlib -Wl,--gc-sections -Wl,-Map="$OUT/bl.map" -T "$BL/bl.ld" \
  $OBJS -o "$OUT/bl.elf" -lgcc
"${PFX}objcopy" -O binary "$OUT/bl.elf" "$OUT/bl.bin"
"${PFX}objcopy" -O ihex "$OUT/bl.elf" "$OUT/bl.hex"
"${PFX}objdump" -d -S "$OUT/bl.elf" > "$OUT/bl.lst"
"${PFX}size" -A "$OUT/bl.elf" | grep -E "^\.(init|blinfo|text|data|bss|noinit|stack)|Total"
SIZE=$(stat -c %s "$OUT/bl.bin")
echo "bootloader: $SIZE of 8192 bytes flash (hw rev $BL_HW_REV, build $BL_BUILD_ID)"
if [ "$SIZE" -gt 8192 ]; then echo "bootloader too big"; exit 1; fi

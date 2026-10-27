#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Build the bootloader (8 KB at 0x08000000). Same compiler choice as tools/build_target.sh.
# Options (environment): OUT=<dir> (default build/bootloader), BOARD=<id or registry name> (default 1 =
# dig2analog-plus-5ch; its hardware id: core/ultra_update.h UPD_HW_ID_*), MCU=<family> (default ch32v20x),
# BL_HW_REV=<0..255> (encoded revision; default from the owning board definition),
# BL_BUILD_ID=<7 chars> (default: the id in bootloader/BL_BUILD_ID, one per bootloader code), EXTRA_CFLAGS.
# Output: bl.elf / .bin / .hex / .map / .lst, size report; fails if it doesn't fit 8 KB or its RAM.
set -e
FW="$(cd "$(dirname "$0")/.." && pwd)"
# the same registered MCU / board selection as the app (tools/build_target.sh)
TARGET_DEFS="$("${PYTHON:-python}" "$FW/tools/gen_targets.py" --shell --board "${BOARD:-1}" --mcu "${MCU:-ch32v20x}")" || exit 2
eval "$TARGET_DEFS"
BL="$FW/bootloader"
XPACK="$(ls -d "$LOCALAPPDATA"/xPacks/xpack-riscv-none-elf-gcc-*/bin 2>/dev/null | sort -V | tail -1)"
if [ -n "$RISCV_GCC" ]; then GCC="$RISCV_GCC"
elif [ -n "$XPACK" ]; then GCC="$XPACK/riscv-none-elf-gcc"
else echo "no RISC-V GCC (set RISCV_GCC)"; exit 1
fi
G="${GCC%.exe}"; PFX="${G%gcc}"      # (a RISCV_GCC ending in .exe, as the release workflow sets it)
OUT="${OUT:-$FW/build/bootloader}"
REV_DEF=""
if [ -n "${BL_HW_REV:-}" ]; then REV_DEF="-DBL_HW_REV=$BL_HW_REV"; fi
HWID_DEF=""                         # board 1: bl_core.h's default UPD_HW_ID_D2A_PLUS_5CH
if [ "$TARGET_BOARD" != 1 ]; then HWID_DEF="-DBL_HW_ID=$TARGET_BOARD"; fi
BL_BUILD_ID="${BL_BUILD_ID:-$(sed -n 's/^id[[:space:]]\{1,\}\([^[:space:]]*\).*/\1/p' "$BL/BL_BUILD_ID" | tr -d '\r')}"
# at most 7 characters, and only ones that stay one word in CFLAGS (a short id padded with spaces split it)
BL_BUILD_ID="$(printf '%s' "$BL_BUILD_ID" | tr -c 'A-Za-z0-9._-' '_' | cut -c1-7)"
mkdir -p "$OUT"
ARCH="-march=rv32imc_zicsr_zifencei -mabi=ilp32"
# -fno-optimize-crc: GCC 15 would turn the bitwise CRC loops into 1.75 KB of lookup tables (speed isn't needed here)
CFLAGS="$ARCH -Os -flto -g -ffunction-sections -fdata-sections -fno-common -msmall-data-limit=8 -ffreestanding \
 -fno-optimize-crc -Wall -Wextra -Wno-unused-parameter -Wno-unterminated-string-initialization -D$TARGET_DEFINE $HWID_DEF $REV_DEF -DBL_BUILD_ID=\"$BL_BUILD_ID\" \
 -I$FW/$TARGET_HAL -I$FW/src/core -I$BL -DMCU_$(printf %s "$TARGET_MCU" | tr a-z A-Z)=1 $EXTRA_CFLAGS"
"$GCC" $CFLAGS -x assembler-with-cpp -c "$BL/bl_start.S" -o "$OUT/bl_start.o"
OBJS="$OUT/bl_start.o"
for s in bl_main bl_hw bl_core bl_session bl_rx; do
  source="$BL/$s.c"
  if [ "$s" = bl_hw ]; then source="$FW/$TARGET_HAL/bl_hw.c"; fi
  "$GCC" $CFLAGS -c "$source" -o "$OUT/$s.o"
  OBJS="$OBJS $OUT/$s.o"
done
"$GCC" -march=rv32imc -mabi=ilp32 -nostartfiles -nostdlib -Wl,--gc-sections -Wl,-Map="$OUT/bl.map" -T "$BL/bl.ld" \
  -L"$FW/$TARGET_HAL" $OBJS -o "$OUT/bl.elf" -lgcc
"${PFX}objcopy" -O binary "$OUT/bl.elf" "$OUT/bl.bin"
"${PFX}objcopy" -O ihex "$OUT/bl.elf" "$OUT/bl.hex"
"${PFX}objdump" -d -S "$OUT/bl.elf" > "$OUT/bl.lst"
"${PFX}size" -A "$OUT/bl.elf" | grep -E "^\.(init|blinfo|text|data|bss|noinit|stack)|Total"
SIZE=$(stat -c %s "$OUT/bl.bin")
# the protected slot of the family (target.json boot_bytes, generated boot_api_layout.ld): 8 KiB V20x, 16 KiB V40x
BL_SLOT=$(( $(sed -n 's/^MCU_BOOT_BYTES = \(0x[0-9A-Fa-f]*\);$/\1/p' "$FW/$TARGET_HAL/boot_api_layout.ld") ))
echo "bootloader: $SIZE of $BL_SLOT bytes flash (hw $TARGET_BOARD rev ${BL_HW_REV:-board-default}, build $BL_BUILD_ID)"
if [ "$SIZE" -gt "$BL_SLOT" ]; then echo "bootloader too big"; exit 1; fi

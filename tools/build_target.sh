#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Build the firmware for the CH32V203 (QingKe V4, RV32IMAC; built as rv32imc, the atomics are not used).
# Compiler: $RISCV_GCC, else xPack riscv-none-elf-gcc (%LOCALAPPDATA%\xPacks\...), else PlatformIO's
# ESP32 RISC-V GCC 8.4 (compile check only).
# Output: build/target/fw.elf, .bin, .hex (flash with WCH-LinkUtility / wlink), .map, .lst and a size report.
# LDSCRIPT=<file>: another linker script; vendor/wch/Link_app.ld links the app behind the bootloader (0x00002100,
# tools/build_release.sh). Default: vendor/wch/Link.ld, a standalone image at 0x00000000.
set -e
FW="$(cd "$(dirname "$0")/.." && pwd)"
XPACK="$(ls -d "$LOCALAPPDATA"/xPacks/xpack-riscv-none-elf-gcc-*/bin 2>/dev/null | sort -V | tail -1)"
if [ -n "$RISCV_GCC" ]; then GCC="$RISCV_GCC"
elif [ -n "$XPACK" ]; then GCC="$XPACK/riscv-none-elf-gcc"
else GCC="$HOME/.platformio/packages/toolchain-riscv32-esp/bin/riscv32-esp-elf-gcc"
fi
PFX="${GCC%gcc}"
if "$GCC" --version | head -1 | grep -q " 8\."; then ARCH="-march=rv32imc -mabi=ilp32"; else ARCH="-march=rv32imc_zicsr_zifencei -mabi=ilp32"; fi
LDARCH="-march=rv32imc -mabi=ilp32"   # selects the rv32imc multilib (libgcc)
OUT="${OUT:-$FW/build/target}"
mkdir -p "$OUT"
echo "compiler: $("$GCC" --version | head -1)"
CFLAGS="$ARCH -Os -g -ffunction-sections -fdata-sections -fno-common -msmall-data-limit=8 \
 -Wall -Wextra -Wno-unused-parameter -DCH32V20x_D6 -I$FW/vendor/wch -I$FW/src/hal -I$FW/src $EXTRA_CFLAGS"
SRC="$FW/src/app/main.c $FW/src/hal/hw.c"
for f in "$FW"/src/core/*.c; do SRC="$SRC $f"; done
# GCC 15 turns bitwise CRC loops into 256-entry tables (-foptimize-crc). The firmware update's CRCs (upd_app.c) run
# once per update frame: built without them (1.8 KB). The CRC-8 / CRC-16 of ultra_protocol.h use nibble tables, which it
# leaves alone. Older compilers don't know the option.
NOCRC=""
COLD="${COLD_CFLAGS--msave-restore}"
if echo "int x;" | "$GCC" -fno-optimize-crc -x c -c - -o "$OUT/nocrc_probe.o" 2>/dev/null; then NOCRC="-fno-optimize-crc"; fi
rm -f "$OUT/nocrc_probe.o"
"$GCC" $CFLAGS -x assembler-with-cpp -c "$FW/vendor/wch/startup_ch32v20x_D6.S" -o "$OUT/startup.o"
"$GCC" $CFLAGS -x assembler-with-cpp -c "$FW/src/hal/isr.S" -o "$OUT/isr.o"
OBJS="$OUT/startup.o $OUT/isr.o"
for s in $SRC; do
  o="$OUT/$(basename "${s%.c}").o"
  f=""
  case "$s" in */upd_app.c) f="$NOCRC" ;; esac
  case "$s" in */cli.c|*/config.c|*/store.c|*/scene.c|*/calstore.c|*/upd_app.c|*/ack.c|*/climit.c|*/mcutemp.c) f="$f $COLD" ;; esac
  "$GCC" $CFLAGS $f -c "$s" -o "$o"
  OBJS="$OBJS $o"
done
LDSCRIPT="${LDSCRIPT:-$FW/vendor/wch/Link.ld}"
"$GCC" $LDARCH -nostartfiles -Wl,--gc-sections -Wl,-Map="$OUT/fw.map" -L"$FW/vendor/wch" -T "$LDSCRIPT" \
  $OBJS -o "$OUT/fw.elf" -lgcc
"${PFX}objcopy" -O binary "$OUT/fw.elf" "$OUT/fw.bin"
"${PFX}objcopy" -O ihex "$OUT/fw.elf" "$OUT/fw.hex"
"${PFX}objdump" -d -S "$OUT/fw.elf" > "$OUT/fw.lst"
"${PFX}size" -A "$OUT/fw.elf" | grep -E "^\.(init|vector|text|data|bss|stack)|Total"
"${PFX}size" "$OUT/fw.elf"

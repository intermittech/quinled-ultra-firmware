#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Release build: bootloader + app behind it + update image + production hex, for every variant below.
#   bash tools/build_release.sh <version> [out dir]        e.g.  bash tools/build_release.sh 0.5.0-beta.1
# Output (default build/release/<version>/):
#   quinled-dig2analog-ultra-5ch-<version>-<variant>.bin / .json   update image (descriptor + body) + metadata
#   quinled-dig2analog-ultra-5ch-<version>-<variant>-factory.hex   production: bootloader + app, verified + confirmed
#   bootloader-rev<N>.bin / .hex                                     the bootloader alone, per board revision
# BUILD_ID (default: git hash if any, else the date) goes into the descriptor and the bootloader info.
set -e
FW="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${1:?usage: build_release.sh <version> [out dir]}"
OUT="${2:-$FW/build/release/$VERSION}"
BUILD_ID="${BUILD_ID:-$(git -C "$FW" rev-parse --short=12 HEAD 2>/dev/null || date +%Y%m%d%H%M)}"
PY="${PYTHON:-python}"
mkdir -p "$OUT"

# variant | compiler flags | board revisions it runs on (mask) | board revision of its factory bootloader
#   hw1: PB11 wired to PWM_B (3 timers). Runs on revision 1 and on revision 2 (the current sense is simply unused).
#   Add "isense|-DHW_ISENSE=1|0x4|2" once the current-sense board is in production.
VARIANTS="hw1||0x6|1"

for v in $VARIANTS; do
  IFS='|' read -r name flags mask rev <<< "$v"
  echo "=== $name (revisions $mask, factory bootloader rev $rev)"
  BL_OUT="$OUT/obj/bl-rev$rev"
  OUT="$BL_OUT" BL_HW_REV="$rev" BL_BUILD_ID="$(printf '%-8.8s' "$BUILD_ID")" bash "$FW/bootloader/build_bl.sh" | tail -1
  cp "$BL_OUT/bl.bin" "$OUT/bootloader-rev$rev.bin"
  cp "$BL_OUT/bl.hex" "$OUT/bootloader-rev$rev.hex"
  APP_OUT="$OUT/obj/app-$name"
  OUT="$APP_OUT" EXTRA_CFLAGS="$flags -DUPD_APP=1 -DFW_VERSION=\"$VERSION\"" LDSCRIPT="$FW/vendor/wch/Link_app.ld" bash "$FW/tools/build_target.sh" | tail -1
  IMG="$OUT/quinled-dig2analog-ultra-5ch-$VERSION-$name.bin"
  "$PY" "$FW/tools/release/mkimage.py" --elf "$APP_OUT/fw.elf" --bin "$APP_OUT/fw.bin" --version "$VERSION" \
    --rev-mask "$mask" --build-id "$BUILD_ID" --out "$IMG"
  "$PY" "$FW/tools/release/mkfactory.py" --bootloader "$OUT/bootloader-rev$rev.bin" --image "$IMG" \
    --out "${IMG%.bin}-factory.hex"
done
echo "release files in $OUT"

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
# BL_BUILD_ID (8 characters): the bootloader info's, one per bootloader code: bootloader/BL_BUILD_ID (in the repo, so a
# release builds from the repo alone; a new id there when, and only when, the bootloader's code changes). The release
# stops when the bootloader it built doesn't match the md5 recorded there. BL_BUILD_ID=... overrides it (a test build:
# no md5 check).
set -e
FW="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${1:?usage: build_release.sh <version> [out dir]}"
OUT="${2:-$FW/build/release/$VERSION}"
BUILD_ID="${BUILD_ID:-$(git -C "$FW" rev-parse --short=12 HEAD 2>/dev/null || date +%Y%m%d%H%M)}"
PY="${PYTHON:-python}"
mkdir -p "$OUT"

# variant | compiler flags | board revisions it runs on (mask) | board revision of its factory bootloader
#   rev04: the dig2analog-ultra-5ch rev 0.4, board revision 3 (core/ultra_update.h UPD_REV_D2A_R04), the only one.
VARIANTS="rev04||0x8|3"

for v in $VARIANTS; do
  IFS='|' read -r name flags mask rev <<< "$v"
  echo "=== $name (revisions $mask, factory bootloader rev $rev)"
  BL_OUT="$OUT/obj/bl-rev$rev"
  OUT="$BL_OUT" BL_HW_REV="$rev" BL_BUILD_ID="${BL_BUILD_ID:-}" bash "$FW/bootloader/build_bl.sh" | tail -1
  if [ -z "${BL_BUILD_ID:-}" ] && [ "${UPPER_RECORDS:-1}" = 1 ]; then
    want="$(sed -n "s/^md5[[:space:]]\{1,\}rev$rev[[:space:]]\{1,\}\([0-9a-f]\{32\}\).*/\1/p" "$FW/bootloader/BL_BUILD_ID" | tr -d '\r')"
    got="$(md5sum "$BL_OUT/bl.bin" | cut -c1-32)"
    if [ "$got" != "$want" ]; then
      echo "error: the bootloader (rev $rev) is md5 $got; bootloader/BL_BUILD_ID records ${want:-none}. Its code changed:" \
           "a new id there and this md5 (or the toolchain isn't the pinned xPack GCC 15.2)" >&2
      exit 1
    fi
    echo "bootloader rev $rev: md5 $got, as bootloader/BL_BUILD_ID records"
  fi
  cp "$BL_OUT/bl.bin" "$OUT/bootloader-rev$rev.bin"
  cp "$BL_OUT/bl.hex" "$OUT/bootloader-rev$rev.hex"
  APP_OUT="$OUT/obj/app-$name"
  OUT="$APP_OUT" PERF=0 EXTRA_CFLAGS="$flags -DUPD_APP=1 -DFW_VERSION=\"$VERSION\"" LDSCRIPT="$FW/vendor/wch/Link_app.ld" bash "$FW/tools/build_target.sh" | tail -1
  # (round 9) reproducible from the repo alone: the same build again under another path gives the same bytes and the
  # same code order (tools/link_order.py; tools/check_repro.sh for any other image). REPRO=0 skips the second build.
  if [ "${REPRO:-1}" = 1 ]; then
    OUT="$APP_OUT-second-build" PERF=0 EXTRA_CFLAGS="$flags -DUPD_APP=1 -DFW_VERSION=\"$VERSION\"" LDSCRIPT="$FW/vendor/wch/Link_app.ld" bash "$FW/tools/build_target.sh" > /dev/null
    for f in fw.bin text_order.ld; do
      cmp -s "$APP_OUT/$f" "$APP_OUT-second-build/$f" || { echo "error: $name: $f differs between two builds of the same source" >&2; exit 1; }
    done
    echo "$name: reproducible (two builds, the same image and code order)"
  fi
  IMG="$OUT/quinled-dig2analog-ultra-5ch-$VERSION-$name.bin"
  "$PY" "$FW/tools/release/mkimage.py" --elf "$APP_OUT/fw.elf" --bin "$APP_OUT/fw.bin" --version "$VERSION" \
    --rev-mask "$mask" --build-id "$BUILD_ID" --out "$IMG"
  "$PY" "$FW/tools/release/mkfactory.py" --bootloader "$OUT/bootloader-rev$rev.bin" --image "$IMG" \
    --out "${IMG%.bin}-factory.hex"
done
echo "release files in $OUT"

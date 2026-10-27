#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Release build: bootloader + app behind it + update image + production hex, for every variant below.
#   bash tools/build_release.sh <version> [out dir]        e.g.  bash tools/build_release.sh 0.7.1
# Output (default build/release/<version>/):
#   quinled-dig2analog-plus-5ch-<version>-<variant>.bin / .json   update image (descriptor + body) + metadata
#   quinled-dig2analog-plus-5ch-<version>-<variant>-factory.hex   production: bootloader + app, verified + confirmed
#   bootloader-rev<N>.bin / .hex                                     the bootloader alone, per board revision
# UR_ANALOG_M=1: also the Diff-Power-UR-Analog-M's (diff-power-ur-analog-m-...,
# bootloader-hw2-rev<N>); UR_DIGITAL_M=1 the Diff-Power-UR-Digital-M's (diff-power-ur-digital-m-..., bootloader-hw3-rev<N>);
# each image's hardware id comes from its ELF (BOARD), never a default.
# BUILD_ID (default: git hash if any, else the date) goes into the descriptor and the bootloader info.
# BL_BUILD_ID (8 characters): the bootloader info's, one per bootloader code: bootloader/BL_BUILD_ID (in the repo, so a
# release builds from the repo alone; a new id there when, and only when, the bootloader's code changes). The release
# stops when the bootloader it built doesn't match the md5 recorded there. BL_BUILD_ID=... overrides it (a test build:
# no md5 check).
set -e
# Release packaging is off: firmware 0.7.1 is a source release. Signed images wait for the bench qualification of the
# bootloaders and line profiles; the V40x Digital-M (MCU=ch32v40x) has no variant below yet.
echo "firmware 0.7.1 is a source release: no release package until the bench qualification is done" >&2
exit 2
FW="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${1:?usage: build_release.sh <version> [out dir]}"
OUT="${2:-$FW/build/release/$VERSION}"
BUILD_ID="${BUILD_ID:-$(git -C "$FW" rev-parse --short=12 HEAD 2>/dev/null || date +%Y%m%d%H%M)}"
PY="${PYTHON:-python}"
mkdir -p "$OUT"

# variant | board revisions it runs on (mask) | board revision of its factory bootloader | hardware
# id (BOARD) | the files' name
#   rev04: the dig2analog-plus-5ch rev 0.4, board revision 3 (core/ultra_update.h UPD_REV_D2A_R04), the only one.
#   ur_analog_m-rev0: the Diff-Power-UR-Analog-M (BOARD=2, hardware id 2; mkimage.py takes it
#   from the ELF, the bootloader from build_bl.sh's BOARD), its first board revision 0 (provisional: no receiver hardware yet).
#   UR_ANALOG_M=1 builds it too; its bootloader's md5 goes into bootloader/BL_BUILD_ID as "md5 hw2-rev0 <md5>".
#   ur_digital_m-rev0: the Diff-Power-UR-Digital-M (BOARD=3, hardware id 3), board
#   revision 0 (v0r1, provisional: not built yet). UR_DIGITAL_M=1 builds it; "md5 hw3-rev0 <md5>" in bootloader/BL_BUILD_ID.
VARIANTS="rev04|0x8|3|1|quinled-dig2analog-plus-5ch"
if [ "${UR_ANALOG_M:-0}" = 1 ]; then VARIANTS="$VARIANTS ur_analog_m-rev0|0x1|0|2|diff-power-ur-analog-m"; fi
if [ "${UR_DIGITAL_M:-0}" = 1 ]; then VARIANTS="$VARIANTS ur_digital_m-rev0|0x1|0|3|diff-power-ur-digital-m"; fi

for v in $VARIANTS; do
  IFS='|' read -r name mask rev hw product <<< "$v"
  echo "=== $name (hardware $hw, revisions $mask, factory bootloader rev $rev)"
  key="rev$rev"
  if [ "$hw" != 1 ]; then key="hw$hw-rev$rev"; fi   # (hardware 1: no hw prefix)
  BL_OUT="$OUT/obj/bl-$key"
  OUT="$BL_OUT" BOARD="$hw" BL_HW_REV="$rev" BL_BUILD_ID="${BL_BUILD_ID:-}" bash "$FW/bootloader/build_bl.sh" | tail -1
  if [ -z "${BL_BUILD_ID:-}" ]; then
    want="$(sed -n "s/^md5[[:space:]]\{1,\}$key[[:space:]]\{1,\}\([0-9a-f]\{32\}\).*/\1/p" "$FW/bootloader/BL_BUILD_ID" | tr -d '\r')"
    got="$(md5sum "$BL_OUT/bl.bin" | cut -c1-32)"
    if [ "$got" != "$want" ]; then
      echo "error: the bootloader ($key) is md5 $got; bootloader/BL_BUILD_ID records ${want:-none}. Its code changed:" \
           "a new id there and this md5 (or the toolchain isn't the pinned xPack GCC 15.2)" >&2
      exit 1
    fi
    echo "bootloader $key: md5 $got, as bootloader/BL_BUILD_ID records"
  fi
  cp "$BL_OUT/bl.bin" "$OUT/bootloader-$key.bin"
  cp "$BL_OUT/bl.hex" "$OUT/bootloader-$key.hex"
  APP_OUT="$OUT/obj/app-$name"
  OUT="$APP_OUT" PERF=0 BOARD="$hw" EXTRA_CFLAGS="-DUPD_APP=1 -DFW_VERSION=\"$VERSION\"" LDSCRIPT="$FW/src/mcu/ch32v20x/Link_app.ld" bash "$FW/tools/build_target.sh" | tail -1
  # reproducible from the repo alone: the same build again under another path gives the same bytes and the
  # same code order (tools/link_order.py; tools/check_repro.sh for any other image). REPRO=0 skips the second build.
  if [ "${REPRO:-1}" = 1 ]; then
    OUT="$APP_OUT-second-build" PERF=0 BOARD="$hw" EXTRA_CFLAGS="-DUPD_APP=1 -DFW_VERSION=\"$VERSION\"" LDSCRIPT="$FW/src/mcu/ch32v20x/Link_app.ld" bash "$FW/tools/build_target.sh" > /dev/null
    for f in fw.bin text_order.ld; do
      cmp -s "$APP_OUT/$f" "$APP_OUT-second-build/$f" || { echo "error: $name: $f differs between two builds of the same source" >&2; exit 1; }
    done
    echo "$name: reproducible (two builds, the same image and code order)"
  fi
  # the Diff-Power-UR-Digital-M's target tests on its compiled code in tools/rv32emu.py: test/test_ur_digital_m_pages.py (every
  # page's answer, the capture path, the status LED) and test/r14/test_ur_digital_m_emulated.py (decoding, frame lengths, the
  # ring's capture and SEND). Its build with the functions as written (LTO=0, no inlining or IPA clones: they keep their
  # arguments). UDTEST=0 skips them.
  if [ "$hw" = 3 ] && [ "${UDTEST:-1}" = 1 ]; then
    API_OUT="$OUT/obj/ur_digital_m-api"
    OUT="$API_OUT" LTO=0 PERF=0 BOARD="$hw" EXTRA_CFLAGS="-fno-inline -fno-ipa-cp -fno-ipa-sra" bash "$FW/tools/build_target.sh" > /dev/null
    r="$("$PY" "$FW/test/test_ur_digital_m_pages.py" "$API_OUT/fw.elf" 2>&1)" ||
      { echo "$r" >&2; echo "error: $name: test/test_ur_digital_m_pages.py failed" >&2; exit 1; }
    echo "$r" | tail -1
    for c in good length ring; do
      r="$("$PY" "$FW/test/r14/test_ur_digital_m_emulated.py" --case "$c" --elf "$API_OUT/fw.elf" 2>&1)" ||
        { echo "$r" >&2; echo "error: $name: test/r14/test_ur_digital_m_emulated.py --case $c failed" >&2; exit 1; }
      echo "$r" | tail -1
    done
    # the local pixel effects (docs/ur_digital_m-firmware.md section 11): the record's flash path and the lit / dark rule, and a
    # poll answer while an effect runs (the strip keeps updating), on the compiled firmware, the presets at +-2.2 % HSI, the encoder's and the renderer's cycles in rv32emu
    # the chain line mode (section 12): AUTO deciding CHAIN on a normal Ultra line, the strip, the answer's slot, the pages, on the compiled firmware;
    # test_ur_digital_m_chain_budget.py: frames back to back at the hub's 300 us reset with the main loop preempted by the tick (every frame captured, 100 us to spare)
    for t in test/test_ur_digital_m_record_flash.py test/test_ur_digital_m_local_poll.py test/test_ur_digital_m_publish.py test/test_ur_digital_m_clock_bounds.py test/test_ur_digital_m_chain_fw.py test/test_ur_digital_m_chain_budget.py tools/ur_digital_m_timing_check.py test/test_ur_digital_m_fx_cycles.py test/test_ur_digital_m_scene_steps.py; do
      r="$("$PY" "$FW/$t" $([ "$t" = test/test_ur_digital_m_record_flash.py ] || [ "$t" = test/test_ur_digital_m_local_poll.py ] || [ "$t" = test/test_ur_digital_m_publish.py ] || [ "$t" = test/test_ur_digital_m_clock_bounds.py ] || [ "$t" = test/test_ur_digital_m_chain_fw.py ] || [ "$t" = test/test_ur_digital_m_chain_budget.py ] && echo "$API_OUT/fw.elf") 2>&1)" ||
        { echo "$r" >&2; echo "error: $name: $t failed" >&2; exit 1; }
      echo "$r" | tail -1
    done
    for t in calibration cal_store no_pacifica reserved_chip; do
      "$PY" "$FW/test/test_ud_$t.py" "$API_OUT/fw.elf"
    done
    "$PY" "$FW/test/test_ur_digital_m_cal_cycles.py" "$APP_OUT/fw.elf"
  fi
  IMG="$OUT/$product-$VERSION-$name.bin"
  "$PY" "$FW/tools/release/mkimage.py" --elf "$APP_OUT/fw.elf" --bin "$APP_OUT/fw.bin" --version "$VERSION" \
    --rev-mask "$mask" --build-id "$BUILD_ID" --out "$IMG"
  "$PY" "$FW/tools/release/mkfactory.py" --bootloader "$OUT/bootloader-$key.bin" --image "$IMG" \
    --out "${IMG%.bin}-factory.hex"
done

# The supported receiver reply binding is USART1/PA9 on both boards.
for board in 2 3; do
  if { [ "$board" = 2 ] && [ "${UR_ANALOG_M:-0}" != 1 ]; } || { [ "$board" = 3 ] && [ "${UR_DIGITAL_M:-0}" != 1 ]; }; then continue; fi
  API_OUT="$OUT/obj/answer-api-$board"
  OUT="$API_OUT" LTO=0 PERF=0 BOARD=$board EXTRA_CFLAGS="-fno-inline -fno-ipa-cp -fno-ipa-sra" bash "$FW/tools/build_target.sh" > /dev/null
  "$PY" "$FW/test/test_answer_uart.py" "$API_OUT/fw.elf" "$board" 1
done
echo "release files in $OUT"

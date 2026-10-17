#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# Release build: bootloader + app behind it + update image + production hex, for every variant below.
#   bash tools/build_release.sh <version> [out dir]        e.g.  bash tools/build_release.sh 0.5.0-beta.1
# Output (default build/release/<version>/):
#   quinled-dig2analog-ultra-5ch-<version>-<variant>.bin / .json   update image (descriptor + body) + metadata
#   quinled-dig2analog-ultra-5ch-<version>-<variant>-factory.hex   production: bootloader + app, verified + confirmed
#   bootloader-rev<N>.bin / .hex                                     the bootloader alone, per board revision
# (round 11, B2) RX=1: also the QuinLED Diff-Power Ultra-Analog receiver's (quinled-diff-power-ultra-analog-...,
# bootloader-hw2-rev<N>); UD=1 the Ultra-Digital receiver's (quinled-diff-power-ultra-digital-..., bootloader-hw3-rev<N>);
# each image's hardware id comes from its ELF (BOARD), never a default.
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

# variant | compiler flags | board revisions it runs on (mask) | board revision of its factory bootloader | hardware
# id | the files' name
#   rev04: the dig2analog-ultra-5ch rev 0.4, board revision 3 (core/ultra_update.h UPD_REV_D2A_R04), the only one.
#   (round 11, B2) rx-rev0: the QuinLED Diff-Power Ultra-Analog receiver (BOARD=2, hardware id 2; mkimage.py takes it
#   from the ELF, the bootloader from build_bl.sh's BL_HW_ID), its first board revision 0 (provisional: no receiver hardware yet).
#   RX=1 builds it too; its bootloader's md5 goes into bootloader/BL_BUILD_ID as "md5 hw2-rev0 <md5>".
#   ud-rev0: the QuinLED Diff-Power Ultra-Digital receiver (BOARD=3, hardware id 3: the protocol 0.2.7 proposal), board
#   revision 0 (v0r1, provisional: not built yet). UD=1 builds it; "md5 hw3-rev0 <md5>" in bootloader/BL_BUILD_ID.
VARIANTS="rev04||0x8|3|1|quinled-dig2analog-ultra-5ch"
if [ "${RX:-0}" = 1 ]; then VARIANTS="$VARIANTS rx-rev0|-DBOARD=2|0x1|0|2|quinled-diff-power-ultra-analog"; fi
if [ "${UD:-0}" = 1 ]; then VARIANTS="$VARIANTS ud-rev0|-DBOARD=3|0x1|0|3|quinled-diff-power-ultra-digital"; fi

for v in $VARIANTS; do
  IFS='|' read -r name flags mask rev hw product <<< "$v"
  echo "=== $name (hardware $hw, revisions $mask, factory bootloader rev $rev)"
  key="rev$rev"
  if [ "$hw" != 1 ]; then key="hw$hw-rev$rev"; fi   # (hardware 1: the files as before)
  BL_OUT="$OUT/obj/bl-$key"
  OUT="$BL_OUT" BL_HW_ID="$hw" BL_HW_REV="$rev" BL_BUILD_ID="${BL_BUILD_ID:-}" bash "$FW/bootloader/build_bl.sh" | tail -1
  if [ -z "${BL_BUILD_ID:-}" ] && [ "${UPPER_RECORDS:-1}" = 1 ]; then
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
  # the Ultra-Digital receiver's target tests on its compiled code in tools/rv32emu.py: test/test_ud_pages.py (every
  # page's answer, the capture path, the status LED) and test/r14/test_ud_emulated.py (decoding, frame lengths, the
  # ring's capture and SEND). Its build with the functions as written (LTO=0, no inlining or IPA clones: they keep their
  # arguments). UDTEST=0 skips them.
  if [ "$hw" = 3 ] && [ "${UDTEST:-1}" = 1 ]; then
    API_OUT="$OUT/obj/ud-api"
    OUT="$API_OUT" LTO=0 PERF=0 EXTRA_CFLAGS="$flags -fno-inline -fno-ipa-cp -fno-ipa-sra" bash "$FW/tools/build_target.sh" > /dev/null
    r="$("$PY" "$FW/test/test_ud_pages.py" "$API_OUT/fw.elf" 2>&1)" ||
      { echo "$r" >&2; echo "error: $name: test/test_ud_pages.py failed" >&2; exit 1; }
    echo "$r" | tail -1
    for c in good length ring; do
      r="$("$PY" "$FW/test/r14/test_ud_emulated.py" --case "$c" --elf "$API_OUT/fw.elf" 2>&1)" ||
        { echo "$r" >&2; echo "error: $name: test/r14/test_ud_emulated.py --case $c failed" >&2; exit 1; }
      echo "$r" | tail -1
    done
    # the local pixel effects (docs/ultradigital-firmware.md section 11): the record's flash path and the lit / dark rule, and a
    # poll answer while an effect runs (the strip keeps updating), on the compiled firmware, the presets at +-2.2 % HSI, the encoder's and the renderer's cycles in rv32emu
    # the chain line mode (section 12): AUTO deciding CHAIN on a normal Ultra line, the strip, the answer's slot, the pages, on the compiled firmware;
    # test_ud_chain_budget.py: frames back to back at the hub's 300 us reset with the main loop preempted by the tick (every frame captured, 100 us to spare)
    for t in test/test_ud_record_flash.py test/test_ud_local_poll.py test/test_ud_publish.py test/test_ud_clock_bounds.py test/test_ud_chain_fw.py test/test_ud_chain_budget.py tools/ud_timing_check.py test/test_ud_fx_cycles.py test/test_ud_scene_steps.py; do
      r="$("$PY" "$FW/$t" $([ "$t" = test/test_ud_record_flash.py ] || [ "$t" = test/test_ud_local_poll.py ] || [ "$t" = test/test_ud_publish.py ] || [ "$t" = test/test_ud_clock_bounds.py ] || [ "$t" = test/test_ud_chain_fw.py ] || [ "$t" = test/test_ud_chain_budget.py ] && echo "$API_OUT/fw.elf") 2>&1)" ||
        { echo "$r" >&2; echo "error: $name: $t failed" >&2; exit 1; }
      echo "$r" | tail -1
    done
  fi
  IMG="$OUT/$product-$VERSION-$name.bin"
  "$PY" "$FW/tools/release/mkimage.py" --elf "$APP_OUT/fw.elf" --bin "$APP_OUT/fw.bin" --version "$VERSION" \
    --rev-mask "$mask" --build-id "$BUILD_ID" --out "$IMG"
  "$PY" "$FW/tools/release/mkfactory.py" --bootloader "$OUT/bootloader-$key.bin" --image "$IMG" \
    --out "${IMG%.bin}-factory.hex"
done

# The receivers' other answer-UART settings (board.h ANSWER_UART; the DI net keeps a 0 R choice between the pins, so the
# firmware follows it): 2 = USART2 / PA2 on both receivers (the console's TX off, its text in a RAM ring), 1 = USART1 /
# PA9 on the Ultra-Digital receiver. Built and tested with every release that builds the receivers, never shipped, so
# the pivot can't rot: the app image behind the bootloader (it fits the slot), the functions-as-written build with
# test/test_answer_uart.py (the burst, the dividers, the pins, the interrupt; the LTO build keeps the console ring) and
# the compiled-code tests of the board (BOARD 3: pages, capture, record flash, local poll; BOARD 2: tools/hal_check.py
# with this setting's interrupt). Sizes: tools/size_gate.py. ANSWER_UART_MATRIX=0 skips them.
if [ "${ANSWER_UART_MATRIX:-1}" = 1 ]; then
  AU_VARIANTS=""
  if [ "${RX:-0}" = 1 ]; then AU_VARIANTS="$AU_VARIANTS rx-au2|2|2"; fi
  if [ "${UD:-0}" = 1 ]; then AU_VARIANTS="$AU_VARIANTS ud-au2|3|2 ud-au1|3|1"; fi
  for v in $AU_VARIANTS; do
    IFS='|' read -r name board au <<< "$v"
    flags="-DBOARD=$board -DANSWER_UART=$au"
    echo "=== $name (answer UART $au on BOARD $board: built and tested, not shipped)"
    APP_OUT="$OUT/obj/app-$name"
    OUT="$APP_OUT" PERF=0 EXTRA_CFLAGS="$flags -DUPD_APP=1 -DFW_VERSION=\"$VERSION\"" LDSCRIPT="$FW/vendor/wch/Link_app.ld" \
      bash "$FW/tools/build_target.sh" | tail -1
    API_OUT="$OUT/obj/au-api-$name"
    apiflags="$flags"
    if [ "$board" = 3 ]; then apiflags="$flags -fno-inline -fno-ipa-cp -fno-ipa-sra"; fi   # (as the ud-api build above)
    OUT="$API_OUT" LTO=0 PERF=0 EXTRA_CFLAGS="$apiflags" bash "$FW/tools/build_target.sh" > /dev/null
    r="$("$PY" "$FW/test/test_answer_uart.py" "$API_OUT/fw.elf" "$board" "$au" "$APP_OUT/fw.elf" 2>&1)" ||
      { echo "$r" >&2; echo "error: $name: test/test_answer_uart.py failed" >&2; exit 1; }
    echo "$r" | tail -1
    if [ "$board" = 3 ]; then
      r="$("$PY" "$FW/test/test_ud_pages.py" "$API_OUT/fw.elf" 2>&1)" ||
        { echo "$r" >&2; echo "error: $name: test/test_ud_pages.py failed" >&2; exit 1; }
      echo "$r" | tail -1
      for c in good length ring; do
        r="$("$PY" "$FW/test/r14/test_ud_emulated.py" --case "$c" --elf "$API_OUT/fw.elf" 2>&1)" ||
          { echo "$r" >&2; echo "error: $name: test/r14/test_ud_emulated.py --case $c failed" >&2; exit 1; }
        echo "$r" | tail -1
      done
      for t in test/test_ud_record_flash.py test/test_ud_local_poll.py; do
        r="$("$PY" "$FW/$t" "$API_OUT/fw.elf" 2>&1)" ||
          { echo "$r" >&2; echo "error: $name: $t failed" >&2; exit 1; }
        echo "$r" | tail -1
      done
    else
      r="$(ANSWER_UART=$au "$PY" "$FW/tools/hal_check.py" "$API_OUT/fw.elf" h2 h6 adc flash option_bytes records vectors 2>&1)" ||
        { echo "$r" >&2; echo "error: $name: tools/hal_check.py failed" >&2; exit 1; }
      echo "$r" | tail -1
    fi
  done
fi
echo "release files in $OUT"

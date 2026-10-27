#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
cd "$(dirname "$0")/.."
: "${WCH_GCC_DIR:?set WCH_GCC_DIR to WCH GCC 15 (tools/toolchain)}"
for pad in "$@"; do
  def="-DUDFX_RENDER_PAD=$pad"; [ "$pad" = 0 ] && def=""
  XW=0 LTO=0 BOARD=3 EXTRA_CFLAGS="-fno-inline -fno-ipa-cp -fno-ipa-sra $def" OUT=build/pad-$pad bash tools/build_target.sh > build/pad-$pad.log 2>&1 && echo "pad $pad built" || { echo "pad $pad FAILED"; tail -3 build/pad-$pad.log; }
done

#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
cd "$(dirname "$0")/.."
export WCH_GCC_DIR="/d/Claude-Folders/Claude-WS2915_dig2analog/tools-ext/mounriver/tc/RISC-V_Embedded_GCC15"
for pad in "$@"; do
  def="-DUDFX_RENDER_PAD=$pad"; [ "$pad" = 0 ] && def=""
  XW=0 LTO=0 EXTRA_CFLAGS="-DBOARD=3 -fno-inline -fno-ipa-cp -fno-ipa-sra $def" OUT=build/pad-$pad bash tools/build_target.sh > build/pad-$pad.log 2>&1 && echo "pad $pad built" || { echo "pad $pad FAILED"; tail -3 build/pad-$pad.log; }
done

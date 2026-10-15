#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# check_repro.sh <out dir>: the image built twice gives the same bytes. tools/build_target.sh with the caller's
# environment (XW, EXTRA_CFLAGS, LDSCRIPT, ...) into <out dir>/a and <out dir>/second-build (another path, so a path
# that leaks into the output shows too); compares what a release ships (fw.bin, fw.hex) and the code's order the
# build computed (text_order.ld, tools/link_order.py). Exit 1 on a difference. tools/build_release.sh checks its app
# images the same way.
#   bash tools/check_repro.sh build/repro                                   the standalone image
#   XW=0 EXTRA_CFLAGS="-DBOARD=2 -DUPD_APP=1" LDSCRIPT=$PWD/vendor/wch/Link_app.ld bash tools/check_repro.sh build/repro
FW="$(cd "$(dirname "$0")/.." && pwd)"
D="${1:?usage: check_repro.sh <out dir>}"
rm -rf "$D/a" "$D/second-build"
mkdir -p "$D" || exit 1                   # (round 9 phase B: a fresh tree has no parent folder for the logs yet)
for o in a second-build; do
  if ! OUT="$D/$o" bash "$FW/tools/build_target.sh" > "$D/$o.log" 2>&1; then
    echo "check_repro: the build into $D/$o failed ($D/$o.log)" >&2
    exit 1
  fi
done
bad=0
for f in fw.bin fw.hex text_order.ld; do
  if ! cmp -s "$D/a/$f" "$D/second-build/$f"; then
    echo "check_repro: $f differs between two builds of the same source" >&2
    bad=1
  fi
done
[ "$bad" = 0 ] && echo "check_repro: two builds, the same image ($(stat -c %s "$D/a/fw.bin") B; $(head -1 "$D/a/text_order.ld" | cut -c1-60)...)"
exit "$bad"

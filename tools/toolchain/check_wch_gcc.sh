#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
# check_wch_gcc.sh <WCH GCC folder> [<the 7z>]: the WCH GCC 15.2 a build uses (tools/build_target.sh XW=1) is the pinned
# one: its version and the files listed in wch-gcc15-win64.sha256 (the compiler, the LTO pieces, as, ld, objcopy,
# objdump, size, the rv32imc_xw libgcc). With the 7z (the archive MounRiver Studio 2.5.0 installs it from): that too.
# Exit 0 = pinned; 1 = not. README.md (this folder): where it comes from.
HERE="$(cd "$(dirname "$0")" && pwd)"
D="${1:?usage: check_wch_gcc.sh <WCH GCC folder> [<the 7z>]}"
LIST="$HERE/wch-gcc15-win64.sha256"
GCC="$D/bin/riscv32-wch-elf-gcc"
V="$("$GCC" --version 2>/dev/null | head -1)"
case "$V" in
  *"(g5115c7e44-dirty) 15.2.0"*) ;;
  *) echo "WCH GCC: $GCC is not the pinned 15.2.0 (g5115c7e44-dirty): '$V'" >&2; exit 1 ;;
esac
bad=0
while read -r sum path; do
  case "$sum" in ''|'#'*) continue ;; esac
  if [ "$path" = "RISC-V Embedded GCC15.7z" ]; then
    [ -n "$2" ] || continue
    f="$2"
  else
    f="$D/$path"
  fi
  got="$(sha256sum "$f" 2>/dev/null | cut -c1-64)"
  if [ "$got" != "$sum" ]; then
    echo "WCH GCC: $path differs from the pinned toolchain (${got:-missing})" >&2
    bad=1
  fi
done < <(sed 's/\r$//' "$LIST")
exit $bad

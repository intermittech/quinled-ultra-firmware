# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compare every analog page/bank/part against the pre-refactor compiled ELF.

Usage: python test/test_rc_pack_equivalence.py before-api.elf after-api.elf
Both: BOARD=2 XW=0 LTO=0, -fno-inline -fno-ipa-cp -fno-ipa-sra.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu

a, b = [Emu(p) for p in sys.argv[1:3]]
for e in (a, b):
    e.call('ultra_init', [e.addr('ul'), 123])
    for name in ('out_now', 'frames', 'rc_windows', 'rc_answers', 'rc_missed', 'rc_aborts'):
        size = e.sym(name)[1]
        e.poke(e.addr(name), bytes((i * 37 + 19) % 256 for i in range(size)))
checks = 0
for page in range(256):
    for part in (0, 1, 2, 255):
        for bank in (0, 1):
            results = []
            for e in (a, b):
                arg = e.addr('din_ring')
                dest = arg + 64
                e.poke(arg, bytes((part, 0, 0, bank)))
                e.poke(dest, bytes(32))
                e.call('rc_page', [page, arg, dest])
                results.append((e.ret, e.peek(dest, 32)))
            assert results[0] == results[1], (page, part, bank, results)
            checks += 1
print(f'Compiled analog pages: {checks} before/after cases pass (including 8 guard bytes)')

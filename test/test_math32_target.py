# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Exercise the real RV32 limb division (including MULHU) against Python integers.
Usage: python test/test_math32_target.py <BOARD=2 no-inline API ELF>
The exhaustive/range host oracle is test_math32.c; this checks the target path.
"""
import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu

e = Emu(sys.argv[1])
r = random.Random(0x3218038)
mask = (1 << 32) - 1
checks = 0
worst = {}
for i in range(1200):
    a, b, d, dh = (r.getrandbits(32) for _ in range(4))
    if i < 400:
        dh &= 255
    if i < 200:
        dh = 0
    d |= 1
    for fn, args, answer in (
        ('m32_div', [a, b, d], ((a << 32) | b) // d),
        ('m32_divwide', [a, b, dh, d], ((a << 32) | b) // ((dh << 32) | d)),
    ):
        e.call(fn, args)
        assert e.ret == answer & mask, (fn, args, e.ret, answer)
        worst[fn] = max(worst.get(fn, 0), e.cycles())
        checks += 1
print(f'math32 RV32: {checks} exact integer comparisons passed; worst estimated cycles {worst}')

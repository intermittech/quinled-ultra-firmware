# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Check the compiled gamma curve against the uncompressed reference.

Build an API image with --undefined=fx_gamma16_of. Effect rounding permits one
16-bit step; 8-bit output, monotonicity and both endpoints remain exact.
"""
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu

for path in sys.argv[1:]:
    e=Emu(path)
    values=[];cycles=[]
    for value in range(256):
        e.call('fx_gamma16_of',[value])
        values.append(e.ret);cycles.append(e.cycles())
    exact = []
    for v in range(256):
        g = math.floor(65280 * (v/255)**2.8 + .5)
        byte = math.floor(255 * (v/255)**2.8 + .5)
        exact.append(min(max(g, byte*256-128), byte*256+127))
    assert all(abs(a-b) <= 1 for a, b in zip(values, exact))
    assert all((a+128)>>8 == (b+128)>>8 for a, b in zip(values, exact))
    assert values[0] == 0 and values[-1] == 65280
    assert all(a <= b for a, b in zip(values, values[1:]))
    print(f'{path}: all 256 gamma values within one step, exact 8-bit bytes; '
          f'{min(cycles)}..{max(cycles)} model cycles/call')

# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Signed microsecond boundaries and rebasing in the compiled BOARD=3 clock.

python test/test_ur_digital_m_clock_bounds.py build/ur_digital_m-api/fw.elf
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu


e = Emu(sys.argv[1])
u = e.addr('ux')
checks = fails = 0
for delta in (-2147483648, -2147483647, -2147482650, -2147482649,
              -1001, -1000, -999, -1, 0, 1, 999, 1000, 2147483647):
    e.call('udfx_init', [u])
    expected = (delta // 1000) & 0xffffffff
    e.call('ux_ms', [u, delta & 0xffffffff])
    checks += 1
    if e.ret != expected:
        print('FAIL ux_ms delta=%d got=%u expected=%u' % (delta, e.ret, expected))
        fails += 1
    e.call('ux_ms_advance', [u, delta & 0xffffffff])
    e.call('ux_ms', [u, delta & 0xffffffff])
    checks += 1
    if e.ret != expected:
        print('FAIL rebased delta=%d got=%u expected=%u' % (delta, e.ret, expected))
        fails += 1
print('test_ud_clock_bounds: %d checks, %d failed' % (checks, fails))
sys.exit(bool(fails))

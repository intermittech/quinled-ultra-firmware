# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Diff-Power-UR-Digital-M r2 pin ownership in compiled application and bootloader code."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from hal_check import Hal

PA, PB, U = 0x40010800, 0x40010c00, 0x40013800
def cfg(e, port, pin):
    return (e.rd(port + (0 if pin < 8 else 4), 4) >> ((pin & 7) * 4)) & 15

def fresh(path):
    e = Hal(path)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 8,
                   PA: 0x44444444, PA + 4: 0x44444444,
                   PB: 0x44444444, PB + 4: 0x44444444})
    return e

e = fresh(sys.argv[1]); e.call('hw_init')
assert cfg(e, PA, 9) == 11 and e.rd(U + 8, 2) == 72
assert cfg(e, PA, 8) == 3 and cfg(e, PB, 1) == 3
assert cfg(e, PB, 10) == 3
assert cfg(e, PA, 1) == 4 and cfg(e, PA, 10) == 4
assert cfg(e, PB, 11) == 0 and cfg(e, PB, 14) == 0
e.writes.clear(); e.call('rc_hw_dir', [1])
assert e.writes.index((PA + 0x14, 1 << 8)) < e.writes.index((PB + 0x10, 1 << 10))
e.call('rc_hw_dir', [0]); assert (PB + 0x14, 1 << 10) in e.writes
e.writes.clear(); e.call('hw_flash_trip')
assert (PB + 0x14, (1 << 1) | (1 << 10)) in e.writes
assert not any(a in (PA + 0x10, PA + 0x14) and v & ((1 << 1) | (1 << 10) | (1 << 17) | (1 << 26))
               for a,v in e.writes)
print('UR_DIGITAL_M application: PA9 reply, PB10 DE, PA8 blank, PB1 mux, PA1/PA10 inputs, PB11/PB14 unused, flash-safe release: PASS')

if len(sys.argv) > 2:
    b = fresh(sys.argv[2]); b.call('hw_blank_early')
    assert (PA + 0x14, 1 << 8) in b.writes and cfg(b, PA, 8) == 3
    assert cfg(b, PA, 1) == 4
    b.call('hw_full_init')
    assert cfg(b, PA, 8) == 3 and cfg(b, PB, 1) == 3 and cfg(b, PB, 10) == 3
    assert cfg(b, PA, 1) == 4 and cfg(b, PA, 10) == 4 and cfg(b, PA, 9) == 4
    assert cfg(b, PB, 11) == 4 and cfg(b, PB, 14) == 4
    print('UR_DIGITAL_M bootloader: early blank on PA8, PB1 direct and PB10 listening, chain pins and unused pins undriven: PASS')

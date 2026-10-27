# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Pivot-A PWM and return timing on compiled BOARD=2 functions in rv32emu.

The register model proves software polarity/off state and the compiled byte
service cost. Baud comes from BRR/PCLK2; electrical timing remains a bench item.
Usage: python test/test_pivot_a.py <BOARD=2 LTO=0 fw.elf> [hw2 bl.elf]
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from hal_check import Hal, start, T1

PA, PB, U, CNT = 0x40010800, 0x40010c00, 0x40013800, 0xe000f008

class ReplyHal(Hal):
    def __init__(self, path):
        super().__init__(path)
        self.count = 0
    def rd(self, addr, n):
        if addr == CNT:
            self.count += 64
            return self.count % 7200
        return super().rd(addr, n)

def cfg(e, port, pin):
    return (e.rd(port + (0 if pin < 8 else 4), 4) >> ((pin & 7) * 4)) & 15

e = ReplyHal(sys.argv[1]); e.call('hw_init')
assert cfg(e, PB, 14) == 11 and cfg(e, PA, 9) == 8
assert cfg(e, PB, 10) == 3
for level in (0, 32768, 65535):
    start(e, 8, 0, level)
    # CH2N with NP=1 follows OC2REF, including the forced LOW trip mode.
    assert e.rd(T1 + 0x20, 2) & 0xf0 == 0xc0
    assert e.rd(T1 + 0x44, 2) == 0x8000  # MOE, no dead-time or break
    e.call('pwm_force_off', [1])
    assert (e.rd(T1 + 0x18, 2) >> 12) & 7 == 4
    assert e.rd(T1 + 0x20, 2) & 0xf0 == 0xc0
    e.call('pwm_force_off', [0])
e.call('hw_flash_trip')
assert (e.rd(T1 + 0x18, 2) >> 12) & 7 == 4
assert e.rd(T1 + 0x20, 2) & 0xf0 == 0xc0
print('PWM R: PB14 CH2N, active-high polarity, zero dead-time, trip/flash LOW: PASS')

buf, sched, poll = 0x20003000, 0x20003100, 0x20003200
payload = bytes(range(32))
for rate, pitch in enumerate((500, 835, 1505)):
    worst_service = 0
    for phase in (0, 6700, 7000, 7100, 7190):
        e = ReplyHal(sys.argv[1]); e.call('hw_init'); e.count = phase
        e.poke(buf, payload); e.mmio[PA + 8] = 1 << 6
        e.call('rc_hw_dir', [1]); e.writes.clear()
        e.mmio[PA + 8] = 0  # disabled RO cannot veto TX
        before = e.count
        e.call('rc_hw_send', [buf, 32, 72 << rate])
        assert e.ret == 0
        assert e.count - before >= 432
        for _ in range(32):
            e.call('rc_uart_isr'); worst_service = max(worst_service, e.cycles())
        assert bytes(v for a,v in e.writes if a == U + 4) == b'\xff' + payload
        e.call('rc_uart_isr'); worst_service = max(worst_service, e.cycles())
        assert e.rd(U + 12, 2) == 0x2048 and cfg(e, PA, 9) == 11
        e.call('rc_uart_isr'); worst_service = max(worst_service, e.cycles())
        assert e.rd(U + 12, 2) == 0x2008 and cfg(e, PA, 9) == 8
    # Include 40 model cycles for HPE entry/exit, a complete poll tick of phase
    # jitter, and the three-us HIGH lead. Every baud divider is rounded at its
    # own rate; bound the full 330-bit burst over the oscillator range.
    worst_burst = 0
    for ppm in range(-22000, 22001, 100):
        ratio = 1 + ppm / 1e6
        rq = round((50 << 16) / ratio)
        e.call('rc_brr', [rq, rate]); brr = e.ret
        bit_us = brr / (72 * ratio)
        worst_burst = max(worst_burst, 330 * bit_us)
    byte_cycles = (72 << rate) * 20
    assert worst_service + 40 < byte_cycles
    slack = pitch - (50 / .978 + 3 / .978 + worst_burst + (worst_service + 40) / (144 * .978))
    assert slack > 0
    # Real compiled schedule: all 16 slots fit before END; turn and END within
    # their protocol windows for every poll phase (the host suite sweeps skew).
    p = bytearray(32); p[3] = 15; p[4] = 16; p[6] = rate
    e.poke(poll, p); e.call('rc_schedule', [sched, 0, 0, 50 << 16, poll])
    turn, check, tx, end, late = [e.rd(sched + 4*i, 4) * 50 for i in range(5)]
    assert 200 <= turn <= 900 and check < 1000 and tx >= 1000 + 15*pitch
    assert 0 <= end - (1000 + 16*pitch) <= 80
    print(f'rate {rate}: ISR {worst_service}+40 < {byte_cycles} cycles; pitch slack {slack:.3f} us; turn {turn} / END {end} us: PASS')

if len(sys.argv) > 2:
    b = Hal(sys.argv[2])
    b.mmio.update({0x40021000: 0x02000002, 0x40021004: 8,
                   PA: 0x44444444, PA + 4: 0x44444444,
                   PB: 0x44444444, PB + 4: 0x44444444})
    b.call('hw_full_init')
    assert cfg(b, PB, 14) == 11 and cfg(b, PA, 9) == 4
    assert b.rd(T1 + 0x20, 2) == 0xc0 and b.rd(T1 + 0x38, 2) == 0
    print('boot feedback: PB14 CH2N, initial zero compare, PA9 input: PASS')

# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compiled round-11 boot hardware checks (no probe): python bootloader/test/test_boot_hw.py <bl.elf>.
Checks register ordering/clock, double option reads, inverse validation, option failure stops and watchdog jump.
The C suite owns the board/session/power-cut scenarios; this checks its target platform implementation too.
"""
from pathlib import Path
import struct
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from rv32emu import Emu

FLASH, OB, RCC, STK = 0x40022000, 0x1ffff800, 0x40021000, 0xe000f000
IWDG, CRC = 0x40003000, 0x40023000

def word_crc(c, w):
    c ^= w
    for _ in range(32):
        c = ((c << 1) ^ (0x04c11db7 if c & 0x80000000 else 0)) & 0xffffffff
    return c

def crc(data):
    c = 0xffffffff
    for (w,) in struct.iter_unpack('<I', data):
        c = word_crc(c, w)
    return c

class Jump(Exception):
    pass

class Boot(Emu):
    def __init__(self, elf):
        super().__init__(elf)
        self.options = [0xa5, 0x3f, 0x12, 0x34, 0xfc, 0xff, 0xff, 0xff]
        self.option_reads = []
        self.option_fault = None
        self.keys = []
        self.ops = 0
        self.fail_at = 0
        self.failure = 'no_eop'
        self.tick = 0
        self.din_until = 0
        self.start_timer = 0
        self.stop_at_app = False
        self.mmio[FLASH + 0x10] = 0x8080
        self.mmio[FLASH + 12] = 0
        self.mmio[FLASH + 0x20] = 0xfffffffc
        self.mmio[RCC + 0x24] = 0x08000000

    def rd(self, a, n):
        if self.stop_at_app and a == 0x2100:
            raise Jump()
        if OB <= a < OB + 16:
            self.option_reads.append((a, self.mmio.get(RCC + 4, 0)))
            i = (a - OB) // 2
            v = self.options[i] | ((self.options[i] ^ 255) << 8)
            if self.option_fault == 'inverse' and i == 3:
                v ^= 0x100
            if self.option_fault == 'unstable' and len(self.option_reads) == 2:
                v ^= 0x101
            return v
        if a == STK + 8:
            t = self.tick
            if self.mmio.get(STK, 0) & 1:
                self.tick = (self.tick + 8) & 0xffffffff
            return t
        if a == 0x40010808:
            return 64 if self.din_until and self.tick <= self.din_until else 0
        return super().rd(a, n)

    def operation(self):
        self.ops += 1
        bad = self.ops == self.fail_at
        self.mmio[FLASH + 12] = (0x10 if self.failure == 'wrp' else 0) if bad else 0x20
        return not bad

    def wr(self, a, n, v):
        self.writes.append((a, v))
        if a == CRC + 8:
            self.mmio[CRC] = 0xffffffff
        elif a == CRC:
            self.mmio[a] = word_crc(self.mmio.get(a, 0xffffffff), v)
        elif a in (FLASH + 4, FLASH + 8, FLASH + 0x24):
            self.keys.append((a, v))
            if len(self.keys) > 1 and self.keys[-2] == (a, 0x45670123) and v == 0xcdef89ab:
                if a == FLASH + 8:
                    self.mmio[FLASH + 16] |= 0x200
                else:
                    self.mmio[FLASH + 16] &= ~(0x80 if a == FLASH + 4 else 0x8000)
        elif a == FLASH + 12:
            self.mmio[a] &= ~(v & 0x30)
        elif a == FLASH + 16:
            if v & 0x60 == 0x60:
                assert v & 0x200 and not v & 0x80, 'options must be unlocked before erase'
                if self.operation():
                    self.options = [255] * 8
            self.mmio[a] = v
        elif OB <= a < OB + 16:
            assert self.mmio[FLASH + 16] & 0x210 == 0x210, 'OPTPG and OPTWRE before each option write'
            if self.operation():
                self.options[(a - OB) // 2] = v & 255
        else:
            self.mmio[a] = v
            if a == STK and v & 1:
                self.start_timer += 1
            if self._mem(a, n)[0] is not None:
                super().wr(a, n, v)

elf = sys.argv[1]
for fault in (None, 'inverse', 'unstable'):
    e = Boot(elf)
    e.option_fault = fault
    e.call('bl_plat_options_read', [0x20001000])
    assert e.ret == (fault is None) and len(e.option_reads) == 16, fault
    assert all(clock == 0 for _, clock in e.option_reads), 'snapshot at reset HSI'
print('option snapshot: 16 safe-clock reads, inverse and unstable-read negatives passed')

want = [0xa5, 0x3f, 0x12, 0x34, 0xac, 0x5a, 0xc3, 0x69]
for failure in ('no_eop', 'wrp'):
    for fail_at in range(10):
        e = Boot(elf)
        e.fail_at, e.failure = fail_at, failure
        e.poke(0x20001000, struct.pack('<8H', *want))
        e.call('bl_plat_options_write', [0x20001000])
        assert (e.ret == 0) == (fail_at == 0), (failure, fail_at, e.ret)
        assert e.ops == (fail_at or 9), 'stop at failed erase/halfword'
        assert e.mmio[FLASH + 16] & 0x82b0 == 0x8080, 'relock and clear OPTER/OPTPG/OPTWRE'
        assert e.mmio[RCC + 4] & 0xf0 == 0, 'restore HSI divider'
        if not fail_at:
            assert e.options == want, 'preserve all non-WRPR option values'
print('option write: success and EOP/WRP failure at erase + every halfword passed')

# Actual main -> app entry with a confirmed descriptor: IWDG must already be running.
e = Boot(elf)
body = bytes(range(256)) * 4
d = bytearray(64)
d[:6] = b'QDU5\x01\x01'
struct.pack_into('>HIIIII', d, 6, 8, 0x000610ff, len(body), crc(body), 0x08002100, 0x2100)
d[28] = 2
struct.pack_into('>I', d, 60, crc(d[:60]))
e.poke(0x08002000, d + b'\xb7\xa5\xc3\xa5' + b'\x39\xe3' * 94)
e.poke(0x08002100, body)
e.stop_at_app = True
try:
    e.call('main', max_steps=10000)
    raise AssertionError('main did not jump')
except Jump:
    wd = [(a, v) for a, v in e.writes if IWDG <= a < IWDG + 16]
    assert wd == [(IWDG, 0x5555), (IWDG+4, 4), (IWDG+8, 2500), (IWDG, 0xaaaa), (IWDG, 0xcccc)], wd
    assert e.option_reads and e.ops == 0, 'protection checked before app, no repeated write'
print('actual main reaches app entry with configured/running IWDG and protection checked')
e.stop_at_app = False
e.poke(0x20000000, bytes(32))
e.call('bl_boot', [0x20001000, 0x20000000, 1, 0x08000000])
assert e.ret == 0, 'confirmed low-DIN decision'
print(f'confirmed low-DIN bl_boot: {e.cycles()} estimated cycles ({e.cycles()/8000:.3f} ms at HSI); excludes startup/jump')

for high in (19999, 20000):
    e.tick, e.din_until, e.start_timer = 0, high * 8, 0
    e.mmio[STK] = 0
    e.poke(0x20000000, bytes(32))
    e.call('bl_boot', [0x20001000, 0x20000000, 1, 0x08000000])
    assert e.ret == (3 if high >= 20000 else 0), (high, e.ret)
    assert e.start_timer == 1, 'one timer start through full qualification loop'
print('compiled DIN qualification: 19.999 ms boots; 20 ms opens session')

e = Boot(elf)
e.call('bl_plat_boot_ticks')
e.call('bl_plat_boot_ticks')
assert e.start_timer == 1 and e.ret > 0, 'DIN timer starts once and advances'
print('DIN reset timer starts once and advances across reads')

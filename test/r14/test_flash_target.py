# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Power cuts in the compiled target's A/B effect store, including sequence wrap.

Uses the project's NOR controller emulator and frozen-record tooling. Cuts are at
every erase/program boundary and within the last two magic halfword programs.
Run: python test/r14/test_flash_target.py [build/r14-measure-b1/fw.elf]
"""
from pathlib import Path
import random
import sys

FW = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FW / "tools"))
from flash_stores import FlashEmu, FLASH_R, CTLR_PG, CTLR_STRT, CTLR_PAGE_ER, UPPER_SPAN
from record_corpus import Board, heads, crc8, same

elf = str(FW / (sys.argv[1] if len(sys.argv) > 1 else "build/r14-measure-b1/fw.elf"))

class PowerCut(Exception):
    pass

class CutEmu(FlashEmu):
    def __init__(self, path):
        super().__init__(path)
        self.mmio[0x40004400] = 0xC0
        self.ops = []
        self.cut_at = None
        self.partial = None
        self.guard = None

    def wr(self, a, n, v):
        ctl = self.mmio.get(FLASH_R + 0x10, 0)
        erase = a == FLASH_R + 0x10 and v & CTLR_STRT and v & CTLR_PAGE_ER
        program = 0x08010000 <= a < 0x0801C000 and ctl & CTLR_PG
        if erase or program:
            target = self.mmio[FLASH_R + 0x14] if erase else a
            assert not self.guard or not self.guard <= target < self.guard + 512, "old bank changed"
            if self.cut_at == len(self.ops):
                if self.partial is not None:
                    assert program and n == 2
                    # Only a prefix of the halfword's bits reaches its programmed value.
                    mask = (1 << self.partial) - 1
                    super().wr(a, n, v | (0xFFFF ^ mask))
                raise PowerCut()
            if program:
                assert self.rd(a, n) == (1 << (8 * n)) - 1, "programming non-erased flash"
            self.ops.append(("erase" if erase else "program", target, n))
        super().wr(a, n, v)

def boot(area=None):
    bd = Board(elf)
    bd.e = CutEmu(elf)
    bd.e.poke(UPPER_SPAN[0], area if area is not None else b"\xff" * UPPER_SPAN[1])
    bd.loads()
    return bd

def save(bd, hs, mask):
    for i, h in enumerate(hs):
        bd.e.poke(bd.ul + i * bd.stride, h)
    bd.e.poke(bd.buf + 300, bytes(crc8(h) for h in hs))
    bd.call("scene_fx_store", [bd.ul, mask, bd.buf + 300])

old_heads, new_heads = heads(random.Random(1401)), heads(random.Random(1402))
checks = cuts = 0
for seq in (1, 0xFFFF):
    bd = boot()
    save(bd, old_heads, 0x1F)
    assert bd.e.ret == 1
    current = 0x08019000
    assert bd.e.peek(current, 4) == b"SFX1"
    bd.e.poke(current + 10, seq.to_bytes(2, "little"))
    old_area = bd.area()
    old = boot(old_area).loads()["fx"]
    assert old["mask"] == 0x1F
    for mask in (0x1F, 0):
        clean = boot(old_area)
        clean.e.ops.clear()
        clean.e.guard = current
        save(clean, new_heads, mask)
        assert clean.e.ret == 1
        ops = clean.e.ops[:]
        want = boot(clean.area()).loads()["fx"]
        assert want["mask"] == mask
        cases = [(k, None) for k in range(len(ops) + 1)]
        cases += [(k, bit) for k in range(len(ops) - 2, len(ops)) for bit in range(1, 16)]
        for k, bit in cases:
            trial = boot(old_area)
            trial.e.ops.clear()
            trial.e.guard = current
            trial.e.cut_at, trial.e.partial = k, bit
            try:
                save(trial, new_heads, mask)
            except PowerCut:
                cuts += 1
            reboot = boot(trial.area())
            got = reboot.loads()["fx"]
            assert same(got, old) or same(got, want), (seq, mask, k, bit, got)
            checks += 1
            save(reboot, new_heads, mask)
            assert reboot.e.ret == 1
            assert same(boot(reboot.area()).loads()["fx"], want), ("retry", seq, mask, k, bit)
            checks += 1
        print(f"effects seq={seq} -> {(seq + 1) & 0xFFFF}, mask=0x{mask:02x}: {len(ops)} flash operations, {len(cases)} interruption/control cases, old-or-new and retry verified", flush=True)
print(f"R14 target flash: {checks} checks, {cuts} power cuts, 0 failures; old bank never programmed/erased", flush=True)

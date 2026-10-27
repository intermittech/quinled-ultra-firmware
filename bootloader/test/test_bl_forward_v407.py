# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 bootloader's downstream gate (TIM2 CH4 on FWD_EN, PA3): it opens (forced HIGH) only after quiet LOW and
stays open through flash work."""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, image
from test_bl_declared_v407 import announce, transfer, committed

TIM2_CHCTLR2, TIM2_CCER, TIM2_CNT = 0x4000001C, 0x40000020, 0x40000024
GPIOA_CFGLR, GPIOA_BSHR, GPIOA_BCR, GPIOB_INDR = 0x40010800, 0x40010810, 0x40010814, 0x40010c08
OCM_TOGGLE, OCM_LO, OCM_HI = 3, 4, 5


class Opened(Exception):
    pass


class Forward(MCU):
    """FWD_EN = TIM2 CH4's output compare: OC4M forced HIGH = open, forced LOW = closed, toggle = the slot plan (closed
    until the 256th edge). opened: the times the gate went to forced HIGH; armed: the times the slot plan was armed"""
    def __init__(self, elf, noisy=False):
        super().__init__(elf)
        self.noisy = noisy
        self.out = 0
        self.ocm = OCM_LO
        self.opened = []
        self.armed = []
        self.forward_ops = []
        self.entering = False
        self.cnt_base = 0.0                              # TIM2 CNT: DIN's falling edges after this time (its last write)
        self._falls = None

    def boot_forward(self, max_steps=6000000):
        """from reset into the main loop, until the gate's first opening (forced HIGH or the slot plan)"""
        self.entering = True
        self.mhz = 20.0
        self.setword('clock_khz', 20000)
        self.setword('options_trusted', 0, 1)
        self.mmio[0x40021024] = 1 << 27
        try: self.call('main', max_steps=max_steps)
        except Opened: pass
        else: raise AssertionError('the gate never opened')
        self.entering = False

    def din(self, t):
        # A short LOW inside a frame must never count as entry quiet.
        return self.noisy and (t < 80 or 140 <= t < 180)

    def fall_times(self):
        """DIN's falling edges, sorted (TIM3 CH1 captures them, TIM2 counts them)"""
        return [80.0, 180.0] if self.noisy else []

    def rd(self, a, n):
        if a == GPIOB_INDR:
            return 16 if self.din(self.now_us) else 0
        if a == TIM2_CNT:
            if self._falls is None:
                self._falls = self.fall_times()
            return (bisect_right(self._falls, self.now_us) - bisect_right(self._falls, self.cnt_base)) & 0xffff
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == GPIOA_BSHR:
            self.out = (self.out | (v & 0xffff)) & ~(v >> 16)
        elif a == GPIOA_BCR:
            self.out &= ~v
        super().wr(a, n, v)
        if a == TIM2_CNT:
            assert v == 0, 'TIM2 CNT written with %d' % v
            self.cnt_base = self.now_us
        if a == TIM2_CHCTLR2:
            ocm = (v >> 12) & 7
            if ocm != self.ocm and ocm in (OCM_HI, OCM_TOGGLE):
                assert (self.mmio.get(GPIOA_CFGLR, 0) >> 12) & 15 == 0xB, 'PA3 is not TIM2 CH4 (AF push-pull)'
                assert self.mmio.get(TIM2_CCER, 0) & (1 << 12), 'TIM2 CH4 output not enabled'
                assert self.out & (1 << 10), 'downstream driver disabled while gate opens'
                assert not self.din(self.now_us), f'gate written at {self.now_us:.1f} us while DIN is HIGH'
                (self.opened if ocm == OCM_HI else self.armed).append(self.now_us)
                self.ocm = ocm
                if self.entering:
                    raise Opened()
            self.ocm = ocm

    def operation(self, *args):
        assert self.ocm == OCM_HI and self.out & (1 << 10), 'forwarding closed during flash work'
        self.forward_ops.append(args[:2])
        return super().operation(*args)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args(); rows = []
    for noisy in (False, True):
        m = Forward(a.elf, noisy)
        m.boot_forward()
        m.installed(image(m.target, version=0x30000))
        assert m.opened and not m.armed, 'bootloader never opens downstream forwarding (forced HIGH)'
        quiet_start = 180 if noisy else 0
        assert m.opened[0] - quiet_start >= 350, m.opened
        assert m.opened[0] < 50000, 'blackout exceeds minimum declared entry bound'
        m.init_session(); img = image(m.target)
        announce(m, img); transfer(m, img)
        assert committed(m, img) and m.ocm == OCM_HI
        assert len(m.opened) == 1, 'flash operations must not toggle the gate'
        rows.append(dict(noisy=noisy, gate_open_us=m.opened[0], flash_ops=len(m.forward_ops), ok=True))
    print(json.dumps(rows, indent=2))
    if a.json: a.json.write_text(json.dumps(rows, indent=2))


if __name__ == '__main__': main()

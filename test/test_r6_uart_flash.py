# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Actual flash_op must pause TX DMA requests and finish the UART byte before changing HCLK.
Usage: python test/test_r6_uart_flash.py build/lto0/fw.elf
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from hal_check import FlashHal, USART2, RCC_CFGR0, dch

class UART(FlashHal):
    def __init__(self, elf, stuck=False):
        super().__init__(elf)
        self.stuck = stuck
        self.mmio[dch(7)] = 0x91
        self.mmio[dch(7)+4] = 300
        self.mmio[USART2+0x14] = 0xc0
        self.mmio[USART2] = 0x80
        self.left = 4  # finite reads to finish the in-flight byte after DMA is paused
        self.slow = []

    def rd(self, a, n):
        if a == USART2 and not self.stuck and not self.mmio[USART2+0x14] & 0x80:
            self.left -= 1
            if self.left <= 0:
                self.mmio[USART2] = 0xc0
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == RCC_CFGR0 and v & 0xf0 == 0x80:
            self.slow.append((self.mmio[USART2+0x14] & 0x80, self.mmio[USART2] & 0x40))
        super().wr(a, n, v)

failed = 0
for stuck in (False, True):
    e = UART(sys.argv[1], stuck)
    e.poke(0x20004000, b'\x34\x12')
    e.call('flash_op', [0x0800ff00, 0x20004000, 1, 0])
    ok = (e.ret == 7 and not e.slow and not e.ops) if stuck else (e.ret == 1 and e.slow == [(0, 0x40)])   # (7: FLASH_ERR_UART)
    ok = ok and e.mmio[dch(7)] == 0x91 and e.mmio[dch(7)+4] == 300
    ok = ok and e.mmio[USART2+0x14] == 0xc0
    print('stuck' if stuck else 'active TX', 'result', e.ret, 'DMAT/TC at clock change', e.slow, 'PASS' if ok else 'FAIL')
    failed += not ok
sys.exit(bool(failed))

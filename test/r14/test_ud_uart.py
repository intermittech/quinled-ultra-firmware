# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""BOARD=3 UART sequencing, including SysTick reload during its HIGH preamble.

Uses the real compiled functions and explicit TXE/TC events, not physical timing.
BOARD=2's test_r6_rc_hal.py intentionally has different transceiver/pin assumptions.
Build the r14-ud-api ELF as described in README.md; optional ELF argument.
"""
from pathlib import Path
import sys

FW = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FW / "tools"))
from hal_check import Hal

U3, PA, PB, CNT = 0x40004800, 0x40010800, 0x40010C00, 0xE000F008
elf = str(Path(sys.argv[1]) if len(sys.argv) > 1 else FW / "build/r14-ud-api/fw.elf")
checks = failures = 0


def check(ok, message):
    global checks, failures
    checks += 1
    if not ok:
        failures += 1
        print("FAIL:", message)


class UartHal(Hal):
    def __init__(self, path):
        super().__init__(path)
        self.phase = self.elapsed = 0
        self.de = 0
        self.blank = 0

    def rd(self, addr, n):
        if addr == CNT:
            self.elapsed += 64
            return (self.phase + self.elapsed) % 7200
        return super().rd(addr, n)

    def wr(self, addr, n, value):
        if addr == PB + 0x10:
            if value & (1 << 14): self.de = 1
            if value & (1 << 30): self.de = 0
        if addr == PB + 0x14 and value & (1 << 14): self.de = 0
        if addr == PA + 0x10 and value & 2: self.blank = 1
        if addr == PA + 0x14 and value & 2: self.blank = 0
        super().wr(addr, n, value)


payload = bytes((i * 7 + 0xA5) & 255 for i in range(32))
scenarios = 0
for phase in (0, 6700, 7000, 7100, 7190):
    for divider in (144, 288, 576):
        e = UartHal(elf)
        e.call("hw_init")
        check(e.rd(U3 + 12, 2) == 0x2008, "USART3 TX only, no burst interrupts at boot")
        check(((e.rd(PB + 4, 4) >> 8) & 15) == 11 and not e.de,
              "PB10 remains AF push-pull while the transceiver is disabled")
        e.poke(0x20004000, payload)
        e.mmio[PA + 8] = 0  # RO is high-Z with DE high; PA6's pull-down reads LOW.
        e.writes.clear()
        e.call("rc_hw_dir", [1])
        check(e.writes.index((PA + 0x14, 2)) < e.writes.index((PB + 0x10, 1 << 14)),
              "blank strip before enabling the return driver")
        check(e.de and not e.blank, "answer mode drives only with the strip held LOW")
        e.writes.clear()
        e.phase, e.elapsed = phase, 0
        e.call("rc_hw_send", [0x20004000, 32, divider], max_steps=10000)
        check(e.ret == 0 and e.rd(U3 + 8, 2) == divider, "answer starts with requested divider")
        check(432 <= e.elapsed - 64 < 432 + 64,
              f"three-microsecond HIGH preamble across reload, phase={phase}, elapsed={e.elapsed}")
        check(e.rd(U3 + 12, 2) & 0x80 and e.de, "driver and TXE interrupt remain active during the burst")
        for _ in range(32):
            e.call("rc_uart_isr")
        check(bytes(v for addr, v in e.writes if addr == U3 + 4) == b"\xff" + payload,
              "one lead byte followed by all 32 payload bytes")
        check(e.de, "driver stays enabled through the last payload byte")
        e.call("rc_uart_isr")
        check(e.rd(U3 + 12, 2) == 0x2048 and e.de, "TXE to TC waits for the final stop bit")
        e.call("rc_uart_isr")
        check(e.rd(U3 + 12, 2) == 0x2008 and not e.de, "TC disables burst interrupts and releases DE")
        check(not e.rd(U3 + 12, 2) & 0xC0, "TC clears both burst-interrupt bits")
        check(not any(addr == U3 + 12 and value & 4 for addr, value in e.writes), "RX remains disabled")
        check(not any(0x40020000 <= addr < 0x40020400 for addr, _ in e.writes), "UART does not use occupied DMA channels")
        for mode in (0, 2):
            e.call("rc_hw_dir", [1])
            e.call("rc_hw_send", [0x20004000, 32, divider])
            e.call("rc_hw_dir", [mode])
            check(not e.rd(U3 + 12, 2) & 0xC0 and not e.de, f"mode {mode} safely cuts an interrupted burst")
        e.call("rc_hw_dir", [1])
        e.writes.clear()
        e.call("rc_hw_send", [0, 0, divider])
        check(e.ret == 0 and not e.de and not any(addr == U3 + 4 for addr, _ in e.writes),
              "zero-length probe emits no payload and releases DE")
        scenarios += 1
        print(f"UD UART phase={phase}, divider={divider}: preamble, payload, TC release and interruption checked")

print(f"R14 UD UART: {scenarios} scenarios, {checks} checks, {failures} failures")
raise SystemExit(bool(failures))

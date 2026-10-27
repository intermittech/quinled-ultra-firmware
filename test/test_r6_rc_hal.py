# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Execute the receiver HAL in the existing RV32 register model (LTO=0 ELF argument).
UART TXE/TC events are injected explicitly: this verifies software sequencing,
not interrupt latency, electrical read-back, VTF routing on silicon, or baud timing.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from hal_check import Hal

U3 = 0x40013800
LEAD = b'\xff'  # core/rc.h RC_LEAD_BYTES: a 0xFF before the answer's SYNC
PA, PB = 0x40010800, 0x40010C00
CNT = 0xE000F008
failures = 0


def check(ok, label):
    global failures
    print(('PASS' if ok else 'FAIL') + ': ' + label)
    failures += not ok


class ReturnHal(Hal):
    def __init__(self, elf):
        super().__init__(elf)
        self.count = 0

    def rd(self, a, n):
        if a == CNT:
            self.count += 64
            return self.count
        return super().rd(a, n)


e = ReturnHal(sys.argv[1])
e.call('hw_init')
check(e.rd(U3 + 0x0C, 2) == 0x2008, 'USART1 UE+TE, RE/TXEIE/TCIE clear at init')
check(((e.rd(PA + 4, 4) >> 4) & 15) == 8, 'PA9 high impedance outside a burst')
e.mmio[PA + 8] = 1 << 6
e.writes.clear()
e.call('rc_hw_dir', [1])
check((PA + 0x14, 1 << 10) in e.writes and (PB + 0x10, 1 << 10) in e.writes,
      'turn: stop downstream driver, enable upstream')
payload = bytes((7 * i + 0xA5) & 255 for i in range(32))
address = 0x20004000
e.poke(address, payload)
e.mmio[PA + 8] = 1 << 6
e.writes.clear()
e.call('rc_hw_send', [address, 32, 577])
check(e.ret == 0 and e.rd(U3 + 8, 2) == 577, 'burst starts with the requested divider')
check(((e.rd(PA + 4, 4) >> 4) & 15) == 11, 'PA9 alternate function during burst')
for _ in range(31 + len(LEAD)):
    e.call('rc_uart_isr')  # transmit-data-empty
check(bytes(v for a, v in e.writes if a == U3 + 4) == LEAD + payload, 'the lead byte, then exactly 32 bytes, in order')
e.call('rc_uart_isr')      # final byte entered shift register
check(e.rd(U3 + 0x0C, 2) == 0x2048 and ((e.rd(PA + 4, 4) >> 4) & 15) == 11,
      'last TXE switches to TC, does not release the final stop bit')
e.call('rc_uart_isr')      # transmission complete, final stop bit finished
check(e.rd(U3 + 0x0C, 2) == 0x2008 and ((e.rd(PA + 4, 4) >> 4) & 15) == 8,
      'TC disables interrupts and releases PA9 to relay')
check(not any(a == U3 + 0x0C and v & 4 for a, v in e.writes), 'USART1 RE stays zero throughout')
check(not any(0x40020000 <= a < 0x40020400 for a, _ in e.writes), 'TX does not touch occupied DMA channels')
for high in (False, True):
    e.mmio[PA + 8] = (1 << 6) if high else 0
    e.writes.clear()
    e.call('rc_hw_send', [address, 0, 72])
    check(e.ret == 0 and not any(a == U3 + 4 for a, _ in e.writes)
          and ((e.rd(PA + 4, 4) >> 4) & 15) == 8,
          f'probe: DIN={int(high)}, no payload or disabled RO readback')

e.writes.clear()
e.call('rc_hw_dir', [2])
check((PA + 0x14, 1 << 10) in e.writes and (PB + 0x10, 1 << 26) in e.writes,
      'quiet: both transceiver drivers disabled')
e.writes.clear()
e.call('rc_hw_dir', [0])
check((PB + 0x14, 1 << 10) in e.writes and (PA + 0x10, 1 << 10) in e.writes,
      'forward: upstream off before downstream on')
print(f'RC HAL register model: {failures} failures')
raise SystemExit(bool(failures))

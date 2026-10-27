# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""A group ANNOUNCE reaches erase through the production decoder and main loop,
with and without a marker; no SELECT, status poll or answer is involved."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import APP, image, frame
from test_bl_wire_v407 import Wire, Done


class GroupWire(Wire):
    def __init__(self, elf, marker):
        super().__init__(elf, 195.2, marker=marker)
        self.out = 0; self.ocm = 4; self.scheduled = False   # ocm: TIM2 CH4's OC4M (FWD_EN): 5 forced HIGH = open
        self.STK = 0xE000F000
        self.mhz = 20.0
        self.setword('clock_khz', 20000)
        self.setword('options_trusted', 0, 1)
        self.mmio[0x40021024] = 1 << 27

    def wr(self, a, n, v):
        if a == 0x40010810: self.out = (self.out | (v & 65535)) & ~(v >> 16)
        elif a == 0x40010814: self.out &= ~v
        elif a == 0x4000001C: self.ocm = (v >> 12) & 7
        super().wr(a, n, v)
        if self.base is not None and not self.scheduled:
            self.scheduled = True
            img = image(self.target)
            for k, f in enumerate([frame(0x85, 7), frame(0x85, 7),
                                   frame(0x81, 7, 0, img[:32]), frame(0x81, 7, 1, img[32:64])]):
                self.add_frame(self.base+int((3000+k*1000)*self.true_mhz), f,
                               lambda tick: 1000000+int((tick-self.base)/self.true_mhz))

    def rd(self, a, n):
        if self.frames and int(self.timer) > self.frames[-1][0]+10000*self.true_mhz:
            raise AssertionError('group ANNOUNCE did not reach an erase')
        return super().rd(a, n)

    def operation(self, kind, address, payload, latency):
        assert kind == 'erase' and address == APP
        assert self.ocm == 5 and self.out & (1 << 10), 'forwarding must remain enabled'
        super().operation(kind, address, payload, latency)
        raise Done()


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('elf', type=Path); p.add_argument('--json', type=Path)
    a = p.parse_args(); rows = []
    for marked in (False, True):
        m = GroupWire(a.elf, marked)
        try: m.call('main', max_steps=3000000)
        except Done: pass
        assert len(m.ops) == 1 and len(m.t0_seen) == 4 and not m.bytes
        rows.append(dict(marker=marked, decoded=len(m.t0_seen), first_operation=m.ops[0][:2], replies=len(m.bytes), ok=True))
    print(json.dumps(rows, indent=2))
    if a.json: a.json.write_text(json.dumps(rows, indent=2))

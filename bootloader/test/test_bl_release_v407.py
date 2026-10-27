# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The compiled hw_reply probe, with late/noisy release and wrap regressions."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from test_bl_status_v407 import Board, GPIOB_INDR
from abi3_model_v407 import UID


class Released(Board):
    def rd(self, a, n):
        if a == GPIOB_INDR:
            us = self.timer / self.mhz - self.origin
            high = us >= self.release and not any(lo <= us < hi for lo, hi in self.noise)
            return (1 << 4) if high else 0
        return super().rd(a, n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path)
    ap.add_argument('--json', type=Path)
    a = ap.parse_args()
    rows = []
    cases = [(150, [], True), (249, [], True), (251, [], False), (440, [], False),
             (449, [], False), (451, [], False), (150, [(200, 230)], True),
             (150, [(240, 251)], False), (150, [(430, 449)], False),
             (150, [(449, 460)], False)]
    for mhz in (191.0, 195.2, 200.0, 204.4):
        for wrap in (False, True):
            for release, noise, expect in cases:
                m = Released(a.elf, UID, mhz)
                m.origin = (0xffffff00 / mhz) if wrap else 0
                m.release, m.noise = release, noise
                m.poke(0x20006000, bytes(32))
                m.advance(m.origin + 100)
                t0 = int(m.origin * mhz) & 0xffffffff
                at = (t0 + int(450 * mhz)) & 0xffffffff
                m.call('hw_reply', [0x20006000, at, round(mhz/2), t0], max_steps=500000)
                enabled = [t/mhz-m.origin for k,t in m.de if k == 'on']
                high_start = max([release] + [hi for lo,hi in noise])
                ok = ((len(m.bytes) == 33 and m.ret == 0 and enabled[0]-high_start >= 20) if expect
                      else (not m.bytes and not enabled and m.ret != 0))
                rows.append(dict(mhz=mhz, wrap=wrap, release=release, noise=noise, expect=expect,
                                 bytes=len(m.bytes), enabled=enabled, ok=bool(ok)))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} release cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__':
    sys.exit(main())

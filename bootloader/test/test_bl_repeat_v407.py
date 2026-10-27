# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Identical and conflicting DATA repeats at every status staging/commit boundary."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from test_bl_status_fail_v407 import fresh
from abi3_model_v407 import frame


def case(elf, phase, changed, padding=False):
    m, img = fresh(elf)
    m.drain()
    if phase == 'partial':
        m.send(frame(0x82, 7, 0, img[64:96]))
    elif phase in ('full', 'reserved', 'page'):
        for i in range(8):
            m.send(frame(0x82, 7, i, img[64 + 32*i:96 + 32*i]))
        if phase != 'full':
            m.poll(run=phase == 'page')
    else:
        m.transfer(img, end=phase == 'committed')
        if phase == 'commit_reserved':
            m.send(frame(0x83, 7, data=img[24:28] + img[20:24] + bytes(24)))
            m.poll(run=False)
    idx = 24 if padding else 0
    data = bytearray(img[64 + idx*32:96 + idx*32].ljust(32, b'\0'))
    if changed:
        data[31 if padding else 0] ^= 1
    before, ops = bytes(m.flash), len(m.ops)
    m.send(frame(0x82, 7, idx, data), run=False)
    st = m.poll(run=False)
    if changed:
        ok = st['state'] == 7 and st['error'] == 7 and st['credit'] == st['retry'] == 0
        m.run_due()
        m.ack()
        ok = ok and len(m.ops) == ops and bytes(m.flash) == before and not m.run_due()
    else:
        ok = st['state'] != 7 and st['error'] == 0 and len(m.ops) == ops
    return dict(phase=phase, changed=changed, padding=padding, ok=bool(ok))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path)
    ap.add_argument('--json', type=Path)
    a = ap.parse_args()
    rows = [case(a.elf, phase, changed) for phase in ('partial', 'full', 'reserved', 'page', 'body',
                                                     'commit_reserved', 'committed') for changed in (False, True)]
    rows += [case(a.elf, phase, changed, True) for phase in ('body', 'commit_reserved', 'committed')
             for changed in (False, True)]
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} repeated-chunk cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__':
    sys.exit(main())

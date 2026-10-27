# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Changed selection discards accounting; reset without RESUME requests restart."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from test_bl_status_fail_v407 import fresh
from abi3_model_v407 import frame


def clean(st):
    return st and all(st[k] == 0 for k in ('error', 'credit', 'mask', 'accepted', 'committed', 'retry', 'crc', 'flags'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path)
    ap.add_argument('--json', type=Path)
    a = ap.parse_args()
    rows = []
    for phase in ('prepared', 'partial', 'failed', 'committed'):
        m, img = fresh(a.elf)
        if phase != 'prepared':
            m.drain()
            m.send(frame(0x82, 7, 0, img[64:96]))
        if phase == 'committed':
            m.transfer(img)
        if phase == 'failed':
            bad = bytearray(img[:32]); bad[16] ^= 1
            m.send(frame(0x81, 7, 0, bad))
        before = m.poll(run=False)
        m.send(frame(0x87, 7, 0, m.uid + m.target + bytes(4)), run=False)
        same = m.poll(run=False)
        kept = all(same[k] == before[k] for k in ('state', 'error', 'mask', 'accepted', 'committed', 'crc', 'flags'))
        # Use a new session and an intervening selection of a different UID.
        for foreign in (False, True):
            if foreign:
                m.select(10, uid=bytes(12))
                assert m.poll(10) is None
            m.select(9 if not foreign else 11)
            st = m.poll(9 if not foreign else 11)
            rows.append(dict(phase=phase, foreign=foreign, ok=bool(kept and clean(st) and st['state'] == 0)))
    for phase in ('prepared', 'revoked', 'partial'):
        m, img = fresh(a.elf)
        if phase != 'prepared':
            m.drain()
        if phase == 'partial':
            m.send(frame(0x82, 7, 0, img[64:96]))
        m = m.restarted(); m.init_session(); m.lock(); m.select(9)
        st = m.poll(9)
        ok = clean(st) and st['state'] == 8
        assert not m.ops
        m.send(frame(0x82, 9, 0, img[64:96]))
        assert not m.ops
        m.start(img, 9)
        ok = ok and m.transfer(img, 9)['state'] == 6
        rows.append(dict(reset=phase, ok=bool(ok)))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} selection/reset cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__':
    sys.exit(main())

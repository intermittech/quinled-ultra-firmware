# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""A failure cancels every unstarted flash reservation.

Run against the production ELF, including the original conflicting ANNOUNCE.
Each case compares the entire flash before/after failure, not just the status.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, image, frame


def fresh(elf):
    m = MCU(elf)
    m.installed(image(m.target, 772, 0x00030000))
    m.init_session()
    m.lock()
    img = image(m.target)
    m.select()
    m.send(frame(0x81, 7, 0, img[:32]))
    m.send(frame(0x81, 7, 1, img[32:64]))
    return m, img


def check(elf, phase, reserved):
    m, img = fresh(elf)
    if phase != 'metadata_erase':
        assert m.poll()['state'] == 2
    if phase in ('program', 'commit'):
        assert m.drain()['state'] == 3
        if phase == 'program':
            for i in range(8):
                m.send(frame(0x82, 7, i, img[64 + 32*i:96 + 32*i]))
        else:
            m.transfer(img, end=False)
            m.send(frame(0x83, 7, data=img[24:28] + img[20:24] + bytes(24)))
    if reserved:
        assert m.poll(run=False)['state'] == 2
    before, ops = bytes(m.flash), len(m.ops)
    bad = bytearray(img[:32])
    bad[16] ^= 1
    m.send(frame(0x81, 7, 0, bad), run=False)
    first = m.poll(run=False)
    cancelled = m.stv('op', 1) == 0 and m.stv('op_due', 1) == 0
    m.advance(m.now_us + 500000)
    m.run_due()
    for f in (frame(0x82, 7, 0, img[64:96]), frame(0x83, 7, data=img[24:28] + img[20:24] + bytes(24)),
              frame(0x81, 7, 1, img[32:64]), frame(0x85, 7)):
        m.send(f)
    last = m.poll()
    ok = (last is not None and first['state'] == last['state'] == 7 and first['error'] != 0 and last['error'] == first['error']
          and first['credit'] == first['retry'] == 0 and cancelled and len(m.ops) == ops
          and bytes(m.flash) == before and not m.run_due())
    return dict(phase=phase, reserved=reserved, ok=bool(ok), cancelled=cancelled,
                first={k:v for k,v in first.items() if k != 'raw'}, new_ops=len(m.ops)-ops)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path)
    ap.add_argument('--json', type=Path)
    a = ap.parse_args()
    rows = [check(a.elf, phase, reserved) for phase in ('metadata_erase', 'body_erase', 'program', 'commit')
            for reserved in (False, True)]
    result = json.dumps(rows, indent=2)
    if a.json:
        a.json.write_text(result)
    print(result)
    return int(not all(r['ok'] for r in rows))


if __name__ == '__main__':
    sys.exit(main())

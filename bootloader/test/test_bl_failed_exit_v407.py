# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""FAILED keeps its error, but ABORT/10 s idle can leave to an intact app.

Adapted from failed_exit_probe.py. Exercise production session code, including
revoked/body-erased apps that must remain in recovery, with whole-flash checks.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, image, frame


def failed(elf, kind):
    m = MCU(elf)
    m.installed(image(m.target, 772, 0x00030000))
    m.init_session()
    m.lock()
    m.select()
    target = bytearray(m.target)
    if kind == 'cross_family':
        target[1:4] = bytes([1, 1, 1])
    img = image(bytes(target), 772, 0x00020000 if kind == 'downgrade' else 0x00040000)
    m.send(frame(0x81, 7, 0, img[:32]))
    m.send(frame(0x81, 7, 1, img[32:64]))
    if kind in ('revoked', 'body_erased'):
        assert m.poll()['state'] == 2  # metadata-sector erase revokes the old app
        if kind == 'body_erased':
            assert m.poll()['state'] == 2
    if kind not in ('downgrade', 'cross_family'):
        bad = bytearray(img[:32])
        bad[16] ^= 1
        m.send(frame(0x81, 7, 0, bad))
    return m


def check(elf, kind, action):
    m = failed(elf, kind)
    error = {'downgrade': 1, 'cross_family': 2}.get(kind, 7)
    intact = kind not in ('revoked', 'body_erased')
    before, ops = bytes(m.flash), len(m.ops)
    first = m.poll()
    # Wrong-session ABORT must not release a selected FAILED session.
    m.send(frame(0x84, 8))
    premature = bool(m.run_due())
    last = m.poll()
    if action == 'abort':
        m.send(frame(0x84, 7), run=False)
    else:
        m.advance(m.now_us + 9_900_000)
        premature |= bool(m.run_due())
        m.advance(m.now_us + 200_000)
    jump = bool(m.run_due())
    retained_error = m.stv('error', 1)
    if not intact:
        m.advance(m.now_us + 60_000_000)
        jump |= bool(m.run_due())
        retained_error = m.stv('error', 1)
    cancelled = m.stv('op', 1) == m.stv('op_due', 1) == 0
    ok = (not premature and jump == intact and first['state'] == last['state'] == 7
          and first['error'] == last['error'] == retained_error == error
          and first['credit'] == last['credit'] == 0 and cancelled
          and len(m.ops) == ops and bytes(m.flash) == before)
    return dict(kind=kind, action=action, expected_error=error, retained_error=retained_error,
                jump_to_app=jump, cancelled=cancelled, new_flash_ops=len(m.ops)-ops,
                flash_unchanged=bytes(m.flash) == before, ok=bool(ok))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path)
    ap.add_argument('--json', type=Path)
    a = ap.parse_args()
    rows = [check(a.elf, kind, action) for kind in
            ('downgrade', 'cross_family', 'conflicting_announce', 'revoked', 'body_erased')
            for action in ('abort', 'silence')]
    result = json.dumps(rows, indent=2)
    if a.json:
        a.json.write_text(result)
    print(result)
    return int(not all(row['ok'] for row in rows))


if __name__ == '__main__':
    sys.exit(main())

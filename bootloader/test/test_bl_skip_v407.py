# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Declared V407 transfers: a node that skips the image (same or older version) waits for END.

It erases nothing, but stays through the erase lead and every DATA pass of its session (a valid frame of the session
is activity), forwarding all the while, and leaves with its result only at END; ten quiet seconds after the last
frame still end it. A 62 KiB image's three passes take ~50 s (docs/update.md: 61 ms block pauses).

    python bootloader/test/test_bl_skip_v407.py <V407 bl.elf> [--json out.json]
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import image, frame
from test_bl_declared_v407 import fresh

SKIP, FAILED = 2, 4
SAME, REJECTED = 2, 3


def run(elf, version, length, passes, quiet_end):
    m = fresh(elf); img = image(m.target, length, version=version)
    m.send(frame(0x81, 7, 0, img[:32])); m.send(frame(0x81, 7, 1, img[32:64]))
    lead = (1 + (length + 4095) // 4096) * 200 + 100
    m.advance(m.now_us + lead * 1000); m.run_due()
    stayed = m.state == SKIP and not m.run_due()
    chunks = (length + 31) // 32
    for _ in range(passes):
        for i in range(chunks):
            m.send(frame(0x82, 7, i, img[64+32*i:96+32*i].ljust(32, b'\0')))
            if i % 8 == 7: m.advance(m.now_us + 61000)
            if m.run_due():
                stayed = False
                break
    stayed = stayed and m.state == SKIP
    if quiet_end:
        m.advance(m.now_us + 9_990_000); kept = not m.run_due()
        m.advance(m.now_us + 20_000); left = bool(m.run_due())
        ok = stayed and kept and left
    else:
        m.send(frame(0x83, 7, data=img[24:28]+img[20:24]+bytes(24)), run=False)
        left = bool(m.run_due())
        ok = stayed and left
    flash_ops = len(m.ops)
    return dict(version=hex(version), length=length, passes=passes, quiet_end=quiet_end, stayed_through_passes=stayed,
                left=left, flash_ops=flash_ops, ok=ok and flash_ops == 0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []
    for version, length, passes, quiet_end in ((0x30000, 63488, 3, False), (0x20000, 63488, 3, False),
                                               (0x30000, 4356, 40, True)):
        row = run(a.elf, version, length, passes, quiet_end)
        rows.append(row); print(json.dumps(row), flush=True)
    if a.json: a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} skip cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__': sys.exit(main())

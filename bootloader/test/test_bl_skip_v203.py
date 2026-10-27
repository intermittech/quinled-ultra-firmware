# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Declared V203 transfers: a node that skips the image (same or older version) waits for END (decided 2026-10-23).

It erases nothing, but stays through the erase lead and every DATA pass of its session (a valid frame of the session is
activity), forwarding all the while, and leaves with its result only at END; ten quiet seconds after the last frame
still end it. A full-slot image's three passes with 21 ms block pauses take ~14 s after a 9.06 s lead (docs/update.md),
well past the ten-second inactivity bound. The pattern of r96's test_bl_skip_v407.py, on the V203 production ELF.

    python bootloader/test/test_bl_skip_v203.py <V203 bl.elf> [--json out.json]
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model import MCU, image, frame  # noqa: E402

SKIP = 2


def fresh(elf, installed_version):
    m = MCU(elf)
    m.installed(image(m.target, 772, installed_version))
    m.init_session()
    return m


def run(elf, installed, version, length, passes, quiet_end):
    m = fresh(elf, installed)
    img = image(m.target, length, version=version)
    m.send(frame(0x81, 7, 0, img[:32]))
    m.send(frame(0x81, 7, 1, img[32:64]))
    lead = (1 + (length + 255) // 256) * 2 * 20 + 100   # the V203's declared lead (20 ms erase bound)
    m.advance(m.now_us + lead * 1000)
    stayed = m.state == SKIP and not m.poll()
    chunks = (length + 31) // 32
    t_first = m.now_us
    for _ in range(passes):
        for i in range(chunks):
            m.send(frame(0x82, 7, i, img[64 + 32 * i:96 + 32 * i].ljust(32, b'\0')))
            if i % 8 == 7:
                m.advance(m.now_us + 21000)
            if m.poll():
                stayed = False
                break
        if not stayed:
            break
    span_ms = round((m.now_us - t_first) / 1000)
    stayed = stayed and m.state == SKIP
    if quiet_end:
        m.advance(m.now_us + 9_990_000)
        kept = not m.poll()
        m.advance(m.now_us + 20_000)
        left = bool(m.poll())
        ok = stayed and kept and left
    else:
        m.send(frame(0x83, 7, data=img[24:28] + img[20:24] + bytes(24)))
        left = bool(m.poll())
        ok = stayed and left
    return dict(installed=hex(installed), version=hex(version), length=length, passes=passes, quiet_end=quiet_end,
                lead_ms=lead, data_span_ms=span_ms, stayed_through_passes=stayed, left=left, flash_ops=len(m.ops),
                ok=ok and len(m.ops) == 0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    full = 0x08010000 - 0x08002100
    rows = []
    for installed, version, length, passes, quiet_end in ((0x30000, 0x30000, full, 3, False),    # same version
                                                          (0x30100, 0x30000, full, 3, False),    # older
                                                          (0x30000, 0x30000, 4356, 40, True)):   # many short passes
        row = run(a.elf, installed, version, length, passes, quiet_end)
        rows.append(row)
        print(json.dumps(row), flush=True)
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} skip cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__':
    sys.exit(main())

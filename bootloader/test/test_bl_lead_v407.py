# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Declared V407 transfers: the whole erase lead is busy time, not inactivity (docs/update.md).

The host sends nothing for erase_units x 2 x erase_max + 100 ms after the final boot ANNOUNCE (62 KiB 3500 ms,
256 KiB 13100 ms, full slot 24900 ms); the modeled erase ends after 0.5 / 2.0 / 3.7 s. Each case checks READY just
before the lead's end and the first DATA chunk accepted at it (polls every 50 ms, or one poll), and on a second node
the ordinary ten-second inactivity counted from the lead's end (one late poll: the end is exact, not the last poll);
with one boot ANNOUNCE pair, and with an identical pair repeated 2 s later (the lead counts from the final one).

    python bootloader/test/test_bl_lead_v407.py <V407 bl.elf> [--json out.json]
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import image, frame
from test_bl_declared_v407 import fresh

RECEIVE, ERASE, FAILED = 1, 3, 4
ERROR_SESSION = 9


def lead_ms(length):
    return (1 + (length + 4095) // 4096) * 200 + 100


def pair(m, img):
    """both boot ANNOUNCE halves; the node's ms clock at the final one (before any erase runs)"""
    m.send(frame(0x81, 7, 0, img[:32]))
    m.send(frame(0x81, 7, 1, img[32:64]), run=False)
    return m.ms()


def node(elf, length, repeat):
    """a node after the final boot ANNOUNCE pair and its real erase; t0 = its ms clock at that pair"""
    m = fresh(elf); img = image(m.target, length)
    t0 = pair(m, img)
    if repeat:
        while m.state == ERASE: m.run_due()
        m.advance(m.now_us + 2_000_000); m.run_due()
        t0 = pair(m, img)
    while m.state == ERASE: m.run_due()
    return m, img, t0


def until(m, ms):
    """advance to the middle of local millisecond `ms`"""
    m.advance(m.now_us + (ms - m.ms()) * 1000 + 500)


def run(elf, length, repeat, dense):
    lead = lead_ms(length)
    m, img, t0 = node(elf, length, repeat)
    erased_ms = m.ms() - t0
    if dense:
        while m.ms() < t0 + lead - 52:
            m.advance(m.now_us + 50_000); m.run_due()
    until(m, t0 + lead - 2); m.run_due()
    ready = m.state == RECEIVE and not m.stv('error', 1)
    until(m, t0 + lead); m.send(frame(0x82, 7, 0, img[64:96]))
    accepted = m.state == RECEIVE and not m.stv('error', 1)
    q, _, q0 = node(elf, length, repeat)
    until(q, q0 + lead + 10_000 - 3); q.run_due()
    kept = q.state == RECEIVE
    until(q, q0 + lead + 10_000 + 3); q.run_due()
    expired = q.state == FAILED and q.stv('error', 1) == ERROR_SESSION
    return dict(length=length, repeat=repeat, dense=dense, lead_ms=lead, erase_done_ms=erased_ms,
                ready_before_lead_end=ready, first_data_accepted=accepted, idle_kept_until_lead_plus_10s=kept,
                idle_expired_after=expired, ok=ready and accepted and kept and expired)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []
    for length in (63488, 262144, 503808):
        for repeat, dense in ((False, False), (False, True), (True, False)):
            row = run(a.elf, length, repeat, dense)
            rows.append(row); print(json.dumps(row), flush=True)
    if a.json: a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} erase-lead cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__': sys.exit(main())

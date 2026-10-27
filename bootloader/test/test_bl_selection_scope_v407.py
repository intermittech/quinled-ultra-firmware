# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""A selection belongs only to its session (docs/update.md "Transfer mode", "Selected / optional status profile").

SELECT for another UID, or for this one, binds only that session: a later session's PING or ANNOUNCE ends it (after
ABORT, after COMMITTED, or with no ending frame at all), and the later session may be a blind one, accepted and
committed here. The excluded session itself never becomes a blind write, also after its ABORT; stale DATA / END /
ABORT of other sessions keep the selection; its own frames keep it alive; ten quiet seconds end it. A selected
transfer that is still running ignores other sessions (its own inactivity ends it). COMMITTED is that session's end:
the next session's frame boots the committed image.

    python bootloader/test/test_bl_selection_scope_v407.py <V407 bl.elf> [--json out.json]
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, WORK, image, frame
from test_bl_declared_v407 import announce, transfer, committed

OTHER = bytes(12)
RECEIVE, FAILED = 1, 4


def node(elf, app=True):
    """a bootloader listening for a requested session (app installed) or in recovery (none)"""
    m = MCU(elf)
    if app:
        m.installed(image(m.target, version=0x30000))
    m.call('bl_app_read', [WORK])
    m.call('bl_sess_init', [m.addr('sess'), 3 if app else 4, WORK, int(m.now_us // 1000)])
    return m


def blind(m, img, session):
    """a declared transfer; True = committed with flash work of its own"""
    before = len(m.ops)
    announce(m, img, session); transfer(m, img, session)
    return committed(m, img) and len(m.ops) > before


def untouched(m, img, session):
    before = bytes(m.flash); ops = len(m.ops)
    announce(m, img, session); transfer(m, img, session)
    return bytes(m.flash) == before and len(m.ops) == ops


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []

    def case(name, fn):
        try:
            ok = bool(fn())
            row = dict(case=name, ok=ok)
        except Exception as e:                       # noqa: BLE001
            row = dict(case=name, ok=False, error=repr(e))
        rows.append(row)
        print(json.dumps(row), flush=True)

    def other_abort_blind():
        m = node(a.elf); img = image(m.target)
        m.select(7, uid=OTHER); m.send(frame(0x84, 7))
        m.advance(m.now_us + 2_002_000); m.run_due()
        return blind(m, img, 8)

    def other_new_session():
        m = node(a.elf); img = image(m.target)
        m.select(7, uid=OTHER)
        return blind(m, img, 8)

    def other_preamble_blind():
        m = node(a.elf); img = image(m.target)
        m.select(7, uid=OTHER); m.send(frame(0x83, 7, data=img[24:28]+img[20:24]+bytes(24)))
        for _ in range(3): m.send(frame(0x85, 8))
        return blind(m, img, 8)

    def other_same_session():
        m = node(a.elf); img = image(m.target)
        m.select(7, uid=OTHER); m.send(frame(0x84, 7))
        return untouched(m, img, 7)

    def other_stale_frames():
        m = node(a.elf); img = image(m.target)
        m.select(7, uid=OTHER)
        for f in (frame(0x82, 8, 0, img[64:96]), frame(0x84, 8), frame(0x83, 8, data=img[24:28]+img[20:24]+bytes(24)),
                  frame(0x85, 0)):
            m.send(f)
        return untouched(m, img, 7)

    def other_kept_alive():
        m = node(a.elf, app=False); img = image(m.target)
        m.select(7, uid=OTHER)
        for _ in range(12):                          # 12 s of the excluded session's frames, 1 s apart
            m.advance(m.now_us + 1_000_000); m.send(frame(0x85, 7))
        return untouched(m, img, 7)

    def other_timeout():
        quiet = []
        for wait_us in (9_990_000, 10_010_000):
            m = node(a.elf, app=False); img = image(m.target)
            m.select(7, uid=OTHER)
            m.advance(m.now_us + wait_us); m.run_due()
            quiet.append(m)
        return untouched(quiet[0], image(quiet[0].target), 7) and blind(quiet[1], image(quiet[1].target), 7)

    def self_revoked_abort_blind():
        m = node(a.elf); img = image(m.target)
        m.lock(); st = m.start(img)
        assert st and st['state'] in (3, 4) and m.stv('op', 1) == 0, st     # READY / RECEIVE: revoked + erased
        m.send(frame(0x84, 7))
        assert m.state == FAILED and not m.run_due(), 'revoked: stays for recovery'
        return blind(m, img, 8)

    def self_listen_blind():
        m = node(a.elf); img = image(m.target)
        m.lock(); m.select(7)
        return blind(m, img, 8)

    def self_running_ignores_others():
        m = node(a.elf); img = image(m.target)
        m.lock(); m.start(img)
        other = image(m.target, length=1000, version=0x30200)
        for f in (frame(0x85, 8), frame(0x81, 8, 0, other[:32]), frame(0x81, 8, 1, other[32:64])):
            m.send(f)
        st = m.transfer(img)
        return st and st['state'] == 6 and committed(m, img) and m.stv('selected', 1) == 1

    def self_committed_next_session():
        m = node(a.elf); img = image(m.target)
        m.lock(); m.start(img); st = m.transfer(img)
        assert st['state'] == 6, st
        for f in (frame(0x82, 8, 0, img[64:96]), frame(0x84, 8)):   # stale: the endpoint stays
            m.send(f, run=False)
            assert not m.run_due(), 'stale frame released COMMITTED'
        m.send(frame(0x85, 8), run=False)
        return bool(m.run_due()) and committed(m, img)

    def self_listen_quiet():
        m = node(a.elf, app=False); img = image(m.target)
        m.lock(); m.select(7)
        m.advance(m.now_us + 10_010_000); m.run_due()
        return blind(m, img, 7)

    def self_failed_quiet():
        m = node(a.elf); img = image(m.target)
        m.lock(); m.start(img); m.send(frame(0x84, 7))
        assert m.state == FAILED and m.poll() is not None, 'FAILED answers its session'
        m.advance(m.now_us + 9_990_000); m.run_due()
        kept = m.stv('selected', 1) == 1
        m.advance(m.now_us + 20_000); m.run_due()
        return kept and m.stv('selected', 1) == 0 and blind(m, img, 8)

    case('other UID, ABORT, quiet, blind session 8 commits', other_abort_blind)
    case('other UID, then blind session 8 without an ending frame', other_new_session)
    case('other UID, END, session 8 PING preamble, blind commits', other_preamble_blind)
    case('other UID: its own session never writes, also after ABORT', other_same_session)
    case('other UID: stale DATA/ABORT/END of session 8 and a confirm PING keep it', other_stale_frames)
    case('other UID: its session\'s frames keep it beyond 10 s', other_kept_alive)
    case('other UID: 10 s quiet ends it (not 9.99 s)', other_timeout)
    case('self, revoked, ABORT, blind session 8 commits', self_revoked_abort_blind)
    case('self, before its ANNOUNCE, blind session 8 commits', self_listen_blind)
    case('self, running transfer ignores session 8 and commits', self_running_ignores_others)
    case('self, COMMITTED, stale frames hold, session 8 PING boots it', self_committed_next_session)
    case('self, before its ANNOUNCE, 10 s quiet: blind session 7 commits', self_listen_quiet)
    case('self, FAILED (revoked): kept at 9.99 s, 10 s quiet ends it', self_failed_quiet)
    if a.json: a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} selection-scope cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__': sys.exit(main())

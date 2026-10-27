# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""COMMITTED is retained until the selected session acknowledges with PING, or until the next session starts.

Session-0 PINGs and stale DATA / END / ABORT / STATUS of other sessions keep the hold through long silence; the
next session's PING or ANNOUNCE (nonzero, another session) ends it: the committed image boots (a node never stays
dark in its bootloader; decided 2026-10-23)."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from test_bl_status_fail_v407 import fresh
from abi3_model_v407 import frame


def committed_node(elf):
    m, img = fresh(elf)
    m.drain()
    assert m.transfer(img)['state'] == 6
    return m, img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf', type=Path)
    a = ap.parse_args()
    m, img = committed_node(a.elf)
    before, ops = bytes(m.flash), len(m.ops)
    # Lost polls, then much longer silence; rate must be reacquired, status retained.
    for delay in (10100000, 60000000):
        m.advance(m.now_us + delay)
        assert not m.run_due(), 'COMMITTED rebooted without selected PING'
        m.lock()
        st = m.poll()
        assert st and st['state'] == 6 and st['committed'] == len(img)-64
        for f in (frame(0x85, 0), frame(0x82, 8, 0, img[64:96]), frame(0x83, 8, data=img[24:28]+img[20:24]+bytes(24)),
                  frame(0x84, 8), frame(0x86, 8, 0, m.uid + bytes([0, 1, 0]) + bytes(17))):
            m.send(f)
            assert not m.run_due(), 'a session-0 PING or a stale frame released COMMITTED'
    assert bytes(m.flash) == before and len(m.ops) == ops
    m.ack()
    assert m.run_due(), 'selected PING did not acknowledge'
    for start in (frame(0x85, 8), frame(0x81, 8, 0, img[:32])):
        m, img = committed_node(a.elf)
        m.send(start, run=False)
        assert m.run_due(), 'the next session did not end the COMMITTED hold'
    print('COMMITTED survives silence, polls, session-0 PING and stale frames; selected ACK or next session exits')


if __name__ == '__main__':
    main()

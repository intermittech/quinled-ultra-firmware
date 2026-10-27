# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 app opens its update-only downstream gate only inside a quiet inter-frame LOW: after >= 350 us of LOW with
no edge (src/mcu/ch32v40x/upd_forward.h; docs/timing.md "Update forwarding and entry"), never partway through a frame.

API builds of main_ur_digital_m.c (the sampled GPIO tick of test_upd_group_chain_v407.py) receive UPD frames with
exact sender gaps after each frame's last falling edge, the trailer decoded 150 us into the gap. Every gate opening is
checked against the line: >= 350 us after the last falling edge and before the next rising edge.
- 349 us gaps never open it; 450 us gaps open it in the first gap after transparency, for ten tick phases;
- a lone edge 349 us into a LOW restarts the count; a host release 250 us after a frame (released HIGH, answer)
  cannot open it until 350 us after the line is LOW again;
- two nodes, 500 us gaps: the whole >= 1000 ms nonzero-session PING preamble, then the entry ANNOUNCE pair; the second
  node receives the preamble and the complete pair (it takes the image) and opens its own gate;
- a marker port (the delay line not in CHAIN) never opens it, however many UPD headers
  come; the 2 s idle close published while DIN is HIGH waits for >= 350 us of quiet LOW: FWD_EN never changes in a HIGH.
Every App runs in CHAIN unless a case says marker; App asserts that FWD_EN never changes while DIN is HIGH.

    python test/test_upd_gate_quiet_v407.py <V407 API app fw.elf> <V407 bl.elf> [--json out.json]
"""
import argparse
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'test'), str(ROOT/'tools'), str(ROOT/'bootloader/test')]
from test_upd_group_chain_v407 import App          # noqa: E402
from abi3_model_v407 import image, frame            # noqa: E402

S_US = 50                                          # samples per us (50 MHz)
QUIET = 350 * S_US
DECODE = 150 * S_US                                # the trailer is decoded this far into the gap


class Gate(App):
    def __init__(self, elf, boot, chain=True):
        self.spans = []                            # (first rise, last fall) sample indices of every frame
        self.opens = []                            # sample index of each opening
        super().__init__(elf, boot, chain=chain)

    def wr(self, a, n, v):
        before = self.forwarding
        super().wr(a, n, v)
        if self.forwarding and not before:
            self.opens.append(self.tick * 2500)

    def bits(self, f):
        out = bytearray()
        for i in range(len(f) * 8):
            n = ((i + 1) * 125 // 2) - (i * 125 // 2)
            hi = 40 if f[i // 8] & (128 >> (i & 7)) else 20
            out.extend(b'\1' * hi + b'\0' * (n - hi))
        return out

    def send(self, f, gap_us, now_ms, pattern=()):
        """the frame, then exactly gap_us of LOW after its last falling edge (`pattern`: (offset_us, high_us) pulses
        inside that LOW); the decode 150 us after the last falling edge; returns the app's action"""
        b = self.bits(f)
        last = max(i for i in range(len(b)) if b[i]) + 1
        start = len(self.samples)
        self.spans.append((start, start + last))
        low = bytearray(gap_us * S_US)
        for offset_us, high_us in pattern:
            at = int(offset_us * S_US)
            low[at:at + int(high_us * S_US)] = b'\1' * int(high_us * S_US)
        self.drive(b[:last] + low[:DECODE])
        self.poke(self.addr('ms_now'), struct.pack('<I', now_ms))
        flags = f[-6] & 15
        self.call('ud_header', [flags, now_ms * 1000, 0, 0])
        action = 0
        if flags & 8:
            self.poke(0x20018000, f)
            self.call('upd_app_frame', [self.addr('upd'), 0x20018000, now_ms])
            action = self.ret
        self.drive(low[DECODE:])
        return action

    def check(self, highs=()):
        """every opening: >= 350 us after the last HIGH sample before it and not inside a frame; `highs`: extra
        HIGH spans (start, end) of the scripted LOWs"""
        bad = []
        edges = sorted(self.spans + list(highs))
        for s in self.opens:
            inside = any(a <= s - 1 < b for a, b in edges)
            last_fall = max([b for a, b in edges if b <= s] or [0])
            if inside or s - last_fall < QUIET:
                bad.append(dict(at_us=s / S_US, low_us=(s - last_fall) / S_US, inside=inside))
        return bad


def ping(session=7):
    return frame(0x85, session)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('app', type=Path); p.add_argument('boot', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []

    def record(name, ok, **extra):
        row = dict(case=name, ok=bool(ok), **extra)
        rows.append(row); print(json.dumps(row), flush=True)

    g = Gate(a.app, a.boot)
    for k in range(30): g.send(ping(), 349, k)
    record('349 us gaps never open the gate', not g.opens and not g.check(), frames=30)

    for phase in range(0, 50, 5):
        g = Gate(a.app, a.boot); g.drive(bytes(phase * S_US))
        for k in range(4): g.send(ping(), 450, k)
        first_gap_end = g.spans[2][0]
        ok = len(g.opens) == 1 and g.opens[0] < first_gap_end and not g.check()
        record('450 us gaps open it in the first gap after transparency', ok, phase_us=phase,
               opened_after_us=(g.opens[0] - g.spans[1][1]) / S_US if g.opens else None)

    g = Gate(a.app, a.boot)
    g.send(ping(), 450, 0)
    g.send(ping(), 1200, 1, pattern=[(349, 0.4)])
    glitch = g.spans[1][1] + 349 * S_US
    ok = len(g.opens) == 1 and g.opens[0] - (glitch + 20) >= QUIET and not g.check([(glitch, glitch + 20)])
    record('an edge 349 us into the LOW restarts the count', ok,
           opened_after_edge_us=(g.opens[0] - glitch - 20) / S_US if g.opens else None)

    g = Gate(a.app, a.boot)
    g.send(ping(), 450, 0)
    g.send(ping(), 2000, 1, pattern=[(250, 1200)])
    high = (g.spans[1][1] + 250 * S_US, g.spans[1][1] + 1450 * S_US)
    ok = len(g.opens) == 1 and g.opens[0] - high[1] >= QUIET and not g.check([high])
    record('a host release 250 us after the frame cannot open it', ok,
           opened_after_low_us=(g.opens[0] - high[1]) / S_US if g.opens else None)

    nodes = [Gate(a.app, a.boot), Gate(a.app, a.boot)]
    img = image(nodes[0].target)
    received = 0; frames_sent = 0; t_us = 0.0; actions = [[], []]
    def chain(f):
        """the frame passes only if node 0's gate is open from its first rise to its last fall"""
        nonlocal received, frames_sent, t_us
        through = nodes[0].forwarding
        events = len(nodes[0].events)
        actions[0].append(nodes[0].send(f, 500, int(t_us // 1000)))
        rise, fall = nodes[0].spans[-1]
        through = through and not any(rise <= e['tick'] * 2500 <= fall + 2500 for e in nodes[0].events[events:])
        if through:
            received += 1
            actions[1].append(nodes[1].send(f, 500, int(t_us // 1000)))
        frames_sent += 1
        t_us += (nodes[0].spans[-1][1] - nodes[0].spans[-1][0]) / S_US + 500
    while t_us < 1_000_000:
        chain(ping())
    preamble = frames_sent; first = preamble - received
    for half in (0, 1):
        chain(frame(0x81, 7, half, img[half * 32:(half + 1) * 32]))
    ok = (received == frames_sent - first and first <= 3 and actions[1][-1] == 1 and actions[0][-1] == 1 and
          nodes[1].opens and not nodes[0].check() and not nodes[1].check())
    record('500 us gaps: 1000 ms preamble and the entry pair reach the next node', ok, preamble_frames=preamble,
           not_forwarded_first=first, second_node_frames=received, second_node_took=actions[1][-1] == 1,
           second_node_opened_us=(nodes[1].opens[0] / S_US) if nodes[1].opens else None)

    # the marker port (two nonzero-session PINGs once opened FWD_EN)
    g = Gate(a.app, a.boot, chain=False)
    for k in range(6): g.send(ping(), 450, k)
    record('marker port: UPD headers never open the gate', not g.opens and not g.forwarding and not g.events, frames=6)

    # the idle close published while DIN is HIGH
    for chain_on in (True, False):
        g = Gate(a.app, a.boot, chain=chain_on)
        for k in range(4): g.send(ping(), 450, k)
        opened = g.forwarding
        err = None
        try:
            g.drive(b'' * 2500)
            g.poke(g.addr('ms_now'), struct.pack('<I', 2011))
            g.call('upd_service')
            before = len(g.events)
            g.drive(b'' * 12500)                    # 250 us more of HIGH: the close must wait
            in_high = g.events[before:]
            fall = len(g.samples)
            g.drive(bytes(25000))                       # 500 us of LOW
        except AssertionError as e:
            err = str(e)
            in_high, fall = [], 0
        closes = [e['tick'] * 2500 for e in g.events if not e['open']]
        if chain_on:
            ok = (err is None and opened and not in_high and not g.forwarding and len(closes) == 1 and
                  closes[0] - fall >= QUIET)
        else:
            ok = err is None and not opened and not g.events
        record(f'idle close published in a HIGH waits for the quiet LOW ({"CHAIN" if chain_on else "marker"})', ok,
               opened=opened, error=err, closed_after_low_us=(closes[0] - fall) / S_US if closes and fall else None)

    if a.json: a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} app gate cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__': sys.exit(main())

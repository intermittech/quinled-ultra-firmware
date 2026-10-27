# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""V203 BOARD 1 / 2 apps: the frame-end gate re-arm, and the UPD-transparent plan's open, write FWD_EN only in a genuine
quiet LOW (decided 2026-10-23), on the compiled poll ISR (tools/rv32emu.py).

The 20 kHz tick (poll_isr) runs on a wire model: HIGH intervals give DIN (PA6) and the falling edges; TIM4 CNT counts the
falling edges since the firmware last wrote it (the re-arm). The tick's first CNT read is its sample at the tick; every
later read in the same call sees the wire 1 us later, so a frame that starts during the ISR is counted there, with DIN
LOW between its bits. Per plan (Ultra, WS2805, WS2915, UPD-transparent); the frame end (the re-arm) comes 200-250 us
after the last falling edge for an Ultra plan (ULTRA_RW_REARM_US: past the hub's release, decided 2026-10-25) and 150-200 us
for a WS2805 / WS2915 plan (its reset gap is >= 280 us):
  valid    a frame, then >= 500 us of LOW, twice: the gate writes (PA1, TIM4 CNT, the toggle setup) equal the reference
           image's (--ref): a WS plan's at the same ticks, an Ultra plan's FT_SHIFT_US later (--ref before the 200 us
           re-arm; --same: a reference with it, no shift); every re-arm at least the plan's end time after the last edge;
  high     DIN HIGH from 60 us after the last edge for 1 ms (a hub's release), over the frame-end tick: no gate write while
           DIN is HIGH, the toggles stopped; no re-arm before the HIGH; the re-arm by the plan's end time after its fall;
  release  (Ultra plans) the same with the HIGH from 150 us after the last edge (the latest release);
  during   a frame starting 0.2 us after the frame-end tick's sample: no re-arm in that tick or inside that frame; the
           re-arm by the plan's end time after its end;
  gap N    a frame N = 10 / 50 / 90 us after the last edge: one frame for the tracker, no re-arm inside it;
  open     (transparent) a frame starting 0.2 us after the sample of the tick that would open the gate: no open.
Checked on every case: no PA1 write while DIN is HIGH.
Usage: python test_gate_rearm_v203.py <app fw.elf> [--ref <fw.elf>] [--same] [--json out]"""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu  # noqa: E402

TIM4_CNT, TIM4_DMAINTENR, TIM4_CH2CVR = 0x40000824, 0x4000080C, 0x40000838
GPIOA_INDR, GPIOA_BSHR = 0x40010808, 0x40010810
DMA4_CFGR, DMA4_CNTR = 0x40020044, 0x40020048
PA1_SET, PA1_CLR = 1 << 1, 1 << 17
TICK_US, ISR_US = 50.0, 1.0
BIT_US, ZERO_US = 1.25, 0.3125
# gate_plan_t: start_open, t1, t2, own_bits, hdr_bits, ws_end (core/proto.h)
PLANS = {'ultra': ((0, 256, 0, 256, 0, 0), 576), 'ws2805': ((0, 40, 0, 40, 0, 1), 200),
         'ws2915': ((1, 32, 112, 112, 32, 1), 272), 'transparent': ((1, 0, 0, 384, 0, 0), 384)}
REARM_US = 200.0                                         # ULTRA_RW_REARM_US
END = {0: (REARM_US, REARM_US + TICK_US), 1: (150.0, 200.0)}   # [ws_end]: the re-arm's window after the last falling edge
FT_SHIFT_US = TICK_US                                    # an Ultra plan's end against a reference before it (4 vs 3 idle ticks)
GATE_REGS = (GPIOA_BSHR, TIM4_CNT, TIM4_DMAINTENR, TIM4_CH2CVR, DMA4_CFGR, DMA4_CNTR)


class Wire:
    def __init__(self):
        self.highs = []

    def frame(self, t0, nbits):
        for i in range(nbits):
            self.highs.append((t0 + i * BIT_US, t0 + i * BIT_US + ZERO_US))
        return self.highs[-1][1]

    def high(self, a, b):
        self.highs.append((a, b))


class Tick(Emu):
    def __init__(self, elf, plan, wire):
        super().__init__(elf)
        self.highs = sorted(wire.highs)
        self.rises = [a for a, _ in self.highs]
        self.falls = sorted(b for _, b in self.highs)
        self.base = 0
        self.t = 0.0
        self.reads = 0
        self.log = []                                    # (tick time, write time, address, value)
        self.problems = []
        self.max_cycles = 0
        self.rearm_ticks = []
        packed = struct.pack('<BxHHHBB', *plan)
        self.poke(self.addr('plan_buf'), packed + packed)
        self.poke(self.addr('plan'), packed)
        self.poke(self.addr('rx_ready'), b'\1')
        self.poke(self.addr('ft') + 3, b'\1')            # ftrack_t.armed: the line was idle before
        self.mmio[DMA4_CNTR] = 0

    def din(self, t):
        i = bisect_right(self.rises, t) - 1
        return i >= 0 and self.highs[i][1] > t

    def now(self):
        return self.t if self.reads <= 1 else self.t + ISR_US

    def rd(self, a, n):
        if a == TIM4_CNT:
            self.reads += 1
            return (bisect_right(self.falls, self.now()) - self.base) & 0xFFFF
        if a == GPIOA_INDR:
            return 64 if self.din(self.now()) else 0
        return super().rd(a, n)

    def wr(self, a, n, v):
        super().wr(a, n, v)
        if a not in GATE_REGS or (a == GPIOA_BSHR and not v & (PA1_SET | PA1_CLR)):
            return
        t = self.t + ISR_US
        if a == TIM4_CNT:
            self.base = bisect_right(self.falls, t) - v
            self.rearm_ticks.append(self.t)
        if a == GPIOA_BSHR and self.din(t):
            self.problems.append(f'PA1 written ({"open" if v & PA1_SET else "close"}) at {t:.1f} us while DIN is HIGH')
        self.log.append((round(self.t, 3), a, v))

    def run(self, t_end):
        while self.t < t_end:
            self.t += TICK_US
            self.reads = 0
            self.call('poll_isr')
            self.max_cycles = max(self.max_cycles, self.cycles())
        return self


def last_fall_before(m, t):
    i = bisect_right(m.falls, t) - 1
    return m.falls[i] if i >= 0 else None


def run_case(elf, name, plan_name, build):
    plan, nbits = PLANS[plan_name]
    w = Wire()
    info = build(w, nbits)
    m = Tick(elf, plan, w).run(info['end'])
    return m, info


def valid(w, nbits):
    a = w.frame(1000.3, nbits)
    b = w.frame(a + 700.0, nbits)
    return dict(end=b + 3000.0)


def case_high(w, nbits, at=60.0):
    a = w.frame(1000.3, nbits)
    w.high(a + at, a + at + 1000.0)
    return dict(end=a + 3000.0, last=a, high=(a + at, a + at + 1000.0))


def gap(n):
    def build(w, nbits):
        a = w.frame(1000.3, nbits)
        b = w.frame(a + n, nbits)
        return dict(end=b + 3000.0, last=a, b_end=b)
    return build


def check_case(elf, ref, plan_name, case, shift=True):
    """one case: (ok, failures, details)"""
    bad = []
    det = {}
    ws = PLANS[plan_name][0][5]
    lo, hi = END[ws]
    if case == 'valid':
        m, _ = run_case(elf, case, plan_name, valid)
        bad += m.problems
        det['re-arm ticks us'] = m.rearm_ticks
        det['max poll_isr cycles'] = m.max_cycles
        if not m.rearm_ticks:
            bad.append('no re-arm')
        early = [t for t in m.rearm_ticks if t - last_fall_before(m, t) < lo]
        if early:
            bad.append(f're-armed under {lo:.0f} us after the last edge at {early[:2]}')
        if ref:
            r, _ = run_case(ref, case, plan_name, valid)
            det['reference max cycles'] = r.max_cycles
            d = FT_SHIFT_US if shift and not ws else 0.0
            # (the transparent plan's open keeps its tick: >= 350 us after the last edge, GATE_QUIET_TICKS one fewer)
            want = [(round(t + (0.0 if a == GPIOA_BSHR and v & PA1_SET else d), 3), a, v) for t, a, v in r.log]
            if want != m.log:
                diff = next((i for i, (x, y) in enumerate(zip(want, m.log)) if x != y), min(len(want), len(m.log)))
                bad.append(f'gate writes differ from the reference (+{d:.0f} us) at write {diff}: '
                           f'{want[diff:diff + 2]} vs {m.log[diff:diff + 2]}')
        return bad, det
    if case == 'high' or case == 'release':
        m, info = run_case(elf, case, plan_name, lambda w, nb: case_high(w, nb, 60.0 if case == 'high' else 150.0))
        bad += m.problems
        h0, h1 = info['high']
        inside = [t for t, a, v in m.log if h0 <= t + ISR_US < h1 and a in (GPIOA_BSHR, TIM4_CNT, TIM4_CH2CVR)]
        if inside:
            bad.append(f'gate written while DIN was HIGH at {inside[:3]}')
        before = [t for t in m.rearm_ticks if info['last'] < t < h0]
        if before:
            bad.append(f're-armed before the HIGH (a release at {h0:.1f}) at {before[:2]}: it would hold the release')
        after = [t for t in m.rearm_ticks if t > h0]
        det['re-arms after the HIGH began'] = after
        if not after or not (h1 + lo <= after[0] <= h1 + hi):
            bad.append(f're-arm after the HIGH: {after[:2]}, want one {lo:.0f}-{hi:.0f} us after its fall at {h1:.1f}')
        return bad, det
    if case == 'during' or case == 'open':
        # find the tick of the first re-arm (or, for open, of the open) on the valid wire, then start a frame there
        r, _ = run_case(elf, case, plan_name, lambda w, nb: dict(end=w.frame(1000.3, nb) + 3000.0))
        if case == 'during':
            t_k = r.rearm_ticks[0] if r.rearm_ticks else None
        else:
            t_k = next((t for t, a, v in r.log if a == GPIOA_BSHR and v & PA1_SET), None)
        if t_k is None:
            return [f'no {"re-arm" if case == "during" else "open"} on the valid wire'], det

        def build(w, nbits):
            a = w.frame(1000.3, nbits)
            b = w.frame(t_k + 0.2, nbits)
            return dict(end=b + 3000.0, last=a, b0=t_k + 0.2, b_end=b)
        m, info = run_case(elf, case, plan_name, build)
        bad += m.problems
        det['tick'] = t_k
        if case == 'during':
            inside = [t for t in m.rearm_ticks if info['b0'] - TICK_US < t <= info['b_end'] + 99.0]
            if inside:
                bad.append(f're-armed at {inside[:3]} us: in the tick the frame began, or inside it '
                           f'({info["b0"]:.1f}-{info["b_end"]:.1f} us)')
            after = [t for t in m.rearm_ticks if t > info['b_end']]
            det['re-arms after'] = after[:2]
            if not after or not (info['b_end'] + lo <= after[0] <= info['b_end'] + hi):
                bad.append(f're-arm after the frame: {after[:2]}, want {lo:.0f}-{hi:.0f} us after {info["b_end"]:.1f}')
        else:
            opens = [t for t, a, v in m.log if a == GPIOA_BSHR and v & PA1_SET and info['b0'] - TICK_US < t <= info['b_end']]
            if opens:
                bad.append(f'opened at {opens[:3]} us with a frame begun ({info["b0"]:.1f} us)')
        return bad, det
    if case.startswith('gap'):
        n = float(case.split()[1])
        m, info = run_case(elf, case, plan_name, gap(n))
        bad += m.problems
        inside = [t for t in m.rearm_ticks if info['last'] <= t <= info['b_end'] + 99.0]
        if inside:
            bad.append(f're-armed inside the joined frame at {inside[:3]} us')
        det['re-arms'] = m.rearm_ticks[:3]
        return bad, det
    raise ValueError(case)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--ref', type=Path)
    p.add_argument('--same', action='store_true', help='the reference has the 200 us re-arm too: no shift')
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []
    for plan in PLANS:
        cases = (['valid', 'high'] + ([] if PLANS[plan][0][5] else ['release']) + ['during', 'gap 10', 'gap 50', 'gap 90'] +
                 (['open'] if plan == 'transparent' else []))
        for case in cases:
            bad, det = check_case(str(a.elf), str(a.ref) if a.ref else None, plan, case, not a.same)
            rows.append(dict(plan=plan, case=case, ok=not bad, failures=bad, details=det))
            print(f"  {plan} {case}: {'ok' if not bad else 'FAIL'} {json.dumps(det)}" +
                  ''.join('\n      ' + f for f in bad), flush=True)
    fails = sum(not r['ok'] for r in rows)
    print('gate re-arm v203: %s' % (f'FAIL, {fails} of {len(rows)}' if fails else f'ok, {len(rows)} cases'))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())

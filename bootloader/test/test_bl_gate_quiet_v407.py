# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 bootloader's downstream gate (TIM2 CH4 on FWD_EN) is written only inside a quiet inter-frame LOW:
  open   forced HIGH (update forwarding) after >= 350 us of LOW with no edge (docs/timing.md "Update forwarding and
         entry": port_gap_us >= 500 us; 350 us is above the 280 us reset recognition and the 250 us late-release reject,
         decided 2026-10-23), never partway through a frame;
  slot   in recovery on a normal Ultra line (the session's gate BL_GATE_ULTRA) the slot plan (closed for the board's own
         256 bits, then toggled open at the 256th edge: CH4CVR 256) re-arms after >= ULTRA_RW_REARM_US (200 us) of LOW
         with no edge after a frame, never in a released HIGH (decided 2026-10-25: a re-arm before the hub's release
         would hold it from the boards behind), and the boards behind keep their slots.

The production ELF runs from reset into its main loop; DIN (PB4) follows a scripted line that starts 100 us after reset
(the boot's DIN escape check and init run first): Ultra frames of 384 bits at 800 kHz (0.4 / 0.8 us HIGH) with sender gaps
of exactly 349 / 500 us after the last falling edge, a lone edge 349 us into a LOW, and a frame whose host releases the
line after its end (the released HIGH and an answer, then LOW again). TIM2 CNT counts the line's falling edges since its
last write (TIM3 CH1 captures them, TIM2 counts the captures).

    python bootloader/test/test_bl_gate_quiet_v407.py <V407 bl.elf> [--json out.json]
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from test_bl_forward_v407 import Forward, Opened, TIM2_CHCTLR2, OCM_HI, OCM_TOGGLE
from abi3_model_v407 import offsets

QUIET_US = 350
REARM_US = 200                                     # ULTRA_RW_REARM_US
FRAME_END = 383 * 1.25 + 0.8                       # an Ultra frame's last falling edge after its first rise
START = 100.0                                      # the scripted line starts after the boot's init
SLOW = 1 / (1 - 0.0455)                            # the boot clock is at most 4.55 % slow (clock.h)
TIM2_CH4CVR = 0x40000040


class Line(Forward):
    """DIN from a list of (start_us, kind, length_us): 'frame' (Ultra bits, ends on its last falling edge), 'high'"""
    def __init__(self, elf, events, ultra=False, until=None):
        super().__init__(elf)
        self.events = events
        self.ultra = ultra                         # the session's gate: BL_GATE_ULTRA from the main loop's start
        self.until = until
        self.gate_writes = []                      # (time, OC4M) of every gate mode write

    def din(self, t):
        for start, kind, length in self.events:
            if start <= t < start + length:
                if kind == 'high':
                    return 1
                bit = int((t - start) / 1.25)
                return int((t - start) - bit * 1.25 < (0.8 if bit % 3 else 0.4))
        return 0

    def fall_times(self):
        out = []
        for start, kind, length in self.events:
            if kind == 'high':
                out.append(start + length)
            else:
                out += [start + k * 1.25 + (0.8 if k % 3 else 0.4) for k in range(384)]
        return sorted(out)

    def _check_app(self, pc, registers):
        if self.ultra and pc == self.addr('bl_rx_init'):
            # BL_GATE_ULTRA, as bl_sess_frame sets it for a valid ordinary Ultra frame in recovery (no decoder here)
            self.poke(self.addr('sess') + offsets()['gate'], (1).to_bytes(4, 'little'))
        super()._check_app(pc, registers)

    def rd(self, a, n):
        if self.until is not None and self.now_us >= self.until:
            raise Opened()                         # (the scripted line is over)
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == TIM2_CHCTLR2:
            self.gate_writes.append((self.now_us, (v >> 12) & 7))
            assert not self.din(self.now_us), f'gate written at {self.now_us:.1f} us while DIN is HIGH'
            if self.ultra and (v >> 12) & 7 == OCM_TOGGLE:
                assert self.mmio.get(TIM2_CH4CVR, 0) == 256, 'slot plan: CH4CVR %d' % self.mmio.get(TIM2_CH4CVR, 0)
        if self.until is None:
            return super().wr(a, n, v)
        entering, self.entering = self.entering, False       # (run on through the openings)
        try:
            super().wr(a, n, v)
        finally:
            self.entering = entering


def frames(gap, count, first=START):
    """frames back to back: each starts `gap` us after the previous one's last falling edge"""
    return [(first + k * (FRAME_END + gap), 'frame', FRAME_END) for k in range(count)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []

    def case(name, events, quiet_from, slack=2.0):
        """opens after >= 350 us of LOW from `quiet_from` (its last edge), within the slow clock's bound, outside
        every frame and HIGH"""
        try:
            m = Line(a.elf, events)
            m.boot_forward()
            assert m.opened, 'the gate never opened'
            t = m.opened[0]
            inside = any(start <= t < start + length for start, _, length in events)
            ok = quiet_from + QUIET_US <= t <= quiet_from + QUIET_US * SLOW + slack and not inside
            row = dict(case=name, opened_us=round(t, 3), low_before_us=round(t - quiet_from, 3), ok=ok)
        except Exception as e:                       # noqa: BLE001
            row = dict(case=name, ok=False, error=repr(e))
        rows.append(row)
        print(json.dumps(row), flush=True)

    def slot(name, events, until, want):
        """the slot plan (recovery on a normal Ultra line): every re-arm (toggle at 256) >= 200 us after the last falling
        edge, inside no frame and no HIGH; want = the expected re-arm count after the first opening"""
        try:
            m = Line(a.elf, events, ultra=True, until=until)
            m.boot_forward(max_steps=40000000)
            falls = m.fall_times()
            arms = [t for t, ocm in m.gate_writes if ocm == OCM_TOGGLE]
            bad = []
            for t in arms:
                last = max((f for f in falls if f <= t), default=None)
                inside = any(start <= t < start + length for start, _, length in events)
                if inside:
                    bad.append(f're-armed inside a frame / HIGH at {t:.1f}')
                elif last is not None and t - last < REARM_US:
                    bad.append(f're-armed {t - last:.1f} us after the last edge at {t:.1f}')
            if any(ocm == OCM_HI for _, ocm in m.gate_writes):
                bad.append('forced open on a normal Ultra line in recovery')
            if len(arms) != want:
                bad.append(f'{len(arms)} re-arms, want {want}')
            row = dict(case=name, rearms_us=[round(t, 1) for t in arms], ok=not bad, failures=bad)
        except Exception as e:                       # noqa: BLE001
            row = dict(case=name, ok=False, error=repr(e))
        rows.append(row)
        print(json.dumps(row), flush=True)

    case('quiet from reset: 350 us from the loop', [], 0.0, slack=120.0)
    case('a frame, then LOW: 350 us after its last falling edge', frames(0, 1), START + FRAME_END)
    case('an edge 349 us into the LOW restarts the count', frames(0, 1) + [(START + FRAME_END + 349, 'high', 0.4)],
         START + FRAME_END + 349.4)
    gaps349 = frames(349, 4)
    case('349 us sender gaps never open it', gaps349, gaps349[-1][0] + FRAME_END)
    case('500 us sender gaps: open in the first gap, before the next frame', frames(500, 3), START + FRAME_END)
    release = frames(0, 1) + [(START + FRAME_END + 250, 'high', 1200)]
    case('a late RW release (250 us), answer, LOW again', release, START + FRAME_END + 250 + 1200)
    # the slot plan: the first opening after 350 us of quiet LOW from the loop, then a re-arm in every gap
    f3 = frames(500, 3, first=600.0)
    slot('slot plan: frames 500 us apart, a re-arm >= 200 us into each gap', f3, f3[-1][0] + FRAME_END + 400, 1 + 3)
    rel = frames(500, 1, first=600.0)
    rel_end = rel[0][0] + FRAME_END
    rel += [(rel_end + 150, 'high', 1500), (rel_end + 2200, 'frame', FRAME_END)]
    slot('slot plan: a release 150 us after the frame: no re-arm before it or in its HIGH', rel,
         rel_end + 2200 + FRAME_END + 400, 1 + 2)
    if a.json: a.json.write_text(json.dumps(rows, indent=2))
    bad = sum(not r['ok'] for r in rows)
    print(f'{len(rows)} bootloader gate cases, {bad} failures')
    return int(bad != 0)


if __name__ == '__main__': sys.exit(main())

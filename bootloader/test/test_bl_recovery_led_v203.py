# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""V203 bootloaders (BOARD 1/2/3): the recovery indicator on the production ELF's whole main loop.

Recovery (no usable app) while no update runs blinks the status LED (PB15, active high) slowly: on for the first second
of the bootloader's ms time base, off for the next, 1 s on / 1 s off (0.5 Hz); otherwise it stays off. The outputs stay
dark. The wire model is test_bl_forward_v203's (production decoder, TIM2 CH2 on FWD_EN); a quiet line (no HIGH within
1.5 ms before / 2 ms after) advances 50 us per SysTick read, so a pass is at most a few such steps. Checked:
  - recovery on an idle line: the LED turns on in the first pass and changes at every whole second of the time base
    (within 1 ms), 8 changes in 7.3 s; after the initialization only the LED's set / reset writes reach a GPIO, no
    timer but the gate's (TIM2 / TIM3) is ever written, no gate problem (test_bl_forward_v203's checks);
  - recovery with ordinary Ultra frames every 100 ms (the slot plan): the same blink;
  - recovery, then an update: the blink until the ANNOUNCE, then off within 1 ms of it and through every erase,
    program and the commit until the reset; the image is committed;
  - recovery with a refused (foreign) image: FAILED, no update runs: the blink goes on;
  - a session the app asked for (app installed): off all the way to its 3 s exit.
Model evidence on the production image, not an electrical qualification.
Usage: python test_bl_recovery_led_v203.py <bl.elf> [--json out]"""
import argparse
from bisect import bisect_left, bisect_right
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model import MCU, Reset, STK, frame, image  # noqa: E402
from test_bl_forward_v203 import Fwd, Wire, session_wire, committed, ultra_frame  # noqa: E402

GPIOA, GPIOB = 0x40010800, 0x40010C00
LED_SET, LED_RESET = 1 << 15, 1 << 31
GATE_TIMERS = (0x40000000, 0x40000400)                   # TIM2 (FWD_EN, edge count), TIM3 (DIN capture)
TIMERS = (0x40000000, 0x40000400, 0x40000800, 0x40000C00, 0x40012C00, 0x40013400, 0x40014C00, 0x40015000, 0x40015400)
STEP_US = 50.0
TOL_US = 1000.0


class Stop(Exception):
    pass


class Led(Fwd):
    def __init__(self, elf, wire, stop_us, enter=False):
        super().__init__(elf, wire, enter=enter)
        self.stop_us = stop_us
        self.led = []                                    # (time, 0 / 1): every change
        self.t_init = None                               # SysTick started (hw_full_init): the ms time base's zero
        self.init_done = False                           # the sampler is on: hw_full_init's GPIO setup is over
        self.stray = []                                  # GPIO / timer writes after the initialization, not the LED's
        self.timers = set()
        self.wkeys = sorted(self.wire_words)
        self.dirty = set()

    def quiet(self):
        k = bisect_right(self.starts, self.now_us)
        nxt = self.starts[k] if k < len(self.starts) else float('inf')
        prev = max(self.ends[:k], default=-1e9)
        return nxt - self.now_us > 2000.0 and self.now_us - prev > 1500.0

    def refresh_ring(self):
        """WireMCU's ring refresh, skipping a quiet span whose ring slots already hold LOW"""
        end = int(self.now_us * 36) // 16
        start = max(self.sample_word, end - 2048)
        i = bisect_left(self.wkeys, start)
        if not self.dirty and (i == len(self.wkeys) or self.wkeys[i] >= end):
            self.sample_word = end
            return end & 2047
        ring = self.addr('hw_ring')
        for word in range(start, end):
            value = self.wire_words.get(word, 0)
            slot = word & 2047
            if value or slot in self.dirty:
                self.poke(ring + 2 * slot, value.to_bytes(2, 'little'))
                if value:
                    self.dirty.add(slot)
                else:
                    self.dirty.discard(slot)
        self.sample_word = end
        return end & 2047

    def rd(self, a, n):
        if self.now_us > self.stop_us:
            raise Stop()
        if a == STK + 8 and self.sampler_enabled and self.opened_at() is not None:
            if self.quiet():
                self.advance(self.now_us + STEP_US)
            return MCU.rd(self, a, n)                    # (no jump to the next frame: the LED's changes stay visible)
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == STK and v & 32:
            self.t_init = self.now_us
        if a == 0x40013000 and v & 64:
            self.init_done = True
        level = None
        if a == GPIOB + 0x10 and v in (LED_SET, LED_RESET):
            level = int(v == LED_SET)
        elif self.init_done and GPIOA <= a < GPIOB + 0x400:
            self.stray.append((round(self.now_us), hex(a), hex(v)))
        if any(b <= a < b + 0x400 for b in TIMERS):
            self.timers.add(a & ~0x3ff)
            if a & ~0x3ff not in GATE_TIMERS:
                self.stray.append((round(self.now_us), hex(a), hex(v)))
        if level is not None and (not self.led or self.led[-1][1] != level):
            self.led.append((self.now_us, level))
        super().wr(a, n, v)

    def level_at(self, t):
        i = bisect_right([x for x, _ in self.led], t) - 1
        return self.led[i][1] if i >= 0 else 0


def run(m):
    try:
        m.call('main', max_steps=400000000)
    except Reset:
        return 'reset'
    except Stop:
        return 'stopped'
    return 'returned'


def blink_check(m, until_us, bad, label=''):
    """changes at every whole second of the time base: on at 0, off at 1 s, ... (each within TOL_US after)"""
    changes = [(t - m.t_init, lv) for t, lv in m.led if lv or t > m.t_init]
    changes = [(t, lv) for t, lv in changes if t >= 0 and t + m.t_init <= until_us]
    if changes and changes[0][1] == 0:
        changes = changes[1:]                            # (hw_full_init's LED off)
    want = int((until_us - m.t_init) // 1e6) + 1
    if len(changes) != want:
        bad.append(f'{label}{len(changes)} LED changes, want {want}: {[(round(t), lv) for t, lv in changes][:12]}')
    worst = 0.0
    for k, (t, lv) in enumerate(changes):
        late = t - k * 1e6
        worst = max(worst, late)
        if lv != (k + 1) % 2 or not 0.0 <= late <= TOL_US:
            bad.append(f'{label}change {k}: LED {lv} at {t / 1000:.3f} ms (want {(k + 1) % 2} at {k * 1000} ms)')
    return [round(t / 1000, 3) for t, _ in changes], round(worst, 1)


def idle_wire(until_us, extra=None):
    w = Wire()
    if extra:
        extra(w)
    w.frames.append(('sentinel', until_us + 300000.0, until_us + 300001.0, None, False))
    return w


def case_idle(elf):
    stop = 7_300_000.0
    m = Led(elf, idle_wire(stop), stop)
    how = run(m)
    bad = list(m.problems)
    if how != 'stopped':
        bad.append(f'left recovery ({how})')
    changes, worst = blink_check(m, stop, bad)
    if m.stray:
        bad.append(f'writes other than the LED after init: {m.stray[:4]}')
    return dict(case='recovery, idle line', ok=not bad, failures=bad, led_changes_ms=changes, worst_late_us=worst,
                timers=sorted(hex(t) for t in m.timers))


def case_ordinary(elf):
    stop = 3_300_000.0

    def frames(w):
        w.low(2000.0)
        while w.t < stop - 100000.0:
            w.frame(ultra_frame(2), 'ordinary', 0.0, ordinary=True)
            w.low(100000.0 - (w.t % 100000.0))
    m = Led(elf, idle_wire(stop, frames), stop)
    how = run(m)
    bad = list(m.problems)
    if how != 'stopped':
        bad.append(f'left recovery ({how})')
    changes, worst = blink_check(m, stop, bad)
    if m.stray:
        bad.append(f'writes other than the LED after init: {m.stray[:4]}')
    return dict(case='recovery, ordinary frames every 100 ms', ok=not bad, failures=bad, led_changes_ms=changes,
                worst_late_us=worst, frames=sum(1 for f in m.wire.frames if f[4]))


def case_update(elf):
    img = image(MCU(elf).target)
    w = session_wire(img, pre_quiet_us=2_400_000.0)
    stop = w.t + 1_000_000.0
    w.frames.append(('sentinel', stop + 300000.0, stop + 300001.0, None, False))
    m = Led(elf, w, stop)
    how = run(m)
    bad = list(m.problems)
    if how != 'reset' or not committed(m, img):
        bad.append(f'the update did not commit ({how})')
    announce = [b for name, a, b, *_ in w.frames if name == 'announce'][-1]
    changes, _ = blink_check(m, announce, bad)
    off = next((t for t, lv in m.led if t >= announce and lv == 0), None)
    on_after = [round(t) for t, lv in m.led if t >= announce and lv == 1]
    if off is None or off - announce > TOL_US:
        bad.append(f'LED not off within 1 ms of the ANNOUNCE (ended {announce:.0f} us, off at {off})')
    if on_after:
        bad.append(f'LED on during the update at {on_after[:4]} us')
    lit = [(k, round(t)) for k, _, t in m.ops[1:] if m.level_at(t)]
    if lit:
        bad.append(f'flash work with the LED on: {lit[:4]}')
    if m.stray:
        bad.append(f'writes other than the LED after init: {m.stray[:4]}')
    return dict(case='recovery, then an update', ok=not bad, failures=bad, led_changes_ms=changes,
                announce_end_us=round(announce), led_off_after_us=None if off is None else round(off - announce, 1),
                flash_ops=len(m.ops), first_op=m.ops[0][0] if m.ops else None)


def case_refused(elf):
    t = bytearray(MCU(elf).target)
    t[0] = (t[0] % 3) + 1                                # another board's image
    img = image(bytes(t))
    stop = 4_200_000.0

    def foreign(w):
        w.low(1_500_000.0)
        for half in (0, 1):
            w.frame(frame(0x81, 7, half, img[32 * half:32 * half + 32]), 'foreign announce')
        for i in range(4):
            w.frame(frame(0x82, 7, i, img[64 + 32 * i:96 + 32 * i]), 'foreign data')
    m = Led(elf, idle_wire(stop, foreign), stop)
    how = run(m)
    bad = list(m.problems)
    state = m.state
    if how != 'stopped' or state != 4:
        bad.append(f'not FAILED in recovery ({how}, state {state})')
    if m.ops:
        bad.append(f'{len(m.ops)} flash operations for a refused image')
    changes, worst = blink_check(m, stop, bad)
    return dict(case='recovery, refused image (FAILED)', ok=not bad, failures=bad, led_changes_ms=changes,
                worst_late_us=worst)


def case_requested(elf):
    stop = 3_600_000.0
    m = Led(elf, idle_wire(stop), stop, enter=True)
    m.installed(image(m.target, 772, 0x00030000))
    how = run(m)
    bad = list(m.problems)
    if how != 'reset':
        bad.append(f'the requested session did not leave at its 3 s wait ({how})')
    lit = [round(t) for t, lv in m.led if lv]
    if lit:
        bad.append(f'LED on outside recovery at {lit[:4]} us')
    return dict(case='session the app asked for (app installed)', ok=not bad, failures=bad,
                left_at_ms=round(m.now_us / 1000, 1), led_on=len(lit))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []
    for case in (case_idle, case_ordinary, case_update, case_refused, case_requested):
        r = case(a.elf)
        rows.append(r)
        print(f"  {r['case']}: {'ok' if r['ok'] else 'FAIL'} " +
              json.dumps({k: v for k, v in r.items() if k not in ('case', 'ok', 'failures')}) +
              ''.join('\n      ' + f for f in r['failures']), flush=True)
    fails = sum(not r['ok'] for r in rows)
    print('bl recovery LED v203: %s' % (f'FAIL, {fails} of {len(rows)}' if fails else f'ok, {len(rows)} cases'))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())

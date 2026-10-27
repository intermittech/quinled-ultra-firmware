# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""V407 bootloader (BOARD 3): the recovery indicator on the production ELF's whole main loop.

Recovery (no usable app) while no update runs blinks the status LED (PA4, active high) slowly: on for the first second
of the bootloader's ms time base, off for the next, 1 s on / 1 s off (0.5 Hz); otherwise it stays off. The outputs stay
dark. The line model is test_bl_wire_v407's (DIN sampled into the DMA ring, the production decoder); a quiet line (no
HIGH within 1.5 ms before / 2 ms after) advances 50 us per SysTick read. Checked:
  - recovery on an idle line: the LED turns on in the first pass and changes at every whole second of the time base
    (within 1 ms), 8 changes in 7.3 s; after the initialization only the LED's set / reset writes reach a GPIO, no
    timer but the gate's (TIM3 capturing DIN, TIM2 counting its edges and driving FWD_EN) is ever written;
  - recovery, then a group update (two PINGs and the ANNOUNCE pair, unmarked): the blink until the ANNOUNCE, then off
    within 1 ms of it and through the erases until 1.5 s later;
  - a session the app asked for (app installed): off all the way to its 3 s exit.
Model evidence on the production image, not an electrical qualification.
Usage: python test_bl_recovery_led_v407.py <bl.elf> [--json out]"""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, Reset, STK, frame, image  # noqa: E402
from test_bl_wire_v407 import Wire, RING  # noqa: E402

GPIOA, GPIOB = 0x40010800, 0x40010C00
LED_SET, LED_RESET = 1 << 4, 1 << 20
GATE_TIMERS = (0x40000000, 0x40000400)                   # TIM2 (edge count, FWD_EN), TIM3 (DIN capture)
TIMERS = (0x40000000, 0x40000400, 0x40000800, 0x40000C00, 0x40012C00, 0x40013400, 0x40014C00, 0x40015000, 0x40015400)
MHZ = 195.2                                              # the model's HCLK = the time base's tick rate
STEP_US = 50.0
TOL_US = 1000.0


class Stop(Exception):
    pass


class Led(Wire):
    def __init__(self, elf, stop_us, schedule=None):
        super().__init__(elf, MHZ, marker=False)
        self.STK = STK
        self.mmio[0x40021024] = 1 << 27                  # power-on reset
        self.mhz = 20.0
        self.setword('clock_khz', 20000)
        self.setword('options_trusted', 0, 1)
        self.stop_us = stop_us
        self.schedule = schedule
        self.led = []                                    # (time, 0 / 1): every change
        self.t_init = None
        self.init_done = False
        self.stray = []
        self.timers = set()
        self.last_busy = -10 ** 12

    def ticks(self, us):
        return int(us * MHZ)

    def quiet(self):
        now = int(self.timer)
        k = bisect_right(self.pulses, (now, 1 << 62))
        nxt = self.pulses[k][0] if k < len(self.pulses) else 1 << 62
        return nxt - now > self.ticks(2000) and now - self.last_edge() > self.ticks(1500)

    def refresh(self):
        """test_bl_wire_v407's ring refresh, skipped over a quiet span whose ring slots already hold LOW"""
        word = (int(self.timer) - self.base) // 64
        start = max(self.filled, word - RING)
        lo, hi = self.base + start * 64, self.base + word * 64
        k = bisect_right(self.pulses, (hi, 1 << 62))
        busy = any(b > lo for _, b in self.pulses[max(0, k - 400):k])
        if not busy and self.last_busy < start - RING:
            self.filled = word
            return word % RING
        if busy:
            self.last_busy = word
        return super().refresh()

    def rd(self, a, n):
        if self.now_us > self.stop_us:
            raise Stop()
        if a == STK + 8 and self.base is not None:
            if self.quiet():
                self.advance(self.now_us + STEP_US)
            return MCU.rd(self, a, n)                    # (no jump to the next frame: the LED's changes stay visible)
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == STK and v & 32:
            self.t_init = self.now_us
        level = None
        if a == GPIOA + 0x10 and v in (LED_SET, LED_RESET):
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
        if a == 0x40013000 and v & 64:
            self.init_done = True
            if self.schedule:
                self.schedule(self)

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


def blink_check(m, until_us, bad):
    """changes at every whole second of the time base: on at 0, off at 1 s, ... (each within TOL_US after)"""
    changes = [(t - m.t_init, lv) for t, lv in m.led if m.t_init <= t <= until_us]
    if changes and changes[0][1] == 0:
        changes = changes[1:]                            # (hw_full_init's LED off)
    want = int((until_us - m.t_init) // 1e6) + 1
    if len(changes) != want:
        bad.append(f'{len(changes)} LED changes, want {want}: {[(round(t), lv) for t, lv in changes][:12]}')
    worst = 0.0
    for k, (t, lv) in enumerate(changes):
        late = t - k * 1e6
        worst = max(worst, late)
        if lv != (k + 1) % 2 or not 0.0 <= late <= TOL_US:
            bad.append(f'change {k}: LED {lv} at {t / 1000:.3f} ms (want {(k + 1) % 2} at {k * 1000} ms)')
    return [round(t / 1000, 3) for t, _ in changes], round(worst, 1)


def case_idle(elf):
    stop = 7_300_000.0
    m = Led(elf, stop)
    how = run(m)
    bad = []
    if how != 'stopped':
        bad.append(f'left recovery ({how})')
    changes, worst = blink_check(m, stop, bad)
    if m.stray:
        bad.append(f'writes other than the LED after init: {m.stray[:4]}')
    return dict(case='recovery, idle line', ok=not bad, failures=bad, led_changes_ms=changes, worst_late_us=worst,
                timers=sorted(hex(t) for t in m.timers))


def case_update(elf):
    at_us = 2_400_000.0
    ends = []

    def schedule(m):
        img = image(m.target)
        ends.append((int(m.timer), m.now_us))
        for k, f in enumerate([frame(0x85, 7), frame(0x85, 7), frame(0x81, 7, 0, img[:32]),
                               frame(0x81, 7, 1, img[32:64])]):
            ends.append(m.add_frame(m.base + m.ticks(at_us + k * 1000), f,
                                    lambda tick: 1000000 + int((tick - m.base) / MHZ)))
    stop = at_us + 1_500_000.0
    m = Led(elf, stop, schedule)
    how = run(m)
    bad = []
    if how != 'stopped':
        bad.append(f'left the update ({how})')
    erases = [op for op in m.ops if op[0] == 'erase']
    if len(erases) < 2:
        bad.append(f'{len(erases)} erases: the ANNOUNCE did not start the update')
    announce = ends[0][1] + (ends[-1] - ends[0][0]) / MHZ    # (the ANNOUNCE's last falling edge, in us)
    changes, _ = blink_check(m, announce, bad)
    off = next((t for t, lv in m.led if t >= announce and lv == 0), None)
    on_after = [round(t) for t, lv in m.led if t >= announce and lv == 1]
    if off is None or off - announce > TOL_US:
        bad.append(f'LED not off within 1 ms of the ANNOUNCE (ended {announce:.0f} us, off at {off})')
    if on_after:
        bad.append(f'LED on during the update at {on_after[:4]} us')
    lit = [(k, round(t)) for k, _, t in m.ops if m.level_at(t)]
    if lit:
        bad.append(f'flash work with the LED on: {lit[:4]}')
    if m.stray:
        bad.append(f'writes other than the LED after init: {m.stray[:4]}')
    return dict(case='recovery, then a group update', ok=not bad, failures=bad, led_changes_ms=changes,
                announce_end_us=round(announce), led_off_after_us=None if off is None else round(off - announce, 1),
                flash_ops=len(m.ops), state=m.state)


def case_requested(elf):
    stop = 3_600_000.0
    m = Led(elf, stop)
    m.installed(image(m.target, version=0x30000))
    magic, enter = 0x51444f48, 0x55504421
    m.poke(0x20000000, struct.pack('<8I', magic, enter, 0, 0, 0, 0, 0, ~(magic ^ enter) & 0xffffffff))
    m.mmio[0x40021024] = 1 << 28                         # a software reset: the hand-over's request holds
    how = run(m)
    bad = []
    if how != 'reset':
        bad.append(f'the requested session did not leave at its 3 s wait ({how})')
    lit = [round(t) for t, lv in m.led if lv]
    if lit:
        bad.append(f'LED on outside recovery at {lit[:4]} us')
    if m.t_init is None:
        bad.append('the session never initialized (no main loop)')
    return dict(case='session the app asked for (app installed)', ok=not bad, failures=bad,
                left_at_ms=round(m.now_us / 1000, 1), led_on=len(lit))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []
    for case in (case_idle, case_update, case_requested):
        r = case(a.elf)
        rows.append(r)
        print(f"  {r['case']}: {'ok' if r['ok'] else 'FAIL'} " +
              json.dumps({k: v for k, v in r.items() if k not in ('case', 'ok', 'failures')}) +
              ''.join('\n      ' + f for f in r['failures']), flush=True)
    fails = sum(not r['ok'] for r in rows)
    print('bl recovery LED v407: %s' % (f'FAIL, {fails} of {len(rows)}' if fails else f'ok, {len(rows)} cases'))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())

# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""V203 bootloaders (BOARD 1/2/3): update forwarding through TIM2 CH2 on FWD_EN (PA1), on the production ELF's whole main.

The wire is sampled into the DMA ring and decoded by the production decoder (abi3_model.WireMCU); PA6 (DIN) reads the wire
level, TIM2 counts its falling edges, and the model follows TIM2 CH2's output mode (forced LOW / HIGH, the slot plan's
toggle at the 256th edge) and PA1's routing. The port-2 wire is AND(DIN, FWD_EN). Checked (docs/timing.md "Update
forwarding and entry"; decided 2026-10-23: >= 350 us of LOW without an edge):
  - FWD_EN is LOW from reset; it opens only after >= 350 us of LOW without an edge, with DIN LOW: never inside a frame or a
    HIGH. Edges every 349 us keep it closed; a LOW of 350 us plus the loop's latency opens it; a HIGH held from reset (a
    released line) keeps it closed until 350 us after its fall;
  - the entry blackout (reset to open, the line LOW) is measured; declared bound 5 ms;
  - then it stays open for the whole stay: every flash operation (erase, program, status writes) runs with TIM2 CH2 forced
    HIGH and PA1 on the timer, no mode change during a busy interval; every update frame after a declared pause is both
    received locally and forwarded whole; a refused image's FAILED wait keeps it open;
  - recovery after ordinary Ultra frames keeps the slot plan (the first 256 bits held LOW, the rest forwarded);
  - a power cut during an erase: the next boot is closed again until its own quiet LOW;
  - every later return to full forwarding (a frame the slot plan held, then the session plan) re-opens only after
    >= 350 us of LOW without an edge, DIN LOW (a short frame, then a frame 200 us later or DIN held HIGH from
    110 us); nothing writes the gate (TIM2 CNT or CH2 mode, after the first re-arm) while DIN is HIGH, the slot plan's
    frame-end re-arm included (DIN HIGH from 80 us after a slot-planned frame).
Model evidence on the production image, not an electrical qualification.
Usage: python test_bl_forward_v203.py <bl.elf> [--json out]"""
import argparse
from bisect import bisect_left, bisect_right
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model import APP, BODY, END, STK, MCU, WireMCU, PowerCut, Reset, frame, image  # noqa: E402

QUIET_US = 350.0
NOISE_END = [0.0]
TIM2_CHCTLR1, TIM2_CCER, TIM2_CNT, TIM2_CH2CVR = 0x40000018, 0x40000020, 0x40000024, 0x40000038
GPIOA_CFGLR, GPIOA_INDR = 0x40010800, 0x40010808
BLACKOUT_BOUND_US = 5000.0
BIT_US, ZERO_US, ONE_US = 1.25, 0.3125, 0.9375      # profile 0 (800 kHz)


class Wire:
    """the incoming line: HIGH intervals (us) and the frames they make"""

    def __init__(self):
        self.highs = []                                  # (rise, fall)
        self.frames = []                                 # (name, start, end, bytes or None, ordinary)
        self.t = 0.0

    def frame(self, data, name, gap_us=500.0, at=None, ordinary=False):
        t0 = self.t if at is None else at
        for i in range(len(data) * 8):
            h = ONE_US if data[i // 8] & (128 >> (i % 8)) else ZERO_US
            self.highs.append((t0 + i * BIT_US, t0 + i * BIT_US + h))
        end = self.highs[-1][1]
        self.frames.append((name, t0, end, bytes(data), ordinary))
        self.t = end + gap_us
        return end

    def high(self, a, b, name):
        self.highs.append((a, b))
        self.frames.append((name, a, b, None, False))
        self.t = max(self.t, b)

    def low(self, us):
        self.t += us


def ultra_frame(slots=1, seq=0):
    """an ordinary Ultra frame: NOP slots (CRC-good) and a header without the UPD flag"""
    from abi3_model import crc
    out = bytearray()
    for k in range(slots):
        s = bytearray(32)
        s[2] = (seq + k) & 255
        s[30:32] = crc(s[:30], 16, 0x1021, 0xffff).to_bytes(2, 'big')
        out += s
    h = bytearray(b'\x5a\xd2\x30' + bytes(4))
    h.append(crc(h, 8, 7, 0))
    return bytes(out + h)


class Fwd(WireMCU):
    def __init__(self, elf, wire, enter=True, flash=None):
        super().__init__(elf, [])
        self.wire = wire
        self.highs = sorted(wire.highs)
        self.rises = [a for a, _ in self.highs]
        self.wire_words = {}
        for a, b in self.highs:
            for sample in range(int(a * 36), int(b * 36)):
                w = sample // 16
                self.wire_words[w] = self.wire_words.get(w, 0) | (1 << (15 - sample % 16))
        self.falling = sorted(b for _, b in self.highs)
        spans = sorted((f[1], f[2]) for f in wire.frames)
        self.starts = [a for a, _ in spans]
        self.ends = [b for _, b in spans]
        self.mode = 4                                    # TIM2 CH2 after reset: no output (FWD_EN pulled LOW)
        self.cc2e = False
        self.pa1_timer = False
        self.base = 0                                    # TIM2 CNT = falling edges since index `base`
        self.ccr = 0
        self.timeline = [(0.0, 0, 0)]                    # (time, kind 0 LOW / 1 HIGH / 2 slot plan, edge base)
        self.flash_checks = []
        self.problems = []
        self.rearmed = False                             # a re-arm (TIM2 CNT written) has run: the main loop's gate
        if flash is not None:
            self.flash[:] = flash
        if not enter:                                    # a power-on reset, no hand-over request
            self.poke(0x20000000, bytes(32))
            self.mmio[0x40021024] = 0x08000000

    def din(self, t):
        i = bisect_right(self.rises, t) - 1
        return i >= 0 and self.highs[i][1] > t

    def _gate_kind(self):
        if not (self.pa1_timer and self.cc2e):
            return 0
        return {5: 1, 4: 0, 3: 2}.get(self.mode, 0)

    def _log(self):
        kind = self._gate_kind()
        base = bisect_right(self.falling, self.now_us) - self.rd_cnt_raw()
        last = self.timeline[-1]
        if (kind, base if kind == 2 else 0) != (last[1], last[2] if last[1] == 2 else 0):
            if kind == 1 and last[1] != 1:
                quiet_from = max([b for a, b in self.highs if b <= self.now_us], default=0.0)
                if self.din(self.now_us) or self.now_us - quiet_from < QUIET_US:
                    self.problems.append(f'FWD_EN opened at {self.now_us:.1f} us after {self.now_us - quiet_from:.1f} us '
                                         f'of LOW (DIN {"HIGH" if self.din(self.now_us) else "LOW"})')
            self.timeline.append((self.now_us, kind, base))

    def rd_cnt_raw(self):
        return (bisect_right(self.falling, self.now_us) - self.edge_base) & 65535

    def rd(self, a, n):
        if a == GPIOA_INDR:
            return 64 if self.din(self.now_us) else 0
        if a == STK + 8 and self.opened_at() is None and self.now_us < 100000.0:
            # (WireMCU fast-forwards a quiet line to 250 us before the next frame: that would hide the entry's quiet
            # LOW. Until FWD_EN opens, time runs instruction by instruction: the first 100 ms.)
            return MCU.rd(self, a, n)
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == TIM2_CNT:
            self.rearmed = True
        if self.rearmed and a in (TIM2_CHCTLR1, TIM2_CNT) and self.din(self.now_us):
            self.problems.append(f'gate written at {self.now_us:.1f} us while DIN is HIGH')
        super().wr(a, n, v)
        if a == TIM2_CHCTLR1:
            self.mode = (v >> 12) & 7
        elif a == TIM2_CCER:
            self.cc2e = bool(v & 0x10)
        elif a == TIM2_CH2CVR:
            self.ccr = v
        elif a == GPIOA_CFGLR:
            self.pa1_timer = (v >> 4) & 15 == 0xB
        if a in (TIM2_CHCTLR1, TIM2_CCER, GPIOA_CFGLR, TIM2_CNT):
            self._log()

    def operation(self, kind, address, payload, latency):
        k = self._gate_kind()
        self.flash_checks.append((kind, self.now_us, k))
        if k != 1:
            self.problems.append(f'{kind} at {self.now_us:.0f} us with FWD_EN {"LOW" if k == 0 else "on the slot plan"}')
        return super().operation(kind, address, payload, latency)

    def level(self, t):
        """FWD_EN at time t"""
        i = bisect_right([x[0] for x in self.timeline], t) - 1
        at, kind, base = self.timeline[i]
        if kind != 2:
            return kind
        # the slot plan: LOW until the CCR-th falling edge counted since `base`
        return int(bisect_right(self.falling, t) - base >= self.ccr) if self.ccr else 0

    def opened_at(self):
        return next((t for t, k, _ in self.timeline if k == 1), None)

    def forwarded(self):
        """per frame: 'whole', 'none' or 'partial' (FWD_EN constant over every HIGH of it, or not)"""
        res = []
        for name, a, b, data, ordinary in self.wire.frames:
            lv = set()
            i = bisect_left(self.rises, a)
            while i < len(self.highs) and self.highs[i][0] < b + 1e-9:
                r, f = self.highs[i]
                lv.add((self.level(r + 1e-6), self.level(f - 1e-6)))
                i += 1
            ups = {x for p in lv for x in p}
            res.append((name, 'whole' if ups == {1} else 'none' if ups == {0} else 'partial', ordinary))
        return res


def session_wire(img, lead_ms=300, pause_ms=25, passes=2, pre_quiet_us=0.0, extra=None):
    """the boot ANNOUNCE pair, the erase lead, DATA passes with block pauses, END (a declared blind profile)"""
    w = Wire()
    if extra:
        extra(w)
    w.low(max(0.0, 60000.0 + pre_quiet_us - w.t))
    for half in (0, 1):
        w.frame(frame(0x81, 7, half, img[32 * half:32 * half + 32]), 'announce')
    w.low(lead_ms * 1000)
    body = img[64:]
    n = (len(body) + 31) // 32
    for p in range(passes):
        for i in range(n):
            w.frame(frame(0x82, 7, i, body[32 * i:32 * i + 32].ljust(32, b'\0'), pass_no=p), 'data' if i % 8 else 'data-after-pause')
            if i % 8 == 7 or i == n - 1:
                w.low(pause_ms * 1000)
    end = img[24:28] + img[20:24] + bytes(24)
    for _ in range(3):
        w.frame(frame(0x83, 7, data=end), 'end')
    w.low(400000)
    return w


def committed(m, img):
    return m.peek(APP, 64) == img[:64] and m.peek(BODY, len(img) - 64) == img[64:]


def run_main(m):
    try:
        m.call('main', max_steps=60000000)
    except Reset:
        return 'reset'
    except PowerCut:
        return 'cut'
    except AssertionError as e:
        if 'wire stream completed' in str(e):
            return 'end of wire'
        raise
    return 'returned'


def case_session(elf, label, extra=None, expect_open_after=None):
    m0 = MCU(elf)
    img = image(m0.target)
    w = session_wire(img, extra=extra)
    m = Fwd(elf, w)
    m.installed(image(m.target, 772, 0x00030000))
    how = run_main(m)
    fw = m.forwarded()
    bad = list(m.problems)
    if how != 'reset' or not committed(m, img):
        bad.append(f'session did not commit ({how})')
    opened = m.opened_at()
    if opened is None:
        bad.append('never opened')
    if expect_open_after is not None and opened is not None and opened < expect_open_after + QUIET_US:
        bad.append(f'opened at {opened:.1f} us, before {expect_open_after:.1f} + 350 us')
    partial = [n for n, s, _ in fw if s == 'partial']
    if partial:
        bad.append(f'partly forwarded: {partial[:4]}')
    upd = [(n, s) for (n, s, o), f in zip(fw, w.frames) if f[3] is not None]
    blocked = [n for n, s in upd if s != 'whole']
    if blocked:
        bad.append(f'update frames not forwarded: {blocked[:4]}')
    received = len(m.received)
    after_pause = [(n, s) for n, s in upd if n == 'data-after-pause']
    got_at = sorted(t for t, *_ in m.received)
    for name, a, b, data, _ in w.frames:
        if name == 'data-after-pause' and not any(b <= t <= b + 3000 for t in got_at[bisect_left(got_at, b):][:3]):
            bad.append(f'the frame after the pause at {b:.0f} us was not received locally')
    ops = len(m.flash_checks)
    if ops == 0:
        bad.append('no flash operation ran')
    modes_in_busy = [t for t, k, _ in m.timeline[1:] if any(t2 <= t <= t2 + 25000 for _, t2, _ in m.flash_checks)]
    if modes_in_busy:
        bad.append(f'FWD_EN changed during flash work at {modes_in_busy[:3]}')
    return dict(case=label, ok=not bad, failures=bad, blackout_us=opened, flash_ops=ops,
                frames=len(w.frames), forwarded=sum(s == 'whole' for _, s, _ in fw), received=received,
                after_pause_forwarded=sum(s == 'whole' for _, s in after_pause), after_pause=len(after_pause))


def noise_349(w):
    """pulses every 349 us of LOW (an edge just before the quiet count completes) for 5 ms, from reset"""
    t = 0.0
    while t < 5000.0:
        w.high(t, t + 1.0, 'noise')
        t += 1.0 + 349.0
    w.t = t
    NOISE_END[0] = t - 349.0


def high_from_reset(w):
    """a released line: HIGH from reset to 2 ms"""
    w.high(0.0, 2000.0, 'held HIGH')


def case_boundary(elf):
    """349 us LOW between edges never opens; a 400 us LOW (350 + the loop's latency) opens before the next pulse"""
    w = Wire()
    t = 0.0
    while t < 4000.0:
        w.high(t, t + 1.0, 'p349')
        t += 350.0
    last349 = t - 350.0 + 1.0
    w.high(last349 + 400.0, last349 + 401.0, 'p400')
    w.high(last349 + 2000.0, last349 + 2001.0, 'tail')
    w.t = last349 + 2001.0
    w.frames.append(('sentinel', w.t + 300000.0, w.t + 300001.0, None, False))
    m = Fwd(elf, w)
    m.installed(image(m.target, 772, 0x00030000))
    m.call('bl_app_read', [0x20004500])
    try:
        m.call('main', max_steps=4000000)
    except Exception:
        pass
    bad = list(m.problems)
    opened = m.opened_at()
    if opened is None or opened < last349 + QUIET_US or opened > last349 + 400.0:
        bad.append(f'the 400 us LOW after the last 349 us gap: opened at {opened}, want in '
                   f'[{last349 + QUIET_US:.0f}, {last349 + 400:.0f}] us')
    return dict(case='edges every 349 us, then 400 us of LOW', ok=not bad, failures=bad, opened_us=opened,
                last_349_edge_us=last349)


def case_failed(elf):
    """a refused image (another target): FAILED wait, ABORT; forwarding stays open, no flash work"""
    m0 = MCU(elf)
    t = bytearray(m0.target)
    t[0] = (t[0] % 3) + 1                               # another board's image
    img = image(bytes(t))
    w = Wire()
    w.low(60000.0)
    for half in (0, 1):
        w.frame(frame(0x81, 7, half, img[32 * half:32 * half + 32]), 'foreign announce')
    w.low(3000000.0)                                    # the FAILED / foreign wait
    for i in range(4):
        w.frame(frame(0x82, 7, i, img[64 + 32 * i:96 + 32 * i]), 'foreign data')
    w.frame(frame(0x84, 7), 'abort')
    w.low(12000000.0)
    m = Fwd(elf, w)
    m.installed(image(m.target, 772, 0x00030000))
    how = run_main(m)
    fw = m.forwarded()
    bad = list(m.problems)
    if m.flash_checks:
        bad.append(f'{len(m.flash_checks)} flash operations for a refused image')
    if any(s != 'whole' for (n, s, o), f in zip(fw, w.frames) if f[3] is not None):
        bad.append(f'not forwarded: {[n for n, s, _ in fw if s != "whole"][:4]}')
    if m.opened_at() is None:
        bad.append('never opened')
    return dict(case='refused image: FAILED / foreign wait, ABORT', ok=not bad, failures=bad, exit=how,
                forwarded=sum(s == 'whole' for _, s, _ in fw), frames=len(fw))


def case_cut(elf):
    """a power cut during an erase; the next boot (power-on, recovery) is closed until its own quiet LOW, then the
    recovery's slot plan after ordinary Ultra frames: the first 256 bits held, the rest forwarded"""
    m0 = MCU(elf)
    img = image(m0.target)
    w = session_wire(img)
    m = Fwd(elf, w)
    m.installed(image(m.target, 772, 0x00030000))
    m.cut_at = 2
    how = run_main(m)
    bad = list(m.problems)
    if how != 'cut':
        bad.append(f'no cut ({how})')
    retained = bytes(m.flash)
    w2 = Wire()
    w2.high(0.0, 100.0, 'pulse at power-up')
    w2.low(1000.0)
    for k in range(6):
        w2.frame(ultra_frame(2, k), 'ordinary', 500.0, ordinary=True)
    w2.low(100000.0)
    w2.frames.append(('sentinel', w2.t + 300000.0, w2.t + 300001.0, None, False))
    r = Fwd(elf, w2, enter=False, flash=retained)
    try:
        r.call('main', max_steps=8000000)
    except Exception:
        pass
    bad += r.problems
    opened = r.opened_at()
    if opened is None or opened < 100.0 + QUIET_US:
        bad.append(f'after the cut: opened at {opened} (want >= 450 us)')
    fw = r.forwarded()
    plans = [s for n, s, o in fw if o]
    # the first ordinary frame comes after the open (whole: the plan is OPEN); the later ones are slot-planned (partial)
    if not plans or plans[0] != 'whole' or any(s != 'partial' for s in plans[1:]):
        bad.append(f'recovery plans: {plans}')
    tail_ok = True
    for (name, a, b, data, ordinary), (_, s, _) in zip(w2.frames, fw):
        if ordinary and s == 'partial':
            # FWD_EN rises at the 256th falling edge: bits 0..255 held, 256.. forwarded
            first_open = next((r0 for r0, f0 in r.highs if a <= r0 <= b and r.level(r0 + 1e-6)), None)
            want = a + 256 * BIT_US
            tail_ok &= first_open is not None and abs(first_open - want) < 0.01
    if not tail_ok:
        bad.append('slot plan does not hold exactly the first 256 bits')
    return dict(case='power cut in an erase; recovery boot and slot plan', ok=not bad, failures=bad,
                reopened_us=opened, plans=plans)


class _Stop(Exception):
    pass


class _Until(Fwd):
    def rd(self, a, n):
        if self.now_us > self.stop_us:
            raise _Stop()
        return super().rd(a, n)


def case_reopen(elf, kind):
    """recovery (a power-on reset, no request) after ordinary Ultra frames, then:
    'continuation': a 128-bit frame the slot plan held (FWD_EN still LOW), the next frame 200 us later;
    'held-high': the same short frame, then DIN HIGH from 110 us after its last falling edge to 1.5 ms;
    'slot-high': DIN HIGH from 80 us after a slot-planned frame (FWD_EN open from its 256th edge), over the re-arm.
    Full forwarding re-opens only after >= 350 us of LOW without an edge with DIN LOW; no gate write while DIN is HIGH."""
    w = Wire()
    w.low(3000.0)
    w.frame(ultra_frame(2), 'ordinary-1', 600.0, ordinary=True)
    end = w.frame(ultra_frame(2), 'ordinary-2', 600.0 if kind != 'slot-high' else 0.0, ordinary=True)
    if kind == 'slot-high':
        w.high(end + 80.0, end + 1500.0, 'held HIGH')
    else:
        end = w.frame(bytes(16), 'short frame', 200.0)
        if kind == 'held-high':
            w.high(end + 110.0, end + 1500.0, 'held HIGH')
        else:
            w.frame(bytes(32), 'continuation', 600.0)
    w.frames.append(('sentinel', w.t + 300000.0, w.t + 300001.0, None, False))
    m = _Until(elf, w, enter=False)
    m.stop_us = w.t + 301000.0                           # (past the sentinel: the quiet line is fast-forwarded to it)
    try:
        m.call('main', max_steps=8000000)
    except _Stop:
        pass
    bad = list(m.problems)
    fw = dict((n, s) for n, s, _ in m.forwarded())
    reopen = next((t for t, k, _ in m.timeline if k == 1 and t > end), None)
    if kind == 'held-high' and fw.get('held HIGH') != 'none':
        bad.append(f"the HIGH after the held short frame: {fw.get('held HIGH')}, want none")
    if kind == 'slot-high' and fw.get('held HIGH') != 'whole':
        bad.append(f"the HIGH over the slot plan's re-arm: {fw.get('held HIGH')}, want whole (never cut)")
    if kind != 'slot-high' and reopen is None:
        bad.append('full forwarding never re-opened')
    return dict(case=f'recovery re-open: {kind}', ok=not bad, failures=bad, last_edge_us=round(end, 3),
                reopened_us=None if reopen is None else round(reopen, 3), forwarded=fw)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = [case_session(a.elf, 'quiet entry: whole session'),
            case_session(a.elf, 'noisy entry: edges every 349 us for 5 ms', noise_349, 4901.0),
            case_session(a.elf, 'released line HIGH from reset to 2 ms', high_from_reset, 2000.0),
            case_boundary(a.elf), case_failed(a.elf), case_cut(a.elf),
            case_reopen(a.elf, 'continuation'), case_reopen(a.elf, 'held-high'), case_reopen(a.elf, 'slot-high')]
    for r in rows:
        print(f"  {r['case']}: {'ok' if r['ok'] else 'FAIL'} " +
              json.dumps({k: v for k, v in r.items() if k not in ('case', 'ok', 'failures')}) +
              ''.join('\n      ' + f for f in r['failures']), flush=True)
    q = rows[0].get('blackout_us')
    print(f"entry blackout (reset to open, quiet LOW): {q:.0f} us of the declared {BLACKOUT_BOUND_US:.0f} us"
          if q else 'entry blackout: not measured')
    if q and q > BLACKOUT_BOUND_US:
        rows.append(dict(case='blackout bound', ok=False, failures=[f'{q} us > {BLACKOUT_BOUND_US} us']))
    fails = sum(not r['ok'] for r in rows)
    print('bl forward v203: %s' % (f'FAIL, {fails} of {len(rows)}' if fails else f'ok, {len(rows)} cases'))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())

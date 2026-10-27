# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Update-only forwarding of the Diff-Power-UR-Digital-M (BOARD 3, CH32V203) on the COMPILED release app, with the main loop
PREEMPTED by the 20 kHz tick (the harness of test_ur_digital_m_chain_budget.py, its scenario hook).

FWD_EN (PA1) is TIM2 CH2's output compare (mcu/ch32v20x/upd_forward.h): every mode write (forced LOW / forced HIGH / the slot
plan's toggle at the 256th falling edge after TIM2's count was zeroed) and DE_OUT (PA10) are logged against the sampled line (the
cycles into the tick); the port-2 wire is AND(line, FWD_EN, DE_OUT), as the board's gate and port 2 make it, and is decoded again
as the next board would. Checked (docs/timing.md "Update forwarding and entry", decided 2026-10-23: >= 350 us of LOW without an
edge; the slot plan's re-arm after ULTRA_RW_REARM_US, decided 2026-10-25):
  - the update transparency opens and closes only after >= 350 us of LOW (12,600 samples) without an edge, with the line LOW:
    never partway through a frame or a release; the slot plan re-arms only after >= 200 us of quiet LOW (after the relay
    window behind an RW frame), never inside a frame; every frame reaches the next board whole, from its 256th falling edge on
    (the slot plan: this board's slot held), or not at all; a released HIGH whole, or cut only by a relay window (port 2
    listens from the turnaround);
  - closed after reset (forced LOW); two UPD headers in a row open it, so the third UPD frame on is forwarded whole;
  - the whole >= 1000 ms nonzero-session PING preamble at 500 us gaps, the entry ANNOUNCE pair, DATA and END pass and decode
    downstream byte for byte; a foreign ANNOUNCE never takes;
  - the first ordinary frame (an RW POLL with its release) passes whole, then the transparency closes; later ordinary frames
    pass from their 256th falling edge on (the slot plan);
  - 349 / 350 us: a LOW of 787 ring words (349.8 us) at a tick does not open, 788 (350.2 us) does; an edge 349.8 us into the
    LOW restarts the count; a late HIGH (a release 300 us after the frame) restarts it too;
  - 2000 ms without UPD frames closes it (before ordinary traffic resumes);
  - a marker port (one receiver: AUTO decides STREAM) never opens it;
  - Lava Lamp 600 px running, 500 and 400 us gaps; max_isr + 60 <= 7,200 cycles.
Model evidence on the compiled image, not an electrical qualification.
Usage: python test_ud_forward_v203.py <ur_digital_m fw.elf> [--json out] [--jobs a,b] [--workers n]"""
import argparse
import json
import os
import struct
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_ur_digital_m_chain_budget as t  # noqa: E402

S = 36                              # samples per us (SPI1 at 36 MHz)
QUIET = 350 * S                     # decided 2026-10-23: >= 350 us of LOW without an edge
QUIET_W = (QUIET + 15) // 16        # MCU_US2W(350) = 788 ring words
GPIOA_CFGLR, GPIOA_BSHR, GPIOA_BCR = 0x40010800, 0x40010810, 0x40010814
TIM2_CHCTLR1, TIM2_CNT = 0x40000018, 0x40000024    # OC2M drives FWD_EN: 4 forced LOW, 5 forced HIGH, 3 the slot plan (toggle)
REARM = 200 * S                     # ULTRA_RW_REARM_US
UPD = 8                             # ULTRA_HDR_UPD
PING, ANNOUNCE, DATA, END = 0x85, 0x81, 0x82, 0x83


def block(typ, session=0, index=0, data=b''):
    f = bytearray(40)
    f[:4] = bytes([typ, session]) + index.to_bytes(2, 'big')
    f[6:6 + len(data)] = data
    f[38:40] = t.crc(f[:38], 16, 0x1021, 0xffff).to_bytes(2, 'big')
    return bytes(f)


def tick_words(k):
    """ring words the harness has written after k ticks (steps of 112, 113, 112, ...)"""
    return 112 * ((k + 1) // 2) + 113 * (k // 2)


class Line(t.Line):
    """the hub's line at 36 MHz, profile 0 (1.25 us bits); frames keep their bytes and spans"""

    def __init__(self):
        super().__init__(150)
        self.spans = []                                 # (name, start sample, end sample = after the last HIGH, bytes)

    def frame_at(self, start, body, flags, name, host=None):
        """a frame starting at sample `start` (the line LOW up to it); returns its end (the sample after the last HIGH)"""
        if len(self.line) < start:
            self.line.extend(bytes(start - len(self.line)))
        assert len(self.line) == start, (len(self.line), start)
        nbits = (len(body) + 8) * 8
        f = body + t.header(0 if host is None else host, flags)
        for _ in range(0 if host is not None else 3):
            end = start + (nbits - 1) * t.PERIOD + (t.H1 if f[-1] & 1 else t.H0)
            g = body + t.header(end // S, flags)
            if (g[-1] & 1) == (f[-1] & 1):
                f = g
                break
            f = g
        for byte in f:
            for k in range(8):
                h = t.H1 if (byte >> (7 - k)) & 1 else t.H0
                self.line.extend(b"\1" * h + b"\0" * (t.PERIOD - h))
        end = start + (nbits - 1) * t.PERIOD + (t.H1 if f[-1] & 1 else t.H0)
        del self.line[end:]                              # the LOW after the last bit belongs to the gap
        self.frame_ends.append((name, end))
        self.spans.append((name, start, end, bytes(f)))
        return end

    def frame(self, body, flags, name, gap_us, release_us=None):
        end = self.frame_at(len(self.line), body, flags, name)
        if release_us is not None:                      # an RW frame: the hub lets go after release_us, retakes after one slot
            self.high_at(end + int(release_us * S), end + (1000 + 500) * S, name + ' release')
        self.low_to(len(self.line) / S + gap_us)
        return end

    def high_at(self, a, b, name):
        if len(self.line) < a:
            self.line.extend(bytes(a - len(self.line)))
        self.line.extend(b"\1" * (b - a))
        self.spans.append((name, a, b, None))

    def frame_ending_at_word(self, end_word, body, flags, name):
        """the frame whose last HIGH sample is the last sample of word end_word - 1 (its LOW run starts at end_word)"""
        nbits = (len(body) + 8) * 8
        host = end_word * 16 // S
        f = body + t.header(host, flags)
        start = end_word * 16 - ((nbits - 1) * t.PERIOD + (t.H1 if f[-1] & 1 else t.H0))
        assert start >= len(self.line), 'no room for the aligned frame'
        end = self.frame_at(start, body, flags, name, host)
        assert end == end_word * 16, (end, end_word * 16)
        return end


def setup(L, npx, effect):
    """seeded qualification (run_once's qualified=1), a few ordinary frames, the effect's CONFIG / PIXEL_FX"""
    L.low_to(3000)
    for i in range(3):
        L.frame(t.slot(bytes([0, 0, 0x10 + i])), 0, 'training', 9000)
    cfg = bytes([0x05, 0, 0x20]) + bytes([64, 0, 0, 66, npx >> 8, npx & 255, 67, 0, 60])
    L.frame(t.slot(cfg), 0, 'config', 9000)
    fx = bytearray([0x0B, 15, 0x21, effect, 0, 0, 0, 0, 128, 128, 0xFF, 255, 0, 0] + [0] * 6 + [255, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    L.frame(t.slot(bytes(fx)), 0, 'fx', 9000)
    for i in range(3):
        L.frame(t.slot(bytes([0, 0, 0x30 + i])), 0, 'ordinary-before', 500)
    L.frame(t.slot(bytes([0x07, 255, 0x40, 0, 1, 0x10, 0, 0, 0])), 1, 'poll-before', 9000, 150)


def foreign_descriptor():
    """a well-formed descriptor of another board's image (BOARD 1): the transit app evaluates it, never takes it"""
    from test_boot_api_app import crc32_words
    d = bytearray(64)
    d[:4] = b'QDU6'
    d[4] = 2
    d[8] = 1                                            # target board 1 (this app is BOARD 3)
    d[15] = 3
    struct.pack_into('>IIIII', d, 16, 0x00030100, 64, 0x12345678, 0x08002100, 0x08002100)
    struct.pack_into('>I', d, 60, crc32_words(bytes(d[:60])))
    return bytes(d)


class Gate(t.Board):
    """the compiled app; FWD_EN (PA1) as TIM2 CH2's mode writes leave it, DE_OUT (PA10) as its GPIO writes"""

    def __init__(self, elf):
        super().__init__(elf)
        self.log = []                                    # (sample, kind 'mode' / 'cnt' / 'deo', value, the tick's ring word)
        self.cfg_at_first = None

    def _live(self):
        """the sample the CPU is at now: the tick's newest ring word plus the cycles run since the tick began"""
        return round(self.model_us * S) + self.cycle_clock // 4

    def rd(self, a, n):
        # The live check (dl_live_moved) reads PA6 and TIM4 several us into the tick: the line at that moment, not at the
        # tick's start, as on the chip (the harness itself sets both at the tick's start only).
        if getattr(self, 'model_us', None) is not None and hasattr(self, 'scenario'):
            line = self.scenario.line.line
            if a == 0x40000824:
                w0 = round(self.model_us * S)
                s1 = min(self._live(), len(line))
                seg = line[max(0, w0 - 1):s1]
                falls = seg.count(b'\x01\x00')
                return (super().rd(a, n) + falls) & 0xFFFF
            if a == 0x40010808:
                s1 = self._live()
                hi = line[s1] if s1 < len(line) else 0
                return (super().rd(a, n) & ~64) | (64 if hi else 0)
        return super().rd(a, n)

    def wr(self, a, n, v):
        ev = None
        if a == TIM2_CHCTLR1:
            ev = ('mode', (v >> 12) & 7)
        elif a == TIM2_CNT:
            ev = ('cnt', v)
        elif a == GPIOA_BSHR and v & ((1 << 10) | (1 << 26)):
            ev = ('deo', 0 if v & (1 << 26) else 1)
        elif a == GPIOA_BCR and v & (1 << 10):
            ev = ('deo', 0)
        elif a == GPIOA_CFGLR:
            self.mmio[a] = v                             # (kept: the pin writes read it back; PA1's nibble is checked)
        super().wr(a, n, v)
        if ev is not None:
            at = getattr(self, 'model_us', None)
            if at is not None:                           # (the boot's writes, before the line model ran: closed, driving)
                self.log.append((round(at * S) + self.cycle_clock // 4, ev[0], ev[1], round(at * S) // 16))
                if ev == ('mode', 5) and self.cfg_at_first is None:
                    self.cfg_at_first = (self.rd(GPIOA_CFGLR, 4) >> 4) & 15


class Scenario:
    board = Gate

    def __init__(self, name, L, expect, extra=None):
        self.name, self.line, self.expect, self.extra = name, L, expect, extra or {}

    def prepare(self, e):
        e.scenario = self

    def observe(self, e):
        pass

    def finish(self, e, result):
        L = self.line.line
        n = len(L)
        bad, opened, rearms = [], [], []
        lvl, deo = [(0, 0)], [(0, 1)]                    # (sample, level) changes of FWD_EN and of DE_OUT

        def quiet(sample, need):
            """the line LOW for `need` samples up to `sample`: None, else the LOW it had"""
            if sample >= need and L.find(1, sample - need, min(n, sample + 1)) == -1:
                return None
            return (sample - L.rfind(1, 0, min(n, sample + 1)) - 1) / S

        def fall_after(c0, k):
            """the k-th falling edge of the line after sample c0 (TIM2 counts falling edges), or None"""
            i = c0
            for _ in range(k):
                i = L.find(b'\x01\x00', i)
                if i < 0:
                    return None
                i += 1
            return i
        log = e.log
        mode, transp, c0 = 4, 0, 0
        for k, (sample, kind, v, word) in enumerate(log):
            if kind == 'deo':
                if v != deo[-1][1]:
                    deo.append((sample, v))
                continue
            if kind == 'cnt':
                c0 = sample
                continue
            nxt = log[k + 1:k + 4]
            rearm = v == 4 and [x[1:3] for x in nxt[:2]] == [('cnt', 0), ('mode', 3)]
            turn = v == 4 and any(x[1] == 'deo' and x[2] == 0 for x in nxt)
            if v == 5 or (v == 4 and transp and not rearm and not turn):
                q = quiet(sample, QUIET)
                if q is not None and lvl[-1][1] != (v == 5):  # (closing a gate a relay window closed already changes nothing)
                    bad.append(f"the transparency {'opened' if v == 5 else 'closed'} at {sample / S:.1f} us after only {q:.1f} us of LOW")
                transp = int(v == 5)
                opened.append(dict(us=round(sample / S, 1), word=word, open=transp))
            if v == 3:
                q = quiet(sample, REARM)
                if q is not None:
                    bad.append(f'the slot plan re-armed at {sample / S:.1f} us after only {q:.1f} us of LOW')
                rearms.append(sample)
            mode = v
            if v in (4, 5):
                if lvl[-1][1] != (v == 5):
                    lvl.append((sample, int(v == 5)))
            else:
                end = next((x[0] for x in log[k + 1:] if x[1] == 'mode'), n)
                tg = fall_after(c0, 256)                 # the toggle: FWD_EN HIGH from this board's slot's last falling edge
                if tg is not None and tg < end:
                    lvl.append((tg, 1))
        out = bytearray(n)
        marks = sorted([(s_, 'f', x) for s_, x in lvl] + [(s_, 'd', x) for s_, x in deo])
        f_on, d_on, at = 0, 1, 0
        for s_, which, x in marks + [(n, 'end', 0)]:
            s_ = min(max(s_, 0), n)
            if f_on and d_on:
                out[at:s_] = L[at:s_]
            at = s_
            if which == 'f':
                f_on = x
            elif which == 'd':
                d_on = x
        listen = []                                      # the relay windows: port 2 listens (DE_OUT low)
        for i, (s_, x) in enumerate(deo):
            if x == 0:
                listen.append((s_, next((s2 for s2, y in deo[i + 1:] if y), n)))
        passed, want = [], []
        for name, a, b, data in self.line.spans:
            seg = out[a:b]
            whole = seg == L[a:b]
            none = seg.find(1) == -1
            tg = fall_after(a, 256) if data is not None else None
            slot = (data is not None and tg is not None and tg < b and out[a:tg].find(1) == -1 and out[tg:b] == L[tg:b]
                    and not whole and not none)
            windowed = data is None and any(la < b and lb > a for la, lb in listen)
            if not (whole or none or slot or windowed):
                bad.append(f'{name} at {a / S:.0f} us cut: partly forwarded')
            if data is not None:
                passed.append((name, whole and not none))
                if whole and not none:
                    want.append(data)
                elif slot and len(data) > 32:
                    want.append(data[32:])               # the rest of the frame after this board's slot
        # the next board's decoder: every forwarded frame's bytes again (the slot plan's: from bit 257 on), from the AND output alone
        decoded = decode(out)
        if decoded != want:
            bad.append(f'downstream decode: {len(decoded)} frames, {len(want)} forwarded'
                       + ('' if len(decoded) != len(want) else ' (bytes differ)'))
        got = [p for _, p in passed]
        if self.expect is not None and got != self.expect:
            first = next(i for i, (g, w) in enumerate(zip(got, self.expect)) if g != w) if len(got) == len(self.expect) else -1
            bad.append(f'forwarded frames differ from the expectation at #{first} '
                       f'({passed[first][0] if first >= 0 else "length"}): got {sum(got)} passed, want {sum(self.expect)}')
        if e.cfg_at_first not in (None, 0xB):
            bad.append(f'PA1 mode {e.cfg_at_first} when opened (want 11: AF push-pull, TIM2 CH2)')
        if mode == 3 or rearms:
            if 'mode' in self.extra and self.extra['mode'] == 1:
                bad.append(f'the slot plan armed on a marker port ({len(rearms)} re-arms)')
        if result['max_isr'] + 60 > 7200:
            bad.append(f"ISR {result['max_isr']} + 60 > 7,200 cycles")
        mode = e.rd(e.addr('uc') + e.O['uc_mode'], 1)
        if 'mode' in self.extra and mode != self.extra['mode']:
            bad.append(f"line mode {mode}, want {self.extra['mode']}")
        for key, check in self.extra.items():
            if key.startswith('check_'):
                msg = check(opened, self.line, passed)
                if msg:
                    bad.append(msg)
        return dict(name=self.name, ok=not bad, failures=bad, transitions=opened[:12], n_transitions=len(opened), rearms=len(rearms),
                    forwarded=sum(got), frames=len(got), max_isr=result['max_isr'], mode=mode,
                    downstream_frames=len(decoded))


def decode(out):
    """the next board's view: HIGH pulses grouped into frames by LOW > 50 us; '1' = HIGH > half a period"""
    frames, bits, i, n = [], [], 0, len(out)
    last_fall = None
    while True:
        r = out.find(1, i)
        if r < 0:
            break
        f = out.find(0, r)
        f = n if f < 0 else f
        if (last_fall is not None and r - last_fall > 50 * S or f - r > 10 * S) and bits:
            frames.append(bits)
            bits = []
        if f - r <= 10 * S:                              # (a longer HIGH is a release or a marker, no bit)
            bits.append(1 if f - r > t.PERIOD // 2 else 0)
        last_fall, i = f, f
    if bits:
        frames.append(bits)
    res = []
    for b in frames:
        if len(b) % 8:
            res.append(bytes([0xEE]) * (len(b) // 8 + 1))
            continue
        res.append(bytes(int(''.join(map(str, b[k:k + 8])), 2) for k in range(0, len(b), 8)))
    return res


def build(job):
    """the scenario of one job: (Line, expected forwarded flags per frame, extra checks)"""
    t.PERIOD, t.H0, t.H1 = 45, 11, 34
    L = Line()
    kind = job['kind']
    npx, effect = job.get('npx', 40), job.get('effect', 4)
    setup(L, npx, effect)
    n_before = len([s for s in L.spans if s[3] is not None])
    exp = [False] * n_before
    extra = {'mode': 2}
    gap = job.get('gap', 500)
    if kind == 'session':
        pre_ms = job.get('preamble_ms', 1000)
        k = 0
        t0 = len(L.line)
        while (len(L.line) - t0) < pre_ms * 1000 * S:
            L.frame(block(PING, 7), UPD, 'ping', gap)
            exp.append(k >= 2)
            k += 1
        d = foreign_descriptor()
        for half in (0, 1):
            L.frame(block(ANNOUNCE, 7, half, d[32 * half:32 * half + 32]), UPD, 'announce', gap)
            exp.append(True)
        L.low_to(len(L.line) / S + 60000)               # the entry wait (>= 50 ms LOW)
        for i in range(8):
            L.frame(block(DATA, 7, i, bytes(range(i, i + 32))), UPD, 'data', gap)
            exp.append(True)
        L.low_to(len(L.line) / S + 21000)               # a block pause
        for _ in range(2):
            L.frame(block(END, 7, 0, bytes(8)), UPD, 'end', gap)
            exp.append(True)
        L.frame(t.slot(bytes([0x07, 255, 0x50, 0, 1, 0x10, 0, 0, 0])), 1, 'first-ordinary', 9000, job.get('release', 150))
        exp.append(True)                                # the first ordinary frame passes whole (and its release)
        for i in range(3):
            L.frame(t.slot(bytes([0, 0, 0x60 + i])), 0, 'ordinary-after', gap)
            exp.append(False)
        if gap < 500:
            # Below the 500 us minimum gap a tick may find 350 us of LOW only a few us before the next frame; the live
            # check then refuses and the gate opens in a later gap. Never partway; from the open on, everything up to
            # the first ordinary frame passes, nothing after it.
            first = n_before + 2
            def check_late(o, L_, P_, first=first, n_before=n_before):
                got = [p for _, p in P_]
                k = next((i for i in range(n_before, len(got)) if got[i]), None)
                if k is None or k > first + 3:
                    return f'not opened by the 6th UPD frame (first forwarded #{k})'
                last = max(i for i, (nm, _) in enumerate(P_) if nm == 'first-ordinary')
                if not all(got[k:last + 1]) or any(got[last + 1:]):
                    return 'not forwarded from the open to the first ordinary frame only'
                return None
            extra['check_late'] = check_late
            exp = None
    elif kind in ('b349', 'b350', 'edge349', 'late'):
        L.low_to(len(L.line) / S + 9000)
        L.frame(block(PING, 7), UPD, 'ping1', 9000)
        exp.append(False)
        # PING 2 ends exactly on a word boundary; its LOW run starts there. A tick lands 787 (349.8 us) or 788 (350.2 us)
        # words into it; the decode is done by then (a light effect: about 260 us).
        k = 1
        while tick_words(k) * 16 < len(L.line) + 2000 * S:
            k += 1
        target = 787 if kind in ('b349', 'edge349') else 788
        end_word = tick_words(k) - target
        end = L.frame_ending_at_word(end_word, block(PING, 7), UPD, 'ping2')
        exp.append(False)
        tick_at = tick_words(k) * 16
        if kind == 'b349':                               # the next frame starts right after that tick: must stay closed
            L.frame_at(tick_at + S, block(PING, 7), UPD, 'ping3-blocked')
            exp.append(False)
            L.low_to(len(L.line) / S + 2000)
            L.frame(block(PING, 7), UPD, 'ping4', 2000)
            exp.append(True)
            extra['check_open'] = lambda o, L_, P_, w=end_word: (None if o and o[0]['open'] and o[0]['word'] > w + 787 + 400
                                                             else f'opened at the 787-word (349.8 us) tick: {o[:1]}')
        elif kind == 'b350':                             # opened at that tick (its ring word): the next frame passes whole
            L.frame_at(tick_at + 60 * S, block(PING, 7), UPD, 'ping3')
            L.low_to(len(L.line) / S + 1000)
            exp.append(True)
            extra['check_open'] = lambda o, L_, P_, w=end_word: (None if o and o[0]['open'] and o[0]['word'] == w + 788
                                                             else f'not opened at the 788-word (350.2 us) tick: {o[:1]}')
        elif kind == 'edge349':                          # an edge 349.8 us into the LOW (a 1 us pulse) restarts the count
            L.high_at(tick_at, tick_at + S, 'pulse')
            pulse_end = tick_at + S
            L.low_to(len(L.line) / S + 1000)
            L.frame(block(PING, 7), UPD, 'ping3', 1000)
            exp.append(True)
            extra['check_open'] = lambda o, L_, P_, p=pulse_end: (None if o and o[0]['open'] and o[0]['us'] >= p / S + 350
                                                              else f'the count did not restart at the edge: {o[:1]}')
        else:                                            # a late release: HIGH from +300 us to +1500 us after PING 2
            L.high_at(end + 300 * S, end + 1500 * S, 'late release')
            rel_end = end + 1500 * S
            L.low_to(len(L.line) / S + 1000)
            L.frame(block(PING, 7), UPD, 'ping3', 1000)
            exp.append(True)
            extra['check_open'] = lambda o, L_, P_, p=rel_end: (None if o and o[0]['open'] and o[0]['us'] >= p / S + 350
                                                            else f'opened before 350 us after the late HIGH: {o[:1]}')
        L.frame(t.slot(bytes([0, 0, 0x70])), 0, 'first-ordinary', 9000)
        exp.append(True)
        L.frame(t.slot(bytes([0, 0, 0x71])), 0, 'ordinary-after', 500)
        exp.append(False)
    elif kind == 'idle':
        for _ in range(3):
            L.frame(block(PING, 9), UPD, 'ping', 10000)
            exp.append(_ >= 2)
        last = L.spans[-1][2]
        L.low_to(last / S + 2150 * 1000)
        L.frame(t.slot(bytes([0, 0, 0x7A])), 0, 'ordinary-after-idle', 500)
        exp.append(False)
        extra['check_close'] = lambda o, L_, P_, e2=last: (
            None if len(o) == 2 and not o[1]['open'] and 2000000 <= o[1]['us'] - e2 / S <= 2100000
            else f'idle close: {o}')
    elif kind == 'marker':
        extra = {'mode': 1}
        L2 = Line()                                      # a marker port from power-up: AUTO decides STREAM
        L2.low_to(3000)
        exp = []
        for i in range(8):
            L2.low_to(len(L2.line) / S + 400)
            m = len(L2.line)
            L2.high_at(m, m + 20 * S, 'marker')
            L2.frame_at(m + 30 * S, block(PING, 7), UPD, 'marker-ping')
            exp.append(False)
            L2.low_to(len(L2.line) / S + 5000)
        L = L2
    if kind != 'marker':
        L.low_to(len(L.line) / S + 3000)
    return L, exp, extra


def run(args):
    elf, job = args
    L, exp, extra = build(job)
    sc = Scenario(job['name'], L, exp, extra)
    O = t.offsets(elf)
    Gate.O = O
    r = t.run_once(elf, O, 500, 0, 150, job.get('npx', 40), job.get('effect', 4), qualified=1, scenario=sc)
    return r


JOBS = [
    dict(name='Lava 600 px, 1000 ms PING preamble at 500 us gaps, entry pair, DATA, END, RW POLL', kind='session',
         npx=600, effect=31, gap=500, preamble_ms=1000),
    dict(name='Lava 600 px, 400 us gaps, late-valid release +150', kind='session', npx=600, effect=31, gap=400,
         preamble_ms=60),
    dict(name='light, 500 us gaps, first ordinary RW release at +250 us', kind='session', gap=500, preamble_ms=40,
         release=250),
    dict(name='349.8 us LOW at a tick: stays closed', kind='b349'),
    dict(name='350.2 us LOW at a tick: opens', kind='b350'),
    dict(name='an edge 349.8 us into the LOW restarts the count', kind='edge349'),
    dict(name='a late HIGH (+300 us) restarts the count', kind='late'),
    dict(name='2000 ms idle close before ordinary traffic', kind='idle'),
    dict(name='marker port: never opens', kind='marker'),
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf')
    p.add_argument('--json', type=Path)
    p.add_argument('--jobs', default='')
    p.add_argument('--workers', type=int, default=5)
    a = p.parse_args()
    jobs = [j for i, j in enumerate(JOBS) if not a.jobs or str(i) in a.jobs.split(',')]
    rows, fails = [], 0
    with ProcessPoolExecutor(max_workers=max(1, min(a.workers, len(jobs)))) as pool:
        for r in pool.map(run, [(a.elf, j) for j in jobs]):
            rows.append(r)
            fails += not r['ok']
            print(f"  {r['name']}: {'ok' if r['ok'] else 'FAIL'}; {r['forwarded']} of {r['frames']} frames forwarded, "
                  f"{r['downstream_frames']} decoded downstream, {r['n_transitions']} FWD_EN changes "
                  f"{[(x['us'], x['open']) for x in r['transitions'][:4]]}, ISR {r['max_isr']}+60/7200, mode {r['mode']}"
                  + ''.join('\n      ' + f for f in r['failures']), flush=True)
    print('ud forward: %s' % (f'FAIL, {fails} of {len(rows)} runs' if fails else f'ok, {len(rows)} runs'))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())

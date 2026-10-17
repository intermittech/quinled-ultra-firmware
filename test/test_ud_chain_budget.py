# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Chain frames back to back on the COMPILED BOARD=3 image, with the main loop PREEMPTED by the 20 kHz tick (the Ultra-Digital receiver alone on a
normal Ultra line, the hub at its default 300 us reset).

test_ud_chain_fw.py runs one whole main-loop pass between two ticks, however many cycles it takes. Here each tick gets 7,200 cycles (144 MHz / 20 kHz):
poll_isr first (its own cycles, the chain scanner's copy included), the main loop the rest, resuming where it was preempted (and sleeping at WFI until
the next tick). So a decode that needs 30 k cycles spans as many ticks as the chip would need, and a frame whose start finds the capture still held is
dropped as on the chip. (Method: review-r4/ud-chain/verify/t_chain_budget.py, ud-fx-verify.)

The line is the hub's normal timing (reset_us between frames, the ESPHome default 300 us; frames back to back while its queue has commands, the
preamble too): 10 DARK frames, CONFIG (V5, 40 px, 60 fps), PIXEL_FX, a burst of 20 NOP frames back to back, a POLL right behind the burst (reset), then
quiet refreshes (9 ms), a POLL after a quiet gap. The clock's rate is poked as measured (the board answers no POLL before it is: test_udchain.c covers
that). The line is shifted against the 50 us tick grid by `phase_us`, in several steps: where the tick falls inside the frame changes the decode's
latency by up to a tick.

Checked: every frame sent is classified (uc.frames == sent), no frame is counted as no Ultra frame (a header read over by the next capture would be) and
both POLLs are answered (DE rises 1,000 .. 1,080 us after the frame's last edge). Three groups:
  1. a light effect (40 px Rainbow), the RW frame's release at +150 us (the protocol's latest), at reset_us and again MARGIN_US shorter (the capture is free
     well before the classification: a frame that starts while it is held is dropped, and the count shows it), `phases` phases against the tick;
  2. the same line with the host letting go of the pair earlier: +2, +5, +20, +40, +60, +100 us after the last edge (the ESPHome hub used to end an RW frame
     with a 2 us LOW; the protocol's lower bound is ULTRA_RW_RELEASE_MIN_US = 60), the pair then floats HIGH (the GM3088's fail-safe): the frame ends at
     the release when the LOW before it was a gap;
  3. a heavy effect (Lava Lamp, 300 px: a render step of 32 pixels is ~20 k cycles) at reset_us with the release at +150 and +5 us, every phase: its main
     loop looks at the line only between two steps, the capture must still be freed before the next frame, and the release HIGH 150 us after an RW frame
     must not start a capture that overwrites the header before it is read.
The runs go in parallel (processes). The slowest classification is printed.

Usage: python test_ud_chain_budget.py <fw.elf> [reset_us=300] [phases=4]
Build the image: XW=0 LTO=0 EXTRA_CFLAGS=-DBOARD=3 OUT=build/ud-opt bash tools/build_target.sh  (or the -fno-inline API image)
"""
import concurrent.futures
import os
import struct
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
sys.path.insert(0, str(FW / "test"))
from mainloop import Emu2  # noqa: E402
from flash_stores import FlashEmu  # noqa: E402
from rv32emu import COST, RET_MAGIC  # noqa: E402
from ud_target import offsets  # noqa: E402

TICK_CYCLES = 7200
ISR_OVERHEAD = 60          # entry / exit, the mret
S_PER_US = 36
MARGIN_US = 100            # the same line with this much less reset must still be classified whole


class Board(FlashEmu, Emu2):
    systick = 0

    def rd(self, a, n):
        if a == 0xE000F008:
            self.systick = (self.systick + 40) % 7200
            return self.systick
        return super().rd(a, n)

    def run_budget(self, pc, stop, budget, x):
        """run from pc until `budget` cycles are used or the pc reaches `stop` (after >= 1 step); returns (pc, cycles, stopped)"""
        self.x = x
        c = 0
        n = 0
        while pc != RET_MAGIC:
            if pc in stop and n > 0:
                return pc, c, True
            if c >= budget:
                return pc, c, False
            n += 1
            h = self.rd(pc, 2)
            if h & 3 != 3:
                npc, cls = self._c(h, pc, x)
            else:
                npc, cls = self._i(self.rd(pc, 4), pc, x)
            c += COST[cls]
            x[0] = 0
            pc = npc
        return pc, c, True


def crc(data, width, poly, init):
    v = init
    m = (1 << width) - 1
    for b in data:
        v ^= b << (width - 8)
        for _ in range(8):
            v = ((v << 1) ^ (poly if v & (1 << (width - 1)) else 0)) & m
    return v


def slot(b, seal=True):
    b = bytearray(b) + bytearray(32 - len(b))
    if seal:
        b[30:32] = crc(b[:30], 16, 0x1021, 0xFFFF).to_bytes(2, "big")
    return bytes(b)


def header(host, flags):
    h = bytearray(b"\x5a\xd2" + bytes([0x20 | flags]) + struct.pack(">I", host & 0xFFFFFFFF))
    h.append(crc(h, 8, 7, 0))
    return bytes(h)


class Line:
    """the hub's line as 36 MHz samples"""

    def __init__(self, rel_us=150):
        self.line = bytearray()
        self.frame_ends = []
        self.rel_us = rel_us                                # the host lets go of the pair this long after an RW frame's last edge

    def low_to(self, us):
        n = int(us * S_PER_US) - len(self.line)
        if n > 0:
            self.line.extend(b"\0" * n)

    def frame(self, slot_bytes, rw=0, kind="frame", gap_us=300):
        line = self.line
        start = len(line)
        for _ in range(3):
            end = start + 319 * 45
            f = slot_bytes + header(0, rw)
            last = f[-1] & 1
            end_s = end + (25 if last else 12)
            f = slot_bytes + header(end_s // S_PER_US, rw)
            if (f[-1] & 1) == last:
                break
        for byte in f:
            for k in range(8):
                h = 25 if (byte >> (7 - k)) & 1 else 12
                line.extend(b"\1" * h + b"\0" * (45 - h))
        t_end = len(line) - (45 - (25 if f[-1] & 1 else 12))
        self.frame_ends.append((kind, t_end))
        if rw:                                              # the hub releases at +rel_us, the pair floats HIGH until it retakes after its window (one slot)
            line.extend(b"\0" * int(self.rel_us * S_PER_US))
            retake_us = 1000 + 460 + 40
            line.extend(b"\1" * (retake_us * S_PER_US - (len(line) - t_end)))
            line.extend(b"\0" * (200 * S_PER_US))
        else:
            line.extend(b"\0" * (gap_us * S_PER_US))

    def words_of(self, a, b):
        out = []
        for w in range(a, b):
            s = self.line[w * 16:(w + 1) * 16]
            if len(s) < 16:
                s = s + b"\0" * (16 - len(s))
            v = 0
            for x in s:
                v = (v << 1) | x
            out.append(v)
        return out


def run_once(elf, O, reset_us, phase_us, rel_us=150, npx=40, effect=4, chip=0):
    L = Line(rel_us)
    seq = [1]

    def nseq():
        seq[0] += 1
        return seq[0]
    L.low_to(3000 + phase_us)
    dark = slot(bytes(30), seal=False)
    for _ in range(10):
        L.frame(dark, 0, "dark", reset_us)
    cfg = bytes([0x05, 0xF0, nseq()]) + bytes([129, 0, chip, 131, npx >> 8, npx & 255, 132, 0, 60])
    L.frame(slot(cfg), 0, "config", reset_us)
    fx = bytes([0x0B, 0xF0, nseq(), effect, 0, 0, 0, 0, 128, 128, 0xFF, 255, 0, 0] + [0] * 6 + [255, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    L.frame(slot(fx), 0, "fx", 9000)
    burst0 = len(L.frame_ends)
    for _ in range(20):                                     # the hub's queue: frames back to back
        L.frame(slot(bytes([0x00, 0xF0, nseq()])), 0, "burst", reset_us)
    burst1 = len(L.frame_ends)
    poll = bytes([0x07, 0xF0, nseq(), 0, 1, 0x81, 0, 3])
    L.frame(slot(poll), 1, "poll_b2b")                      # right behind the burst's last frame
    L.low_to(len(L.line) / S_PER_US + 9000)
    for _ in range(4):
        L.frame(slot(bytes([0x00, 0xF0, nseq()])), 0, "nop", 9000)
    poll = bytes([0x07, 0xF0, nseq(), 0, 1, 0x81, 0, 3])
    L.frame(slot(poll), 1, "poll_quiet")
    L.low_to(len(L.line) / S_PER_US + 9000)
    for _ in range(3):
        L.frame(slot(bytes([0x00, 0xF0, nseq()])), 0, "nop", 9000)
    T_US = len(L.line) // S_PER_US + 2000
    L.low_to(T_US)
    sent = len(L.frame_ends)

    e = Board(elf)
    e.poke(0x08010000, b"\xff" * 0xC000)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 0x08, 0x40004400: 0xC0, 0x40010C08: 1 << 10, 0x40010808: 1 << 3})
    base, size = e.sym("main")
    wfi = [q for q in range(base, base + size, 2) if e.rd(q, 4) == 0x10500073]
    stop = set(wfi)
    pc, _ = e.run_to(e.start("main"), stop, max_steps=3000000)
    main_x = list(e.x)
    ux = e.addr("ux")
    e.wr(ux + O["ux_r_have"], 1, 1)                         # the clock's rate is measured (nominal: ux_init's)
    ring = e.addr("dl_ring")
    uc = e.addr("uc")
    edges, prev = 0, 0
    de, de_end = 0, None
    de_rises = []
    frames_at = []                                          # (word, uc.frames) after each tick
    wpos = 0
    ticks = 0
    total_words = T_US * S_PER_US // 16
    busy_ticks = longest_busy = cur_busy = 0
    while wpos < total_words:
        step = 113 if ticks % 2 else 112
        ticks += 1
        new = L.words_of(wpos, wpos + step)
        for i, v in enumerate(new):
            w = wpos + i
            rxv = 0 if de else v
            e.wr(ring + 2 * (w % 512), 2, rxv)
            for j in range(16):
                b = (rxv >> (15 - j)) & 1
                edges += prev and not b
                prev = b
            if de and de_end is not None and w >= de_end:
                de = 0
                de_end = None
                saved = list(e.x)
                x = e.start("dl_tx_done")
                e.x[2] = (main_x[2] - 256) & ~15
                e.run_to(x, set(), max_steps=100000)
                e.x = saved
        wpos += step
        rx = wpos % 512
        e.mmio[0x40020020] = 512 - rx if rx else 512
        e.mmio[0x40020034] = 512 - ((rx + 2) % 512)
        e.mmio[0x40000824] = edges & 0xFFFF
        e.writes.clear()
        x = e.start("poll_isr")
        e.x[2] = (main_x[2] - 256) & ~15                    # the ISR's frame below the preempted main loop's (as on the chip)
        e.run_to(x, set(), max_steps=2000000)
        isr_c = e.cycles()
        tick_writes = list(e.writes)
        e.writes.clear()
        budget = TICK_CYCLES - isr_c - ISR_OVERHEAD
        pc, used, stopped = e.run_budget(pc, stop, budget, main_x)
        main_x = list(e.x)
        if not stopped:
            cur_busy += 1
            busy_ticks += 1
            longest_busy = max(longest_busy, cur_busy)
        else:
            cur_busy = 0
        for a, v in tick_writes + list(e.writes):
            if a == 0x40000824 and v == 0:
                edges = 0
            elif a == 0x40010C10 and v & (1 << 14):
                de = 1
                de_end = wpos + 760
                de_rises.append(wpos)
        frames_at.append((wpos, struct.unpack_from("<H", e.peek(uc + O["uc_frames"], 2))[0]))
    frames = frames_at[-1][1]

    classified_at = []
    last = 0
    for w, f in frames_at:                                  # the tick after which uc.frames went up
        while last < f:
            classified_at.append(w * 16)
            last += 1
    ends = [s for _, s in L.frame_ends]
    delay = [(c - s) / S_PER_US for c, s in zip(classified_at, ends)]
    burst_delay = delay[burst0:burst1] if len(delay) >= burst1 else []
    answers = {}
    for name in ("poll_b2b", "poll_quiet"):
        t = [s for k, s in L.frame_ends if k == name][0]
        r = [w for w in de_rises if t < w * 16 < t + 2000 * S_PER_US]
        answers[name] = (r[0] * 16 - t) / S_PER_US if r else None
    worst = max(burst_delay) if burst_delay else 9999.0
    nonultra = struct.unpack_from("<H", e.peek(uc + O["uc_nonultra"], 2))[0]
    return dict(sent=sent, frames=frames, worst=worst, answers=answers, longest_busy=longest_busy, busy_ticks=busy_ticks,
                burst_ok=len(burst_delay) == burst1 - burst0, nonultra=nonultra, delays=[round(d) for d in delay], ends=[round(t / S_PER_US) for t in ends])


def job(args):
    """one run in a process of its own: (label, elf, reset_us, phase_us, rel_us, npx, effect) -> (label, result)"""
    label, elf, *case = args
    return label, run_once(elf, offsets(elf), *case)


def main():
    elf = sys.argv[1]
    reset_us = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    nph = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    jobs = []
    for rs in (reset_us, reset_us - MARGIN_US):                         # 1. a light effect, the release at +150 us
        for ph in range(nph):
            jobs.append((f"light, release +150, reset {rs}, phase {ph * 50 // nph}", elf, rs, ph * 50 // nph, 150, 40, 4))
    for rel in (2, 5, 20, 40, 60, 100):                                 # 2. the host lets go earlier
        for ph in (0, 37):
            jobs.append((f"light, release +{rel}, reset {reset_us}, phase {ph}", elf, reset_us, ph, rel, 40, 4))
    for ph in range(0, 50, 7):                                          # 3. a heavy effect
        jobs.append((f"Lava Lamp 300 px, release +150, reset {reset_us}, phase {ph}", elf, reset_us, ph, 150, 300, 31))
    for ph in (0, 37):
        jobs.append((f"Lava Lamp 300 px, release +5, reset {reset_us}, phase {ph}", elf, reset_us, ph, 5, 300, 31))
    for ph in (0, 14, 37):                                              # 4. Pacifica, which never leaves the main loop idle (4-pixel steps, udfx.h UX_CHUNK_HEAVY)
        jobs.append((f"Pacifica 300 px, release +150, reset {reset_us}, phase {ph}", elf, reset_us, ph, 150, 300, 18))
    for chip, np in ((0, 600), (3, 500), (5, 400), (7, 600)):
        for ph in range(nph):
            phase = ph * 50 // nph
            jobs.append((f"Pacifica chip {chip}, {np} px, reset {reset_us}, phase {phase}",
                         elf, reset_us, phase, 150, np, 18, chip))
    print(f"{len(jobs)} runs of the line (a light effect and a heavy one, the host's release at +2 .. +150 us, phases against the 50 us tick); image {elf}")
    fails = 0
    workers = max(1, min(len(jobs), os.cpu_count() or 1, 8))
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as pool:
        for label, r in pool.map(job, jobs):
            ok = (r["frames"] == r["sent"] and r["burst_ok"] and r["nonultra"] == 0 and
                  all(v is not None and 1000.0 <= v <= 1080.0 for v in r["answers"].values()))
            a = ", ".join(f"{k} {'NOT answered' if v is None else '%+.0f us' % (v - 1000.0)}" for k, v in r["answers"].items())
            print(f"  {label}: {r['frames']} of {r['sent']} frames classified, {r['nonultra']} no-Ultra (the slowest {r['worst']:.0f} us after its last edge); "
                  f"answers from their slots: {a}  {'ok' if ok else 'FAIL'}")
            fails += not ok
    print("budget: %s" % ("FAIL, %d runs" % fails if fails else "ok"))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())

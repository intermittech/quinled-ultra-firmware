# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""BLANK_N and the strip after a poll answer while a local effect runs, on the COMPILED BOARD=3 firmware (tools/rv32emu.py).

(Adapted from ud-fx-verify's harness/t_blank_target.py, which found the bug: the answer's SEND pulled BLANK_N LOW with DE,
dline never asks for its release in local mode, and the effect froze at the first poll answer.)

The real main() boots to its WFI; then every 20 kHz tick: RX DMA's new words go into dl_ring, DMA1 CH2 / CH3 CNTR and TIM4 CNT
are set, poll_isr() runs (tick_now, ms_now, dl_tick -> dline_tick, dl_pins, SEND -> rc_hw_dir / rc_hw_answer_local, udfx_tick),
then one pass of the real main loop (dl_main -> ud_decode -> ud_frame -> udfx_config / udfx_pixel_fx; udfx_render;
udfx_service). The hub's line: LOW, a CONFIG marker frame (WS2812B V5, 40 px, 50 fps), a PIXEL_FX marker frame (Rainbow, the
whole strip), then an RW POLL (page 0x81) with the hub's release. USART3 TC is emulated by calling dl_tx_done() after the
burst. The strip: BLANK_N (PA1, from the MMIO writes) AND (MUX_SEL PA10 ? the TX DMA's replay : the line). Frames are counted
on it: the effect must go on updating after the answer, BLANK_N must end HIGH, and every frame must be a whole 40-pixel one.

Build the image (the functions as written, as test_ud_pages.py):
  XW=0 LTO=0 EXTRA_CFLAGS="-DBOARD=3 -fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/ud-api bash tools/build_target.sh
Run: python test/test_ud_local_poll.py build/ud-api/fw.elf [microseconds]   (tools/build_release.sh runs it for UD=1)
"""
import struct
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
from mainloop import Emu2  # noqa: E402
from flash_stores import FlashEmu  # noqa: E402

checks = fails = 0


def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print("FAIL:", what)


class Board(FlashEmu, Emu2):
    systick = 0

    def rd(self, a, n):
        if a == 0xE000F008:              # SysTick CNT (POLL_CNT: rc_hw_send's preamble wait) advances as it is read
            self.systick = (self.systick + 40) % 7200
            return self.systick
        return super().rd(a, n)


def crc(data, width, poly, init):
    v = init
    m = (1 << width) - 1
    for b in data:
        v ^= b << (width - 8)
        for _ in range(8):
            v = ((v << 1) ^ (poly if v & (1 << (width - 1)) else 0)) & m
    return v


def slot(b):
    b = bytearray(b) + bytearray(32 - len(b))
    b[30:32] = crc(b[:30], 16, 0x1021, 0xFFFF).to_bytes(2, "big")
    return bytes(b)


def header(host, flags):
    h = bytearray(b"\x5a\xd2" + bytes([0x20 | flags]) + struct.pack(">I", host & 0xFFFFFFFF))
    h.append(crc(h, 8, 7, 0))
    return bytes(h)


S_PER_US = 36  # the receiver's samples per us (HSI exact here)
line = bytearray()  # one byte per sample


def low_to(us):
    n = int(us * S_PER_US) - len(line)
    if n > 0:
        line.extend(b"\0" * n)


def marker(slot_bytes, rw, retake_us=1900):
    line.extend(b"\1" * (20 * S_PER_US))
    line.extend(b"\0" * (10 * S_PER_US))
    start = len(line)
    # the last edge: 319 bits of 45 samples, then the last bit's HIGH; TIME = host us there (host = receiver us here)
    for _ in range(3):
        end = start + 319 * 45
        f = slot_bytes + header(0, 1 if rw else 0)
        last = f[-1] & 1
        end_s = end + (27 if last else 11)
        f = slot_bytes + header(end_s // S_PER_US, 1 if rw else 0)
        if (f[-1] & 1) == last:
            break
    for byte in f:
        for k in range(8):
            h = 27 if (byte >> (7 - k)) & 1 else 11
            line.extend(b"\1" * h + b"\0" * (45 - h))
    t_end = len(line)
    if rw:
        line.extend(b"\0" * (150 * S_PER_US))
        line.extend(b"\1" * (retake_us * S_PER_US - (len(line) - t_end)))
    line.extend(b"\0" * (320 * S_PER_US))


def words_of(a, b):
    out = []
    for w in range(a, b):
        s = line[w * 16:(w + 1) * 16]
        if len(s) < 16:
            s = s + b"\0" * (16 - len(s))
        v = 0
        for x in s:
            v = (v << 1) | x
        out.append(v)
    return out


def main():
    elf = sys.argv[1]
    seq = [1]

    def nseq():
        seq[0] += 1
        return seq[0]
    # the hub's line
    low_to(3000)
    cfg = bytes([0x05, 0xF0, nseq()]) + bytes([129, 0, 0, 131, 0, 40, 132, 0, 50])  # CONFIG: chip 0 (V5), 40 px, 50 fps
    marker(slot(cfg), 0)
    low_to(5000)
    # PIXEL_FX: type 0x0B, LIGHT 15, effect 4 (Rainbow), start 0, speed 128, intensity 128, palette 0xFF, c1 red, bri 255
    fx = bytes([0x0B, 0xF0, nseq(), 4, 0, 0, 0, 0, 128, 128, 0xFF, 255, 0, 0] + [0] * 6 + [255, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    marker(slot(fx), 0)
    low_to(45000)
    poll = bytes([0x07, 0xF0, nseq(), 0, 1, 0x81, 0, 0])  # POLL: index 0 of 1, page 0x81, rate 0, argument 0 (part 0)
    marker(slot(poll), 1)
    T_US = int(sys.argv[2]) if len(sys.argv) > 2 else 130000
    low_to(T_US)

    e = Board(elf)
    e.poke(0x08010000, b"\xff" * 0xC000)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 0x08, 0x40004400: 0xC0, 0x40010C08: 1 << 10, 0x40010808: 1 << 3})
    base, size = e.sym("main")
    wfi = [q for q in range(base, base + size, 2) if e.rd(q, 4) == 0x10500073]
    stop = set(wfi)
    pc, n = e.run_to(e.start("main"), stop, max_steps=3000000)
    check(e.rd(e.addr("dl_on"), 1) == 1, "the delay line did not start (dl_on)")
    main_x = list(e.x)
    ring = e.addr("dl_ring")
    bn_low, mux, de, de_end = 1, 0, 0, None   # (hw_init: BLANK_N LOW, the mux direct)
    edges, prev = 0, 0
    strip = bytearray()
    events = []
    wpos = 0
    ticks = 0
    isr_cycles = []
    total_words = T_US * S_PER_US // 16
    while wpos < total_words:
        step = 113 if ticks % 2 else 112
        ticks += 1
        new = words_of(wpos, wpos + step)
        for i, v in enumerate(new):
            w = wpos + i
            # the strip now: BLANK_N AND (delayed: the TX DMA's word = ring[(w + 2) % 512] before RX writes w; direct: the line)
            tx = e.rd(ring + 2 * ((w + 2) % 512), 2)
            rxv = 0 if de else v
            s = 0 if bn_low else (tx if mux else rxv)
            strip.extend(((s >> (15 - j)) & 1) for j in range(16))
            e.wr(ring + 2 * (w % 512), 2, rxv)
            for j in range(16):
                b = (rxv >> (15 - j)) & 1
                edges += prev and not b
                prev = b
            if de and de_end is not None and w >= de_end:   # USART3 TC: rc_release + dl_tx_done
                de = 0
                de_end = None
                if "dl_tx_done" in e.syms:       # (an LTO image has no such symbol: only the tick's cycles are meaningful there)
                    x = e.start("dl_tx_done")
                    e.run_to(x, set(), max_steps=100000)
                    e.x = main_x
        wpos += step
        rx = wpos % 512
        e.mmio[0x40020020] = 512 - rx if rx else 512
        e.mmio[0x40020034] = 512 - ((rx + 2) % 512)
        e.mmio[0x40000824] = edges & 0xFFFF
        e.writes.clear()
        x = e.start("poll_isr")
        e.run_to(x, set(), max_steps=2000000)
        isr_cycles.append(e.cycles())
        e.x = main_x
        pc, _ = e.run_to(pc, stop, max_steps=5000000)
        main_x = list(e.x)
        for a, v in e.writes:
            if a == 0x40010810:          # GPIOA BSHR
                if v & (1 << 1):
                    bn_low = 0; events.append((wpos, "BLANK_N HIGH (BSHR)"))
                if v & (1 << 17):
                    bn_low = 1; events.append((wpos, "BLANK_N LOW (BSHR)"))
                if v & (1 << 10):
                    mux = 1; events.append((wpos, "MUX delayed"))
                if v & (1 << 26):
                    mux = 0; events.append((wpos, "MUX direct"))
            elif a == 0x40010814:        # GPIOA BCR
                if v & (1 << 1):
                    bn_low = 1; events.append((wpos, "BLANK_N LOW (BCR: rc_hw_dir ANSWER)"))
                if v & (1 << 10):
                    mux = 0; events.append((wpos, "MUX direct (BCR)"))
            elif a == 0x40010C10 and v & (1 << 14):   # GPIOB BSHR: DE high
                de = 1
                de_end = wpos + 1575
                events.append((wpos, "DE high (the answer)"))
            elif a == 0x40010C14 and v & (1 << 14):
                de = 0
    for w, ev in events:
        print(f"  {w / 2250:7.2f} ms  {ev}")
    # frames on the strip: split at LOW >= 280 us, count frames of 40 x 24 bits
    frames = []
    i, n_s = 0, len(strip)
    low = 0
    cur = None
    while i < n_s:
        if strip[i]:
            if cur is None or low >= 280 * S_PER_US:
                cur = [i, 0]
                frames.append(cur)
            h = 0
            while i < n_s and strip[i]:
                h += 1; i += 1
            cur[1] += 1
            low = 0
        else:
            low += 1; i += 1
    local = [f for f in frames if f[1] == 40 * 24]
    print(f"strip: {len(frames)} frames, {len(local)} of 960 bits (40 px RGB); their starts (ms): "
          f"{[round(f[0] / 36000, 1) for f in local]}")
    after = [f for f in local if f[0] / 36000 > 46.0]
    expect = int((T_US / 1000 - 46) / 20)
    print(f"local frames after the poll (46 .. {T_US / 1000:.0f} ms, 50 fps expected ~{expect}): {len(after)}")
    check(any("DE high" in ev for _, ev in events), "the poll was not answered (DE never rose)")
    check(len(local) >= 3, f"only {len(local)} local frames on the strip")
    check(len(after) >= expect - 2, f"the effect stopped after the answer: {len(after)} frames of ~{expect}")
    check(len(frames) == len(local), f"{len(frames) - len(local)} frames on the strip that are not whole local ones")
    phases = [(f[0] / 36000.0) % 20.0 for f in local]       # (50 fps: the host grid is 20 ms; the first frames sit on it too)
    check(phases and max(phases) - min(phases) < 0.3, f"the local frames are off the 20 ms grid: phases {[round(p, 1) for p in phases]}")
    check(not bn_low, "BLANK_N ended LOW: the strip line is held, the effect frozen")
    check(not any("BLANK_N LOW" in ev for _, ev in events if _ > 40 * 2250), "BLANK_N was pulled LOW during the local effect")
    srt = sorted(isr_cycles)
    print(f"the 20 kHz tick on the compiled image (poll_isr, rv32emu cycles): median {srt[len(srt) // 2]}, 99th percentile {srt[len(srt) * 99 // 100]}, worst {srt[-1]} "
          f"(a tick is 7,200 cycles); mean {sum(isr_cycles) / len(isr_cycles):.0f} = {sum(isr_cycles) / len(isr_cycles) / 72:.1f} % of the CPU")
    print(f"ud local poll: {checks} checks, {fails} failed")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())

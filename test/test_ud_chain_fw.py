# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""CHAIN line mode on the COMPILED BOARD=3 firmware (tools/rv32emu.py): a normal Ultra line, AUTO (the factory default), a local effect, a POLL.

(Adapted from test_ud_local_poll.py's harness.) The real main() boots to its WFI; then every 20 kHz tick: RX DMA's new words go
into dl_ring, DMA1 CH2 / CH3 CNTR and TIM4 CNT are set (TIM4 = the falling edges, zeroed by the firmware's own DLINE_A_EDGES_ZERO
write: the emulated register takes it), poll_isr() runs (dl_tick -> dline_tick: the chain scanner, dl_pins, SEND, udfx_tick), then
one pass of the real main loop (dl_main -> ud_chain_decode -> udc_classify -> ws_decode on the target, udfx_*, udc_*).
The hub's line: LOW, the preamble (6 DARK frames), a CONFIG slot (WS2812B V5, 40 px, 50 fps), a PIXEL_FX slot (Rainbow, the whole
strip), NOPs, then an RW POLL (page 0x81 part 3, index 0 of 1) with the hub's release and retake. There are no markers on the line.
The strip: BLANK_N (PA1, from the MMIO writes) AND (MUX_SEL PA10 ? the TX DMA's replay : the line).

Checks: AUTO decided CHAIN (uc.mode), BLANK_N held until then and the strip LOW until the first effect frame, every strip frame is a
whole 40-pixel local frame (nothing of the Ultra line, the release or the answer ever shows), the answer starts inside its Ultra
window slot (t0 + 1000 us .. + 80 us from the frame's last falling edge), the answer's bytes are page 0x81 part 3 with mode CHAIN,
the effect goes on updating after the answer, BLANK_N ends HIGH.

Build the image (the functions as written, as test_ud_pages.py):
  XW=0 LTO=0 EXTRA_CFLAGS="-DBOARD=3 -fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/ud-api bash tools/build_target.sh
Run: python test/test_ud_chain_fw.py build/ud-api/fw.elf [microseconds]   (tools/build_release.sh runs it for UD=1)
"""
import collections
import os
import struct
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
sys.path.insert(0, str(FW / "test"))
from mainloop import Emu2  # noqa: E402
from flash_stores import FlashEmu  # noqa: E402
from ud_target import offsets  # noqa: E402

checks = fails = 0
PROFILE = bool(os.environ.get("UDCHAIN_PROFILE"))


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


def slot(b, seal=True):
    b = bytearray(b) + bytearray(32 - len(b))
    if seal:
        b[30:32] = crc(b[:30], 16, 0x1021, 0xFFFF).to_bytes(2, "big")
    return bytes(b)


def header(host, flags):
    h = bytearray(b"\x5a\xd2" + bytes([0x20 | flags]) + struct.pack(">I", host & 0xFFFFFFFF))
    h.append(crc(h, 8, 7, 0))
    return bytes(h)


S_PER_US = 36  # the receiver's samples per us (HSI exact here)
line = bytearray()  # one byte per sample
frame_ends = []     # (kind, last falling edge sample) of every hub frame


def low_to(us):
    n = int(us * S_PER_US) - len(line)
    if n > 0:
        line.extend(b"\0" * n)


def chain_frame(slot_bytes, rw=0, kind="frame", gap_us=380):
    """one Ultra frame: the slot + the header trailer (TIME = the host us at the last edge), the pulses at the WS2915 timing"""
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
    frame_ends.append((kind, t_end))
    if rw:
        line.extend(b"\0" * (150 * S_PER_US))
        retake_us = 1000 + 460 + 120
        line.extend(b"\1" * (retake_us * S_PER_US - (len(line) - t_end)))
    line.extend(b"\0" * (gap_us * S_PER_US))


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
    dark = slot(bytes(30), seal=False)                      # 256 zero bits: never a valid slot
    for _ in range(6):                                      # the start-up preamble of a one-board line
        chain_frame(dark, 0, "dark")
    cfg = bytes([0x05, 0xF0, nseq()]) + bytes([129, 0, 0, 131, 0, 40, 132, 0, 50])  # CONFIG: chip 0 (V5), 40 px, 50 fps
    chain_frame(slot(cfg), 0, "config")
    # PIXEL_FX: type 0x0B, LIGHT 15, effect 4 (Rainbow), start 0, speed 128, intensity 128, palette 0xFF, c1 red, bri 255
    fx = bytes([0x0B, 0xF0, nseq(), 4, 0, 0, 0, 0, 128, 128, 0xFF, 255, 0, 0] + [0] * 6 + [255, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    chain_frame(slot(fx), 0, "fx")
    nop = bytes([0x00, 0xF0, nseq()])
    while len(line) < 55000 * S_PER_US:
        chain_frame(slot(nop), 0, "nop", gap_us=9000)
    poll = bytes([0x07, 0xF0, nseq(), 0, 1, 0x81, 0, 3])   # POLL: index 0 of 1, page 0x81, rate 0, argument 3 (the line)
    chain_frame(slot(poll), 1, "poll")
    poll_end = frame_ends[-1][1]
    low_to(len(line) / S_PER_US + 600)
    while len(line) < 140000 * S_PER_US:
        chain_frame(slot(nop), 0, "nop", gap_us=9000)
    T_US = int(sys.argv[2]) if len(sys.argv) > 2 else 150000
    low_to(T_US)

    e = Board(elf)
    e.poke(0x08010000, b"\xff" * 0xC000)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 0x08, 0x40004400: 0xC0, 0x40010C08: 1 << 10, 0x40010808: 1 << 3})
    base, size = e.sym("main")
    wfi = [q for q in range(base, base + size, 2) if e.rd(q, 4) == 0x10500073]
    stop = set(wfi)
    pc, n = e.run_to(e.start("main"), stop, max_steps=3000000)
    check(e.rd(e.addr("dl_on"), 1) == 1, "the delay line did not start (dl_on)")
    e.wr(e.addr("ux") + offsets(elf)["ux_r_have"], 1, 1)  # the clock's rate is measured (it needs two anchors a second apart; an unmeasured one answers no POLL)
    main_x = list(e.x)
    ring = e.addr("dl_ring")
    bn_low, mux, de, de_end = 1, 0, 0, None   # (hw_init: BLANK_N LOW, the mux direct)
    edges, prev = 0, 0
    strip = bytearray()
    events = []
    wpos = 0
    ticks = 0
    isr_cycles = []
    worst_tick = [0, None]
    total_words = T_US * S_PER_US // 16
    de_sample = None
    uc_sym = e.addr("uc") if "uc" in e.syms else None
    while wpos < total_words:
        step = 113 if ticks % 2 else 112
        ticks += 1
        new = words_of(wpos, wpos + step)
        for i, v in enumerate(new):
            w = wpos + i
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
        hist = collections.Counter() if PROFILE else None
        e.run_to(x, set(), max_steps=2000000, hist=hist)
        isr_cycles.append(e.cycles())
        if PROFILE and e.cycles() > worst_tick[0]:
            worst_tick[0] = e.cycles()
            worst_tick[1] = hist
        e.x = main_x
        pc, _ = e.run_to(pc, stop, max_steps=5000000)
        main_x = list(e.x)
        for a, v in e.writes:
            if a == 0x40000824 and v == 0:   # TIM4 CNT: dline's DLINE_A_EDGES_ZERO (dl_hw_edges_zero)
                edges = 0
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
                de_sample = wpos * 16
                events.append((wpos, "DE high (the answer)"))
            elif a == 0x40010C14 and v & (1 << 14):
                de = 0
    for w, ev in events:
        print(f"  {w / 2250:7.2f} ms  {ev}")
    if PROFILE and worst_tick[1]:
        by = collections.Counter()
        names = sorted((a, n) for n, (a, sz) in e.syms.items() if sz)
        for pc, c in worst_tick[1].items():
            best = None
            for a, n in names:
                if a <= pc:
                    best = n
                else:
                    break
            by[best] += c
        print("the worst tick,", worst_tick[0], "cycles, by function:", by.most_common(8))
        print("hottest pcs:", " ".join("%x:%d" % (pc, c) for pc, c in worst_tick[1].most_common(24)))
    if uc_sym is None:                   # an LTO image: report the tick's cost only
        srt = sorted(isr_cycles)
        print(f"the 20 kHz tick (poll_isr, rv32emu cycles): median {srt[len(srt) // 2]}, 99th percentile {srt[len(srt) * 99 // 100]}, worst {srt[-1]}; "
              f"mean {sum(isr_cycles) / len(isr_cycles):.0f} = {sum(isr_cycles) / len(isr_cycles) / 72:.1f} % of the CPU")
        return 0
    uc_mode = e.rd(uc_sym, 1)
    check(uc_mode == 2, f"AUTO did not decide CHAIN: uc.mode {uc_mode}")
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
          f"{[round(f[0] / 36000, 1) for f in local][:12]} ...")
    first_fx = [i for i, (k, _) in enumerate(frame_ends) if k == "fx"][0]
    second = frame_ends[1][1]
    bn_high_ev = [w for w, ev in events if ev.startswith("BLANK_N HIGH")]
    check(bool(bn_high_ev) and bn_high_ev[0] * 16 > second, "BLANK_N was released before the second Ultra frame ended (AUTO deciding)")
    check(bool(bn_high_ev) and bn_high_ev[0] * 16 < second + 3000 * S_PER_US, "BLANK_N released late")
    check(len(frames) == len(local), f"{len(frames) - len(local)} strip frames that are not whole local ones (a leak of the line, the release or the answer)")
    check(len(local) >= 3, f"only {len(local)} local frames")
    if local:
        check(local[0][0] > frame_ends[first_fx][1], "a strip frame before the PIXEL_FX frame ended")
    after = [f for f in local if f[0] > poll_end + 3000 * S_PER_US]
    check(len(after) >= 3, f"the effect stopped after the answer: {len(after)} frames")
    check(not bn_low, "BLANK_N ended LOW")
    # the answer
    check(de_sample is not None, "the POLL was not answered (DE never rose)")
    if de_sample is not None:
        dt = (de_sample - poll_end) / S_PER_US
        print(f"  the answer's DE rise {dt:.1f} us after the POLL frame's last falling edge (slot 0: 1000 .. 1080 us)")
        check(995 <= dt <= 1085, f"the answer starts {dt:.1f} us after t0")
    ans = e.peek(e.addr("ud_ans"), 32)
    check(ans[0] == 0xA5, "the answer's SYNC")
    check(ans[1] == 0x81 and ans[2] >> 4 == 0, f"the answer is page 0x{ans[1]:02X}, index {ans[2] >> 4}")
    data = ans[6:30]
    check(data[1] == 2 and data[0] == 0, f"page 0x81 part 3: config {data[0]}, mode {data[1]} (CHAIN)")
    check(int.from_bytes(data[4:6], "big") >= 10, f"Ultra frames counted: {int.from_bytes(data[4:6], 'big')}")
    check(int.from_bytes(data[10:12], "big") == 1, f"windows: {int.from_bytes(data[10:12], 'big')}")
    check(data[2] & 0x02 == 0, "flags: non-Ultra frames seen on a clean Ultra line")
    srt = sorted(isr_cycles)
    print(f"the 20 kHz tick on the compiled image (poll_isr, rv32emu cycles): median {srt[len(srt) // 2]}, 99th percentile {srt[len(srt) * 99 // 100]}, worst {srt[-1]} "
          f"(a tick is 7,200 cycles); mean {sum(isr_cycles) / len(isr_cycles):.0f} = {sum(isr_cycles) / len(isr_cycles) / 72:.1f} % of the CPU")
    print(f"ud chain fw: {checks} checks, {fails} failed")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())

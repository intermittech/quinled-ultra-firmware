# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run the compiled firmware functions in tools/rv32emu.py and report executed instructions / estimated cycles.

    python tools/measure.py            (after tools/build_target.sh and its measurement build, see below)
    FW_ELF=<path>/fw.elf python tools/measure.py   (another build, e.g. OUT=... EXTRA_CFLAGS=-DISENSE_AMP_GAIN=50)

The build uses LTO: most measured functions are inlined into their only caller (core/lto.h). The ISR rows run
in FW_ELF itself; the rest in its measurement build FW_ELF_MEASURE (default: <FW_ELF's folder>-measure/fw.elf), the
same build with MEASURE=1, where those functions stay out of line:
    MEASURE=1 OUT=build/target-measure bash tools/build_target.sh
An LTO=0 build has every function: then FW_ELF alone.
"""
import os
import random
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rv32emu import Emu  # noqa: E402

FW = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ELF = os.environ.get("FW_ELF") or os.path.join(FW, "build", "target", "fw.elf")
MELF = os.environ.get("FW_ELF_MEASURE") or os.path.join(os.path.dirname(ELF) + "-measure", "fw.elf")
F_CPU = 144e6
FS = 36e6
rng = random.Random(1)


def frame_samples(nbits_total, own, hint_us=60.0, t0h=(280, 320), t1h=(680, 820), per=(1240, 1260)):
    """16-bit words, MSB first: 60 us idle, then the frame (own bits first), sampled at 36 MHz."""
    bits = [rng.getrandbits(1) for _ in range(nbits_total)]
    edges = []
    t = hint_us * 1e3
    for b in bits:
        h = rng.uniform(*(t1h if b else t0h))
        edges.append((t, t + h))
        t += rng.uniform(*per)
    end = edges[own - 1][1] + 60e3                    # copy ends ~60 us after our last bit
    ts = 1e9 / FS
    n = int(end / ts) // 16 * 16
    tt = np.arange(n) * ts
    lvl = np.zeros(n, dtype=np.uint8)
    for r, f in edges:
        if r > end:
            break
        lvl[(tt >= r) & (tt < f)] = 1
    words = np.packbits(lvl.reshape(-1, 16), axis=1, bitorder="big")       # 2 bytes per word, MSB first
    w16 = (words[:, 0].astype(np.uint16) << 8) | words[:, 1]
    return bits, w16


def pack(bits):
    out = bytearray((len(bits) + 7) // 8)
    for i, b in enumerate(bits):
        if b:
            out[i >> 3] |= 0x80 >> (i & 7)
    return bytes(out)


# ---------------------------------------------------------------------------------------------- Ultra
def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def crc8(d):
    c = 0
    for b in d:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def u_header(t_us):
    h = bytearray(struct.pack(">HBI", 0x5AD2, 0x20, t_us))           # VERSION 2 (trailer), flags 0
    return bytes(h + bytes([crc8(h)]))


def u_slot(typ, light, flags, seq, payload):
    s = bytearray(32)
    s[0], s[1], s[2] = typ, (light << 4) | flags, seq
    s[3:3 + len(payload)] = payload
    s[30:32] = struct.pack(">H", crc16(s[:30]))
    return bytes(s)


def u_def(light, seq, roles, gamma=2.8, flags=0):
    p = bytearray(27)
    for i, r in enumerate(roles):
        p[i >> 1] |= (r & 15) << (0 if i & 1 else 4)
    p[3:5] = struct.pack(">H", int(gamma * 1000))
    p[5] = flags
    for i in range(5):
        p[6 + 2 * i:8 + 2 * i] = struct.pack(">H", 0)
        p[16 + 2 * i:18 + 2 * i] = struct.pack(">H", 65535)
    return u_slot(1, light, 0, seq, p)


def u_set(light, seq, m, a, dur, flags=1, start=0):
    p = struct.pack(">H5H5H", m, *a, *([65535] * 5)) + struct.pack(">I", dur)[1:] + struct.pack(">h", start)
    return u_slot(2, light, flags, seq, p)


def u_fx(light, seq, fx, params):
    return u_slot(3, light, 0, seq, bytes([fx]) + struct.pack(">i", 0) + params)


def u_opt(light, seq, cw_k, ww_k, warm_k, assist):
    """LIGHT_OPT: dim to warm, mireds x 10, red assist Q16"""
    m10 = lambda k: int(1e7 / k + 0.5)
    return u_slot(9, light, 0, seq, struct.pack(">BHHHH", 1, m10(cw_k), m10(ww_k), m10(warm_k), int(assist * 65535)))


def measure_ultra(emu, scratch, copy):
    rows = []
    # ---- decode of an Ultra capture: our slot (256 bits) at the frame start, downstream slots after it
    bits, w16 = frame_samples(256 * 4 + 64, 256)
    assert len(w16) <= 1792
    emu.poke(copy, w16.astype("<u2").tobytes())
    emu.poke(scratch, struct.pack("<BBH", 17, 80, 1800))
    out = scratch + 16
    n = emu.call("ws_decode_frame", [copy, len(w16) * 16, 256, scratch, out])
    ok = emu.ret == 256 and emu.peek(out, 32) == pack(bits[:256])
    rows.append(("Ultra capture decode (256-bit slot)", n, emu.cycles(), "ok" if ok else "MISMATCH"))
    # ---- the header trailer: the last 576 words up to the frame end (ISR tail copy), last 64 bits
    bits, w16 = frame_samples(256 * 4 + 64, 256 * 4 + 64)
    tail = w16[-576:]
    emu.poke(copy, tail.astype("<u2").tobytes())
    n = emu.call("ws_decode_last", [copy, len(tail) * 16, scratch, out])
    ok = emu.ret >= 64 and emu.peek(out, 8) == pack(bits[-64:])
    rows.append(("Ultra header decode (last 64 bits of 576 words)", n, emu.cycles(),
                 ("ok, %d bits in the window" % emu.ret) if ok else "MISMATCH"))

    ul = emu.addr("ul")
    buf = scratch + 256
    R, G, B, W, CW, WW, BR = 2, 3, 4, 5, 6, 7, 1

    def hdr(tick, t_us):                                # (header CRC-8 is inlined in main.c, ~7 bytes)
        n = emu.call("ultra_on_header", [ul, tick, t_us])
        return n, emu.cycles()

    def slot(s, tick):                                  # main applies frx.slot: CRC checked in frx_capture
        emu.poke(buf, s)
        n = emu.call("ultra_apply_slot" if "ultra_apply_slot" in emu.syms else "ultra_on_slot", [ul, buf, tick])
        return n, emu.cycles()

    def render(tick):
        n = emu.call("ultra_render", [ul, tick, scratch + 300])
        return n, emu.cycles()

    def scenario(name, cmds, ticks):
        emu.call("ultra_init", [ul, 1])
        t = 1000
        hdr(t, 50_000_000)
        for s in cmds:
            slot(s, t)
        worst = (0, 0)
        for dt in ticks:
            n, c = render(t + dt)
            if c > worst[1]:
                worst = (n, c)
        rows.append(("ultra_render: " + name, worst[0], worst[1], ""))

    hdr(1000, 50_000_000)
    n, c = hdr(1100, 50_005_000)
    rows.append(("Ultra header: clock sync", n, c, ""))
    emu.call("ultra_init", [ul, 1])
    hdr(1000, 50_000_000)
    slot(u_def(0, 1, [R, G, B, CW, WW]), 1000)
    n, c = slot(u_set(0, 2, 40000, [65535, 30000, 1000, 20000, 50000], 2000), 1000)
    rows.append(("Ultra slot: SET (CRC + new ramp)", n, c, ""))

    mid = [20 * k for k in range(1, 1000, 97)]          # inside the 2 s ramps
    rgbww = [R, G, B, CW, WW]
    scenario("idle (no light defined)", [], [5])
    scenario("RGBWW light, steady", [u_def(0, 1, rgbww), u_set(0, 2, 40000, [65535, 30000, 1000, 20000, 50000], 0)], [100])
    scenario("RGBWW light, smooth transition", [u_def(0, 1, rgbww), u_set(0, 2, 40000, [65535, 30000, 1000, 20000, 50000], 2000)], mid)
    scenario("RGBWW light, via-off transition", [u_def(0, 1, rgbww), u_set(0, 2, 40000, [65535, 30000, 1000, 20000, 50000], 0),
                                                 u_set(0, 3, 50000, [1000, 30000, 65535, 0, 0], 2000, 3)], mid)
    scenario("RGB + CW/WW constant brightness, transition",
             [u_def(0, 1, [R, G, B, 0, 0]), u_def(1, 2, [0, 0, 0, CW, WW], 2.8, 2),
              u_set(0, 3, 40000, [65535, 30000, 1000, 0, 0], 2000), u_set(1, 4, 40000, [0, 0, 0, 20000, 50000], 2000)], mid)
    scenario("5 single-channel lights, all fading",
             [u_def(i, 10 + i, [BR if j == i else 0 for j in range(5)]) for i in range(5)] +
             [u_set(i, 20 + i, 30000 + i * 1000, [65535] * 5, 2000) for i in range(5)], mid)
    scenario("RGBWW, color loop effect", [u_def(0, 1, rgbww), u_set(0, 2, 40000, [65535] * 5, 0),
                                          u_fx(0, 3, 7, struct.pack(">I", 6000)[1:] + struct.pack(">H", 65535))], mid)
    scenario("RGBWW, flicker effect", [u_def(0, 1, rgbww), u_set(0, 2, 40000, [65535] * 5, 0),
                                       u_fx(0, 3, 3, struct.pack(">HHH", 62258, 983, 16))], mid)
    scenario("RGBWW constant brightness + dim to warm, red assist, transition",
             [u_def(0, 1, [R, G, B, WW, CW], 2.8, 2), u_opt(0, 2, 6500, 2700, 1800, 0.3),
              u_set(0, 3, 65535, [0, 0, 0, 65535, 30000], 0), u_set(0, 4, 3000, [0, 0, 0, 65535, 30000], 2000)], mid)
    scenario("5 lights, breathe effect each",
             [u_def(i, 10 + i, [BR if j == i else 0 for j in range(5)]) for i in range(5)] +
             [u_set(i, 20 + i, 50000, [65535] * 5, 0) for i in range(5)] +
             [u_fx(i, 30 + i, 6, struct.pack(">I", 4000)[1:] + struct.pack(">HH", 10000, 65535)) for i in range(5)], mid)
    return rows


# ---------------------------------------------------------------------------------------------- current sense
def measure_isense(emu, scratch):
    """main loop side of core/isense.c: isense_ms() every ms, isense_apply() per output update (worst of a run)."""
    IS = emu.addr("is")
    req = scratch + 3000
    rows = []

    def run(name, raw, ms, duty=40000):
        emu.poke(req, struct.pack("<5H", *([duty] * 5)))
        wm = wa = 0
        for _ in range(ms):
            na = emu.call("isense_apply", [IS, req, req + 16])
            wa = max(wa, emu.cycles())
            nm = emu.call("isense_ms", [IS, raw * 20, 20, raw + 20])
            if emu.cycles() >= wm:
                wm, nmm = emu.cycles(), nm
            if emu.cycles() >= wa:
                naa = na
        rows.append(("isense_ms, %s (every 1 ms, worst)" % name, nmm, wm, ""))
        rows.append(("isense_apply, %s (per output update, worst)" % name, naa, wa, ""))

    emu.call("isense_init", [IS, 1, 10000, 8000])
    run("no model, 2 A (regression windows)", 500, 64)
    run("no model, limiting 12 A of content", 3000, 64)
    # a stored standard test (isense_cal_t: levels, pct[4], zero, ma[5][4]), as the boot loads it (cal_load)
    full = (2500, 2600, 2400, 3000, 2800)
    ma = [m * k // 4 for m in full for k in (1, 2, 3, 4)]
    emu.poke(scratch + 3100, struct.pack("<B4Bxh20H", 4, 25, 50, 75, 100, 40, *ma))
    emu.call("isense_cal_set", [IS, scratch + 3100])
    run("calibrated, limiting", 2000, 64)
    emu.call("isense_cal_start", [IS, 16384])
    run("calibration running", 300, 160)
    return rows


# ---------------------------------------------------------------------------------------------- 0.6.0 monitoring
def measure_monitor(emu, scratch):
    """0.6.0 in the main loop: supply dips and the energy every ms, the soft start per output update, the channel
    health once a second (worst of a run)."""
    rows = []
    vd, en, ss, hl, IS = (emu.addr(n) for n in ("vdip", "en", "ss", "hl", "is"))

    def worst(calls):
        wn = wc = 0
        for fn, args in calls:
            n = emu.call(fn, args)
            if emu.cycles() >= wc:
                wn, wc = n, emu.cycles()
        return wn, wc

    emu.poke(vd, bytes(84))                                          # (static, zero: vdip_init isn't linked)
    emu.call("vdip_ms", [vd, 300, 2604, 1, 2000])                     # the reference: 24 V
    n, c = worst([("vdip_ms", [vd, t, 2604 + (t & 3), 1, 2000]) for t in range(301, 400)])
    rows.append(("vdip_ms, steady supply (every 1 ms)", n, c, ""))
    dip = [("vdip_ms", [vd, t, 2080, 1, 6000]) for t in range(400, 410)]
    dip += [("vdip_ms", [vd, t, 2604, 1, 2000]) for t in range(410, 414)]
    n, c = worst(dip)
    rows.append(("vdip_ms, a dip: start / its end into the log (worst)", n, c, ""))
    emu.call("energy_init", [en, 1000, 0])
    n, c = worst([(f, a) for t in range(1, 50) for f, a in (("energy_ms", [en, t, 24000, 2000, 1]),
                                                            ("energy_due", [en, t, 0]))])
    rows.append(("energy_ms / energy_due (every 1 ms, worst of the two)", n, c, ""))
    emu.call("ss_init", [ss, 50])
    n, c = worst([("ss_factor", [ss, t, 1, 1]) for t in range(1000, 1400)])
    rows.append(("ss_factor, soft start running (per output update)", n, c, ""))
    full = (2500, 2600, 2400, 3000, 2800)
    ma = [m * k // 4 for m in full for k in (1, 2, 3, 4)]
    emu.poke(scratch + 3100, struct.pack("<B4Bxh20H", 4, 25, 50, 75, 100, 40, *ma))
    emu.call("isense_init", [IS, 1, 10000, 8000])
    emu.call("isense_cal_set", [IS, scratch + 3100])
    emu.call("health_init", [hl])
    n = emu.call("health_run", [hl, IS, scratch + 3100])
    rows.append(("health_run (once a second)", n, emu.cycles(), ""))
    emu.call("health_test_start", [hl, scratch + 3100])
    n = emu.call("health_test_done", [hl, scratch + 3100])
    rows.append(("health_test_done (after a calibration test)", n, emu.cycles(), ""))
    return rows


def emulators():
    """(the image itself, the one with every measured function: its measurement build unless it has them all)"""
    rel = Emu(ELF)
    need = ("ws_decode_frame", "dither_build", "ultra_render", "ultra_apply_slot", "isense_apply", "stagger_slew")
    if all(f in rel.syms for f in need):
        return rel, rel
    if not os.path.exists(MELF):
        sys.exit("%s has inlined the measured functions (LTO): build %s with MEASURE=1 (see the top of this file)"
                 % (ELF, MELF))
    return rel, Emu(MELF)


def isense_init(emu, src):
    """isense_init(&is, 1, 10000, 8000) in emu; when LTO inlined it there, in src and the struct copied over"""
    if "isense_init" in emu.syms:
        emu.call("isense_init", [emu.addr("is"), 1, 10000, 8000])
        return
    a, n = src.sym("is")
    src.call("isense_init", [a, 1, 10000, 8000])
    assert emu.sym("is")[1] == n
    emu.poke(emu.addr("is"), src.peek(a, n))


def main():
    rel, emu = emulators()
    meas = emu
    scratch = emu.addr("din_ring")                     # 4 KB, unused by the functions measured here
    copy = emu.addr("din_copy")
    rows = []

    # ---- decoder: WS2915 (112 bits) and WS2805 (40 bits) slots, chain with downstream bits after ours
    for name, own, total in (("WS2915 slot (112 bits)", 112, 112 + 80 * 10), ("WS2805 slot (40 bits)", 40, 40 * 10)):
        bits, w16 = frame_samples(total, own)
        assert len(w16) <= 1024
        emu.poke(copy, w16.astype("<u2").tobytes())
        emu.poke(scratch, struct.pack("<BBH", 17, 80, 1800))
        out = scratch + 16
        n = emu.call("ws_decode_frame", [copy, len(w16) * 16, own, scratch, out])
        ok = emu.ret == own and emu.peek(out, (own + 7) // 8) == pack(bits[:own])
        rows.append((name + " decode", n, emu.cycles(), "ok" if ok else "MISMATCH"))

    # ---- dither table for one channel (worst = longest pattern, 100 kHz) and the 25 kHz default;
    # dither_opts_t {mode, threshold16, floor_policy, floor} = <iHxxiHxx>, default floor 29 counts (skip)
    opts = scratch + 2048
    emu.poke(opts, struct.pack("<iHxxiHxx", 0, 655, 0, 29))
    for steps, ln, v in ((1440, 50, 0x7123), (5760, 12, 0x7123), (65535, 1, 0x7123), (5760, 12, 20), (1440, 50, 20)):
        n = emu.call("dither_build", [v, steps, ln, opts, 3, scratch, 3])
        what = "pulse skipping" if v < 100 else "dither"
        rows.append(("dither_build %s, steps %d, table %d (x5 per update)" % (what, steps, ln), n, emu.cycles(), ""))

    # ---- smoothing step (5 channels, ramp running)
    sm = emu.addr("sm")
    emu.poke(sm, struct.pack("<5H5H5HxxIIIIB", *([0] * 5), *([0] * 5), *([60000, 1000, 30000, 257, 9]), 100, 400, 100, 400, 1))
    n = emu.call("smooth_update", [sm, 300])
    rows.append(("smooth_update (5 channels, every 1 ms)", n, emu.cycles(), ""))

    # ---- WS2805 gamma 2.2 / 2.8 (ultra_gamma, no tables): 5 channels per frame; linear is a multiply
    tot_n = tot_c = 0
    for v in (17, 114, 187, 220, 255):
        tot_n += emu.call("gamma_apply", [2, v])
        tot_c += emu.cycles()
    rows.append(("gamma_apply 2.8, 5 channels (per WS2805 frame)", tot_n, tot_c, ""))

    # ---- MCU temperature: conversion + filter of one pair of results (main loop, once a second); ~35 C at 3.3 V
    mt = emu.addr("mt")
    emu.call("mcutemp_init", [mt])
    for ts2, vr2 in ((3368, 2978), (3366, 2979)):
        n = emu.call("mcutemp_sample", [mt, ts2, vr2])
        c = emu.cycles()
    rows.append(("mcutemp_sample (once a second)", n, c, "%.1f C" % (struct.unpack("<h", emu.peek(mt + 6, 2))[0] / 10)))

    # ---- poll ISR paths: in the image itself (the ISRs are never inlined)
    emu = rel
    i_img = len(rows)
    EDGE_CNT = 0x40000824                              # TIM4->CNT: DIN edges (TIM3 capture -> ITR2)
    DMA1_CH2_CNTR = 0x40020020
    sym = emu.addr
    emu.mmio[DMA1_CH2_CNTR] = 2048 - 700

    def isr(cnt, setup=None):
        emu.mmio[EDGE_CNT] = cnt
        if setup:
            setup()
        return emu.call("poll_isr"), emu.cycles()

    # every poll tick also sums the current-sense result (isense_tick), so all poll ISR rows below include it
    ADC1_STATR, ADC1_RDATAR = 0x40012400, 0x4001244C
    isense = True
    isense_init(emu, meas)
    emu.mmio[ADC1_RDATAR] = 500                        # ~2 A
    FT = sym("ft")                                     # ftrack_t: last_cnt, idle, armed (+3), cap_state (+4), ...
    emu.wr(sym("rx_ready"), 1, 1)
    emu.wr(FT + 3, 1, 1)                               # armed
    plan = sym("plan")
    emu.poke(plan, struct.pack("<BxHHHBx", 1, 32, 112, 112, 32))   # gate_plan_t: start_open, t1, t2, own, hdr
    n, c = isr(0)
    rows.append(("poll ISR, line idle", n, c, ""))
    # every 20th tick (1 ms): the ADC injected group, the last sequence's 4 results read, the next one started
    emu.wr(sym("adc_ready"), 1, 1)
    emu.wr(sym("inj_run"), 1, 1)
    emu.mmio[ADC1_STATR] = 0x4                         # JEOC
    for a in range(0x4001243C, 0x4001244C, 4):         # IDATAR1-4
        emu.mmio[a] = 1234
    sub = [k for k in emu.syms if k.startswith("sub.")]  # poll_isr's static ms divider (GCC numbers it)
    assert len(sub) == 1, sub
    emu.wr(sym(sub[0]), 2, 19)
    n, c = isr(0)
    rows.append(("poll ISR, line idle, 1 ms tick (ADC injected read + start)", n, c, ""))
    n, c = isr(5)
    rows.append(("poll ISR, first edges of a frame (note ring pos)", n, c, ""))
    n, c = isr(60)
    rows.append(("poll ISR, frame running", n, c, ""))
    n, c = isr(120)
    rows.append(("poll ISR, own bits complete", n, c, ""))
    emu.wr(sym("cap_start"), 2, 100)
    n, c = isr(160)
    rows.append(("poll ISR, ring copy (600 words)", n, c, ""))
    n, c = isr(160)
    n, c = isr(160)
    n, c = isr(160)
    rows.append(("poll ISR, WS frame end (re-arm + tail copy: auto, unlocked)", n, c, ""))
    # an Ultra frame (4 boards: 1088 bits) ends: gate re-arm + the 576-word tail copy (the header trailer)
    emu.wr(FT + 3, 1, 1)                               # armed
    emu.mmio[DMA1_CH2_CNTR] = 2048 - 1500
    n, c = isr(5)
    n, c = isr(1088)
    n, c = isr(1088)
    n, c = isr(1088)
    n, c = isr(1088)
    rows.append(("poll ISR, Ultra frame end (re-arm + 576-word tail copy)", n, c, ""))
    # longest Ultra capture (1728 words, full-spec timing with hiccups): ring copy inside the poll ISR
    emu.wr(FT + 4, 1, 2)                               # cap_state: copy next tick
    emu.wr(sym("cap_start"), 2, 100)
    emu.mmio[DMA1_CH2_CNTR] = 2048 - (100 + 1728)
    n, c = isr(400)
    rows.append(("poll ISR, ring copy (1728 words, longest Ultra capture)", n, c, ""))
    if isense:
        emu.mmio[ADC1_RDATAR] = 4095                   # a short: 2nd tick over the fast-trip level
        isr(400)
        n, c = isr(400)
        rows.append(("poll ISR, current sense fast trip (outputs forced off)", n, c, ""))
        emu.mmio[ADC1_RDATAR] = 500

    # adaptive stagger phase slew: TIM2 / TIM3 update ISRs while moving (100 kHz, 1440 counts, 50-entry tables)
    emu.wr(sym("cur_steps"), 4, 1440)
    emu.wr(sym("cur_len"), 1, 50)
    emu.wr(sym("cur_stagger"), 1, 2)
    emu.wr(sym("slew_tgt2"), 2, 480)
    emu.wr(sym("slew_tgt3"), 2, 960)
    emu.mmio[0x40012C24] = 700        # TIM1->CNT
    emu.mmio[0x40000024] = 30         # TIM2->CNT just after its wrap
    emu.mmio[0x40000424] = 30         # TIM3->CNT
    emu.mmio[0x40020034] = 60         # DMA1 ch3 CNTR (TIM3 table position)
    n = emu.call("slew2_isr")
    rows.append(("slew ISR TIM2 (per PWM period while moving)", n, emu.cycles(), ""))
    n = emu.call("slew3_isr")
    rows.append(("slew ISR TIM3 incl. W2 CCR3 shift (per period while moving)", n, emu.cycles(), ""))
    if meas is not rel:
        rows[i_img:] = [(r[0], r[1], r[2], (r[3] + " [image]").strip()) for r in rows[i_img:]]
    emu = meas

    # ---- frame parse + config check
    emu.poke(scratch, pack([rng.getrandbits(1) for _ in range(112)]))
    n = emu.call("proto_parse", [1, scratch, scratch + 64])
    rows.append(("proto_parse WS2915", n, emu.cycles(), ""))

    rows += measure_ultra(emu, scratch, copy)

    # ---- adaptive stagger: offset search (main loop, at most every 50 ms) and one phase-slew step
    duty, peak, res = scratch + 400, scratch + 420, scratch + 440
    emu.poke(peak, struct.pack("<5H", 3000, 3000, 3000, 5000, 5000))
    for name, d in (("example mix 83/70/42/9/34 %", (0.83, 0.70, 0.42, 0.09, 0.34)),
                    ("all five at 42 %", (0.42,) * 5), ("RGB only 50 %", (0.5, 0.5, 0.5, 0, 0))):
        emu.poke(duty, struct.pack("<5H", *[int(x * 65535) for x in d]))
        n = emu.call("stagger_choose_max", [duty, peak, 819, 21845, 43691, 0, res])
        o2, o3 = struct.unpack("<HH", emu.peek(res, 4))
        rows.append(("stagger_choose: " + name, n, emu.cycles(), "-> %.3f / %.3f T" % (o2 / 65536, o3 / 65536)))
    # spread spectrum: dither table with varying periods, one channel (x5 per update)
    arr = scratch + 1024
    for steps, ln in ((5760, 32), (1440, 50), (65535, 32)):
        emu.call("stagger_spread_arr", [steps, ln, arr])
        emu.poke(opts, struct.pack("<iHxxiHxx", 0, 655, 0, 29))
        min_len = max((65535 + steps - 1) // steps, (144000000 // 2000) // steps)
        # mid level (fast path), just above the floor / off-time near it (carried error, two passes), below the
        # floor (pulse skipping)
        fl = 29 * 65535 // steps
        for what, v in (("mid", 0x7123), ("just above floor", fl * 103 // 100), ("below floor", fl // 3),
                        ("off-time near floor", 65535 - fl * 103 // 100)):
            n = emu.call("dither_build_var", [v, steps, arr, ln, min_len, opts, 3, scratch, 6, 0])
            rows.append(("dither_build_var (spread) %s, steps %d, table %d (x5)" % (what, steps, ln), n, emu.cycles(), ""))
    n = emu.call("stagger_slew", [100, 2000, 11, 5760, 11, 1])
    rows.append(("stagger_slew (per period while moving)", n, emu.cycles(), ""))

    if isense:
        rows += measure_isense(emu, scratch)
        rows += measure_monitor(emu, scratch)

    print("image: %s" % ELF)
    if meas is not rel:
        print("rows not marked [image]: its measurement build %s" % MELF)
    print("%-52s %8s %8s %9s" % ("function / path", "instr", "cycles~", "time"))
    for name, n, c, note in rows:
        print("%-52s %8d %8d %7.1f us %s" % (name, n, c, c / F_CPU * 1e6, note))
    return rows


if __name__ == "__main__":
    main()

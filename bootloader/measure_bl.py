# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run the compiled bootloader's functions in tools/rv32emu.py and report executed instructions / estimated cycles.

    bash bootloader/build_bl.sh  &&  python bootloader/measure_bl.py
    BL_ELF=<path>/bl.elf python bootloader/measure_bl.py

Measures the fast boot path (at 8 MHz, before any clock is touched), the streaming decoder per sample word (budget
at 96 MHz with 24 MHz sampling: 64 cycles per 16-sample word) and the session's work per UPD frame.
"""
import os
import random
import struct
import sys

FW = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(FW, "tools"))
from rv32emu import Emu  # noqa: E402

ELF = os.environ.get("BL_ELF") or os.path.join(FW, "build", "bootloader", "bl.elf")
CRC_DATAR = 0x40023000
FS = float(os.environ.get("BL_FS", "36e6"))      # DIN sample rate
F_CPU = float(os.environ.get("BL_FCPU", "144e6"))
THR = {24e6: (13, 53, 2400), 36e6: (17, 80, 3600), 18e6: (10, 40, 1800)}
rng = random.Random(7)


def crc32_words(data, crc=0xFFFFFFFF):
    for i in range(0, len(data), 4):
        crc ^= struct.unpack_from("<I", data, i)[0]
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
    return crc


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


def descriptor(body, version=0x000500FF):
    d = bytearray(64)
    d[0:4] = b"QDU5"
    d[4], d[5] = 1, 1
    struct.pack_into(">HIIIII", d, 6, 0b110, version, len(body), crc32_words(body), 0x08002100, 0x00002100)
    d[28] = 1
    struct.pack_into(">I", d, 60, crc32_words(bytes(d[:60])))
    return bytes(d)


def upd_frame(typ, session, index, data32, flags=0):
    f = bytearray(48)
    f[0], f[1] = typ, session
    struct.pack_into(">H", f, 2, index)
    f[5] = flags
    f[6:38] = data32
    struct.pack_into(">H", f, 38, crc16(f[:38]))
    h = bytearray(8)
    struct.pack_into(">HBI", h, 0, 0x5AD2, (2 << 4) | 0x08, 12345)
    h[7] = crc8(h[:7])
    f[40:48] = h
    return bytes(f)


def wave(frame, gap_us=300.0):
    """16-bit words at 24 MHz: the hub's '0' 340/960 ns, '1' 700/650 ns, +-40 ns jitter, then the reset gap"""
    lv = []
    t = 0.0
    frac = 0.0

    def put(level, ns):
        nonlocal frac
        s = ns * 1e-9 * FS + frac
        n = int(s)
        frac = s - n
        lv.extend([level] * n)

    put(0, 150e3)
    for byte in frame:
        for k in range(8):
            one = (byte >> (7 - k)) & 1
            e = rng.uniform(-40, 40)
            put(1, (700 if one else 340) + e)
            put(0, (650 if one else 960) - e)
    put(0, gap_us * 1e3)
    while len(lv) % 16:
        lv.append(0)
    words = []
    for i in range(0, len(lv), 16):
        w = 0
        for b in lv[i:i + 16]:
            w = (w << 1) | b
        words.append(w)
    return words


def main():
    emu = Emu(ELF)
    s = emu.syms
    print(f"bootloader {ELF}")

    # ---------------------------------------------------------------- fast boot path (8 MHz, no PLL)
    body = bytes(rng.getrandbits(8) for _ in range(30548))
    desc = descriptor(body)
    emu.poke(0x08002000, desc + struct.pack("<HH", 0xA5B7, 0xA5C3) + b"\x39\xE3" * 94)
    emu.mmio[CRC_DATAR] = struct.unpack(">I", desc[60:64])[0]   # what the CRC unit returns for the descriptor
    app, ho = 0x20003000, 0x20000000
    n1 = emu.call("bl_app_read", [app])
    c1 = emu.cycles()
    n2 = emu.call("bl_decide", [app, ho, 1, 0x0C000000])
    c2 = emu.cycles()
    print(f"fast path (8 MHz): bl_app_read {n1} instr / {c1} cycles + bl_decide {n2} / {c2} -> action {emu.ret} (0 = jump);"
          f" {(c1 + c2) / 8:.1f} us at 8 MHz")

    # ---------------------------------------------------------------- decoder per word
    timing = 0x20003100
    emu.poke(timing, struct.pack("<HHI", *THR[FS]))
    rx, fr, buf = s["rx"][0], s["frame"][0], 0x20003200
    frames = [upd_frame(0x82, 7, i, bytes(rng.getrandbits(8) for _ in range(32))) for i in range(4)]
    words = []
    for f in frames:
        words += wave(f)
    emu.call("bl_rx_init", [rx, timing])
    tot_i = tot_c = got = 0
    for i in range(0, len(words), 128):
        chunk = words[i:i + 128]
        emu.poke(buf, struct.pack(f"<{len(chunk)}H", *chunk))
        tot_i += emu.call("bl_rx_feed", [rx, buf, len(chunk), fr])
        tot_c += emu.cycles()
        got += emu.ret
    budget = F_CPU * 16 / FS
    print(f"decoder at {FS / 1e6:.0f} MHz sampling, {F_CPU / 1e6:.0f} MHz CPU: {len(words)} words "
          f"({len(words) * 16 / FS * 1e3:.2f} ms, 4 UPD frames + 300 us gaps) -> {got} frames; "
          f"{tot_c / len(words):.1f} cycles/word = {100 * tot_c / len(words) / budget:.0f} % of the CPU on average")
    # inside a frame only: one frame's words from its first edge to its last
    fw = wave(frames[0], gap_us=0.0)[int(150e-6 * FS / 16):]
    emu.call("bl_rx_init", [rx, timing])
    emu.poke(buf, struct.pack(f"<{len(wave(frames[0])[:int(150e-6 * FS / 16)])}H", *wave(frames[0])[:int(150e-6 * FS / 16)]))
    c_in = 0
    for i in range(0, len(fw), 128):
        chunk = fw[i:i + 128]
        emu.poke(buf, struct.pack(f"<{len(chunk)}H", *chunk))
        emu.call("bl_rx_feed", [rx, buf, len(chunk), fr])
        c_in += emu.cycles()
    print(f"         inside a frame: {c_in / len(fw):.1f} cycles/word = {100 * c_in / len(fw) / budget:.0f} % of the CPU "
          f"({c_in / 384:.0f} cycles per bit of {F_CPU * 1.325e-6:.0f})")

    # ---------------------------------------------------------------- session work per frame
    sess = s["sess"][0]
    emu.poke(app, struct.pack("<BBBBBBxxIII", 1, 1, 1, 0, 0, 2, 0x000400FF, len(body), 0))
    emu.call("bl_sess_init", [sess, 3, app, 0])                    # BL_ACT_SESSION
    tail = 0x20003400
    for idx in (0, 1):
        emu.poke(tail, upd_frame(0x81, 9, idx, desc[32 * idx:32 * idx + 32]))
        emu.call("bl_sess_frame", [sess, 384, tail, 1])
    print(f"ANNOUNCE -> accept + erase plan: {emu.cycles()} cycles (+ the erases themselves)")
    costs = []
    for idx in range(16):
        emu.poke(tail, upd_frame(0x82, 9, idx, body[32 * idx:32 * idx + 32]))
        emu.call("bl_sess_frame", [sess, 384, tail, 2])
        costs.append(emu.cycles())
    print(f"DATA frame: {min(costs)}-{max(costs)} cycles ({min(costs) / F_CPU * 1e6:.1f}-{max(costs) / F_CPU * 1e6:.1f} us;"
          f" the max includes a page program's register work, not its 2 ms)")
    ok = emu.peek(0x08002100, 512) == body[:512]
    print(f"2 pages programmed through the flash registers (emulated memory): {'match' if ok else 'MISMATCH'}")


if __name__ == "__main__":
    main()

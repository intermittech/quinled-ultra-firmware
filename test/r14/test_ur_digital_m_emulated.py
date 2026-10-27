# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Round-14 tests of the real compiled BOARD=3 app in rv32emu.

Build: XW=0 LTO=0 BOARD=3 EXTRA_CFLAGS='-fno-inline -fno-ipa-cp -fno-ipa-sra'
       OUT=build/r14-ur_digital_m-api bash tools/build_target.sh
Run: python test/r14/test_ur_digital_m_emulated.py [--case good|length|features|all]
No copied implementation: calls ud_decode/ud_frame/ud_rc_page from the target ELF.
"""
import argparse
from pathlib import Path
import struct
import sys

FW = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FW / "tools"))
from rv32emu import Emu
sys.path.insert(0, str(FW / "test"))
from ur_digital_m_target import offsets

checks = failures = 0

def check(ok, message):
    global checks, failures
    checks += 1
    if not ok:
        failures += 1
        print("FAIL:", message)

def crc(data, width, poly, init):
    value = init
    mask = (1 << width) - 1
    for b in data:
        value ^= b << (width - 8)
        for _ in range(8):
            value = ((value << 1) ^ (poly if value & (1 << (width - 1)) else 0)) & mask
    return value

def frame(kind=7, value=0, rate=0, flags=1, host=123456):
    slot = bytearray(32)
    slot[0:3] = bytes([kind, 255 if kind in (6, 7) else 0, 23])
    if kind == 7:
        slot[3:8] = bytes([0, 1, value, rate, 0])
    else:
        slot[3] = value
    slot[30:] = crc(slot[:30], 16, 0x1021, 0xFFFF).to_bytes(2, "big")
    h = bytearray(b"\x5a\xd2" + bytes([0x30 | flags]) + struct.pack(">I", host))
    h.append(crc(h, 8, 7, 0))
    return bytes(slot + h)

def wave(data, nbits=None):
    nbits = len(data) * 8 if nbits is None else nbits
    samples = [0] * 360  # ten microseconds after marker's falling edge
    for i in range(nbits):
        high = 25 if data[i // 8] & (0x80 >> (i % 8)) else 9
        samples.extend([1] * high + [0] * (48 - high))
    samples.extend([0] * 1664)  # 46.2 us end detection; even a 384-bit capture fits 1280 words
    samples.extend([0] * (-len(samples) % 16))
    words = []
    for i in range(0, len(samples), 16):
        value = 0
        for b in samples[i:i + 16]:
            value = (value << 1) | b
        words.append(value)
    return struct.pack("<" + "H" * len(words), *words)

ELF = str(FW / "build/r14-ur_digital_m-api/fw.elf")   # --elf: another build of the same flags


def fresh():
    e = Emu(ELF)
    e.call("dline_init", [e.addr("dl"), 0, e.addr("dl_cap")])
    return e

def decode(e, data, nbits=None):
    w = wave(data, nbits)
    e.poke(0x20003000, w)
    e.call("ud_decode", [0x20003000, len(w) // 2, 25000], max_steps=2000000)
    return e.ret

def direct(e, data, tick=25000):
    e.poke(0x20003000, data)
    e.call("ud_frame", [0x20003000, len(data) * 8, tick], max_steps=2000000)
    return e.ret

def good():
    for rate in range(3):
        for page in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0x80, 0xFF):
            e = fresh()
            r = decode(e, frame(value=page, rate=rate))
            answer = e.peek(e.addr("ud_ans"), 32)
            check(r == 1, f"valid POLL rate {rate} page {page} gets answer")
            check(answer[0] == 0xA5 and crc(answer[:30], 16, 0x1021, 0xFFFF) == int.from_bytes(answer[30:], "big"),
                  f"valid answer CRC, rate {rate} page {page}")
    for n in (0, 1, 7, 8, 255, 256, 311, 312, 318, 319):
        check(decode(fresh(), frame(), n) == 0, f"truncated capture, {n} bits")
    for bit in range(320):
        f = bytearray(frame())
        f[bit // 8] ^= 0x80 >> (bit % 8)
        check(direct(fresh(), f) == 0, f"corruption at frame bit {bit}")
    for flags in (0, 4, 5, 8, 9, 12, 13):
        check(direct(fresh(), frame(flags=flags)) == 0, f"POLL ignored under header flags {flags}")
    for base in (0, 0xFFFFFF00):
        e = fresh()
        e.call("ud_clock", [base, 0xFFFF0000])
        e.call("ud_clock", [(base + 1000000) & 0xFFFFFFFF, (0xFFFF0000 + 20000) & 0xFFFFFFFF])
        rate = int.from_bytes(e.peek(e.addr("rate_q16"), 4), "little")
        check(rate == 50 << 16, f"clock recovery across host/tick wrap: {rate}")
    print("UR_DIGITAL_M good cases: 48 page/rate combinations, 10 truncations, 320 single-bit corruptions, flags and clock wraps")

def lengths():
    valid = frame()
    check(decode(fresh(), valid) == 1, "control: complete 320-bit frame accepted")
    for n in (321, 328, 336, 352, 383, 384):
        e = fresh()
        r = decode(e, valid + bytes(8), n)
        print(f"UR_DIGITAL_M capture length {n} bits, valid 320-bit prefix, no valid final trailer: answer={r}")
        check(r == 0, f"malformed {n}-bit frame must not apply an embedded POLL")
    e = fresh()
    f = frame(kind=6, value=7, flags=0) + bytes(8)
    decode(e, f, 328)
    # vdip_t: ref_q12/start_ms/run/count/in_dip/n_log/head/planned (offset 15).
    planned = e.peek(e.addr("vdip") + 15, 1)[0]
    print(f"UR_DIGITAL_M 328-bit IDENTIFY 7 with invalid final trailer: planned={planned}")
    check(planned == 0, "malformed frame must not set the planned-power-cut flag")

def features():
    e = fresh()
    e.poke(0x20003100, bytes([1]) + bytes(23))
    e.poke(0x20003200, bytes(24))
    e.call("ud_rc_page", [4, 0x20003100, 0x20003200])
    page = e.ret
    bits = int.from_bytes(e.peek(0x20003200, 4), "big")
    before = bytes(e.ram)
    e.writes.clear()
    direct(e, frame(kind=6, value=5, flags=0))
    after = bytes(e.ram)
    # Enumerate the target's persistent symbols changed by this command.
    changed = []
    for name, (addr, size) in e.syms.items():
        if size and 0x20000000 <= addr < 0x20003000:
            off = addr - 0x20000000
            if before[off:off + size] != after[off:off + size]:
                changed.append(name)
    print(f"UR_DIGITAL_M IDENTITY feature VERSION_BLINK={bool(bits & 0x400)}; IDENTIFY 5 MMIO writes={e.writes}; changed symbols={sorted(changed)}")
    # A command may schedule a future blink, so this observation alone is not a failure.
    check(page == 4, "IDENTITY features page supported")
    check(e.rd(e.addr("led_lead"), 2) == 1000, "board IDENTIFY 5 schedules the advertised version indication")

def ring():
    """Capture the actual wire samples through compiled dline_tick before ud_decode."""
    for nbits in (320, 321, 328, 384):
        e = fresh()
        dl, ring_addr = e.addr("dl"), e.addr("dl_ring")
        # 400 us LOW, 20 us marker, 10 us LOW, frame, total 150 us LOW, release HIGH.
        wire = bytes(1800) + b"\xff\xff" * 45 + wave(frame() + bytes(8), nbits)
        wire += bytes(468) + b"\xff\xff" * 2250
        wire += bytes(-len(wire) % 2)
        words = struct.unpack("<" + "H" * (len(wire) // 2), wire)
        written = edges = last = captures = answer = sends = cap_words = 0
        while written < len(words):
            stop = min(written + 112, len(words))
            for pos in range(written, stop):
                value = words[pos]
                e.wr(ring_addr + (pos & 511) * 2, 2, value)
                for bit in range(15, -1, -1):
                    level = (value >> bit) & 1
                    edges += last and not level
                    last = level
            written = stop
            rx = written & 511
            e.call("dline_tick", [dl, ring_addr, rx, (rx + 2) & 511, edges & 0xFFFF])
            actions = e.ret
            if actions & 16:
                captures += 1
                cap_words = e.rd(dl + offsets(ELF)["dl_cap_words"], 2)
                e.call("ud_decode", [e.addr("dl_cap"), cap_words, written // 112], max_steps=2000000)   # (dline's capture buffer: the array dl.cap points to)
                answer = e.ret
                e.call("dline_arm", [dl, answer])
            if actions & 32:
                sends += 1
                break
        print(f"UR_DIGITAL_M actual ring: sent bits={nbits}, captures={captures}, captured words={cap_words}, decoded answer={answer}, SEND actions={sends}")
        check(captures == 1, f"control: {nbits}-bit wire frame reached actual capture")
        check(answer == (nbits == 320) and sends == (nbits == 320), f"only the exact 320-bit POLL may be answered ({nbits} bits)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--case", choices=("good", "length", "features", "ring", "all"), default="all")
    p.add_argument("--elf", default=ELF)
    a = p.parse_args()
    ELF = a.elf
    for name, fn in (("good", good), ("length", lengths), ("features", features), ("ring", ring)):
        if a.case in (name, "all"):
            fn()
    print(f"R14 UR_DIGITAL_M target: {checks} checks, {failures} failures")
    raise SystemExit(bool(failures))

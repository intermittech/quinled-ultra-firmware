# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Full BOARD=3 release poll ISR, driven by real 36-MHz ring samples.

Uses rv32emu's documented instruction-class cycle estimate, not silicon timing.
The main loop is serviced between sample batches; this measures separate ISR and
main-loop costs, not preemptive wall-clock latency. Stop at SEND (UART sequencing
is exercised separately by test_ur_digital_m_uart.py). During flash, use flash_poll_isr at
half HCLK and do not service the main loop.
"""
import argparse
import struct
from pathlib import Path
from test_ur_digital_m_emulated import FW, frame, wave
from mainloop import Emu2
from flash_stores import FlashEmu
import sys
sys.path.insert(0, str(FW / "test"))
from ur_digital_m_target import offsets

p = argparse.ArgumentParser()
p.add_argument("--release-dir", type=Path, default=FW / "build/release/0.7.1")
p.add_argument("--sweep", action="store_true", help="all 113 word offsets and 16 sample offsets of one POLL")
args = p.parse_args()
release = args.release_dir
elf = release / "obj/app-ur_digital_m-rev0/fw.elf"
O = offsets(elf)


class Board(FlashEmu, Emu2):
    cnt = 0
    de = 0

    def rd(self, a, n):
        if a == 0xE000F008:
            self.cnt += 8
            return self.cnt % 7200
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == 0x40010C10:
            if v & (1 << 10): self.de = 1
            if v & (1 << 26): self.de = 0
        if a == 0x40010C14 and v & (1 << 10): self.de = 0
        if a == 0x4001380C: self.mmio[a] = v
        super().wr(a, n, v)

    def interrupt(self, name):
        # Preserve the suspended main register file; ISR stack lies below main's.
        main_regs = self.x
        pc = self.start(name)
        self.x[2] = main_regs[2] - 16
        self.run_to(pc, set(), max_steps=200000)
        cost = self.cycles()
        self.x = main_regs
        return cost


def boot():
    e = Board(str(elf))
    e.poke(0x08010000, b"\xff" * 0xC000)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 8, 0x40004400: 0xC0,
                   0x40010C08: 1 << 10, 0x40010808: 1 << 3})
    desc = (release / "diff-power-ur-digital-m-0.7.1-ur_digital_m-rev0.bin").read_bytes()[:64]
    e.poke(0x08002000, desc + b"\xff" * 192)
    info = (release / "bootloader-hw3-rev0.bin").read_bytes()[16:48]
    assert info[:4] == b"QDB2"
    e.poke(0x08000010, info)
    base, size = e.sym("main")
    stop = {q for q in range(base, base + size, 2) if e.rd(q, 4) == 0x10500073}
    assert len(stop) == 1
    pc, _ = e.run_to(e.start("main"), stop, max_steps=2000000)
    assert e.rd(e.addr("dl_on"), 1)
    # This measures a digital-port STREAM session, not AUTO discovery. The first
    # marker in AUTO establishes the mode; the existing stream POLL fixtures
    # assume that decision has already happened (CHAIN has its own budget suite).
    uc, dl, ux = e.addr("uc"), e.addr("dl"), e.addr("ux")
    e.wr(uc + O['uc_mode'], 1, 1)  # UDC_STREAM
    e.wr(uc + O['uc_cfg'], 1, 1)   # CONFIG 75 STREAM
    e.wr(ux + O['ux_cfg'] + (75 - 64) * 2, 2, 1)
    e.wr(dl + O['dl_ch_on'], 1, 0)
    e.wr(dl + O['dl_ch_hold'], 1, 0)
    return e, pc, stop


def words(wire):
    wire += bytes(-len(wire) % 2)
    return struct.unpack("<" + "H" * (len(wire) // 2), wire)


scenarios = [("idle", bytes(18000), False)]
for value in (0x00, 0xFF, 0xAA, 0x55):
    scenarios.append((f"pixels-{value:02x}", bytes(1800) + wave(bytes([value]) * 120) + bytes(1800), False))
for rate in range(3):
    scenarios.append((f"poll-rate{rate}", bytes(1800) + b"\xff\xff" * 45 + wave(frame(rate=rate))
                      + bytes(468) + b"\xff\xff" * 2250, True))

if args.sweep:
    original = words(scenarios[-3][1])
    scenarios = []
    for word_offset in range(113):
        for sample_offset in range(16):
            shifted = [0] * word_offset
            carry = 0
            for value in original:
                shifted.append((value >> sample_offset) | carry)
                carry = ((value << (16 - sample_offset)) & 0xFFFF) if sample_offset else 0
            if sample_offset: shifted.append(carry)
            wire = struct.pack("<" + "H" * len(shifted), *shifted)
            scenarios.append((f"poll-w{word_offset}-s{sample_offset}", wire, True))

failures = checks = ticks = 0
for flash in (False, True):
    for name, wire, expect_send in scenarios:
        e, pc, stop = boot()
        ring = e.addr("dl_ring")
        data = words(wire)
        written = edges = last = k = max_poll = max_main = 0
        while written < len(data):
            # 112/113 words alternate: exactly 1800 samples per 50-us tick.
            stop_word = min(written + 112 + (k & 1), len(data))
            for pos in range(written, stop_word):
                v = data[pos]
                e.wr(ring + (pos & 511) * 2, 2, v)
                for bit in range(15, -1, -1):
                    level = (v >> bit) & 1
                    edges += bool(last and not level)
                    last = level
            written = stop_word
            e.mmio[0x40020020] = 512 - (written & 511)
            e.mmio[0x40020034] = 512 - ((written + 2) & 511)
            e.mmio[0x40000824] = edges & 0xFFFF
            e.mmio[0x40010808] = (1 << 3) | (last << 6)
            cost = e.interrupt("flash_poll_isr" if flash else "poll_isr")
            max_poll = max(max_poll, cost)
            ticks += 1
            if not flash:
                pc, _ = e.run_to(pc, stop, max_steps=200000)
                max_main = max(max_main, e.cycles())
            k += 1
            if e.de: break
        budget = 3600 if flash else 7200
        checks += 3
        if max_poll >= budget:
            failures += 1
            print(f"FAIL: {name} flash={flash} exceeds modeled ISR budget")
        if bool(e.de) != (expect_send and not flash):
            failures += 1
            print(f"FAIL: {name} flash={flash} send={e.de}")
        fault_count = e.rd(e.addr("uds") + 2, 1)
        pending_fault = e.rd(e.addr("dl_fault_pend"), 1)
        if fault_count or pending_fault:
            failures += 1
            print(f"FAIL: {name} flash={flash} faults={fault_count} pending={pending_fault}")
        print(f"UR_DIGITAL_M full ISR {name}, flash={int(flash)}: ticks={k}, peak={max_poll}/{budget} cycles "
              f"({max_poll / (72 if flash else 144):.3f} us), main peak={max_main} cycles, SEND={e.de}", flush=True)

print(f"R14 UR_DIGITAL_M full ISR: {len(scenarios) * 2} scenarios, {ticks} ticks, {checks} checks, {failures} failures")
raise SystemExit(bool(failures))

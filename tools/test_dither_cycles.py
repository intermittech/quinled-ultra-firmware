# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Target timing regression probes for two output-equivalent mutation survivors.

Usage: python tools/test_dither_cycles.py <MEASURE=1 fw.elf>
Reference: GCC 15.2 (WCH's, XW), default rev04 LTO measurement build, rv32emu COST.
These are emulator cycle estimates, not scope measurements. The upper bounds are reviewed
baselines: frx_capture 1,057, dither_build_var 1,149 (-fno-ipa-modref: +1, a flash trade),
dither_build 728 (XW's flags,
--param gcse-unrestricted-cost=0: +3, a flash trade within the limits); a new compiler/configuration requires a
reviewed baseline.
"""
import argparse
import struct
from rv32emu import Emu

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("elf")
parser.add_argument("--frx-elf", help="optional default rev04 LTO=0 ELF for the frame receiver budget")
args = parser.parse_args()
failures = 0
for function, level, policy, length, limit, expected in (
        ("dither_build", 28963, 2, 50, 728, 636),
        ("dither_build_var", 65184, 0, 32, 1149, None)):
    emu = Emu(args.elf)
    dst, opts, arr = 0x20004000, 0x20004300, 0x20004400
    emu.poke(opts, struct.pack("<iHxxiHxx", 2, 655, policy, 29))
    if function == "dither_build":
        emu.call(function, [level, 1440, length, opts, 3, dst, 3])
    else:
        emu.call("stagger_spread_arr", [5760, length, arr])
        emu.call(function, [level, 5760, arr, length, 12, opts, 3, dst, 3, 0])
    cycles = emu.cycles()
    table = struct.unpack("<150H", emu.peek(dst, 300))
    if function == "dither_build_var":
        periods = struct.unpack("<32H", emu.peek(arr, 64))
        correct = all(table[3 * i] == (level * (periods[i] + 1) + 32767) // 65535 for i in range(length))
    else:
        correct = all(table[3 * i] == expected for i in range(length))
    print(f"{function}: {cycles} estimated cycles, cap {limit}, correct table {correct}")
    if cycles > limit or not correct:
        if not correct:
            print("active entries:", table[0:3 * length:3])
        print("FAIL target timing/output contract:", function)
        failures += 1
if args.frx_elf:
    emu = Emu(args.frx_elf)
    rx, ws, bits = 0x20001000, 0x20002800, 0x20002900
    slot = bytearray(32)
    crc = 0xFFFF
    for byte in slot[:30]:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    slot[30], slot[31] = crc >> 8, crc & 255
    emu.poke(bits, slot)
    emu.call("frx_init", [rx, 3, 0])
    emu.call("frx_capture", [rx, 7, 4, bits, 256, 256, 0, ws])
    cycles = emu.cycles()
    correct = emu.ret == 0 and emu.peek(rx + 20, 1) == b"\x01"
    print(f"frx_capture (LTO=0): {cycles} estimated cycles, cap 1057, valid Ultra capture {correct}")
    if cycles > 1057 or not correct:
        print("FAIL target timing/output contract: frx_capture")
        failures += 1
raise SystemExit(bool(failures))

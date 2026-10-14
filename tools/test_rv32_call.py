# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Verify Emu.call's RV32 ABI with actual nine/ten-argument PWM calls.

python tools/test_rv32_call.py <MEASURE=1 fw.elf> [--emulator <rv32emu.py>]
The optional emulator path permits a negative control against the unchanged base.
"""
import argparse
import importlib.util
from pathlib import Path
import struct
import sys

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("elf")
p.add_argument("--emulator", default=str(Path(__file__).with_name("rv32emu.py")))
a = p.parse_args()
sys.dont_write_bytecode = True  # A supplied comparison emulator may be in a read-only base tree.
spec = importlib.util.spec_from_file_location("review_emu", a.emulator)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
failures = 0
for stride, right_aligned in ((3, 0), (5, 1)):
    e = module.Emu(a.elf)
    dst, opts, arr = 0x20004000, 0x20004300, 0x20004400
    e.poke(dst, struct.pack("<64H", *([0xA55A] * 64)))
    e.poke(opts, struct.pack("<iHxxiHxx", 2, 655, 2, 29))
    e.poke(arr, struct.pack("<4H", *([799] * 4)))
    e.call("dither_build_var", [16384, 800, arr, 4, 1, opts, 0, dst, stride, right_aligned])
    table = struct.unpack("<64H", e.peek(dst, 128))
    expected = 600 if right_aligned else 200
    passed = all(v == (expected if i in (0, stride, 2 * stride, 3 * stride) else 0xA55A)
                 for i, v in enumerate(table))
    print(f"RV32 stack arguments: stride={stride}, right_aligned={right_aligned}: {'pass' if passed else 'FAIL'}")
    failures += not passed
raise SystemExit(bool(failures))

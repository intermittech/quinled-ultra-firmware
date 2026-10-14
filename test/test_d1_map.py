# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""(0.6.5, D1) the reset breadcrumbs on the target: where the block is, and the fault handler.

    python test/test_d1_map.py <app fw.elf> <its fw.map> <bootloader bl.map>

- The app's .noinit (main.c bcrumb) lies outside the update hand-over block (0x20000000-0x2000001F) and outside the
  RAM the bootloader uses (bootloader/bl.ld: its .data / .bss / .noinit buffers up to _end_noinit, its stack from
  _susrstack), so a watchdog or software reset through the bootloader keeps it; and below the app's own stack.
- isr.S trap_entry (mtvec, unified mode: no vector table since 0.6.5) sends an exception to its HardFault_Handler, and
  that stores the trap's cause and PC at bcrumb + 12 / + 8 (run in tools/rv32emu.py: the CSR reads are no-ops there,
  so it stores the registers' 0; the words around stay).
"""
import os
import re
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from rv32emu import Emu  # noqa: E402

elf, app_map, bl_map = sys.argv[1:4]
fails = 0


def check(ok, what):
    global fails
    print(("ok    " if ok else "FAIL  ") + what)
    fails += not ok


def sym(text, name):
    m = re.search(r"^\s+0x([0-9a-f]+)\s+(?:PROVIDE \()?" + re.escape(name) + r"\b", text, re.M)
    return int(m.group(1), 16) if m else None


app = open(app_map, encoding="utf-8", errors="replace").read()
bl = open(bl_map, encoding="utf-8", errors="replace").read()
m = re.search(r"^\.noinit\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)", app, re.M)
check(m is not None, "the app has a .noinit section")
noinit, size = (int(m.group(1), 16), int(m.group(2), 16)) if m else (0, 0)
bl_end, bl_stack = sym(bl, "_end_noinit"), sym(bl, "_susrstack")
app_stack = sym(app, "_susrstack")
print("app .noinit 0x%08x + %u, bootloader RAM up to 0x%08x, its stack from 0x%08x, app stack from 0x%08x" %
      (noinit, size, bl_end or 0, bl_stack or 0, app_stack or 0))
check(noinit >= 0x20000020, "after the hand-over block")
check(bl_end is not None and bl_stack is not None and bl_end <= noinit and noinit + size <= bl_stack,
      "outside the bootloader's buffers and stack")
check(app_stack is not None and noinit + size <= app_stack, "below the app's stack (stack_paint stays clear of it)")
check(size >= 16, "holds bcrumb_t (%u bytes)" % size)

e = Emu(elf)
bc = e.addr("bcrumb")
check(bc == noinit, "bcrumb is the .noinit block (0x%08x)" % bc)
hf = e.addr("HardFault_Handler")
x, pc = [0] * 32, e.addr("trap_entry")
x[5] = 2                                                 # mcause 2, illegal instruction (csrr: a no-op, t0 kept)
for _ in range(12):
    if pc == hf:
        break
    h = e.rd(pc, 2)
    pc, _ = e._c(h, pc, x) if h & 3 != 3 else e._i(e.rd(pc, 4), pc, x)
    x[0] = 0
check(pc == hf, "an exception: trap_entry -> isr.S HardFault_Handler (0x%x)" % hf)
e.poke(bc, b"\xA5" * 16)
try:
    e.call("HardFault_Handler", max_steps=40)
    check(False, "the handler waits for the watchdog")
except RuntimeError:
    check(True, "the handler waits for the watchdog")
words = struct.unpack("<4I", e.peek(bc, 16))
check(words == (0xA5A5A5A5, 0xA5A5A5A5, 0, 0), "cause and PC stored at +12 / +8, magic and uptime kept (%s)" %
      " ".join("%08x" % w for w in words))
print("D1 map / fault handler: %s (%d failed)" % ("passed" if not fails else "FAILED", fails))
sys.exit(1 if fails else 0)

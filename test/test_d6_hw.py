# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""ADC freshness and DMA fault counters (mcu/ch32v20x/hw.c hw_adc_ms / hw_dma_check, mcu/ch32v20x/hw.h hw_diag), run in
tools/rv32emu.py on an LTO=0 image (the functions keep their names there):

    python test/test_d6_hw.py <LTO=0 fw.elf>

- hw_adc_ms (the poll tick, once a ms): STRT clear = no regular PA7 conversion started since the last ms -> hw_diag[0];
  JEOC clear while a sequence runs -> hw_diag[1] (its values not taken); healthy (STRT | JEOC): nothing counted, the
  values taken; STATR is written with only JEOC and STRT cleared (rc_w0: ~0x14), before the next sequence starts.
- hw_dma_check (the main loop, once a second): each TEIFx counts on hw_diag[1 + x], INTFCR gets exactly the TEIF bits
  read (no other channel's flags cleared).
- The wiring, a source contract (the host models stub hw.c): main.c's temp_service calls hw_dma_check, the status
  snapshot points at hw_diag.
"""
import os
import re
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from rv32emu import Emu  # noqa: E402

ADC1_STATR, ADC1_IDATAR1 = 0x40012400, 0x4001243C
DMA1_INTFR, DMA1_INTFCR = 0x40020000, 0x40020004
fails = 0


def check(ok, what):
    global fails
    print(("ok    " if ok else "FAIL  ") + what)
    fails += not ok


e = Emu(sys.argv[1])
diag = e.addr("hw_diag")


def counts():
    return struct.unpack("<9H", e.peek(diag, 18))


e.wr(e.addr("adc_ready"), 1, 1)
for a in range(ADC1_IDATAR1, ADC1_IDATAR1 + 16, 4):
    e.mmio[a] = 1000
cases = [  # (inj_run, STATR, expected [0], [1] increments, values taken)
    (1, 0x14, 0, 0, True, "healthy: a regular conversion started, the sequence done"),
    (1, 0x04, 1, 0, True, "STRT clear: the current sense stale"),
    (1, 0x10, 0, 1, False, "JEOC clear: the sequence not done"),
    (1, 0x00, 1, 1, False, "both"),
    (0, 0x10, 0, 0, False, "no sequence ran yet (boot): only STRT checked"),
    (0, 0x00, 1, 0, False, "no sequence, STRT clear"),
]
for inj, sr, d0, d1, taken, what in cases:
    e.wr(e.addr("inj_run"), 1, inj)
    e.mmio[ADC1_STATR] = sr
    n0 = e.rd(e.addr("vs_n"), 4)
    c0 = counts()
    e.writes.clear()
    e.call("hw_adc_ms")
    c1 = counts()
    w = [v for a, v in e.writes if a == ADC1_STATR]
    check(c1[0] - c0[0] == d0 and c1[1] - c0[1] == d1 and (e.rd(e.addr("vs_n"), 4) - n0 == 1) == taken and
          w == [0xFFFFFFEB], "%s (diag +%d +%d, STATR <- %s)" % (what, c1[0] - c0[0], c1[1] - c0[1],
                                                                  " ".join("%08x" % v for v in w)))
e.wr(diag, 2, 0xFFFF)
e.wr(e.addr("inj_run"), 1, 1)
e.mmio[ADC1_STATR] = 0x04
e.call("hw_adc_ms")
check(counts()[0] == 0, "the counter wraps (u16)")

e.wr(diag, 18, 0)
e.mmio[DMA1_INTFR] = 0x0F00F00F | 0x00000880                 # every flag of ch1 / ch4 / ch7, TEIF2 + TEIF3 (bits 7, 11)
e.writes.clear()
e.call("hw_dma_check")
c = counts()
w = [v for a, v in e.writes if a == DMA1_INTFCR]
check(c[2:] == (1, 1, 1, 1, 0, 0, 1), "TEIF1/2/3/4/7 counted per channel (%s)" % (c[2:],))
check(w == [(0x0F00F00F | 0x00000880) & 0x08888888], "INTFCR clears exactly the TEIF bits read (%s)" %
      " ".join("%08x" % v for v in w))
e.mmio[DMA1_INTFR] = 0x00000000
e.call("hw_dma_check")
check(counts()[2:] == c[2:], "no flags: nothing counted")

def unity(path):
    """the file's text with each local #include "x.c" in place (src/app/main.c's units)"""
    with open(path, encoding="utf-8") as f:
        return re.sub(r'^#include "(\w+\.c)"$', lambda m: unity(os.path.join(os.path.dirname(path), m.group(1))),
                      f.read(), flags=re.M)


src = unity(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "app", "main.c"))
m = re.search(r"static void temp_service\(void\)\s*\{(.*?)\n\}", src, re.S)
check(m is not None and "hw_dma_check();" in m.group(1), "main.c: temp_service calls hw_dma_check (once a second)")
check("stat.hwd = hw_diag;" in src, "main.c: the status points at hw_diag")
print("D6 ADC / DMA counters: %s (%d failed)" % ("passed" if not fails else "FAILED", fails))
sys.exit(1 if fails else 0)

# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""the main loop's render step, per call, racing the beam: the longest and the mean (cycles) for each effect: python test/measure_steps.py [effect ...]"""
import os
import sys
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_ud_fx_cycles as T
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from rv32emu import Emu

NPX = int(os.environ.get("NPX", "300"))


def main():
    elf = os.path.join(tempfile.mkdtemp(), "ud_fx_bench.elf")
    T.build(elf)
    e = Emu(elf)

    def cyc(fn, args=()):
        e.call(fn, list(args))
        return e.cycles(), e.ret
    nfx = cyc("b_fx_count")[1]
    fxs = [int(a) for a in sys.argv[1:]] or list(range(1, nfx))
    for fx in fxs:
        cyc("b_setup", [0, NPX, fx, 0])
        cyc("b_race_reset")
        steps = []
        for k in range(1500):
            cyc("b_race_tick")
            while True:
                c, more = cyc("b_render_once")
                if k >= 300:
                    steps.append(c)
                if not more:
                    break
        work = [c for c in steps if c > 200]
        print("effect %2d: %5d render steps, longest %6d, mean %6.0f cycles" % (fx, len(work), max(work) if work else 0, sum(work) / max(len(work), 1)))


main()

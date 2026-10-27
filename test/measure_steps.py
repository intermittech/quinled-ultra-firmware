# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""the main loop's render step, per call, racing the beam: the longest and the mean (cycles) for each effect: python test/measure_steps.py [effect ...]"""
import os
import sys
import tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_ur_digital_m_fx_cycles as T
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from rv32emu import Emu

NPX = int(os.environ.get("NPX", "300"))


def fire_cases(e):
    """Maximum udfx_render latency, including commands, prep and four-step catch-up."""
    def call(fn, *args):
        e.call(fn, list(args))
        return e.cycles(), e.ret
    worst = 0
    for chip, npx in ((0, 600), (3, 500), (5, 400)):
        for mode in range(6):
            call('b_setup', chip, npx, 4 if mode == 2 else 16, 60)
            call('b_race_reset')
            peak = prep = 0
            for k in range(2400):
                call('b_race_tick')
                if k == 400:
                    if mode == 5:
                        call('b_fire_four')
                        call('b_scene_at', 1000)
                    elif mode:
                        call('b_fire_change', mode)
                    else:
                        call('b_scene_at', 1000)       # four-step catch-up without a command
                while True:
                    _, preparing = call('b_preparing')
                    c, more = call('b_render_once')
                    peak = max(peak, c)
                    if preparing:
                        prep = max(prep, c)
                    if not more:
                        break
            print(f'Fire chip={chip} px={npx} fade={mode}: prep max {prep}, all steps max {peak} cycles', flush=True)
            worst = max(worst, peak)
    assert worst <= 8000, f'Fire main-loop step {worst} > 8000 cycles'


def main():
    elf = os.path.join(tempfile.mkdtemp(), "ud_fx_bench.elf")
    T.build(elf)
    e = Emu(elf)
    if '--fire' in sys.argv:
        fire_cases(e)
        return

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

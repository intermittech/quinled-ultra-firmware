# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The hub-locked HSI trim (src/core/hsitrim.c, CH32V40x) against a plant model, compiled for RV32 and run in rv32emu:
factory HSI error -2.2..+2.2 %, the boot trim 12 steps low, true step 1800..2500 ppm (the boot trim needs >= 1794 ppm a step for +2.2 %) (either sign of the nominal
direction is not modelled: the boot trim presumes RM p.23's ordering, a silicon check), measurement noise +-60 ppm.
Checks: the true clock never exceeds nominal after the first measurement, and the controller settles within one
step below nominal (at most step + margin + hysteresis) in a few measurements.

    python test/test_hsitrim_v407.py <out dir>
"""
import json
import pathlib
import random
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'tools'))
sys.path.insert(0, str(FW / 'test'))
from rv32emu import Emu                      # noqa: E402
from ur_digital_m_target import target_gcc   # noqa: E402

BENCH = r'''
#include "core/hsitrim.h"
hsitrim_t t;
void b_init(unsigned trim) { hsitrim_init(&t, trim, 2000); }
unsigned b_update(int err) { return hsitrim_update(&t, err); }
unsigned b_locked(void) { return t.locked; }
'''


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    src = out / 'hsitrim_bench.c'
    src.write_text(BENCH)
    elf = out / 'hsitrim.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-nostdlib', '-I', str(FW / 'src'),
                    '-Wl,-e,b_init', '-T', str(FW / 'test/fx_rv.ld'), str(src), str(FW / 'src/core/hsitrim.c'),
                    '-o', str(elf)], check=True, timeout=120)
    e = Emu(str(elf))
    rng = random.Random(407)
    worst_final, worst_iters, cases = 0, 0, 0
    for factory in [x / 1000 for x in range(-22, 23, 2)]:
        for step in (1800, 1900, 2000, 2200, 2500):   # 12 x step must exceed 2.2 % / 1.022: step >= 1794 ppm
            for trial in range(4):
                trim = 16 - 12
                e.call('b_init', [trim])
                f = lambda tr: (1 + factory) * (1 + step * (tr - 16) / 1e6)
                assert f(trim) < 1.0, ('boot above nominal', factory, step)
                settled = 0
                for it in range(40):
                    err = (f(trim) - 1) * 1e6 + rng.uniform(-60, 60)
                    e.call('b_update', [int(round(err)) & 0xffffffff])
                    trim = e.ret
                    assert f(trim) <= 1.0 - 50e-6, ('above nominal', factory, step, it, trim, f(trim))
                    e.call('b_locked')
                    settled = settled + 1 if e.ret else 0
                    if settled >= 3:
                        break
                final = (1 - f(trim)) * 1e6
                assert settled >= 3 and final <= step + 150 + 100 + 150, ("not settled", factory, step, final)
                worst_final = max(worst_final, final - step)
                worst_iters = max(worst_iters, it + 1)
                cases += 1
    result = dict(cases=cases, worst_final_minus_step_ppm=round(worst_final, 1), worst_measurements=worst_iters)
    (out / 'hsitrim_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()

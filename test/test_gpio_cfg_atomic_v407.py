# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 GPIO configuration words are shared across priority levels: PA9 (the answer UART: the USART7 handler releases
it, the tick drives it for an answer) and PA15 (the ARGB output: the tick switches it per frame) sit in GPIOA's CFGHR.
Every runtime change of a pin's nibble must be atomic against every other one, or a preempted read-modify-write puts a
stale nibble back (PA9 driving a released relay line).

The production routines (hw.c, hw_dline.h) are compiled for RV32 and run in rv32emu with mstatus.MIE modeled: the
higher-priority routine is injected before every instruction of the lower one, as soon as MIE allows it (a masked
interrupt stays pending until MIE returns), for every pair of writers. Both pins must end as their last writer set them.

    python test/test_gpio_cfg_atomic_v407.py <out dir> [--src <firmware tree>]
"""
import argparse
import json
import pathlib
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
from ur_digital_m_target import target_gcc   # noqa: E402

BENCH = r'''
#include "mcu/ch32v40x/hw_dline.h"
#include "mcu/ch32v40x/hw.h"
uint8_t buf[4];
void b_init(void) { }
void b_argb_start(void) { argb_hw_start(buf, 3u, 250u, 70u, 135u); }
void b_argb_stop(void) { argb_hw_stop(); }
void b_release(void) { rc_hw_dir(RC_DIR_FORWARD); }
void b_answer(void) { rc_hw_dir(RC_DIR_ANSWER); }
void b_answer_local(void) { rc_hw_answer_local(); }
'''
CFGHR = 0x40010804
PA9_SH, PA15_SH = 4, 28
# the nibble each routine leaves its pin with
SETS = {'b_argb_start': (PA15_SH, 0xB), 'b_argb_stop': (PA15_SH, 0x3), 'b_release': (PA9_SH, 0x4),
        'b_answer': (PA9_SH, 0xB), 'b_answer_local': (PA9_SH, 0xB)}
PAIRS = [(lo, hi) for lo in ('b_argb_start', 'b_argb_stop') for hi in ('b_release', 'b_answer')] + \
        [(lo, hi) for lo in ('b_release', 'b_answer', 'b_answer_local') for hi in ('b_argb_start', 'b_argb_stop')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    import rv32emu as rv
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'gpio_bench.c').write_text(BENCH)
    elf = a.out / 'gpio_bench.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-fno-common',
                    '-ffunction-sections', '-fdata-sections', '-nostdlib', '-DBOARD=3', '-DMCU_CH32V40X=1',
                    '-I', str(src / 'src'), '-I', str(src / 'src/core'),
                    '-I', str(src / 'src/mcu/ch32v40x'), '-Wl,--gc-sections', '-Wl,-e,b_init',
                    '-T', str(src / 'test/fx_rv.ld'), str(a.out / 'gpio_bench.c'), str(src / 'src/mcu/ch32v40x/hw.c'),
                    '-o', str(elf)], check=True, timeout=120)

    class Core(rv.Emu):
        def __init__(self, path):
            super().__init__(path)
            self.mstatus = 0x8                                  # MIE: a level-1 handler with nesting on, or main

        def wr(self, addr, n, v):
            super().wr(addr, n, v)
            if self._mem(addr, n)[0] is None:
                self.mmio[addr] = v & ((1 << (8 * n)) - 1)

        def _i(self, ins, pc, x):
            if ins & 0x7f == 0x73 and (ins >> 12) & 7:
                f3, num, src_reg = (ins >> 12) & 7, ins >> 20, (ins >> 15) & 31
                operand = src_reg if f3 & 4 else x[src_reg]
                old = self.mstatus if num == 0x300 else 0
                if num == 0x300 and (f3 & 3 == 1 or src_reg):
                    self.mstatus = {1: operand, 2: old | operand, 3: old & ~operand}[f3 & 3] & 0xffffffff
                x[(ins >> 7) & 31] = old
                return pc + 4, 'alu'
            return super()._i(ins, pc, x)

        def run(self, fn, sp, inject=None):
            """fn to its return; inject = (step, handler): the handler runs before that step once MIE is set"""
            x = [0] * 32
            x[1], x[2], x[3] = rv.RET_MAGIC, sp, self.gp
            pc, n, pending = self.addr(fn), 0, False
            while pc != rv.RET_MAGIC:
                if inject and n == inject[0]:
                    pending = True
                if pending and self.mstatus & 8:
                    pending = False
                    saved = self.mstatus
                    self.mstatus = 0                            # the deepest level: MIE off (QK3 2.2)
                    self.run(inject[1], sp - 512)
                    self.mstatus = saved                        # mret: MIE from MPIE
                h = self.rd(pc, 2)
                pc, _ = self._c(h, pc, x) if h & 3 != 3 else self._i(self.rd(pc, 4), pc, x)
                x[0] = 0
                n += 1
                if n > 100000:
                    raise RuntimeError('step limit')
            if inject and (pending or n <= inject[0]):          # masked to the end, or due at the return: taken then
                self.run(inject[1], sp - 512)
            return n

    sp = rv.RAM_BASE + rv.RAM_SIZE - 64
    bad, cases = [], 0
    for lo, hi in PAIRS:
        steps = Core(str(elf)).run(lo, sp)
        for k in range(steps + 1):
            e = Core(str(elf))
            cfg = 0x44444443                                    # PA8 output, PA9..PA15 inputs
            for f in (lo, hi):                                  # each pin starts opposite to its writer's value
                sh, v = SETS[f]
                cfg = (cfg & ~(0xF << sh)) | ((0x3 if sh == PA15_SH else 0x4) if v == 0xB else 0xB) << sh
            e.mmio[CFGHR] = cfg
            before = e.mmio[CFGHR]
            e.run(lo, sp, (k, hi))
            got = e.mmio[CFGHR]
            want = before
            for f in (lo, hi):
                sh, v = SETS[f]
                want = (want & ~(0xF << sh)) | (v << sh)
            cases += 1
            if got != want:
                bad.append(dict(low=lo, high=hi, step=k, got=hex(got), want=hex(want)))
    result = dict(cases=cases, failures=len(bad), first=bad[:4])
    (a.out / 'gpio_cfg_atomic.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    if bad:
        sys.exit(1)


if __name__ == '__main__':
    main()

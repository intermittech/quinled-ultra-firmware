# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407's temperature sample against DS p.84 (17.1 us) under both readings of the ADC clock (RM p.25: ADCPRE
"PCLK2 divided by" = 100 / 8 = 12.5 MHz; CFGR0.ADCSRC = 0 "HCLK" = 200 / 8 = 25 MHz). hw.c (hw_adc_init, hw_temp_start,
hw_adc_ms, hw_temp_read) is compiled for RV32 and run in rv32emu against an ADC and SysTick model whose conversion of
252 ADC cycles takes 4,032 HCLK cycles (12.5 MHz) or 2,016 (25 MHz): the temperature may only be reported when its
239.5-cycle sample is >= 17.1 us (12.5 MHz: 19.2 us), never at 25 MHz (9.58 us). Also: SMP16 / SMP17 = 239.5 cycles.

    python test/test_adc_clock_v407.py <out dir> [--src <firmware tree>]
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
#include "mcu/ch32v40x/hw.h"
void b_init(void) { }
void b_adc_init(void) { hw_adc_init(); }
void b_temp_start(void) { hw_temp_start(); }
unsigned b_ms(void) { return hw_adc_ms(); }
unsigned b_temp(void) { uint32_t a, b; return hw_temp_read(&a, &b); }
'''
ADC = 0x40012400
STATR, CTLR2, SAMPTR1 = ADC, ADC + 0x08, ADC + 0x0C
STK_CNT, STK_CMP = 0xE000F008, 0xE000F010
JEOC, JSWSTART, RSTCAL, CAL = 1 << 2, 1 << 21, 1 << 3, 1 << 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    import rv32emu as rv
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'adc_bench.c').write_text(BENCH)
    elf = a.out / 'adc_bench.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-fno-common',
                    '-ffunction-sections', '-fdata-sections', '-nostdlib', '-DBOARD=3', '-DMCU_CH32V40X=1',
                    '-I', str(src / 'src'), '-I', str(src / 'src/core'),
                    '-I', str(src / 'src/mcu/ch32v40x'), '-Wl,--gc-sections', '-Wl,-e,b_init',
                    '-T', str(src / 'test/fx_rv.ld'), str(a.out / 'adc_bench.c'), str(src / 'src/mcu/ch32v40x/hw.c'),
                    '-o', str(elf)], check=True, timeout=120)

    class Board(rv.Emu):
        conv = 4032

        def __init__(self, path):
            super().__init__(path)
            self.t, self.start = 0, None
            self.mmio.update({STK_CMP: 9999, STATR: 0})

        def _i(self, ins, pc, x):
            self.t += 1                                         # (one HCLK cycle per instruction: enough here)
            return super()._i(ins, pc, x)

        def _c(self, h, pc, x):
            self.t += 1
            return super()._c(h, pc, x)

        def rd(self, addr, n):
            if addr == STK_CNT:
                return self.t % 10000
            if addr == STATR:
                done = self.start is not None and self.t - self.start >= self.conv
                return self.mmio.get(STATR, 0) | (JEOC if done else 0)
            if addr == CTLR2:
                return self.mmio.get(CTLR2, 0) & ~(RSTCAL | CAL)  # calibrations done
            return super().rd(addr, n)

        def wr(self, addr, n, v):
            super().wr(addr, n, v)
            if self._mem(addr, n)[0] is None:
                if addr == STATR:                               # RW0
                    if not v & JEOC:
                        self.start = None
                    self.mmio[STATR] = self.mmio.get(STATR, 0) & v & ~JEOC
                    return
                self.mmio[addr] = v
                if addr == CTLR2 and v & JSWSTART:
                    self.start = self.t

    rows, bad = {}, []
    for name, conv, allowed in (('12.5 MHz (PCLK2 / 8)', 4032, True), ('25 MHz (HCLK / 8)', 2016, False)):
        e = Board(str(elf))
        e.conv = conv
        e.call('b_adc_init')
        smp = (e.mmio.get(SAMPTR1, 0) >> 18) & 7, (e.mmio.get(SAMPTR1, 0) >> 21) & 7
        e.call('b_temp_start')
        got = 0
        for k in range(4):                                      # schedule, convert, collect
            e.call('b_ms')
            e.t += 6000
            e.call('b_temp')
            got |= e.ret
        sample_us = 239.5 / (252 * 200 / conv)
        rows[name] = dict(sample_us=round(sample_us, 2), temperature_reported=bool(got), smp16_17=smp)
        if bool(got) != allowed or smp != (7, 7):
            bad.append(name)
    result = dict(rows=rows, failures=bad)
    (a.out / 'adc_clock_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

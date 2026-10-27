# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 delay line's tick cost: core/dline.c compiled for the family (-DMCU_CH32V40X=1, the firmware's -Os) and run
in rv32emu, one dline_tick per 50 us = 156 new ring words at 50 MHz, on line content that maximises the scan: 800 kHz
pixel data (edges in every word), a marker frame (zeroing + capture), the chain scanner on (its raw copy too), late
ticks (2 and 3 ticks of words at once). The modeled cycles use rv32emu's V4B-derived cost table: the QingKe V3V has no
published cycle table (static prediction only), so these are model numbers to compare with the 10,000-cycle tick at
200 MHz, not a silicon certificate.

    python test/test_dline_cycles_v407.py <out dir>
"""
import json
import pathlib
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'tools'))
sys.path.insert(0, str(FW / 'test'))
from rv32emu import Emu                      # noqa: E402
from ur_digital_m_target import target_gcc   # noqa: E402

BENCH = r'''
#include "core/dline.h"
dline_t d;
uint16_t ring[512 + 64] __attribute__((aligned(4)));
uint16_t cap[DLINE_CAP_WORDS] __attribute__((aligned(4)));
uint32_t acts;
void b_init(unsigned chain) { dline_init(&d, 0, cap); d.rec = 0; d.direct = 0; d.bl = 0; if (chain) { d.ch_on = 1; d.cpos = 0; } }
void b_tick(unsigned rx, unsigned edges) { acts |= dline_tick(&d, ring, rx & 511u, (rx + 2u) & 511u, edges); }
'''


def pixel_word(k):
    # 800 kHz WS2812 bits at 50 MHz: 62.5 samples a bit, '1' 40 HIGH / '0' 20 HIGH: edges in every word
    out = 0
    for b in range(16):
        s = k * 16 + b
        pos = s % 125
        bit_high = 40 if (s // 125) % 2 else 20
        hi = (pos % 62.5) < bit_high
        out |= (0x8000 >> b) if hi else 0
    return out


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    (out / 'dline_cyc_bench.c').write_text(BENCH)
    elf = out / 'dline_cyc.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-DBOARD=3', '-DMCU_CH32V40X=1',
                    '-fno-builtin', '-fno-common', '-nostdlib', '-msmall-data-limit=400', '-DDLINE_RAMCODE=',
                    '-I', str(FW / 'src'), '-Wl,-e,b_init', '-T', str(FW / 'test/fx_rv.ld'),
                    str(out / 'dline_cyc_bench.c'), str(FW / 'src/core/dline.c'), '-o', str(elf)], check=True, timeout=120)
    e = Emu(str(elf))
    ring = e.addr('ring')
    rows = {}
    for name, chain, step in (('pixels', 0, 156), ('pixels+chain', 1, 156), ('pixels late x2', 0, 312),
                              ('pixels late x3', 0, 468), ('marker+frame', 0, 156), ('chain frames', 1, 156)):
        e.call('b_init', [chain])
        words = [pixel_word(k) for k in range(512)]
        if name == 'marker+frame':
            words = [0] * 160 + [0xffff] * 60 + [0] * 4 + [pixel_word(k) for k in range(288)]
        if name == 'chain frames':
            words = [0] * 200 + [pixel_word(k) for k in range(250)] + [0] * 62
        e.poke(ring, struct.pack('<512H', *words))
        rx, peak, edges = 0, 0, 0
        for t in range(60):
            rx += step
            edges += step * 2
            # the ring holds the line: rewrite the words RX wrote since the last tick (as the DMA would)
            e.call('b_tick', [rx, edges & 0xffff])
            peak = max(peak, e.cycles())
            e.poke(ring, struct.pack('<512H', *words))
        rows[name] = peak
    budget = 10000
    result = dict(model='rv32emu (V4B cost table)', tick_cycles_200mhz=budget, dline_tick_peak=rows,
                  worst_fraction=round(max(rows.values()) / budget, 3))
    (out / 'dline_cycles_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

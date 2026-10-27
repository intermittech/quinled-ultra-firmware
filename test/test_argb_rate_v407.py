# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compile actual chip presets and the encoder; measure its free-running frame ceiling.

These frame lengths are timing scenarios, not new buffer/admission limits.
Delay-line replay is deliberately outside the local symbol table.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
FW = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(FW/'test'), str(FW/'tools')]
from test_argbenc_v407 import MODEL
from ur_digital_m_target import target_gcc
from rv32emu import Emu

BENCH = r'''
#include "core/ur_digital_m/udfx.c"
volatile uint32_t m_now, m_start, m_n, m_per, m_t0, m_t1, m_starts, m_stops, m_running, m_hang;
ws_enc_t e;
ws_lut_t L;
ws_chip_t c;
uint8_t fb[2500];
void b_init(unsigned id, unsigned npx)
{
 const ux_chip_t *p = &ux_chips[id];
 c.s=p->s; c.t0h=p->t0h; c.t1h=p->t1h; c.bpp=p->bpp; c.reset_w=p->reset_w;
 ws_argb_q16 = 2u*65536u;
 ws_lut_build(&L, &c); m_now=m_starts=m_running=m_hang=0;
 ws_enc_init(&e, &c, fb, npx, 0, 0);
}
void b_fill(unsigned w) { m_now=w; e.ready=1; ws_ring_fill(&e, &L, 0, w); }
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=Path)
    ap.add_argument('--src', type=Path, default=FW)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out/'model.h').write_text(MODEL)
    (a.out/'rate.c').write_text(BENCH)
    elf = a.out/'rate.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin',
                    '-ffunction-sections', '-fdata-sections', '-nostdlib', '-DMCU_CH32V40X=1', '-DBOARD=3',
                    f'-DWS_ARGB_HW="{(a.out/"model.h").as_posix()}"', '-DWS_RAMCODE=',
                    '-I', str(a.src/'src'), '-I', str(a.src/'src/core'), 
                    '-Wl,-e,b_init', '-Wl,--gc-sections', '-T', str(FW/'test/fx_rv.ld'),
                    str(a.out/'rate.c'), str(a.src/'src/core/ur_digital_m/argbenc.c'), '-o', str(elf)],
                   check=True, timeout=120)
    e = Emu(str(elf))
    def val(n):
        return struct.unpack('<I', e.peek(e.addr(n), 4))[0]
    rows = []
    for chip, npx, bpp, lo, hi in ((0,800,3,40.8,41.3), (3,600,4,40.8,41.5), (5,500,5,39.2,39.7)):
        e.call('b_init', [chip, npx])
        starts, prev = [], 0
        for tick in range(20000):
            e.call('b_fill', [tick*10000//64])
            now = val('m_starts')
            if now != prev:
                starts.append(val('m_start')*0.32)
                prev = now
                if now == 32:
                    break
        fps = (len(starts)-1)*1e6/(starts[-1]-starts[0])
        bit_ns, one_ns = val('m_per')*5, val('m_t1')*5
        ok = bit_ns == 1250 and lo <= fps <= hi
        if chip in (0,5):
            ok = ok and one_ns == 650 and bit_ns-one_ns >= 580
        rows.append(dict(chip=chip, pixels=npx, bpp=bpp, bit_ns=bit_ns, one_ns=one_ns, fps=fps, ok=ok))
    board = json.loads((a.src/'src/board/ur_digital_m/board.json').read_text())
    px = board['bindings']['ch32v40x']['overrides']['descriptor']['pixel']
    descriptor_ok = px.get('bit_ns', board['descriptor']['pixel'][0]['bit_ns']) == 1250
    result = dict(rows=rows, descriptor_ok=descriptor_ok)
    (a.out/'rate.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    return int(not descriptor_ok or not all(r['ok'] for r in rows))


if __name__ == '__main__':
    sys.exit(main())

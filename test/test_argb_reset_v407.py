# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 ARGB output's reset (latch) LOW, on the hardware's clock rather than the tick's: the encoder
(src/core/ur_digital_m/argbenc.c) compiled for RV32 and run in rv32emu against a serialiser model with an explicit
time in HCLK cycles. Each tick's snapshot (limit, the RX word count read before the scan) is older than the moment the
encoder runs by a varying ISR time (0..34 us, the scan's worst; late ticks of 2 and 3 tick lengths too), so a frame
starts later than its snapshot. Checked on every tick, for reset lengths 944 words (302.08 us) and 250 words (80 us),
two frame lengths, the free and the gridded frame rate:
- the encoder's LOW count never exceeds the line's real LOW at the snapshot (what udfx's switch back to the replay
  and the next frame's start both rely on);
- every frame starts at least the reset after the real end of the frame before (the strip latches);
- a source handoff (ws_enc_init at a tick, the replay LOW from that snapshot) and a frame cut by a re-init both give
  the first frame after them the whole reset.
The profile minimums (tools/ur_digital_m_timing_check.py, e.g. 280 us) sit below the configured resets, so a whole
configured reset satisfies them.

    python test/test_argb_reset_v407.py <out dir> [--src <firmware tree>]
"""
import argparse
import json
import pathlib
import random
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
from ur_digital_m_target import target_gcc   # noqa: E402

MODEL = r'''
#include <stdint.h>
#define ARGB_LEAD_CYC 200u
extern volatile uint32_t m_t, m_start, m_end, m_n, m_per, m_starts, m_running, m_extra, m_fail;
static inline uint32_t argb_hw_now(uint32_t w) { (void)w; return m_t / 64u + 1u; }
static inline void argb_hw_start(const uint8_t *b, uint32_t n, uint32_t per, uint32_t t0, uint32_t t1)
{ (void)b; (void)t0; (void)t1; m_start = m_t; m_n = n; m_per = per; m_end = m_t + ARGB_LEAD_CYC + n * 8u * per + m_extra;
  m_starts++; m_running = 1; }
static inline uint32_t argb_hw_left(void)
{ uint32_t c = m_t - m_start, s = c <= ARGB_LEAD_CYC ? 0u : (c - ARGB_LEAD_CYC) / (8u * m_per) + 1u;
  return !m_running || s >= m_n ? 0u : m_n - s; }
static inline uint32_t argb_hw_done(void) { return m_running && (int32_t)(m_t - m_end) >= 0; }
#define ARGB_ST_RUN 0u
#define ARGB_ST_OK 1u
#define ARGB_ST_FAIL 2u
static inline uint32_t argb_hw_status(void) { return argb_hw_done() ? ARGB_ST_OK : ARGB_ST_RUN; }
static inline void argb_hw_stop(void)
{ if (m_running && (int32_t)(m_t - m_end) < 0) m_end = m_t;          /* a cut frame: LOW from the stop */
  m_running = 0; }
'''
BENCH = r'''
#include "core/ur_digital_m/wsenc.h"
volatile uint32_t m_t, m_start, m_end, m_n, m_per, m_starts, m_running, m_extra, m_fail;
ws_enc_t e;
ws_lut_t L;
ws_chip_t c;
uint8_t fb[2000];
uint32_t st[4];
void b_init(unsigned npx, unsigned reset_w, unsigned period_w, unsigned w0, unsigned t)
{ c.s = MCU_SYM_BIT; c.t0h = MCU_SYM_NS(361u); c.t1h = MCU_SYM_NS(694u); c.bpp = 3; c.reset_w = (uint16_t)reset_w;
  ws_lut_build(&L, &c); m_t = t; ws_enc_init(&e, &c, fb, npx, w0, period_w); }
void b_fill(unsigned limit, unsigned t)
{ m_t = t; (void)ws_ring_fill(&e, &L, 0, limit); st[0] = e.state; st[1] = e.low_w; st[2] = e.lost; st[3] = e.ready; }
void b_ready(void) { e.ready = 1; }
'''
TICK = 10000                                    # HCLK cycles per 50 us tick at 200 MHz
AGE_MAX = 6800                                  # 34 us of scan before the encoder runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    from rv32emu import Emu
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'argb_reset_model.h').write_text(MODEL)
    (a.out / 'argb_reset_bench.c').write_text(BENCH)
    elf = a.out / 'argb_reset.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-nostdlib',
                    '-DMCU_CH32V40X=1', '-DBOARD=3', f'-DWS_ARGB_HW="{(a.out / "argb_reset_model.h").as_posix()}"',
                    '-DWS_RAMCODE=', '-I', str(src / 'src'), '-Wl,-e,b_init', '-T', str(src / 'test/fx_rv.ld'),
                    str(a.out / 'argb_reset_bench.c'), str(src / 'src/core/ur_digital_m/argbenc.c'), '-o', str(elf)],
                   check=True, timeout=120)
    e = Emu(str(elf))

    def var(name, value=None):
        addr = e.addr(name)
        if value is not None:
            e.poke(addr, struct.pack('<I', value & 0xffffffff))
        return struct.unpack('<I', e.peek(addr, 4))[0]

    rng = random.Random(4072)
    bad, frames, ticks, short, kinds = [], 0, 0, None, {}
    for reset_w in (944, 250):
        for npx, period_w in ((3, 0), (100, 0), (3, 3125), (100, 3125 * 4)):
            for run in range(3):
                t_snap = 1_000_000 + rng.randrange(TICK)
                var('m_running', 0)
                var('m_extra', rng.choice((0, 0, 250)))             # (a serialiser one bit longer than computed)
                e.call('b_init', [npx, reset_w, period_w, t_snap // 64, t_snap + rng.randrange(AGE_MAX)])
                low_from = t_snap                                   # handoff: the replay LOW at least from here
                prev_starts, open_frame = var('m_starts'), False
                for k in range(1500):
                    t_snap += TICK if rng.random() > 0.08 else TICK * rng.choice((2, 3))
                    age = rng.choice((0, AGE_MAX, rng.randrange(AGE_MAX), rng.randrange(400)))
                    if rng.random() < 0.4:
                        e.call('b_ready')
                    if rng.random() < 0.002:                       # a re-init (pause / restart) at a random moment
                        running = var('m_running')
                        e.call('b_init', [npx, reset_w, period_w, t_snap // 64, t_snap + age])
                        if running:
                            low_from, open_frame = var('m_end'), False    # cut now: LOW from the stop
                        continue
                    e.call('b_fill', [t_snap // 64, t_snap + age])
                    st = struct.unpack('<4I', e.peek(e.addr('st'), 16))
                    ticks += 1
                    starts = var('m_starts')
                    if starts != prev_starts:
                        frames += 1
                        gap = var('m_start') - low_from
                        short = gap / 200 if short is None else min(short, gap / 200)
                        if gap < reset_w * 64:
                            bad.append(dict(kind='start', reset_w=reset_w, npx=npx, gap_us=gap / 200))
                        prev_starts, open_frame = starts, True
                    if not var('m_running'):
                        if open_frame:
                            low_from, open_frame = var('m_end'), False
                        if st[0] == 0 and st[1] != 0 and st[1] * 64 > t_snap - low_from:
                            bad.append(dict(kind='low_w', reset_w=reset_w, npx=npx, low_w=st[1],
                                            real_w=(t_snap - low_from) / 64))
    for b in bad:
        kinds[b['kind']] = kinds.get(b['kind'], 0) + 1
    result = dict(ticks=ticks, frames=frames, failures=len(bad), by_kind=kinds, shortest_reset_us=short, first=bad[:4])
    (a.out / 'argb_reset_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    if bad or frames < 100:
        sys.exit(1)


if __name__ == '__main__':
    main()

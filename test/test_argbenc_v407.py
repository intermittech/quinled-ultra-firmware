# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 local-effects encoder (src/core/ur_digital_m/argbenc.c) against a model of the ARGB serialiser, compiled
for RV32 (-DMCU_CH32V40X=1) and run in rv32emu with 50 us ticks of 156 ring words (plus late ticks). Checks: a frame
starts only with the renderer's frame ready, a whole reset of LOW after the previous frame's end and its grid word; it
starts at the first tick at or after that; no start while one runs; the end is the computed duration; the bytes the
DMA has read are released (e->i) in order; a serialiser that never ends its cycle counts the frame lost and restarts
with a full reset; the bit timing is the chip's 10 ns symbols at the clock estimate. Also the fill's peak modeled
cycles per tick (rv32emu's V4B cost table, the bench's state copy included): the encoder's share of the 200 MHz tick.

    python test/test_argbenc_v407.py <out dir>
"""
import json
import pathlib
import random
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'tools'))
sys.path.insert(0, str(FW / 'test'))
from rv32emu import Emu                      # noqa: E402
from ur_digital_m_target import target_gcc   # noqa: E402

MODEL = r'''
#include <stdint.h>
#define ARGB_LEAD_CYC 200u
extern volatile uint32_t m_now, m_start, m_n, m_per, m_t0, m_t1, m_starts, m_stops, m_running, m_hang;
static inline uint32_t argb_hw_now(uint32_t w) { (void)w; return m_now + 1u; }   /* (the read lands in the tick's word) */
static inline void argb_hw_start(const uint8_t *b, uint32_t n, uint32_t per, uint32_t t0, uint32_t t1)
{ (void)b; m_start = m_now; m_n = n; m_per = per; m_t0 = t0; m_t1 = t1; m_starts++; m_running = 1; }
static inline uint32_t m_cycles(void) { return (m_now - m_start) * 64u; }
static inline uint32_t argb_hw_left(void)
{ uint32_t c = m_cycles(), sent = c <= ARGB_LEAD_CYC ? 0u : (c - ARGB_LEAD_CYC) / (8u * m_per) + 1u;
  return sent >= m_n ? 0u : m_n - sent; }
static inline uint32_t argb_hw_done(void)
{ return m_running && !m_hang && m_cycles() >= ARGB_LEAD_CYC + m_n * 8u * m_per; }
static inline void argb_hw_stop(void) { if (m_running) m_stops++; m_running = 0; }
#define ARGB_ST_RUN 0u
#define ARGB_ST_OK 1u
#define ARGB_ST_FAIL 2u
static inline uint32_t argb_hw_status(void) { return argb_hw_done() ? ARGB_ST_OK : ARGB_ST_RUN; }
'''
BENCH = r'''
#include "core/ur_digital_m/wsenc.h"
volatile uint32_t m_now, m_start, m_n, m_per, m_t0, m_t1, m_starts, m_stops, m_running, m_hang;
ws_enc_t e;
ws_lut_t L;
ws_chip_t c;
uint8_t fb[2000];
uint32_t state[12];
void b_init(unsigned npx, unsigned w0, unsigned period_w)
{ c.s = MCU_SYM_BIT; c.t0h = MCU_SYM_NS(361u); c.t1h = MCU_SYM_NS(694u); c.bpp = 3; c.reset_w = 944;
  ws_lut_build(&L, &c); m_running = m_starts = m_stops = m_hang = 0; m_now = w0; ws_enc_init(&e, &c, fb, npx, w0, period_w); }
void b_ready(void) { e.ready = 1; }
void b_fill(unsigned limit)
{ m_now = limit; (void)ws_ring_fill(&e, &L, 0, limit);
  state[0] = e.state; state[1] = e.i; state[2] = e.low_w; state[3] = e.next_w; state[4] = e.start_w; state[5] = e.end_w;
  state[6] = e.frames; state[7] = e.lost; state[8] = m_starts; state[9] = m_per; state[10] = e.ready; state[11] = m_running; }
'''


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    (out / 'argb_model.h').write_text(MODEL)
    (out / 'argbenc_bench.c').write_text(BENCH)
    elf = out / 'argbenc.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-nostdlib',
                    '-DMCU_CH32V40X=1', '-DBOARD=3', f'-DWS_ARGB_HW="{(out / "argb_model.h").as_posix()}"',
                    '-DWS_RAMCODE=', '-I', str(FW / 'src'), '-Wl,-e,b_init', '-T', str(FW / 'test/fx_rv.ld'),
                    str(out / 'argbenc_bench.c'), str(FW / 'src/core/ur_digital_m/argbenc.c'), '-o', str(elf)],
                   check=True, timeout=120)
    e = Emu(str(elf))
    st = e.addr('state')

    peak = [0]

    def fill(w):
        e.call('b_fill', [w & 0xffffffff])
        peak[0] = max(peak[0], e.cycles())
        return struct.unpack('<12I', e.peek(st, 48))

    def var(name, value=None):
        a = e.addr(name)
        if value is not None:
            e.poke(a, struct.pack('<I', value))
        return struct.unpack('<I', e.peek(a, 4))[0]

    rng = random.Random(407)
    checks, frames_total = 0, 0
    # the bit period at the boot clock estimate: 125 x 1.952 cycles / 10 ns = 244
    for npx, period_w in ((100, 0), (600, 0), (300, 3125 * 16), (50, 1563)):
        w0 = 1000
        e.call('b_init', [npx, w0, period_w])
        w, last_end, prev_starts, first = w0, None, 0, True
        run_first = frames_total
        for tick in range(4000):
            w += 156 if rng.random() > 0.05 else rng.randint(157, 400)   # late ticks too
            if rng.random() < 0.3:
                e.call('b_ready')
            s = fill(w)
            state, i, low, next_w, start_w, end_w, frames, lost, starts, per, ready, running = s
            if starts != prev_starts:                                   # a frame started at this tick
                assert starts == prev_starts + 1 and state == 1 and running, s
                assert start_w == w + 1, s                              # (argb_hw_now: one word ahead)
                assert per == 244 and var('m_t0') == 70 and var('m_t1') == 135, (per, var('m_t0'), var('m_t1'))
                dur = (200 + npx * 3 * 8 * per + 63) // 64 + 1
                assert end_w == w + 1 + dur, (end_w, w, dur)
                if last_end is not None:
                    assert w - last_end >= 944, ('reset too short', w - last_end)
                if period_w:
                    assert (next_w - w) % 1 == 0 and next_w > w, s
                frames_total += 1
                prev_starts = starts
                first = False
            if state == 1:
                assert 0 <= i <= npx * 3, s
            if state == 0 and running == 0 and prev_starts and last_end != end_w:
                last_end = end_w
            checks += 1
        assert frames_total > run_first, (npx, period_w)   # (every run sends frames)
    # a serialiser that never finishes: the frame is lost after its end + 1 ms, then a full reset
    e.call('b_init', [100, 0, 0])
    var('m_hang', 1)
    e.call('b_ready')
    w = 0
    lost_seen = False
    for tick in range(200):
        w += 156
        s = fill(w)
        if s[7]:
            lost_seen = True
            assert s[0] == 0 and s[2] == 0 and not s[11], s
            break
    assert lost_seen
    var('m_hang', 0)
    result = dict(checks=checks, frames=frames_total, lost_case=True, fill_peak_cycles=peak[0])
    (out / 'argbenc_v407.json').write_text(json.dumps(result))
    print(json.dumps(result))


if __name__ == '__main__':
    main()

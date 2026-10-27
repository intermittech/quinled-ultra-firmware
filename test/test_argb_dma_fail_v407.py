# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""A failed ARGB DMA frame must not count as sent. Two parts, both compiled for RV32 and run in rv32emu:

1. The driver (src/mcu/ch32v40x/hw_dline.h) against register states: the end of cycle (FRHIF) with bytes left in DMA1
   ch10's CNTR, or a transfer error (TEIF10 in EXTEM_INTFR), is a failure; FRHIF with CNTR = 0 and no error is the only
   success; a start and a stop clear channel 10's flags (a stale error must not fail the next frame).
2. The encoder (src/core/ur_digital_m/argbenc.c) against a serialiser model that ends its cycle normally while its DMA
   failed (bytes left, or a transfer error): the frame is lost (counted), its bytes are not reported sent, the renderer
   must make it again (ready cleared), and the next frame starts a whole reset after the stop.

On a tree whose driver has no status (9b34a28), part 1 uses its end-of-cycle check and part 2 its argb_hw_done.

    python test/test_argb_dma_fail_v407.py <out dir> [--src <firmware tree>]
"""
import argparse
import json
import pathlib
import struct
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
from ur_digital_m_target import target_gcc   # noqa: E402

DRIVER = r'''
#include "mcu/ch32v40x/hw_dline.h"
uint8_t buf[8];
void b_init(void) { }
unsigned b_status(void)
{
#ifdef ARGB_ST_OK
    return argb_hw_status();
#else
    return argb_hw_done() ? 1u : 0u;              /* (a driver without a status: the end of cycle alone) */
#endif
}
void b_start(void) { argb_hw_start(buf, 8u, 250u, 70u, 135u); }
void b_stop(void) { argb_hw_stop(); }
'''
MODEL = r'''
#include <stdint.h>
#define ARGB_LEAD_CYC 200u
#define ARGB_ST_RUN 0u
#define ARGB_ST_OK 1u
#define ARGB_ST_FAIL 2u
extern volatile uint32_t m_t, m_start, m_end, m_n, m_per, m_starts, m_running, m_left, m_teif;
static inline uint32_t argb_hw_now(uint32_t w) { (void)w; return m_t / 64u + 1u; }
static inline void argb_hw_start(const uint8_t *b, uint32_t n, uint32_t per, uint32_t t0, uint32_t t1)
{ (void)b; (void)t0; (void)t1; m_start = m_t; m_n = n; m_per = per; m_end = m_t + ARGB_LEAD_CYC + n * 8u * per;
  m_starts++; m_running = 1; m_teif = 0; }
static inline uint32_t argb_hw_left(void) { return m_left; }
static inline uint32_t argb_hw_done(void) { return m_running && (int32_t)(m_t - m_end) >= 0; }   /* FRHIF */
static inline uint32_t argb_hw_status(void)
{ if (m_teif) return ARGB_ST_FAIL; if (!argb_hw_done()) return ARGB_ST_RUN; return m_left ? ARGB_ST_FAIL : ARGB_ST_OK; }
static inline void argb_hw_stop(void) { if (m_running && (int32_t)(m_t - m_end) < 0) m_end = m_t; m_running = 0; }
'''
BENCH = r'''
#include "core/ur_digital_m/wsenc.h"
volatile uint32_t m_t, m_start, m_end, m_n, m_per, m_starts, m_running, m_left, m_teif;
ws_enc_t e;
ws_lut_t L;
ws_chip_t c;
uint8_t fb[600];
uint32_t st[5];
void b_init(unsigned npx, unsigned w0, unsigned t)
{ c.s = MCU_SYM_BIT; c.t0h = MCU_SYM_NS(361u); c.t1h = MCU_SYM_NS(694u); c.bpp = 3; c.reset_w = 944;
  ws_lut_build(&L, &c); m_t = t; m_running = 0; ws_enc_init(&e, &c, fb, npx, w0, 0); }
void b_fill(unsigned limit, unsigned t)
{ m_t = t; (void)ws_ring_fill(&e, &L, 0, limit); st[0] = e.state; st[1] = e.i; st[2] = e.lost; st[3] = e.ready;
  st[4] = e.low_w; }
void b_ready(void) { e.ready = 1; }
'''
INTFR, INTFCR, CNTR10, STATR = 0x400200F0, 0x400200F4, 0x400200B4, 0x4002501C
TICK = 10000


def build(out, name, text, srcs, src, extra=()):
    (out / (name + '.c')).write_text(text)
    elf = out / (name + '.elf')
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-fno-builtin', '-fno-common',
                    '-ffunction-sections', '-fdata-sections', '-nostdlib', '-DBOARD=3', '-DMCU_CH32V40X=1',
                    '-I', str(src / 'src'), '-I', str(src / 'src/core'), 
                    '-I', str(src / 'src/mcu/ch32v40x'), *extra, '-Wl,--gc-sections', '-Wl,-e,b_init',
                    '-T', str(src / 'test/fx_rv.ld'), str(out / (name + '.c')), *[str(src / s) for s in srcs],
                    '-o', str(elf)], check=True, timeout=120)
    return elf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    import rv32emu as rv
    a.out.mkdir(parents=True, exist_ok=True)
    bad = []

    # 1. the driver
    class Regs(rv.Emu):
        def wr(self, addr, n, v):
            super().wr(addr, n, v)
            if self._mem(addr, n)[0] is None:
                if addr == INTFCR:
                    self.mmio[INTFR] = self.mmio.get(INTFR, 0) & ~v
                else:
                    self.mmio[addr] = v

    d = Regs(str(build(a.out, 'argb_driver', DRIVER, [], src)))
    for name, frhif, cntr, flags, want in (('running', 0, 5, 0, 0), ('sent', 1, 0, 0, 1), ('bytes left', 1, 123, 0, 2),
                                           ('transfer error', 0, 5, 1 << 11, 2), ('error at the end', 1, 0, 1 << 11, 2),
                                           ('other channel error', 1, 0, 1 << 7, 1)):
        d.mmio.update({STATR: frhif, CNTR10: cntr, INTFR: flags})
        d.call('b_status')
        if d.ret != want:
            bad.append(dict(part='driver', case=name, got=d.ret, want=want))
    for fn in ('b_start', 'b_stop'):
        d.mmio.update({INTFR: 0xF00 | (1 << 7), STATR: 0, CNTR10: 8})
        d.call(fn)
        if d.mmio[INTFR] & 0xF00 or not d.mmio[INTFR] & (1 << 7):
            bad.append(dict(part='driver', case=fn + ' clears only channel 10', intfr=hex(d.mmio[INTFR])))

    # 2. the encoder
    (a.out / 'argb_fail_model.h').write_text(MODEL)
    e = rv.Emu(str(build(a.out, 'argb_fail', BENCH, ['src/core/ur_digital_m/argbenc.c'], src,
                         [f'-DWS_ARGB_HW="{(a.out / "argb_fail_model.h").as_posix()}"', '-DWS_RAMCODE='])))

    def var(n, v=None):
        if v is not None:
            e.poke(e.addr(n), struct.pack('<I', v))
        return struct.unpack('<I', e.peek(e.addr(n), 4))[0]

    for case in ('sent', 'bytes left', 'transfer error'):
        t = 2_000_000
        e.call('b_init', [100, t // 64, t])
        var('m_left', 0)
        var('m_starts', 0)
        var('m_teif', 0)
        e.call('b_ready')
        stop_at, started = None, False
        for k in range(400):
            t += TICK
            if var('m_running') and case != 'sent' and t - var('m_start') > 3 * TICK:
                var('m_left', 123 if case == 'bytes left' else var('m_left'))
                if case == 'transfer error':
                    var('m_teif', 1)
                    var('m_left', 60)
            e.call('b_fill', [t // 64, t + 3000])
            s = struct.unpack('<5I', e.peek(e.addr('st'), 20))
            if var('m_starts') == 1:
                started = True
            if started and not var('m_running') and stop_at is None:
                stop_at = var('m_end')
                if case == 'sent':
                    if s[2] or s[1] != 300:
                        bad.append(dict(part='encoder', case=case, lost=s[2], sent=s[1]))
                else:
                    if s[2] != 1 or s[1] != 0 or s[3] != 0 or s[0] != 0:
                        bad.append(dict(part='encoder', case=case, state=s[0], sent=s[1], lost=s[2], ready=s[3]))
                    e.call('b_ready')                    # the renderer's frame again
            if var('m_starts') == 2:
                if var('m_start') - stop_at < 944 * 64:
                    bad.append(dict(part='encoder', case=case, retry_gap_us=(var('m_start') - stop_at) / 200))
                break
            if stop_at is not None and case == 'sent':
                break
        else:
            bad.append(dict(part='encoder', case=case, error='no retry'))
        if stop_at is None:
            bad.append(dict(part='encoder', case=case, error='frame never ended'))
    result = dict(failures=len(bad), details=bad)
    (a.out / 'argb_dma_fail_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    if bad:
        sys.exit(1)


if __name__ == '__main__':
    main()

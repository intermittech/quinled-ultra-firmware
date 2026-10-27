# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 Digital-M's whole tick ISR against its budget, on the compiled app ELFs (both compilers) in rv32emu.

The tick is SysTick at 20 kHz: 50 us x 200 MHz = 10,000 HCLK cycles. The rule (as the V203's 7,200-cycle tick at
144 MHz): the worst modeled ISR + 60 cycles (entry / exit) <= the tick's cycles, here 9,940. What runs: the trap stub
(isr.S, mcause 12) into poll_isr: the ADC's millisecond work (poll_sub at its wrap every tick), the delay line's scan
with the chain scanner on over 800 kHz pixel data in every word, and the local effects' encoder: idle, a frame start
on every tick (a LOW of a whole reset and the renderer's frame ready re-seeded each tick), a frame's completion, a DMA
failure; and the CHAIN gate's relay window (the board at any position of its line): the turnaround (TURN: the gate
closed, port 2 listens, the line above sampled and driven from the relay node) and forward-again (FWD) on every tick over
the same pixel words, and the slot plan's re-arm after 200 us of quiet LOW (the live check, TIM2 re-armed) on every tick.
The words per tick: 156, 157 and the exact 50 MHz schedule (156 / 156 / 156 / 157). Added on top: the answer
UART's handler (priority 0: it preempts the tick) for five bytes a tick (1 Mbaud: 10 us a byte) and its TC, and one
console byte (same group as the tick: it can delay it once). rv32emu's V4B-derived cost table, not V3V silicon timing
(no published cycle table; HPE save / restore and fetch stalls are not modeled): a model gate.

    python test/test_poll_budget_v407.py <out dir> <app fw.elf> [<app fw.elf> ...] [--src <firmware tree>]
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

TICK_CYCLES = 200_000_000 // 20_000
BUDGET = TICK_CYCLES - 60
SETUP = r'''
#include "core/dline.h"
#include "core/ur_digital_m/udfx.h"
#include <stddef.h>
unsigned b_size(void) { return sizeof(dline_t); }
void b_init(dline_t *d, uint16_t *cap, unsigned chain, unsigned local)
{ dline_init(d, 0, cap); d->rec = 0; d->direct = 0; d->bl = 0; d->ch_on = (uint8_t)chain; d->cpos = 0;
  d->local = (uint8_t)local; }
unsigned b_offsets(unsigned i)
{ static const unsigned o[] = { offsetof(udfx_t, local), offsetof(udfx_t, e), offsetof(ws_enc_t, state),
  offsetof(ws_enc_t, ready), offsetof(ws_enc_t, nbytes), offsetof(ws_enc_t, reset_w), offsetof(ws_enc_t, fb),
  offsetof(ws_enc_t, end_w), offsetof(ws_enc_t, low_w), offsetof(ws_enc_t, period_w), offsetof(ws_enc_t, next_w),
  offsetof(udfx_t, npx), offsetof(udfx_t, chain), offsetof(udfx_t, period_w), offsetof(ws_enc_t, fb_next) }; return o[i]; }
unsigned b_dl_offsets(unsigned i)
{ static const unsigned o[] = { offsetof(dline_t, ch_rel), offsetof(dline_t, ch_dir), offsetof(dline_t, ch_fwd),
  offsetof(dline_t, ch_st), offsetof(dline_t, ch_lz), offsetof(dline_t, wpos) }; return o[i]; }
'''
# encoder field index -> offset table position
LOCAL, ENC, STATE, READY, NBYTES, RESET_W, FB, END_W, LOW_W, PERIOD_W, NEXT_W, NPX, CHAIN, UPERIOD, FB_NEXT = range(15)
DMA2_CNTR, DMA3_CNTR, TIM4_CNT, ADC_STATR = 0x40020020, 0x40020034, 0x40000824, 0x40012400
TIM2_CHCTLR2, TIM2_CNT, GPIOB_INDR = 0x4000001C, 0x40000024, 0x40010C08
CH_REL, CH_DIR, CH_FWD, CH_ST, CH_LZ, WPOS = range(6)        # dline_t offsets (b_dl_offsets)
ARGB_STATR, DMA10_CNTR, DMA_EXT = 0x4002501C, 0x400200B4, 0x400200F0
USART7_CTLR1, USART1_CTLR1 = 0x40001C0C, 0x4001380C


def pixel_word(k):
    out = 0
    for b in range(16):
        s = k * 16 + b
        bit_high = 40 if (s // 125) % 2 else 20
        out |= (0x8000 >> b) if (s % 125) % 62.5 < bit_high else 0
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('elfs', type=pathlib.Path, nargs='+')
    ap.add_argument('--src', type=pathlib.Path, default=FW)
    a = ap.parse_args()
    src = a.src.resolve()
    sys.path.insert(0, str(src / 'tools'))
    import rv32emu as rv
    rv.RAM_SIZE, rv.FLASH_SIZE = 65536 * 4, 0x80000              # (the V407's SRAM and flash in the model)
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'poll_setup.c').write_text(SETUP)
    setup_elf = a.out / 'poll_setup.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-DBOARD=3',
                    '-DMCU_CH32V40X=1', '-DDLINE_RAMCODE=', '-fno-builtin', '-fno-common', '-nostdlib',
                    '-I', str(src / 'src'), '-I', str(src / 'src/core'), '-Wl,-e,b_init',
                    '-T', str(src / 'test/fx_rv.ld'), str(a.out / 'poll_setup.c'), str(src / 'src/core/dline.c'),
                    '-o', str(setup_elf)], check=True, timeout=120)
    setup = rv.Emu(str(setup_elf))
    setup.call('b_size')
    dsize = setup.ret
    off = []
    for i in range(15):
        setup.call('b_offsets', [i])
        off.append(setup.ret)
    doff = []
    for i in range(6):
        setup.call('b_dl_offsets', [i])
        doff.append(setup.ret)

    class Trap(rv.Emu):
        cause = 0x8000000C

        def _i(self, ins, pc, x):
            if ins == 0x30200073:                                   # mret: back to the caller
                return rv.RET_MAGIC, 'jump'
            if ins & 0x7f == 0x73 and ins >> 20 == 0x342:           # csrr mcause
                x[(ins >> 7) & 31] = self.cause
                return pc + 4, 'alu'
            return super()._i(ins, pc, x)

    words = struct.pack('<512H', *[pixel_word(k) for k in range(512)])
    rows, worst = {}, 0
    for elf in a.elfs:
        name, elf_worst = elf.parent.name, 0
        for local in ('idle', 'local', 'start', 'complete', 'dma error', 'turn', 'fwd', 'rearm'):
            for step in (156, 157, 0):
                e = Trap(str(elf))

                def put(sym_addr, v, n=4):
                    e.poke(sym_addr, (v & ((1 << (8 * n)) - 1)).to_bytes(n, 'little'))

                relay = local in ('turn', 'fwd', 'rearm')
                setup.call('b_init', [e.addr('dl'), e.addr('dl_cap'), 1, int(local != 'idle' and not relay)])
                e.poke(e.addr('dl'), setup.peek(e.addr('dl'), dsize))
                put(e.addr('dl_on'), 1, 1)
                put(e.addr('adc_ready'), 1, 1)
                put(e.addr('inj_run'), 1, 1)
                ux, enc = e.addr('ux'), e.addr('ux') + off[ENC]
                if relay:
                    put(e.addr('gate_slot'), 1, 1)                  # CHAIN decided: the slot plan
                if local != 'idle' and not relay:
                    put(ux + off[LOCAL], 1, 1)
                    put(ux + off[NPX], 3, 2)
                    put(ux + off[CHAIN], 1, 1)
                    put(ux + off[UPERIOD], 31250)
                    fbuf = e.addr('ux_frames') if 'ux_frames' in e.syms else e.addr('ux_pool')   # (FX_V407: two frames)
                    for idx, v, n in ((STATE, 0, 1), (READY, 1, 1), (NBYTES, 9, 4), (RESET_W, 944, 2),
                                      (FB, fbuf, 4), (FB_NEXT, fbuf, 4), (LOW_W, 1000, 4), (PERIOD_W, 31250, 4),
                                      (NEXT_W, 0, 4)):
                        put(enc + off[idx], v, n)
                    e.poke(e.addr('ux_lut'), bytes([133, 36, 69]))
                rx, peak = 0, 0
                for tick in range(80):
                    rx = rx + step if step else (tick + 1) * 625 // 4
                    put(e.addr('poll_sub'), 19)                     # the ms wrap: the ADC's injected group too
                    e.mmio.update({DMA2_CNTR: 512 - (rx & 511), DMA3_CNTR: 512 - ((rx + 2) & 511),
                                   TIM4_CNT: (rx * 2) & 0xffff, ADC_STATR: 4, ARGB_STATR: 0, DMA10_CNTR: 3,
                                   DMA_EXT: 0})
                    e.poke(e.addr('dl_ring'), words if local != 'rearm' else bytes(1024))
                    dl = e.addr('dl')
                    w = int.from_bytes(e.peek(dl + doff[WPOS], 4), 'little')
                    if local == 'turn':                             # the turnaround due now: the relay node drives up
                        put(dl + doff[CH_REL], 1, 1)
                        put(dl + doff[CH_DIR], w)
                        e.mmio[GPIOB_INDR] = 1 << 4                 # (the line above released: HIGH)
                    elif local == 'fwd':                            # forward-again due now
                        put(dl + doff[CH_REL], 2, 1)
                        put(dl + doff[CH_FWD], w)
                    elif local == 'rearm':                          # 200 us of quiet LOW, edges since the last re-arm
                        put(dl + doff[CH_REL], 0, 1)
                        put(dl + doff[CH_ST], 0, 1)
                        put(dl + doff[CH_LZ], 0xFFFF, 2)
                        e.mmio.update({TIM2_CNT: 5, TIM2_CHCTLR2: 3 << 12, GPIOB_INDR: 0, TIM4_CNT: 7})
                    if local == 'start':                            # a frame start's path on every tick
                        for idx, v, n in ((STATE, 0, 1), (READY, 1, 1), (LOW_W, 1000, 4), (PERIOD_W, 31250, 4),
                                          (NEXT_W, rx - 1, 4)):
                            put(enc + off[idx], v, n)
                    if local in ('complete', 'dma error'):          # a running frame that ends now
                        put(enc + off[STATE], 1, 1)
                        put(enc + off[END_W], rx + 100)
                        if local == 'complete':
                            e.mmio.update({ARGB_STATR: 1, DMA10_CNTR: 0})
                        else:
                            e.mmio.update({DMA_EXT: 1 << 11})
                    e.cause = 0x8000000C
                    e.call('trap_entry', max_steps=200000)
                    peak = max(peak, e.cycles())
                rows[f'{name} {local} words={step or "50MHz"}'] = peak
                elf_worst = max(elf_worst, peak)
        # the nested answer bytes (VTF slot 1 straight into rc_uart_entry) and one console byte
        e = Trap(str(elf))
        put = lambda addr, v: e.poke(addr, (v & 0xffffffff).to_bytes(4, 'little'))   # noqa: E731
        put(e.addr('rc_left'), 40)
        put(e.addr('rc_next'), e.addr('ux_frames') if 'ux_frames' in e.syms else e.addr('ux_pool'))
        e.mmio[USART7_CTLR1] = 1 << 7
        e.call('rc_uart_entry')
        rc_byte = e.cycles()
        put(e.addr('rc_left'), 0)
        e.call('rc_uart_entry')
        rc_tc = e.cycles()
        put(e.addr('con_w'), 10)
        put(e.addr('con_r'), 0)
        e.cause = 0x80000035
        e.call('trap_entry')
        con = e.cycles()
        rows[f'{name} answer byte / TC / console byte'] = [rc_byte, rc_tc, con]
        nested = 5 * rc_byte + rc_tc + con
        rows[f'{name} worst tick + nested'] = elf_worst + nested
        worst = max(worst, elf_worst + nested)
    result = dict(model='rv32emu (V4B cost table)', tick_cycles=TICK_CYCLES, budget=BUDGET, worst=worst,
                  margin=BUDGET - worst, rows=rows)
    (a.out / 'poll_budget_v407.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    sys.exit(0 if worst <= BUDGET else 1)


if __name__ == '__main__':
    main()

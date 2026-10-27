# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 Digital-M at any position of a CHAIN line, on the compiled API app (rv32emu): the port-2 gate and the relay
window as main_ur_digital_m.c's tick and hw.c drive the pins (docs/ur_digital_m-firmware.md; the design: the board holds its
own slot, forwards the rest, and the boards behind answer through its relay window).

FWD_EN is TIM2 CH4's output compare (OC4M forced LOW = closed, forced HIGH = open, toggle at CNT 256 = the slot plan);
DE_OUT PA10 drives port 2, DE_IN PA11 the line above, PA9 the answer UART's TX, PB4 the input's RO. Checked:
  - the turnaround (DLINE_A_CH_TURN): the gate closed, DE_OUT low, then the line above sampled with DE_IN still low:
    HIGH (released) -> DE_IN high, the relay node drives the line above; LOW (still driven) -> DE_IN stays low, the armed
    answer dropped, PIXEL BUSY + 1; a turnaround later than DIR_MAX is not taken;
  - this board's burst inside a window: DE stays high at its TC (the boards behind have later slots), PA9 floats again;
  - forward-again (DLINE_A_CH_FWD): DE_IN low, DE_OUT high, PA9 floating, the chain scanner waits out a HIGH;
  - the slot plan's re-arm: after >= ULTRA_RW_REARM_US of LOW (the chain scanner IDLE), CHAIN decided, no window, not
    update-transparent, the plan not armed since the last edge, and the line LOW with no edge since the tick read TIM4:
    closed, CNT 0, CH4CVR 256, toggle; each missing condition alone: no re-arm;
  - STREAM decided: the gate forced LOW (a marker port forwards nothing); CHAIN decided: no gate write from the main loop.
Model evidence on the production code paths (API build: LTO 0, no inlining), not an electrical qualification.

    python test/test_ud_relay_v407.py <out dir> <V407 API app fw.elf> [...]
"""
import argparse
import json
import pathlib
import subprocess
import sys

FW = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
sys.path.insert(0, str(FW / 'tools'))
from ur_digital_m_target import target_gcc   # noqa: E402
import rv32emu as rv                          # noqa: E402

GPIOA, GPIOB_INDR = 0x40010800, 0x40010C08
BSHR, BCR, CFGHR = GPIOA + 0x10, GPIOA + 0x14, GPIOA + 0x04
TIM2_CHCTLR2, TIM2_CNT, TIM2_CH4CVR = 0x4000001C, 0x40000024, 0x40000040
TIM4_CNT, USART7_CTLR1 = 0x40000824, 0x40001C0C
DMA2_CNTR, DMA3_CNTR = 0x40020020, 0x40020034
CLOSED, TOGGLE = 4 << 12, 3 << 12
SETUP = r'''
#include "core/dline.h"
#include "core/ur_digital_m/udchain.h"
#include <stddef.h>
void b_init(void) { }
unsigned b_off(unsigned i)
{ static const unsigned o[] = { offsetof(dline_t, ch_rel), offsetof(dline_t, ch_dir), offsetof(dline_t, ch_fwd),
  offsetof(dline_t, ch_st), offsetof(dline_t, ch_lz), offsetof(dline_t, wpos), offsetof(dline_t, ch_arm),
  offsetof(dline_t, ch_on), offsetof(udc_t, relay_busy) }; return o[i]; }
'''
NAMES = ('ch_rel', 'ch_dir', 'ch_fwd', 'ch_st', 'ch_lz', 'wpos', 'ch_arm', 'ch_on', 'relay_busy')
WORDS_TICK = 156                                            # 50 us of 50 MHz samples, 16 a word (one tick's scan)
REARM_W = (200 * 50 + 15) // 16                             # MCU_US2W(ULTRA_RW_REARM_US)
LATE_W = (400 * 50 + 15) // 16                              # DLINE_W_CH_TURN_LATE


class App(rv.Emu):
    def __init__(self, elf):
        super().__init__(str(elf))
        self.log = []                                       # (address, value) of every peripheral write

    def wr(self, a, n, v):
        if 0x40000000 <= a < 0x50000000:
            self.log.append((a, v))
        super().wr(a, n, v)

    def put(self, addr, v, n=4):
        self.poke(addr, (v & ((1 << (8 * n)) - 1)).to_bytes(n, 'little'))

    def get(self, addr, n=4):
        return int.from_bytes(self.peek(addr, n), 'little')

    def gate_writes(self):
        return [(a, v) for a, v in self.log if a in (TIM2_CHCTLR2, TIM2_CNT, TIM2_CH4CVR)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=pathlib.Path)
    ap.add_argument('elfs', type=pathlib.Path, nargs='+')
    a = ap.parse_args()
    rv.RAM_SIZE, rv.FLASH_SIZE = 65536 * 4, 0x80000
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'relay_setup.c').write_text(SETUP)
    setup_elf = a.out / 'relay_setup.elf'
    subprocess.run([target_gcc(), '-march=rv32imc_zicsr_zifencei', '-mabi=ilp32', '-Os', '-DBOARD=3', '-DMCU_CH32V40X=1',
                    '-DDLINE_RAMCODE=', '-fno-builtin', '-fno-common', '-nostdlib', '-I', str(FW / 'src'),
                    '-I', str(FW / 'src/core'), '-Wl,-e,b_init', '-T', str(FW / 'test/fx_rv.ld'), str(a.out / 'relay_setup.c'),
                    '-o', str(setup_elf)], check=True, timeout=120)
    setup = rv.Emu(str(setup_elf))
    off = {}
    for i, n in enumerate(NAMES):
        setup.call('b_off', [i])
        off[n] = setup.ret
    rows, bad = [], 0

    def record(elf, case, ok, **extra):
        nonlocal bad
        bad += not ok
        row = dict(elf=elf.parent.name, case=case, ok=bool(ok), **extra)
        rows.append(row)
        print(json.dumps(row), flush=True)

    for elf in a.elfs:
        def fresh():
            e = App(elf)
            e.call('upd_boot')                              # the gate's timer and pins (upd_forward_init)
            e.call('dline_init', [e.addr('dl'), 0, e.addr('dl_cap')])
            e.put(e.addr('dl_on'), 1, 1)
            e.put(e.addr('dl') + off['ch_on'], 1, 1)
            e.mmio.update({DMA2_CNTR: 512, DMA3_CNTR: 510})
            for _ in range(8):                              # a quiet line for 400 us: the scanners settle
                tick(e)
            return e

        def tick(e, rel=None, dir_ago=0, fwd_in=None, slot=None, lz=None, st=None, din=0, upd_open=0, armed=False):
            """one dl_tick over the next tick's LOW words; rel / slot / lz / st: set before (None: as they are). lz is the
            chain scanner's LOW run before the tick (the tick's words add WORDS_TICK)"""
            dl = e.addr('dl')
            w = e.get(dl + off['wpos'])
            rx = w + WORDS_TICK
            if st is not None:
                e.put(dl + off['ch_st'], st, 1)
            if lz is not None:
                e.put(dl + off['ch_lz'], lz, 2)
            if rel is not None:
                e.put(dl + off['ch_dir'], rx - dir_ago)
                e.put(dl + off['ch_fwd'], rx if fwd_in is None else rx + fwd_in)
                e.put(dl + off['ch_rel'], rel, 1)
            if slot is not None:
                e.put(e.addr('gate_slot'), slot, 1)
            e.put(e.addr('upd_forward_open'), upd_open, 1)
            e.put(e.addr('upd_forward_wanted'), upd_open, 1)
            e.mmio.update({DMA2_CNTR: 512 - (rx & 511), DMA3_CNTR: 512 - ((rx + 2) & 511), TIM4_CNT: 7,
                           GPIOB_INDR: (1 << 4) if din else 0,
                           TIM2_CNT: 0 if armed else 3, TIM2_CHCTLR2: TOGGLE if armed else CLOSED})
            e.poke(e.addr('dl_ring'), bytes(1024))           # (a LOW line)
            e.log.clear()
            e.call('poll_isr', max_steps=400000)              # (main's tick: dl_tick inside; dline.c has a static dl_tick too)
            return e

        dl = None
        # the turnaround, the line above released
        e = tick(fresh(), rel=1, din=1)
        dl = e.addr('dl')
        lg = e.log
        ok = ((TIM2_CHCTLR2, CLOSED) in lg and (BCR, 1 << 10) in lg and (BSHR, 1 << 11) in lg and
              lg.index((TIM2_CHCTLR2, CLOSED)) < lg.index((BSHR, 1 << 11)) and lg.index((BCR, 1 << 10)) < lg.index((BSHR, 1 << 11)) and
              e.get(dl + off['ch_rel'], 1) == 2 and e.get(e.addr('uc') + off['relay_busy'], 2) == 0)
        record(elf, 'turnaround, the line above released: gate closed, DE_OUT low, then DE_IN high', ok,
               writes=[hex(x) for x, _ in lg][:12])
        # the turnaround, the line above still driven
        e = fresh()
        e.put(e.addr('dl') + off['ch_arm'], 1, 1)
        tick(e, rel=1, din=0)
        lg = e.log
        ok = ((TIM2_CHCTLR2, CLOSED) in lg and (BCR, 1 << 10) in lg and (BSHR, 1 << 11) not in lg and
              e.get(e.addr('dl') + off['ch_arm'], 1) == 0 and e.get(e.addr('uc') + off['relay_busy'], 2) == 1)
        record(elf, 'turnaround, the line above still driven: no relay, the answer dropped, BUSY + 1', ok)
        # a turnaround later than DIR_MAX: not taken (missed)
        e = tick(fresh(), rel=1, dir_ago=LATE_W + 4, din=1)
        ok = not any(a_ in (BCR, BSHR) for a_, _ in e.log) and e.get(e.addr('dl') + off['ch_rel'], 1) == 0
        record(elf, 'a turnaround later than DIR_MAX: not taken', ok)
        # this board's burst inside a window: DE stays high at its TC, PA9 floats
        e = fresh()
        e.mmio[GPIOB_INDR] = 1 << 4
        e.call('rc_hw_relay', [1])
        e.call('rc_hw_answer_local')
        e.put(e.addr('rc_left'), 0)
        e.mmio[USART7_CTLR1] = 1 << 6                       # TCIE set: the burst's last stop bit is out
        e.log.clear()
        e.call('rc_uart_isr')
        lg = e.log
        pa9 = [v for a_, v in lg if a_ == CFGHR]
        ok = (BCR, 1 << 11) not in lg and bool(pa9) and ((pa9[-1] >> 4) & 15) == 0x4
        record(elf, 'own burst in a window: DE stays high at TC, PA9 floating again', ok, writes=[hex(x) for x, _ in lg][:8])
        # forward-again
        e = tick(fresh(), rel=2, din=1)
        lg = e.log
        pa9 = [v for a_, v in lg if a_ == CFGHR]
        ok = ((BCR, 1 << 11) in lg and (BSHR, 1 << 10) in lg and bool(pa9) and ((pa9[-1] >> 4) & 15) == 0x4 and
              e.get(e.addr('dl') + off['ch_rel'], 1) == 0)
        record(elf, 'forward-again: DE_IN low, DE_OUT high, PA9 floating', ok)
        # the re-arm and each condition missing alone
        e = tick(fresh(), slot=1, st=0, lz=REARM_W - WORDS_TICK)
        gw = e.gate_writes()
        ok = gw[-4:] == [(TIM2_CHCTLR2, CLOSED), (TIM2_CNT, 0), (TIM2_CH4CVR, 256), (TIM2_CHCTLR2, TOGGLE)]
        record(elf, 're-arm after 200 us of quiet LOW: closed, CNT 0, CH4CVR 256, toggle', ok,
               writes=[(hex(x), hex(y)) for x, y in gw])
        for name, kw in (('199.7 us of LOW', dict(lz=REARM_W - WORDS_TICK - 1)), ('not CHAIN', dict(slot=0)),
                         ('in a window', dict(rel=2, fwd_in=100000)), ('update-transparent', dict(upd_open=1)),
                         ('DIN HIGH now', dict(din=1)), ('armed, no edge since', dict(armed=True)),
                         ('the scanner in a frame', dict(st=1, lz=0))):
            k = dict(slot=1, st=0, lz=REARM_W)
            k.update(kw)
            e = tick(fresh(), **k)
            record(elf, f'no re-arm: {name}', not any(x == TIM2_CNT for x, _ in e.gate_writes()),
                   writes=[(hex(x), hex(y)) for x, y in e.gate_writes()])
        # STREAM decided: the gate forced LOW; CHAIN decided: wanted, no gate write from the main loop
        e = fresh()
        e.put(e.addr('gate_slot'), 1, 1)
        e.log.clear()
        e.call('ud_line_apply', [2])                        # UDC_A_STREAM
        record(elf, 'STREAM: the gate forced LOW, no slot plan', e.get(e.addr('gate_slot'), 1) == 0 and (TIM2_CHCTLR2, CLOSED) in e.log)
        e = fresh()
        e.log.clear()
        e.call('ud_line_apply', [1])                        # UDC_A_CHAIN
        record(elf, 'CHAIN: the slot plan wanted, no gate write from the main loop',
               e.get(e.addr('gate_slot'), 1) == 1 and not e.gate_writes())
    (a.out / 'ud_relay_v407.json').write_text(json.dumps(rows, indent=2))
    print(f'ud relay v407: {len(rows)} cases, {bad} failures')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

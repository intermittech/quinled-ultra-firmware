# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compiled main-loop regression: a suppressed BUSY burst cannot authorize flash.

Hardware/session stubs isolate the production main-loop boundary. Release timing
itself is covered against the production hardware driver by test_bl_release_v407.
"""
import argparse
from pathlib import Path
import subprocess
import sys
FW = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(FW/'tools'), str(FW/'test')]
from rv32emu import Emu
from ur_digital_m_target import target_gcc

STUBS = r'''
#include "bl_core.h"
#include "bl_hw.h"
#include "bl_rx.h"
uint16_t hw_ring[HW_RING_WORDS];
volatile uint8_t hw_flash_stall;
volatile unsigned pos, operation_authorized;
volatile int reply_result;
void hw_blank_early(void) {}
uint32_t hw_reset_flags_take(void) { return 0; }
uint32_t bl_reset_kind(uint32_t f) { (void)f; return BL_RST_POWER; }
void hw_crc_on(void) {}
bl_action_t bl_boot(bl_app_t *a, upd_handover_t *h, uint32_t r, uint32_t f)
{ (void)a; (void)h; (void)r; (void)f; return BL_ACT_RECOVERY; }
void hw_full_init(void) {}
bl_action_t bl_after_verify(bl_app_t *a) { (void)a; return BL_ACT_RECOVERY; }
void bl_ho_seal(upd_handover_t *h) { (void)h; }
void bl_sess_init(bl_sess_t *s, bl_action_t m, const bl_app_t *a, uint32_t t)
{ (void)a; (void)t; s->mode=m; s->state=BL_S_ERASE; s->gate=BL_GATE_OPEN; }
void bl_rx_init(bl_rx_t *r,const bl_rx_timing_t *t) { (void)t; r->locked_period=1; r->run=0; }
void bl_rx_resync(bl_rx_t *r) { (void)r; }
void hw_gate_open(void) {}
void hw_gate_ultra(void) {}
uint32_t hw_ring_pos(void) { return pos++ ? 1 : 0; }
uint16_t hw_edges(void) { return 0; }
void hw_kick(void) {}
void hw_led(int on) { (void)on; }
uint32_t hw_ticks(void) { return 1000; }
uint32_t hw_ms(void) { return 0; }
int bl_rx_feed(bl_rx_t *r,const uint16_t *w,uint32_t n,bl_frame_t *f)
{ (void)r; (void)w; (void)n; f->bad=0; f->nbits=384; return 1; }
void bl_sess_frame(bl_sess_t *s,uint32_t n,const uint8_t *f,uint32_t t)
{ (void)n; (void)f; (void)t; s->st.reply=1; s->st.op=1; s->st.op_due=1; }
int hw_reply(const uint8_t *a,uint32_t at,uint16_t brr T0_ARG)
{ (void)a; (void)at; (void)brr; return reply_result; }
int bl_sess_poll(bl_sess_t *s,uint32_t t) { (void)t; operation_authorized=s->st.op_due; return 1; }
void bl_ho_exit(upd_handover_t *h,const bl_sess_t *s) { (void)h; (void)s; }
void hw_soft_reset(void) { for (;;) {} }
void hw_jump_app(void) { for (;;) {} }
'''

class Done(Exception):
    pass

class Board(Emu):
    def _i(self, instruction, pc, regs):
        if pc == self.addr('hw_soft_reset'):
            raise Done()
        return super()._i(instruction, pc, regs)
    def _c(self, instruction, pc, regs):
        if pc == self.addr('hw_soft_reset'):
            raise Done()
        return super()._c(instruction, pc, regs)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('out',type=Path)
    ap.add_argument('--src',type=Path,default=FW)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    sig=', uint32_t t0' if 'uint32_t t0' in (a.src/'bootloader/bl_hw.h').read_text() else ''
    (a.out/'stubs.c').write_text(STUBS.replace('T0_ARG',sig))
    elf=a.out/'main.elf'
    subprocess.run([target_gcc(),'-march=rv32imc_zicsr_zifencei','-mabi=ilp32','-Os','-nostdlib',
      '-DCH32V407','-DMCU_CH32V40X=1','-DBL_HW_ID=3','-I',str(a.src/'bootloader'),'-I',str(a.src/'src/core'),
      '-I',str(a.src/'src/mcu/ch32v40x'),'-Wl,-e,main','-T',str(FW/'test/fx_rv.ld'),
      str(a.out/'stubs.c'),str(a.src/'bootloader/bl_main.c'),'-o',str(elf)],check=True,timeout=120)
    bad=0
    for rc in (0,-1):
        m=Board(str(elf));m.poke(m.addr('reply_result'),(rc & 0xffffffff).to_bytes(4,'little'))
        try:m.call('main',max_steps=100000)
        except Done:pass
        authorized=int.from_bytes(m.peek(m.addr('operation_authorized'),4),'little')
        ok=authorized == (rc==0);bad+=not ok
        print(dict(reply_result=rc,flash_authorized=authorized,ok=ok))
    return int(bool(bad))


if __name__=='__main__':
    sys.exit(main())

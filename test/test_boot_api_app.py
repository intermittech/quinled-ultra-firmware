# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Actual release app handover/confirmation with boot code overlaid in rv32emu.

Flash latency is a modeled input, never a hardware qualification claim.
"""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

FW = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(FW/'tools'), str(FW/'test')]
from rv32emu import RET_MAGIC, COST
from test_ur_digital_m_chain_budget import Board

KEYS = 'ping_seen ping_ms startup_ok confirm_done transparent info confirmed desc_ok st_next'.split()
DIGITAL = 'e max_px npx chip local dark calibration ready state nbytes frames low_w reset_w bpp take owned ch_hold'.split()
GCC = Path(os.environ.get('WCH_GCC_DIR', 'WCH_GCC_DIR-not-set'))/'bin/riscv32-wch-elf-gcc.exe'   # WCH's GCC 15 (tools/toolchain)

def offsets(board, folder):
    obj = folder/f'offsets{board}-{os.getpid()}.o'
    cmd = [str(GCC), '-march=rv32imc_zicsr_zifencei_xw', '-mabi=ilp32', '-Os', '-c',
           '-DCH32V20x_D6', '-DUPD_APP=1', '-DUPD_CRC_ULTRA', f'-DBOARD={board}',
           '-DMON_EXTRA=0', '-DCONSOLE_FULL=0', *['-I'+str(FW/p) for p in ('src/mcu/ch32v20x','src','src/core')],
           '-I'+str(folder/f'app{board}-wch/gen'), str(FW/'test/boot_api_app_offsets.c'), '-o',str(obj)]
    subprocess.run(cmd,check=True,capture_output=True)
    d=obj.read_bytes(); shoff=struct.unpack_from('<I',d,32)[0]
    ents,count,names=struct.unpack_from('<HHH',d,46)
    secs=[struct.unpack_from('<10I',d,shoff+i*ents) for i in range(count)]
    strs=secs[names][4]
    for s in secs:
        if d[strs+s[0]:d.index(b'\0',strs+s[0])] == b'.rodata.update_offsets':
            return dict(zip(KEYS+(DIGITAL if board==3 else ['ack_active','ack_led_only']),
                            struct.unpack_from('<%dI'%(s[5]//4),d,s[4])))
    raise AssertionError('offset section')

def crc32_words(d):
    c=0xffffffff
    for i in range(0,len(d),4):
        c ^= int.from_bytes(d[i:i+4],'little')
        for _ in range(32): c=((c<<1)^(0x04c11db7 if c&0x80000000 else 0))&0xffffffff
    return c

class Reset(Exception): pass

class App(Board):
    active=False
    irqs=True
    elapsed=0
    busy_until=0
    program_us=100
    no_write=False
    programmed=False
    in_isr=False

    def _i(self, ins, pc, x):
        if ins&127==0x73 and ins>>20==0x800:
            f=(ins>>12)&7; v=x[(ins>>15)&31]
            if f==2 and v&0x88: self.irqs=True
            if f==3 and v&0x88: self.irqs=False
        return super()._i(ins,pc,x)

    def rd(self,a,n):
        if self.active and self.programmed and 0x08002040<=a<0x08002100:
            assert self.elapsed>=self.busy_until, 'physical readback before BSY cleared'
        if a==0x4002200c and self.active and self.programmed:
            return 1 if self.elapsed<self.busy_until else 0x20
        return super().rd(a,n)

    def wr(self,a,n,v):
        if a==0xe000e060: self.flash_mode=bool(v)
        if a==0xe000e048 and v==0xbeef0080 and self.active:
            self.reset_irq=self.irqs
            raise Reset()
        if self.active and 0x08002040<=a<0x08002100:
            self.programmed=True
            self.busy_until=self.elapsed+self.program_us
            self.program_start=self.elapsed
            if self.board==3:
                ux=self.addr('ux'); enc=ux+self.O['e']
                assert self.rd(enc+self.O['state'],1)==0
                assert self.rd(enc+self.O['low_w'],4)>=self.rd(enc+self.O['reset_w'],2)+512
                assert not any(self.peek(self.addr('ux_pool'),self.rd(ux+self.O['max_px'],2)*self.rd(ux+self.O['chip']+self.O['bpp'],1)))
            if self.no_write: return
        super().wr(a,n,v)

    def invoke(self,fn,args=()):
        pc=self.start(fn,args); regs=self.x
        while pc!=RET_MAGIC:
            pc,_,_=self.run_budget(pc,set(),3600 if self.flash_mode else 7200,regs)
            regs=self.x
            if pc!=RET_MAGIC: self.tick(regs[2])
        return regs[10]

    def tick(self,sp=0x20004f00):
        self.elapsed+=50
        self.wpos+=112 if (self.elapsed//50)&1 else 113
        if self.board==3:
            for i in range(113): self.poke(self.addr('dl_ring')+2*((self.wpos-i)%512),b'\0\0')
            rx=self.wpos%512
            self.mmio[0x40020020]=512-rx if rx else 512
            self.mmio[0x40020034]=512-((rx+2)%512)
        if self.irqs:
            saved=self.x
            pc=self.start('flash_poll_isr' if self.flash_mode else 'poll_isr')
            self.x[2]=(sp-256)&~15
            self.in_isr=True
            self.run_to(pc,set(),max_steps=200000)
            self.in_isr=False
            self.x=saved

def fresh(board,folder,O,updated=False):
    e=App(str(folder/f'app{board}-wch/fw.elf')); e.board=board;e.O=O
    e.flash[:8192]=(folder/f'bl{board}-wch/bl.bin').read_bytes().ljust(8192,b'\xff')
    e.poke(0x08010000,b'\xff'*0xc000)
    target=e.flash[21:37];d=bytearray(256);d[:]=b'\xff'*256;d[:64]=b'\0'*64
    d[:4]=b'QDU6';d[4]=2;d[6]=d[7]=target[7];d[8:15]=target[:7];d[15]=3
    struct.pack_into('>IIIII',d,16,0x00030000,16,0x12345678,0x08002100,0x08002100)
    d[36:39]=bytes([2,3,1]);struct.pack_into('>I',d,60,crc32_words(d[:60]));d[64:66]=b'\xb7\xa5'
    e.poke(0x08002000,d)
    if updated:
        words=[0x51444f48,0,1,0x00030000,0,0,0]
        words.append((~(words[0]^words[1]^words[2]^words[3]^words[4]))&0xffffffff)
        e.poke(0x20000000,struct.pack('<8I',*words))
    e.mmio.update({0x40021000:0x02000002,0x40021004:8,0x40004400:0xc0,
                   0x40013800:0xc0,0x40010c08:1<<10,0x40010808:1<<3,
                   0x4002001c:2048,0x40020020:512,0x40020034:510})
    base,size=e.sym('main');stop={p for p in range(base,base+size,2) if e.rd(p,4)==0x10500073}
    e.main_pc,_=e.run_to(e.start('main'),stop,max_steps=3000000)
    e.main_x=e.x[:];e.stop=stop;e.wpos=0;e.elapsed=0;e.active=True
    assert e.rd(e.addr('upd')+O['desc_ok'],1)==1
    e.wr(e.addr('ms_now'),4,1000)
    e.wr(e.addr('tick_now'),4,20000)
    e.flash_mode=False
    if board==3:
        # Model STREAM selection; AUTO's undecided hold has already ended after
        # valid update headers. Exercise the empty local-pixel configuration too.
        e.wr(e.addr('dl')+O['ch_hold'],1,0)
    return e

def authorize(e,age=0):
    u=e.addr('upd');o=e.O
    e.wr(u+o['ping_seen'],1,1);e.wr(u+o['confirm_done'],1,1)
    e.wr(u+o['ping_ms'],4,(e.rd(e.addr('ms_now'),4)-age)&0xffffffff)

def action(e,a):
    for _ in range(2400):
        e.invoke('upd_action',(a,))
        if e.board!=3 or not e.rd(e.addr('cal_io')+e.O['take'],1): return
        for _ in range(20): e.tick()
    raise AssertionError('action never completed')

def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    rows=[]
    for b in (1,2,3):
        O=offsets(b,a.folder)
        for case in ('confirm','expired','readback','stuck','handover','last-ms'):
            e=fresh(b,a.folder,O)
            authorize(e,100 if case=='expired' else 99 if case=='last-ms' else 0)
            e.no_write=case=='readback';e.program_us=100000 if case=='stuck' else 100
            reset=False
            try: action(e,1 if case=='handover' else 2)
            except Reset: reset=True
            confirmed=e.rd(e.addr('upd')+O['confirmed'],1)
            if case=='confirm': assert confirmed and e.programmed,(b,case)
            if case in ('expired','readback'): assert not confirmed and not reset,(b,case)
            if case=='expired': assert not e.programmed
            if case=='stuck': assert reset and not confirmed and e.elapsed-e.program_start<=20000,(b,case,e.elapsed)
            if case=='handover': assert reset and not e.reset_irq and e.elapsed<=50000,(b,case,e.elapsed)
            if case=='last-ms':
                assert not reset
                if b==3: assert not confirmed and not e.programmed and e.elapsed<=2000,(b,case,e.elapsed)
                else: assert confirmed and e.programmed and e.elapsed<=20000,(b,case,e.elapsed)
            rows.append(dict(board=b,case=case,elapsed_us=e.elapsed,confirmed=confirmed,reset=reset))
            print(rows[-1],flush=True)
    a.output.write_text(json.dumps(rows,indent=2)+'\n')

if __name__=='__main__': main()

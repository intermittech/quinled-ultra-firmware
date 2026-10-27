# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Rejected CONFIG values must not cancel an active digital calibration."""
import sys
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from flash_stores import fresh
from ur_digital_m_cal_target import offsets
elf=sys.argv[1];e=fresh(elf);O=offsets(elf);ux=e.addr('ux');c=e.addr('ucal');buf=e.addr('dl_ring')
e.call('udfx_init',[ux]);e.wr(c+O['phase'],1,1);e.wr(c+O['pending'],1,1)
cal=e.peek(c,e.sym('ucal')[1])
for seq,(cfg,value) in enumerate(((64,8),(67,1001),(75,3),(76,4),(64,0)),1):
 f=bytearray(32);f[:6]=bytes([5,0,seq,cfg,value>>8,value&255]);e.poke(buf,f)
 e.call('ud_slot',[buf,0]);assert e.peek(c,len(cal))==cal,(cfg,value,'cancelled calibration')
f=bytearray(32);f[:6]=bytes([5,0,6,66,0,120]);e.poke(buf,f);e.call('ud_slot',[buf,0])
assert e.rd(c+O['phase'],1)==8 and e.rd(c+O['error'],1)==5,'accepted change must cancel safely'
print('CONFIG admission: invalid/unchanged values preserve job; accepted change cancels')
e.call('udfx_init',[ux]);e.wr(c+O['phase'],1,1);e.wr(c+O['pending'],1,1)
cal=e.peek(c,e.sym('ucal')[1])
f[:6]=bytes([5,255,7,21,0,0]);e.poke(buf,f);e.call('ud_slot',[buf,0])
assert e.peek(c,len(cal))==cal,'default input ceiling must preserve calibration'
f[:6]=bytes([5,255,8,21,3,232]);e.poke(buf,f);e.call('ud_slot',[buf,0])
assert e.rd(c+O['phase'],1)==8 and e.rd(c+O['error'],1)==5,'changed input ceiling must cancel calibration'

for seq,(target,cfg) in enumerate(((0,68),(0,69),(255,21)),30):
 e.call('udfx_init',[ux]);e.wr(c+O['phase'],1,1);e.wr(c+O['pending'],1,1)
 cal=e.peek(c,e.sym('ucal')[1])
 def status():
  e.poke(buf+64,bytes(64));e.call('ud_rc_page',[0,buf+64,buf+96]);assert e.ret==0
  return e.peek(buf+96,24)
 before=status()
 f[:6]=bytes([5,target,seq,cfg,255,255]);e.poke(buf,f);e.call('ud_slot',[buf,0])
 assert e.peek(c,len(cal))==cal,'unknown current cancelled calibration'
 # Command SEQ is acknowledged, but no value becomes accepted or changes the applied CRC.
 assert status()==before,'unknown current changed status/applied CRC'
print('CONFIG unknown-current sentinels rejected without cancelling calibration')

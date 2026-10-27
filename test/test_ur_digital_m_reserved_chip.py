# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Reserved chip must never select a fallback or revive a saved scene/model."""
import sys
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from flash_stores import fresh
from ur_digital_m_cal_target import offsets
elf=sys.argv[1];e=fresh(elf);ux=e.addr('ux');buf=e.addr('dl_ring');c=e.addr('ucal')
e.call('udfx_init',[ux]);e.call('udfx_config',[ux,64,6]);assert e.ret==1
before=e.peek(ux,e.sym('ux')[1]);e.call('udfx_config',[ux,64,7]);assert e.ret==0
assert e.peek(ux,len(before))==before,'reserved CONFIG mutated receiver'
# The wire dispatcher must reject it before cancelling an active calibration.
O=offsets(elf);e.wr(c+O['phase'],1,1);e.wr(c+O['pending'],1,1)
cal=e.peek(c,e.sym('ucal')[1])
f=bytearray(32);f[:6]=bytes([5,0,1,64,0,7]);e.poke(buf,f)
e.call('ud_slot',[buf,0]);assert e.peek(c,len(cal))==cal,'reserved CONFIG aborted calibration'
e.call('udfx_rec_pack',[ux,1,buf]);e.call('udfx_rec_valid',[buf]);assert e.ret==1
r=bytearray(e.peek(buf,128));r[8:10]=b'\x07\x00'
def seal(r):
 crc=0xffff
 for b in r[:-2]:
  crc^=b<<8
  for _ in range(8):crc=((crc<<1)^(0x1021 if crc&0x8000 else 0))&65535
 r[-2:]=crc.to_bytes(2,'little')
seal(r);e.poke(buf,r);e.call('udfx_rec_valid',[buf]);assert e.ret==0,'reserved saved scene accepted'
e.poke(0x08010000,r);e.call('ufx_load',[buf]);assert e.ret==0,'invalid scene selected at boot'
print('reserved chip: CONFIG unchanged, CRC-valid record rejected, boot ignores record')

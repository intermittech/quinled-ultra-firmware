# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Execute the actual six bootloader descriptor checkers in rv32emu."""
import json,pathlib,struct,sys
FW=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(FW/'tools'))
from rv32emu import Emu
folder=pathlib.Path(sys.argv[1]);checks=0

def seal(d):
    crc=0xffffffff
    for i in range(0,60,4):
        crc^=int.from_bytes(d[i:i+4],'little')
        for _ in range(32):crc=((crc<<1)^(0x04c11db7 if crc&0x80000000 else 0))&0xffffffff
    d[60:64]=crc.to_bytes(4,'big')
    return d
for b in (1,2,3):
 for compiler in ('xpack','wch'):
    p=folder/f'bl{b}-{compiler}';e=Emu(str(p/'bl.elf'));t=(p/'bl.bin').read_bytes()[21:37]
    d=bytearray(64);d[:4]=b'QDU6';d[4]=2;d[6]=d[7]=t[7];d[8:15]=t[:7];d[15]=2
    struct.pack_into('>IIIII',d,16,0x00030000,16,0xdeadbeef,0x08002100,0x08002102)
    d[36:39]=bytes([2,3,1]);seal(d)
    def check(x,expected):
        global checks
        e.poke(0x20004000,x);flash=bytes(e.flash);e.writes.clear();e.call('bl_desc_check',[0x20004000])
        assert e.ret==expected,(b,compiler,e.ret,expected,x.hex())
        assert bytes(e.flash)==flash and not e.writes,'descriptor check touched hardware'
        checks+=1
    check(d,0)
    for pos in range(64):
        x=d.copy();x[pos]^=1;check(x,1) # CRC/magic failure, every descriptor byte
    for pos in (8,9,10,11,12,14,15):
        x=d.copy();x[pos]^=128;check(seal(x),2)
    for lo,hi in ((t[7]+1,255),(0,t[7]-1),(9,8)):
        x=d.copy();x[6:8]=bytes([lo,hi]);check(seal(x),2)
    for features in (1,4,8,16,255):
        x=d.copy();x[13]=features;check(seal(x),2)
    for pos in (5,39,56,57,58,59):
        x=d.copy();x[pos]=1;check(seal(x),10)
    x=d.copy();x[4]=1;check(seal(x),10)
    for pos,value in ((20,0),(20,3),(20,57092),(20,0xfffffffc),(28,0x08000000),(32,0x080020fe),(32,0x08002110),(32,0x08002101)):
        x=d.copy();struct.pack_into('>I',x,pos,value);check(seal(x),3)
    # Version ranges and informational app format bytes do not narrow acceptance.
    x=d.copy();x[6]=0;x[7]=255;x[13]=0;x[15]=1;x[36:39]=b'\xff\xff\xff';check(seal(x),0)
print(f'{checks} compiled boot descriptor checks PASS; all reject paths leave flash/MMIO untouched')

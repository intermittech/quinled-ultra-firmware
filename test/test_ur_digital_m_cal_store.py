# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""UDC1 compiled pack/load/pagepair: every halfword cut, tombstones, CRC, wrap,
old/unknown formats, natural-dark policy and failed-write retention."""
import sys,struct
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from flash_stores import fresh,FlashEmu,FLASH_R
from ur_digital_m_cal_target import offsets
elf=sys.argv[1];O=offsets(elf);e=fresh(elf);A=e.addr;c=A('ucal');buf=A('dl_ring');ux=A('ux');enc=ux+O['enc']
PAGES=(0x08012000,0x08013000);checks=0
def check(ok,msg):
    global checks
    checks+=1
    if not ok:raise AssertionError(msg)
def model(generation=1,flags=9):
    b=bytearray(40)
    struct.pack_into('<18H',b,0,120,100,100,0x88,24000,12000,2000,2100,2200,0,0,123,2,30,35,32,generation,generation)
    b[36:39]=bytes([0,3,flags]);return bytes(b)
def load(em=e):
    em.call('ucal_load',[c]);return em.ret,em.peek(c,40)
def record(em=e,seq=1):
    em.call('udcal_pack',[c,seq,buf]);return em.peek(buf,64)
e.poke(c,model());r=record();e.call('udcal_valid',[buf]);check(e.ret==1,'valid record')
check(r[:4]==b'UDC1' and r[8:47]==model()[:39] and r[50]==3 and r[51]==1,'record layout')
for bpp,order in ((4,0x688),(5,0x4688)):
    m=bytearray(model());m[36]=3 if bpp==4 else 5;m[37]=bpp
    struct.pack_into('<H',m,6,order)
    for j in range(3,bpp):struct.pack_into('<H',m,12+2*j,2000)
    e.poke(c,m);record();e.call('udcal_valid',[buf]);check(e.ret==1,'record bpp '+str(bpp))
e.poke(c,model())
for byte in range(64):
    x=bytearray(r);x[byte]^=1;e.poke(buf,x);e.call('udcal_valid',[buf]);check(e.ret==0,'CRC covers byte '+str(byte))
for magic in (b'CAL2',b'UDC2'):
    x=bytearray(r);x[:4]=magic;e.poke(PAGES[0],x);check(load()[0]==0,'unknown magic ignored')
for p in PAGES:e.poke(p,b'\xff'*256)
load();e.poke(c,model());e.call('ucal_store',[c]);check(e.ret==1,'first store')
base=[e.peek(p,256) for p in PAGES]
e.poke(c,model(2));e.call('ucal_store',[c]);check(e.ret==1,'second store');new=e.peek(PAGES[0]+64,64)
for cut in range(33):
    q=fresh(elf)
    for p,b in zip(PAGES,base):q.poke(p,b)
    q.poke(PAGES[0]+64,new[:cut*2]+b'\xff'*(64-cut*2))
    found,m=load(q)
    check(found==1 and int.from_bytes(m[32:34],'little')==(2 if cut==32 else 1),'cut '+str(cut))
    q.poke(c,model(3));q.call('ucal_store',[c]);check(q.ret==1,'recover write '+str(cut))
    check(load(q)[1][32:34]==b'\3\0','recover newest '+str(cut))
# A completed clear can never resurrect the preceding valid calibration.
e.poke(c,model(2,0));e.call('ucal_store',[c]);check(e.ret==1,'tombstone store')
tomb=e.peek(PAGES[0]+128,64)
for cut in range(33):
    q=fresh(elf);q.poke(PAGES[0],base[0]);q.poke(PAGES[1],base[1])
    q.poke(PAGES[0]+64,tomb[:cut*2]+b'\xff'*(64-cut*2))
    found,m=load(q);check(found==1 and bool(m[38]&1)==(cut<32),'tombstone cut '+str(cut))
for p in PAGES:e.poke(p,b'\xff'*256)
e.poke(c,model(5));old=record(seq=0xFFFFFFFF);e.poke(c,model(6));new=record(seq=0)
e.poke(PAGES[0],old);e.poke(PAGES[1],new);check(load()[1][32:34]==b'\6\0','sequence wrap')
# Valid CRC but bad format/ranges/reserved bytes must still fail.
def crc(b):
    n=0xffff
    for x in b:
        n^=x<<8
        for _ in range(8):n=((n<<1)^(0x1021 if n&0x8000 else 0))&65535
    return n
for off,value in ((50,2),(51,0),(45,2),(46,0x89),(44,7),(44,8),(12,0),(14,0),(20,0)):
    x=bytearray(r);x[off]=value
    if off in (12,14,20):x[off+1]=0
    x[62:64]=struct.pack('<H',crc(x[:62]));e.poke(buf,x);e.call('udcal_valid',[buf]);check(e.ret==0,'range/version '+str(off))
for sentinel in (65535,0xe339):
    x=bytearray(r);x[62:64]=struct.pack('<H',sentinel);e.poke(buf,x);e.call('udcal_valid',[buf]);check(e.ret==0,'erased CRC')
# Real application's save policy: no output changes to create an opportunity.
e=fresh(elf);A=e.addr;c=A('ucal');ux=A('ux');enc=ux+O['enc'];dl=A('dl')
e.wr(A('flash_boot'),1,0) # cal_flash_service runs only after boot housekeeping
e.call('udfx_init',[ux]);load();e.poke(c,model(flags=11));e.wr(dl+O['dl_state'],1,0)
e.wr(A('ms_now'),4,5000)
for local,dark,phase in ((0,0,0),(1,0,0),(1,1,5),(1,1,8)):
    e.wr(ux+O['local'],1,local);e.wr(ux+O['dark'],1,dark);e.wr(c+O['phase'],1,phase)
    e.wr(enc+O['low_w'],4,5000);e.wr(enc+O['reset_w'],2,680)
    before=[e.peek(p,256) for p in PAGES];pool=e.peek(A('ux_pool'),2000)
    e.call('cal_flash_service');check(before==[e.peek(p,256) for p in PAGES] and pool==e.peek(A('ux_pool'),2000),'defer without blink')
e.wr(c+O['phase'],1,9);e.wr(enc+O['ready'],1,0);e.call('cal_flash_service')
check(not e.rd(c+O['flags'],1)&2,'natural dark saved')
check(load()[0]==1,'natural dark valid flash record')
# A controller program error must not advance the record sequence or replace
# the previous valid generation. Retrying can later commit the same RAM model.
class RejectFlash(FlashEmu):
    fail=False
    def rd(self,a,n):
        if self.fail and a==FLASH_R+0x0C:return 0x30  # CH32 EOP plus WRPRTERR
        return super().rd(a,n)
q=RejectFlash(elf);q.mmio[0x40004400]=0xC0
for p in PAGES:q.poke(p,b'\xff'*256)
q.call('ucal_load',[c]);q.poke(c,model(1));q.call('ucal_store',[c]);check(q.ret==1,'error fixture first write')
q.poke(c,model(2));q.fail=True;q.call('ucal_store',[c]);check(q.ret!=1,'program error returned')
q.fail=False;check(load(q)[1][32:34]==b'\1\0','program error retains old generation')
q.poke(c,model(2));q.call('ucal_store',[c]);check(q.ret==1,'retry after program error')
check(load(q)[1][32:34]==b'\2\0','retry commits new generation')
print('UDC1:',checks,'checks passed')

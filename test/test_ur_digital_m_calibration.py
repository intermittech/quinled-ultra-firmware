# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Real compiled calibration, adapter, wire pages and bounded foreground jobs.
Run with a BOARD=3 no-inline API ELF. Fake ADC is independent of its state machine;
the physical frame delay is included. Encoder publication and no-request behavior
are checked directly on main_ur_digital_m's adapter and its real frame buffer.
"""
import sys,struct
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from flash_stores import fresh
from ur_digital_m_cal_target import offsets
elf=sys.argv[1]; O=offsets(elf);e=fresh(elf);A=e.addr
c=A('ucal');ux=A('ux');enc=ux+O['enc'];dl=A('dl');cfg=A('dl_ring');arg=cfg+96;out=cfg+128
checks=0;worst={}
def check(v,msg):
    global checks
    checks+=1
    if not v: raise AssertionError(msg)
def call(fn,args=()):
    e.call(fn,args);worst[fn]=max(worst.get(fn,0),e.cycles());return e.ret
def C(key,n=1):return e.rd(c+O[key],n)
def W(key,v,n=1):e.wr(c+O[key],n,v)
def config(n=120,bpp=3):
    b=bytearray(40);struct.pack_into('<6H',b,0,n,0,0,(0x88,0x688,0x4688)[bpp-3],24000,12000);b[37]=bpp;e.poke(cfg,b)
def request(seq=1,n=120,bpp=3):
    config(n,bpp);call('udcal_request',[c,seq,cfg,600 if bpp==3 else 500 if bpp==4 else 400,2250,0,0])
request()
ms=0;sent=0;held=0;frames=0
while C('phase') not in (9,10) and ms<5500:
    ms+=1
    if C('pending')==1 and not sent:sent=ms+21;W('pending',2)
    if sent and ms>=sent:
        # Count selected bytes independently by spatial distribution.
        k=C('k',2);pos=C('pos',2);mask=C('mask');duty=C('duty');n=e.rd(c+O['candidate']+4,2)
        lit=(1 if pos and pos<=120 else 0) if pos else k
        held=lit*mask.bit_count()*20*duty/255 if k else 0
        sent=0;frames+=1;call('udcal_latched',[c,ms])
    # exact average, 20 samples with 1/20 mA resolution
    call('udcal_feed',[c,round((100+held)*20),20,65536,ms,0,24000,12000])
    call('udcal_step',[c,65536])
    if C('phase')==8 and C('finish'):call('udcal_restored',[c])
check(C('phase')==9 and C('error')==0,f'completed phase={C("phase")} error={C("error")}')
check(e.rd(c+2,2)==120,'detected 120')
check(abs(e.rd(c+12,2)-2400)<25,'bus coefficient')
for part in (0,1,2):
    e.poke(arg,bytes([part,1])+bytes(22));call('ud_rc_page',[6,arg,out]);check(e.ret==6,'page supported')
    d=e.peek(out,24)
    if part==0:check(d[0]==3 and d[1]==2 and int.from_bytes(d[9:11],'big')==e.rd(c+12,2),'coefficient page')
    if part==1:check(d[0]==2 and d[1]&1 and d[2]==9 and int.from_bytes(d[8:10],'big')==120,'metadata page')
e.poke(arg,b'\3');call('ud_rc_page',[6,arg,out]);check(e.ret==255,'unsupported part')
saved=e.peek(c,40);call('udcal_request',[c,1,cfg,600,2250,ms,0]);check(C('phase')==9 and e.peek(c,40)==saved,'duplicate SEQ')
call('udfx_init',[ux]);e.wr(ux+O['npx'],2,120);e.poke(ux+O['pack']+O['order'],bytes(range(5)))
for name,v in (('vs_bus',24000),('vs_vb',12000)):
    e.wr(A(name)+O['vs_valid'],1,1);e.wr(A(name)+O['vs_mv'],4,v)
call('cal_match');check(e.rd(c+O['flags'],1)&1,'matching fingerprint retained')
e.wr(A('vs_bus')+O['vs_mv'],4,26400);call('cal_match')
check(e.rd(c+O['flags'],1)&5==5,'post-calibration voltage marks stale')
call('udcal_page',[c,0,out]);check(e.peek(out,2)==bytes(2),'stale coefficients not advertised as usable')
e.wr(A('vs_bus')+O['vs_mv'],4,24000);call('cal_match');check(not e.rd(c+O['flags'],1)&4,'reference voltage recovers')
epoch=e.rd(c+O['epoch'],2);e.wr(ux+O['npx'],2,121);call('cal_match')
check(not e.rd(c+O['flags'],1)&1 and e.rd(c+O['epoch'],2)==epoch+1,'changed geometry invalidates and increments epoch')

# Real adapter with a completed full-reset frame, one slice at a time.
e=fresh(elf);A=e.addr;c=A('ucal');ux=A('ux');enc=ux+O['enc'];dl=A('dl');cfg=A('dl_ring');arg=cfg+96;out=cfg+128
call('udfx_init',[ux]);call('udfx_config',[ux,66,600])
e.wr(ux+O['npx'],2,600);e.wr(ux+O['local'],1,1);e.wr(dl+O['dl_state'],1,0)
e.wr(enc+O['state'],1,0);e.wr(enc+O['low_w'],4,5000);e.wr(enc+O['reset_w'],2,680)
e.poke(A('ux_pool'),bytes([73])*2000)
# No explicit request: dark state makes ordinary render a no-op.
e.wr(ux+O['dark'],1,1);before=e.peek(A('ux_pool'),2000);call('cal_work')
check(e.peek(A('ux_pool'),2000)==before,'no unsolicited output')
request(n=600);call('cal_work')
for _ in range(38):call('cal_work')
check(C('pending')==2 and e.rd(enc+O['ready'],1)==1,'publish one full cap frame')
check(e.peek(A('ux_pool'),1800)==bytes(1800),'full black bytes')
e.wr(enc+O['ready'],1,0);e.wr(enc+O['frames'],4,1)
e.wr(enc+O['low_w'],4,680);call('cal_work');check(C('pending')==2,'wait for reset to leave delay ring')
e.wr(enc+O['low_w'],4,1192);call('cal_work');check(C('sampling')==1,'latch acknowledged')
call('flash_may',[0]);check(e.ret==0,'no program during job')
call('flash_may',[1]);check(e.ret==0,'no erase during job')
call('life_admit');check(e.ret==0,'no dip store during job')
W('duty',255);W('mask',7);W('k',32,2);W('pending',1);e.wr(c+O['candidate']+4,2,600)
for _ in range(38):call('cal_work')
check(sum(e.peek(A('ux_pool'),1800))==32*3*255,'distributed raw channels bypass gamma/dither')

# Five bytes/pixel is the largest raw-render slice. Full masks at a safe low
# duty exercise every channel rather than taking the all-black fast path.
e.wr(enc+O['ready'],1,0);e.wr(ux+O['chip']+O['chip_bpp'],1,5)
e.wr(ux+O['npx'],2,400)
e.poke(ux+O['pack']+O['order'],bytes(range(5)))
W('cap',400,2);W('duty',64);W('mask',31);W('k',32,2);W('pending',1)
e.wr(c+O['candidate']+4,2,400)
for _ in range(25):call('cal_work')
check(sum(e.peek(A('ux_pool'),2000))==32*5*64,'five-channel raw slice')
e.wr(enc+O['ready'],1,0);e.poke(ux+O['pack']+O['order'],bytes([4,3,2,1,0]))
W('mask',1);W('pending',1)
for _ in range(25):call('cal_work')
expected=bytearray(2000)
for j in range(32):expected[(j*400//32)*5+4]=64
check(e.peek(A('ux_pool'),2000)==expected,'resolved reverse wire order selects only logical red')

# The background Fire scene advances while calibration owns the wire. Measure
# every preparation slice, including catch-up, and ensure restoration does not
# reset the scene's anchor or RESTART counter.
e.wr(enc+O['ready'],1,0);W('pending',0);W('sampling',0)
f=bytearray(32);f[:4]=bytes([11,15,20,16]);f[4:8]=struct.pack('>i',-500)
f[8:11]=bytes([255,255,255]);f[11:14]=bytes([255,0,0]);f[20]=128;f[28]=55
e.poke(arg,f);call('udfx_pixel_fx',[ux,arg,0])
for t in range(200):
    call('udfx_service',[ux,A('dl'),t*4,0,256])
    call('cal_work')
seg=ux+O['seg'];anchor=e.rd(seg+O['anchor'],4);restarts=e.rd(seg+O['restarts'],2)
pool=e.peek(A('ux_pool'),2000)
# AUTO/CHAIN handoff may change the normal render quota while calibration owns
# the wire. Neither direct render nor background preparation may touch it.
call('udfx_chain',[ux,1])
for _ in range(40):
    call('udfx_render',[ux]);call('cal_work')
check(e.peek(A('ux_pool'),2000)==pool,'scene preparation never publishes a test frame')
W('phase',8);W('error',6);W('finish',1);call('cal_work')
check(C('phase')==10 and not e.rd(ux+O['calibration'],1),'restore releases ownership')
check(e.rd(seg+O['anchor'],4)==anchor and e.rd(seg+O['restarts'],2)==restarts,'restore preserves Fire anchor/restarts')

# Request admission through the actual IDENTIFY handler, without publishing any
# frame on invalid AUTO/update/argument requests. Cancellation also uses the wire.
e=fresh(elf);A=e.addr;c=A('ucal');ux=A('ux');enc=ux+O['enc'];dl=A('dl');arg=A('dl_ring')
call('udfx_init',[ux]);e.wr(ux+O['npx'],2,120);e.wr(A('dl_on'),1,1)
for name,v in (('vs_bus',24000),('vs_vb',12000)):
    e.wr(A(name)+O['vs_valid'],1,1);e.wr(A(name)+O['vs_mv'],4,v)
def identify(seq,action=2,bad=0):
    f=bytearray(32);f[:5]=bytes([6,0,seq,action,bad]);e.poke(arg,f);call('ud_slot',[arg,0])
pool=e.peek(A('ux_pool'),2000)
identify(1);check(C('phase')==10 and C('error')==12 and not C('pending'),'undecided AUTO rejected')
e.wr(A('uc')+O['mode'],1,2);e.wr(A('upd')+O['upd_run'],1,1)
identify(2);check(C('error')==12 and not C('pending'),'update rejected')
e.wr(A('upd')+O['upd_run'],1,0)
identify(3,bad=1);check(C('error')==13 and not C('pending'),'reserved argument rejected')
check(e.peek(A('ux_pool'),2000)==pool,'rejected requests never change output bytes')
identify(4);check(C('phase')==1 and C('pending')==1,'explicit wire request queued')
identify(4);check(C('phase')==1,'wire request duplicate does not restart')
identify(5,4);check(C('phase')==8 and C('error')==6,'wire cancel goes through black restore')
print('ur_digital_m calibration target:',checks,'checks,',ms,'ms,',frames,'frames; worst cycles',worst)
for fn in ('udcal_feed','udcal_step','cal_work','ud_rc_page','ud_slot'):
    check(worst[fn]<=8000,fn+' exceeds 8000 cycles: '+str(worst[fn]))

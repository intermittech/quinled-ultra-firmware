# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Actual release CHAIN input, idle PING, metadata readback and UID/INDEX replies."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import struct
import sys
import test_ur_digital_m_chain_budget as t
from test_boot_api_app import crc32_words, offsets

class Handover(Exception): pass

def block(typ,session=0,index=0,data=b''):
    f=bytearray(40);f[:4]=bytes([typ,session])+index.to_bytes(2,'big');f[6:6+len(data)]=data
    f[38:40]=t.crc(f[:38],16,0x1021,0xffff).to_bytes(2,'big')
    return f

class Wire(t.Line):
    def packet(self,data,flags,name,gap=500,slots=1):
        start=len(self.line)
        end=start+((len(data)+8)*8-1)*t.PERIOD+t.H1
        f=data+t.header(end//36,flags)
        for b in f:
            for bit in range(7,-1,-1):
                hi=t.H1 if b>>bit&1 else t.H0
                self.line.extend(b'\1'*hi+b'\0'*(t.PERIOD-hi))
        last=len(self.line)-(t.PERIOD-(t.H1 if f[-1]&1 else t.H0))
        self.frame_ends.append((name,last))
        if flags&1:
            self.low_to(last/36+60)
            self.line.extend(b'\1'*max(0,int((1000+500*slots)*36)-(len(self.line)-last)))
        self.low_to(len(self.line)/36+gap)

class Receiver(t.Board):
    irqs=True
    def __init__(self,elf):
        super().__init__(elf);self.replies=[];self.status_writes=[]
    def _i(self,ins,pc,x):
        if ins&127==0x73 and ins>>20==0x800:
            f=(ins>>12)&7;v=x[(ins>>15)&31]
            if f==2 and v&0x88: self.irqs=True
            if f==3 and v&0x88: self.irqs=False
        return super()._i(ins,pc,x)
    def wr(self,a,n,v):
        if a==0xe000e048 and v==0xbeef0080:
            self.scenario.on_reset(self)
        if a==0x40010c10 and v&(1<<10) and hasattr(self,'model_us'):
            self.replies.append(dict(time_us=self.model_us,answer=self.peek(self.addr('ud_ans'),32).hex()))
        if 0x08002040<=a<0x08002100:
            self.status_writes.append(dict(address=hex(a),value=v,time_us=getattr(self,'model_us',0)))
        super().wr(a,n,v)

class Scenario:
    board=Receiver
    def __init__(self,folder,uid,index,profile,phase,session=0):
        self.folder=folder;self.uid=uid;self.index=index;self.session=session
        t.PERIOD,t.H0,t.H1=((45,11,34),(60,15,45),(90,23,68),(180,45,135))[profile]
        self.line=Wire();self.line.low_to(3000+phase)
        # Qualification is seeded here; the separate 200-case training grid
        # proves acquisition. These cases isolate update dispatch and replies.
        for i in range(3): self.line.packet(t.slot(bytes([0,0,i])),0,'training',9000)
        self.action_ms=None;self.taking=session==255
        if self.taking:
            d=self.image_descriptor((folder/'bl3-wch/bl.bin').read_bytes()[21:37])
            struct.pack_into('>I',d,16,0x00030100);struct.pack_into('>I',d,60,crc32_words(d[:60]))
            for _ in range(3):
                for half in (0,1): self.line.packet(block(0x81,1,half,d[32*half:32*(half+1)]),8,'announce',500)
            self.line.low_to(len(self.line.line)/36+60000)
        else:
            for _ in range(3): self.line.packet(block(0x85,session),8,'ping',120000)
        for part,(page,arg) in enumerate(((0,0),(4,0),(4,2),(5,0))):
            for seq in range(3):
                # Position in the chain and response-window INDEX are independent.
                poll=t.slot(bytes([7,255,seq+0x41+3*part,index,index+1,page,0,arg]))
                self.line.packet(poll,1,'poll-'+str(page),10000,index+1)
        self.seen_confirm=None
    @staticmethod
    def image_descriptor(target):
        d=bytearray(b'\xff'*256);d[:64]=b'\0'*64
        d[:4]=b'QDU6';d[4]=2;d[6]=d[7]=target[7];d[8:15]=target[:7];d[15]=3
        struct.pack_into('>IIIII',d,16,0x00030000,16,0x12345678,0x08002100,0x08002100)
        d[36:39]=bytes([2,3,1]);struct.pack_into('>I',d,60,crc32_words(d[:60]));d[64:66]=b'\xb7\xa5'
        return d
    def prepare(self,e):
        e.scenario=self
        e.flash[:8192]=(self.folder/'bl3-wch/bl.bin').read_bytes().ljust(8192,b'\xff')
        d=self.image_descriptor(e.flash[21:37])
        e.poke(0x08002000,d)
        self.descriptor=bytes(d[:64]);self.target=bytes(e.flash[21:40])
        for i,b in enumerate(self.uid): e.mmio[0x1ffff7e8+i]=b
    def observe(self,e):
        if self.seen_confirm is None and e.rd(0x08002042,2)==0xa5c3:
            self.seen_confirm=e.model_us
        if self.taking and self.action_ms is None and e.rd(e.addr('cal_io')+self.O['take'],1):
            self.action_ms=e.model_us
    def on_reset(self,e):
        assert self.taking and self.action_ms is not None and not e.irqs
        words=struct.unpack('<8I',e.peek(0x20000000,32))
        assert words[0]==0x51444f48 and words[1]==0x55504421
        assert words[7]==(~(words[0]^words[1]^words[2]^words[3]^words[4]))&0xffffffff
        enc=e.addr('ux')+self.O['e']
        assert e.rd(enc+self.O['state'],1)==0 and e.rd(enc+self.O['frames'],4)>0
        assert e.rd(enc+self.O['low_w'],4)>=e.rd(enc+self.O['reset_w'],2)+512
        assert e.model_us-self.action_ms<=50000
        self.result=dict(handover_us=e.model_us-self.action_ms,irq_masked=True,request=hex(words[1]))
        raise Handover()
    def finish(self,e,result):
        result.update(uid=self.uid.hex(),index=self.index,session=self.session,
                      writes=e.status_writes,replies=e.replies,confirmed_us=self.seen_confirm)
        assert result['max_isr']+60<=7200,result
        assert bool(self.seen_confirm)==(self.session==0),result
        answers=[bytes.fromhex(r['answer']) for r in e.replies]
        assert answers,result
        for a in answers:
            assert a[2]>>4==self.index and int.from_bytes(a[4:6],'big')==t.crc(self.uid,16,0x1021,0xffff),a.hex()
            assert int.from_bytes(a[30:32],'big')==t.crc(a[:30],16,0x1021,0xffff)
        identity=[a for a in answers if a[1]==0x04 and 0x44<=a[3]<=0x46]
        target=[a for a in answers if a[1]==0x04 and 0x47<=a[3]<=0x49]
        update=[a for a in answers if a[1]==0x05]
        assert identity and all(a[6:18]==self.uid for a in identity),result
        assert target and all(a[6:25]==self.target for a in target),result
        assert update and all(bool(a[7]&2)==(self.session==0) for a in update),result
        assert all(a[7]&1 and a[10:22]==self.descriptor[16:28] and a[22:26]==self.descriptor[60:64] for a in update),result
        return result

def case(job):
    folder,profile,index,session=job;folder=Path(folder)
    elf=str(folder/'app3-wch/fw.elf');O=t.offsets(elf)
    scenario=Scenario(folder,bytes(range(index,index+12)),index,profile,25,session)
    scenario.O=offsets(3,folder) if session==255 else None
    try: r=t.run_once(elf,O,500,25,qualified=1,profile=profile,scenario=scenario)
    except Handover: r=scenario.result
    r.update(profile=profile,index=index,session=session)
    return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--profiles',default='0,1,2,3');a=p.parse_args();rows=[]
    jobs=[(str(a.folder),profile,index,session) for profile in map(int,a.profiles.split(','))
          for index,session in ((1,0),(2,7),(1,255))]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(case,jobs):
            rows.append(r);print(json.dumps(r),flush=True)
            a.output.write_text(json.dumps(rows,indent=2)+'\n')

# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compiled lossless-run decoder across clock, sample and 50 us tick phases.
Model cycles are not a silicon timing certificate. Uses the production source.
"""
import json, math, os, pathlib, struct, subprocess, sys
FW=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(FW/'tools'));sys.path.insert(0,str(FW/'test'))
from rv32emu import Emu
from ur_digital_m_target import target_gcc
out=pathlib.Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
board=int(sys.argv[2]) if len(sys.argv)>2 else 3
assert board in (1,2,3)
elf=out/'line.elf'
subprocess.run([target_gcc(),'-march=rv32imc_zicsr_zifencei','-mabi=ilp32',
               '-Os' if board==3 else '-O2',f'-DBOARD={board}',
               '-fno-builtin','-fno-common','-nostdlib','-Wl,-e,b_reset','-T',str(FW/'test/fx_rv.ld'),str(FW/'test/line_profile_bench.c'),str(FW/'src/core/ws_decode.c'),'-o',str(elf)],check=True,timeout=60)
e=Emu(str(elf));samples=e.addr('samples');dest=e.addr('decoded')
# Four training payloads plus changing bytes exercise both classes and word boundaries.
data=bytes([0x55,0xaa,0,255])*12
bits=[(x>>(7-i))&1 for x in data for i in range(8)]
rows=[];checks=0
for profile,(period,k,D) in enumerate(zip([1250,5000/3,2500,5000],[22,30,45,90],[218,292,440,588])):
    e.call('b_threshold',[profile]);assert e.ret==k
    peak=0
    for ppm in (-22000,0,22000):
      f=36*(1+ppm/1e6); quantum=1000/f
      for phase in range(16):
       # Edges at every sample phase. Vary the first service point across a tick.
       for distortion in (-D,D):
        first=16+phase/16; end=math.ceil(first+len(bits)*period/quantum)+32
        words=[0]*((end+15)//16)
        for i,bit in enumerate(bits):
          start=first+i*period/quantum
          high=(period*(.75 if bit else .25)+(10 if distortion>0 else -10)+distortion)/quantum
          for j in range(math.ceil(start),math.ceil(start+high)):
            words[j//16]|=0x8000>>(j%16)
        e.call('b_reset');pos=0;chunk=1+phase*7
        while pos<len(words):
          n=min(chunk,len(words)-pos);e.poke(samples,struct.pack('<'+'H'*n,*words[pos:pos+n]));e.call('b_feed',[n]);peak=max(peak,e.cycles());pos+=n;chunk=112
        e.call('b_profile');assert e.ret==profile,(profile,ppm,phase,e.ret)
        e.call('b_decode',[0]);assert e.ret==384 and e.peek(dest,48)==data,(profile,ppm,phase,distortion,e.ret,e.peek(dest,48).hex())
        e.call('b_decode',[1]);assert e.ret==64 and e.peek(dest,8)==data[-8:]
        checks+=1
    Q=1000/(36*.978)
    rows.append(dict(profile=profile,threshold=k,M0_ns=round(1000*k/(36*1.022)-(period/4+10+D)-Q,3),M1_ns=round(period*.75-10-D-1000*k/(36*.978)-Q,3),feed_peak_model_cycles=peak))
(out/'profiles.json').write_text(json.dumps(dict(board=board,cases=checks,profiles=rows),indent=2))
print(json.dumps(dict(board=board,cases=checks,profiles=rows),indent=2))

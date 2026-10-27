# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compare the actual UR_DIGITAL_M diagnostic service before/after schema compression.

Arguments: baseline API ELF, current API ELF, baseline console_schema.json.
All values (including signed extrema) are matched by source expression; only
the schema's field grouping/width representation may change. No UART model or
live hardware is needed: intercept the write call, after real formatting/CRC.
"""
import json
from pathlib import Path
import random
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rv32emu import Emu
import decode_console as decoder

class Capture(Emu):
    def __init__(self,path):
        super().__init__(path);self.lines=[]
    def hook(self,pc,x):
        if pc==self.addr('uart_room'):
            x[10]=256;return x[1],'jump'
        if pc==self.addr('uart_write'):
            a=x[10];s=bytearray()
            while self.rd(a,1):s.append(self.rd(a,1));a+=1
            self.lines.append(bytes(s));return x[1],'jump'
    def _i(self,ins,pc,x):return self.hook(pc,x) or super()._i(ins,pc,x)
    def _c(self,ins,pc,x):return self.hook(pc,x) or super()._c(ins,pc,x)

def capture(path,seed):
    e=Capture(path);rng=random.Random(seed)
    for name in ['ux','ucal','dl','en','pw','vdip','bc','uds','mt','vs_bus','vs_vb','upd',
                 'i_avg','cpu_peak','ovr_late','saves','flash_err','reset_flags','fw_version','upd_res','bl_prot','uc','hw_diag']:
        if name in e.syms:
            a,n=e.syms[name];e.poke(a,bytes(rng.randrange(256) if seed>1 else seed*255 for _ in range(n)))
    for name,value in [('up_s',6),('ms_now',6000),('con_ms',0),('con_status_s',0),('con_part',0),('con_flash_err',0)]:
        a,n=e.syms[name];e.poke(a,value.to_bytes(n,'little'))
    e.call('con_banner')
    e.call('console_service') # live, then start the minute dump
    for _ in range(30):e.call('console_service')
    return e.lines

def semantic(lines,schema):
    decoder.SCHEMA=schema;decoder.TABLE={r['record']:r for r in schema['records']}
    values={}
    for line in lines:
        d=decoder.decode(line);r=decoder.TABLE[d['record']]
        for f in r['fields']:
            if 'expr' in f:key=f['expr']
            elif r.get('ud_emit'):
                base=r['base'].lstrip('&');key=base+('' if f['member'].startswith('[') else '.')+f['member']
            else:key=r['record']+':'+f['name']
            key=(key,d['index']);v=d['fields'][f['name']]
            if key in values:assert values[key]==v,key
            values[key]=v
    return values

old=json.loads(Path(sys.argv[3]).read_text())
new=json.loads((Path(__file__).resolve().parents[1]/'tools/console_schema.json').read_text())
count=0
for seed in range(34):
    a=semantic(capture(sys.argv[1],seed),old)
    b=semantic(capture(sys.argv[2],seed),new)
    assert a==b,(seed,[(k,a.get(k),b.get(k)) for k in a.keys()|b.keys() if a.get(k)!=b.get(k)][:8])
    count+=len(a)
print(f'UR_DIGITAL_M console: {count} semantic values identical in 34 full compiled service dumps, including zero/all-ones states')

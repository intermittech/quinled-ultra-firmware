# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Worst raw-channel slice on the actual LTO release image, all pixel caps.
The API test covers reductions, pages, preparation and restoration separately.
"""
import sys
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from test_ur_digital_m_chain_budget import Board
from ur_digital_m_cal_target import offsets
elf=sys.argv[1];O=offsets(elf);worst=0
for bpp,cap in ((3,600),(4,500),(5,400)):
    e=Board(elf);e.poke(0x08010000,b'\xff'*0xC000)
    e.mmio.update({0x40021000:0x02000002,0x40021004:8,0x40004400:0xC0,0x40010C08:1<<10,0x40010808:1<<3})
    start,size=e.sym('main');wfi={a for a in range(start,start+size,2) if e.rd(a,4)==0x10500073}
    e.run_to(e.start('main'),wfi,max_steps=3000000)
    c=e.addr('ucal');ux=e.addr('ux');enc=ux+O['enc'];dl=e.addr('dl')
    e.wr(dl+O['dl_state'],1,0);e.wr(dl+O['dl_ch_win'],4,0)
    e.wr(ux+O['local'],1,1);e.wr(ux+O['npx'],2,cap);e.wr(ux+O['chip']+O['chip_bpp'],1,bpp)
    e.poke(ux+O['pack']+O['order'],bytes(range(5)))
    e.wr(enc+O['state'],1,0);e.wr(enc+O['ready'],1,0)
    e.wr(enc+O['reset_w'],2,680);e.wr(enc+O['low_w'],4,5000)
    for field,n,value in (('phase',1,1),('pending',1,1),('cap',2,cap),('limit',2,2250),
                          ('mask',1,(1<<bpp)-1),('duty',1,64),('k',2,32)):
        e.wr(c+O[field],n,value)
    e.wr(c+O['candidate']+4,2,cap)
    cycles=[]
    for _ in range(1+(cap+15)//16):
        e.call('cal_work');cycles.append(e.cycles())
    assert e.rd(enc+O['ready'],1)==1 and not e.rd(c+O['error'],1),'frame not admitted'
    assert sum(e.peek(e.addr('ux_pool'),cap*bpp))==32*bpp*64,'raw frame'
    peak=max(cycles);worst=max(worst,peak);assert peak<=8000,(bpp,peak)
    print('release calibration raw slice:',bpp,cap,peak,'cycles')
print('release calibration cycles: passed, worst',worst)

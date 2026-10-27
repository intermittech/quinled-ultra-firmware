# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Offsets compiled from the real target types, not hand-maintained ABI guesses."""
import os,sys,struct,subprocess,tempfile
from pathlib import Path
from ur_digital_m_target import target_gcc
FW=Path(__file__).resolve().parents[1]
KEYS='size model_size candidate phase error pending finish duty mask k pos cap stage sampling limit now_ms block_ms start_ms flags count epoch enc pack order calibration begun state low_w reset_w ready frames lost nbytes i w dl_state dl_wpos dl_ch_on dl_ch_win dl_local vs_valid vs_mv mode local dark want_stream want_local npx chip seg seg_size anchor restarts chip_bpp'.split()
KEYS += ['upd_run','upd_transparent','reconf']
def offsets(elf):
    with tempfile.TemporaryDirectory() as td:
        obj=Path(td)/'uc.o'
        cmd=[target_gcc(),'-march=rv32imac_zicsr_zifencei','-mabi=ilp32','-Os','-c','-DCH32V20x_D6','-DUPD_CRC_ULTRA',
             '-DBOARD=3','-DMON_EXTRA=0','-DCONSOLE_FULL=0']
        cmd += ['-I'+str(p) for p in (FW/'src/mcu/ch32v20x',FW/'src',FW/'src/core',Path(elf).parent/'gen')]
        subprocess.run(cmd+[str(FW/'test/ur_digital_m_cal_offsets.c'),'-o',str(obj)],check=True)
        data=obj.read_bytes()
    shoff=struct.unpack_from('<I',data,32)[0]; ents,n,idx=struct.unpack_from('<HHH',data,46)
    secs=[struct.unpack_from('<10I',data,shoff+i*ents) for i in range(n)]
    strings=secs[idx][4]
    for s in secs:
        name=data[strings+s[0]:data.index(b'\0',strings+s[0])]
        if name==b'.rodata.uc_offs':
            values=struct.unpack_from('<%dI'%(s[5]//4),data,s[4]);assert len(values)==len(KEYS)
            return dict(zip(KEYS,values))
    raise AssertionError('offsets missing')

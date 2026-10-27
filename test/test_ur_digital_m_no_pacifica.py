# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Optional BOARD 3 flash trade: ID 18 stays reserved and cannot play from flash."""
import sys
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from flash_stores import fresh
from ur_digital_m_target import offsets
e=fresh(sys.argv[1]);O=offsets(sys.argv[1]);ux=e.addr('ux');buf=e.addr('dl_ring');out=buf+256
e.call('udfx_init',[ux]);e.call('udfx_page',[ux,1,out])
assert not e.peek(out,24)[18//8]&(1<<(18%8)), 'Pacifica advertised'
f=bytearray(32);f[:4]=bytes([11,15,1,18]);f[20]=255;e.poke(buf,f)
e.call('udfx_pixel_fx',[ux,buf,0]);assert e.peek(ux+O['ux_seq5'],5)==bytes(5), 'unsupported command admitted'
# Pack a valid old-style preset record, then replace its saved effect ID and CRC.
e.call('udfx_rec_pack',[ux,1,buf]);r=bytearray(e.peek(buf,128))
# 8 + twelve u16 CONFIG slots = first saved 23-byte effect; mask follows four.
r[32]=18;r[124]=1
crc=65535
for b in r[:126]:
    crc ^= b<<8
    for _ in range(8):crc=((crc<<1)^(0x1021 if crc&0x8000 else 0))&65535
r[126:128]=crc.to_bytes(2,'little');e.poke(buf,r)
e.call('udfx_rec_valid',[buf]);assert e.ret, 'fixture record CRC'
e.call('udfx_load',[ux,buf]);assert not e.rd(ux+O['ux_stored_have'],1), 'stored Pacifica admitted'
print('optional Pacifica removal: capability, command and stored preset checks passed')

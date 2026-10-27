# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Actual RV32 page builders: generated descriptor paging and immutable QDB2 identity."""
import sys,struct,json
from pathlib import Path
FW=Path(__file__).resolve().parents[1];sys.path.insert(0,str(FW/'tools'))
from flash_stores import fresh
from board_descriptor import encode
checks=0
for elf,board in zip(sys.argv[1:],(2,3)):
 e=fresh(elf);A=e.addr;buf=A('din_ring' if board==2 else 'dl_ring');arg=buf+512;out=arg+32
 pagefn='rc_page' if board==2 else 'ud_rc_page'
 def page(n,args=b''):
  global checks
  e.poke(arg,args+bytes(24-len(args)));e.poke(out,bytes(24));e.call(pagefn,[n,arg,out]);checks+=1
  return e.ret,e.peek(out,24)
 desc=encode(json.loads((FW/'src/board'/('ur_analog_m' if board==2 else 'ur_digital_m')/'board.json').read_text()))
 got=b''
 for part in range((len(desc)+23)//24):
  p,b=page(15,bytes([part]));assert p==15;got+=b
 assert got[:len(desc)]==desc and not any(got[len(desc):]);checks+=1
 assert page(15,bytes([(len(desc)+23)//24]))[0]==255
 assert page(4,b'\2')[0]==255
 target=bytes([board,1,1,1,3,0,0,1])+struct.pack('>II',0x08002100,57088)
 bl=b'QDB2\1'+target+bytes([0x88,10,20,5])+b'0123456'
 d=bytearray(256);d[:64]=bytes(64);d[64:]=bytes([255])*192
 d[:4]=b'QDU6';d[4]=2;d[7]=255;d[8:15]=target[:7];d[15]=1
 struct.pack_into('>IIIII',d,16,0x00030000,16,0x12345678,0x08002100,0x08002102)
 d[36:39]=bytes([2,3,1]);e.poke(buf,d)
 e.call('upd_crc32_words',[0xFFFFFFFF,buf,15]);struct.pack_into('>I',d,60,e.ret)
 d[64:70]=struct.pack('<HHH',0x1111,0x3333,0x7777) # overwritten below using pinned status constants
 hdr=(FW/'src/core/ultra_update.h').read_text()
 import re
 status={k:int(v,16) for k,v in re.findall(r'#define\s+(UPD_ST_\w+)\s+(0x[0-9A-Fa-f]+)',hdr)}
 d[64:70]=struct.pack('<HHH',*(status[x] for x in ('UPD_ST_VERIFIED','UPD_ST_TRIAL_SEEN','UPD_ST_CONFIRMED')))
 e.poke(0x08002000,d);e.poke(buf+256,bl)
 e.call('upd_app_read_info',[A('upd'),0x08002000,buf+256])
 p,b=page(4,b'\2');assert p==4 and b==bl[5:24]+bytes([2,1,5,0,0]),(board,p,b.hex())
 p,b=page(5);assert p==5 and b[0]==1 and b[1]==7 and b[2]==1 and b[3]==0 and b[4:16]==d[16:28] and b[16:20]==d[60:64] and b[20:]==bytes(4)
 assert page(5,b'\1')[0]==255
 # Whole immutable tuple and geometry bytes are copied, including unknown future values.
 e.poke(buf+256+5+5,b'\x8F\xC9\xFE')
 p,b=page(4,b'\2');assert b[:19]==e.peek(buf+261,19)
 for n in range(20):
  if n in (8,14):continue
  assert page(n,bytes([0,0,0,1]))[0]==255
 print('identity/descriptor/FW_UPDATE board',board,'passed')
print('M1a immutable page checks:',checks)

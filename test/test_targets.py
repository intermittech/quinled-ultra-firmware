# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Target selection/registry regression checks; no toolchains or third-party YAML parser needed."""
import copy
import importlib.util
import subprocess
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
spec=importlib.util.spec_from_file_location('targets',ROOT/'tools/gen_targets.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
import json
reg=m.definitions()
subprocess.run([sys.executable,str(ROOT/'tools/gen_targets.py'),'--check'],check=True)
for board in ('1','2','3','d2a_plus','ur_analog_m','ur_digital_m'):
 r=subprocess.run([sys.executable,str(ROOT/'tools/gen_targets.py'),'--shell','--board',board],capture_output=True,text=True)
 assert r.returncode==0 and 'TARGET_MCU=ch32v20x' in r.stdout,(board,r.stderr)
for args in (['--board','0'],['--board','4'],['--mcu','unknown'],['--board','../1']):
 r=subprocess.run([sys.executable,str(ROOT/'tools/gen_targets.py'),'--shell',*args],capture_output=True,text=True)
 assert r.returncode!=0,args
for channels in (15,20):
 r=copy.deepcopy(reg);r['boards']['ur_analog_m']['channels']=channels
 generated=m.generate(r)
 assert f'#define BOARD_CHANNELS {channels}\n' in generated['src/board/ur_analog_m/board_gen.h']
for rel,text in m.generate(reg).items():
 if rel.startswith('src/board/'):
  assert '->' not in text and 'ch32' not in text and 'IRQ' not in text,rel
for key,value in (('table_offset',49),('entry_offset_min',64),('entry_offset_min',81),
                  ('boot_bytes',65540),('flash_base',0x08000001),('link_base',1)):
 r=copy.deepcopy(reg);r['mcus']['ch32v20x']['boot_api'][key]=value
 try: m.generate(r)
 except ValueError: pass
 else: raise AssertionError(('invalid boot API layout accepted',key,value))
r=copy.deepcopy(reg);r['mcus']['ch32v20x']['boot_api']['table_offset']=52
generated=m.generate(r)
assert 'MCU_BOOT_API_TABLE_ADDR 0x08000034u' in generated['src/mcu/ch32v20x/boot_api_layout.h']
assert 'MCU_BOOT_API_OFFSET = 0x34;' in generated['src/mcu/ch32v20x/boot_api_layout.ld']
print('target registry: generation, valid/invalid selectors, 15/20-channel board facts and register-free boards passed')

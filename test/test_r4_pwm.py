# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run from firmware: python test/test_r4_pwm.py <fw.elf>.
Checks the compiled PWM restart: every mode write while tripped stays force-low.
No cycle or analog pulse-width claim is made by this register-level regression.
"""
import struct, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rv32emu import Emu
e=Emu(sys.argv[1])
e.poke(e.addr('set'),bytes([0,8,0,10,0,4,0,20,0,1,0,0])+struct.pack('<I',5760))
e.poke(e.addr('sext'),b'\xff'*7)
e.wr(e.addr('pwm_forced'),1,1)
e.mmio[0x40000400]=0x81 # TIM3 keeps running for DIN capture
e.call('apply_settings')
masks={0x40012C18:0x7070,0x4000001C:0x7000,0x4000041C:0x7070}
bad=[(hex(a),hex(v)) for a,v in e.writes if a in masks and v&masks[a] != 0x4040&masks[a]]
assert any(a in masks for a,v in e.writes), 'no PWM mode writes tested'
print('unsafe PWM-mode writes while force-off is latched:',bad)
sys.exit(bool(bad))

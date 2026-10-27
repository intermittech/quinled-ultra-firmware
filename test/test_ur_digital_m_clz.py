# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The BOARD 3 compact leading-zero lookup is exact for every byte."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rv32emu import Emu
for elf in sys.argv[1:]:
    e=Emu(elf)
    for x in range(256):
        e.call('ws_clz8',[x])
        assert e.ret==8-x.bit_length(),(x,e.ret)
    print('digital leading-zero table: 256 exact byte results')

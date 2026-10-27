# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Release-only absence gate; pass one or more app-slot ELF paths.
Wide accumulators may compile to native RV32 operations, but compiler double-word
arithmetic helpers and the retired release text formatter must not return.
"""
import re
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from rv32emu import Emu

assert len(sys.argv) > 1, 'supply the twelve release ELF paths'
for path in sys.argv[1:]:
    symbols = Emu(path).syms
    bad = [s for s in symbols if re.match(r'^__\w*di[234](?:\.|$)', s)
           or s.split('.')[0] in ('status_part', 'cz_text', 'cz_pool')]
    assert not bad, (path, bad)
    print(Path(path).parent.name + ': no double-word arithmetic helpers or retired text tables')
print(f'release symbols: {len(sys.argv)-1} images passed')

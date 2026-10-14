# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compiler-publication contracts, not a race simulation. Optional argument: firmware root."""
from pathlib import Path
import re
import sys

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]


def function(path, name):
    s = (root / path).read_text()
    s = re.sub(r'/\*[\s\S]*?\*/|//[^\n]*', '', s)
    m = re.search(r'\b' + name + r'\s*\([^;{}]*\)\s*\{', s)
    start = end = m.end()
    depth = 1
    while depth:
        depth += (s[end] == '{') - (s[end] == '}')
        end += 1
    return s[start:end - 1]


barrier = r'BARRIER\s*\(\s*\)\s*;'
checks = [
    ('src/app/main.c', 'rc_tick', barrier + r'[\s\S]*?rc_s\.dir_tick', 'schedule acquire'),
    ('src/app/main.c', 'rc_tick', barrier + r'\s*rc_state\s*=\s*RC_IDLE', 'window release'),
    ('src/app/main.c', 'rc_arm', barrier + r'\s*rc_state\s*=\s*RC_ARMED', 'window publish'),
    ('src/hal/hw.c', 'rc_hw_send', r'rc_left\s*=[^;]+;\s*' + barrier +
     r'[\s\S]*?USART3->CTLR1\s*=[^;]*TXEIE', 'UART cursor publish'),
    ('src/hal/hw.c', 'rc_uart_isr', r'^\s*' + barrier, 'UART cursor acquire'),
]
failed = 0
for path, name, pattern, desc in checks:
    ok = re.search(pattern, function(path, name)) is not None
    print(('PASS' if ok else 'FAIL') + ': ' + desc)
    failed += not ok
raise SystemExit(bool(failed))

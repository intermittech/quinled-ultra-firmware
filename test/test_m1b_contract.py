# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Source publication guards supplement the preempted application diagnostic."""
from pathlib import Path
import re

FW=Path(__file__).resolve().parents[1]
def read(name):
    s=(FW/name).read_text()
    return re.sub(r"/\*[\s\S]*?\*/|//[^\n]*","",s)

def ordered(source,*parts):
    pos=0
    for part in parts:
        m=re.search(part,source[pos:])
        assert m,part
        pos+=m.end()

d=read("src/core/dline.c")
c=read("src/core/ur_digital_m/udchain.c")
a=read("src/app/main_ur_digital_m.c")
ordered(d,r"d->cap\[.*?=",r"DL_BARRIER\(\);",r"d->ch_hn = \(uint16_t\)hn;")
ordered(c,r"hn = \*\(volatile uint16_t \*\)&d->ch_hn;",
        r'__asm__ volatile \("" ::: "memory"\)',r"ws_line_feed\(")
ordered(c,r"if \(acquiring\) dline_ch_free\(d\);",r"ws_line_decode\(")
ordered(c,r"ws_rise_tail\(",r"if \(!acquiring\) dline_ch_free\(d\);",r"ls_frame\(")
ordered(d[d.index("void dline_ch_free"):],r"DL_BARRIER\(\);",r"d->ch_ready = 0;")
ordered(a[a.index("if (a & DLINE_A_CH_READY)"):],
        r"dl_ch_tick = tick_now;",r"BARRIER\(\);",r"dl_ch_ready = 1;")
ordered(a[a.index("if (dl_ch_ready)"):],
        r"dl_ch_ready = 0;",r"BARRIER\(\);",r"ud_chain_decode\(\);")
ordered(a[a.index("static void ud_chain_decode(void)\n{"):],
        r"end_w = dl.ch_end, ready_tick = dl_ch_tick;",r"udc_classify\(",
        r"ud_answer\(",r"BARRIER\(\);",r"dline_ch_done\(")
ordered(d[d.index("void dline_ch_done"):],
        r"d->ch_win = win_w;",r"d->ch_tx = tx_w;",r"DL_BARRIER\(\);",r"d->ch_arm =")
assert "d->cap+DLINE_CH_TAIL" in c
print("M1b raw/acquisition ownership, metadata snapshot, answer publication: passed")

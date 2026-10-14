#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Diff-Power ultra receiver's return channel (BOARD=2) on the compiled firmware, in the RV32IMC emulator.

    FW_ELF=<BOARD=2 image>/fw.elf FW_ELF_MEASURE=<its MEASURE=1 build>/fw.elf python tools/measure_rc.py

Main loop (the measurement build): what rc_arm does at the frame end of an RW frame, before its deadline
(ULTRA_RW_DIR_MAX_US = 900 us after the frame's last edge; the frame end is handled 100-150 us + the header decode
after it): the last edge in the tail copy (worst: the edge at the copy's start), the POLL check, the schedule, each page,
the framing. Poll ISR (the image itself): every path of the window (rc_tick). The USART3 ISR per byte.
Cycles as tools/measure.py counts them (rv32emu.py), at 144 MHz."""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rv32emu import Emu  # noqa: E402

FW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ELF = os.environ.get("FW_ELF") or os.path.join(FW, "build", "target", "fw.elf")
MELF = os.environ.get("FW_ELF_MEASURE") or os.path.join(os.path.dirname(ELF) + "-measure", "fw.elf")
SYSTICK_CNT = 0xE000F008
GPIOA_INDR = 0x40010808
USART3_STATR, USART3_CTLR1 = 0x40004800, 0x4000480C
PAGES = (("STATUS", 0x00), ("OUTPUTS", 0x01), ("ACKS", 0x02), ("ERRORS", 0x03), ("IDENTITY", 0x04),
         ("CALIBRATION", 0x06), ("CAL_TEST", 0x07), ("POWER", 0x08), ("SUPPLY", 0x09), ("HEALTH", 0x0A),
         ("ENERGY", 0x0B), ("unknown (0x0C)", 0x0C))


class EmuT(Emu):
    """SysTick's counter runs (the turnaround check / the burst wait RC_PREAMBLE_CYC on it)"""
    t = 0

    def rd(self, a, n):
        if a == SYSTICK_CNT:
            self.t += 8
            return self.t & 0xFFFFFFFF
        return Emu.rd(self, a, n)


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def poll(index=0, slots=8, page=0, rate=0, seq=7):
    s = bytearray(32)
    s[0], s[1], s[2] = 0x07, 0xF0, seq
    s[3:7] = bytes((index, slots, page, rate))
    s[30:32] = struct.pack(">H", crc16(s[:30]))
    return bytes(s)


def main():
    rows = []
    m = EmuT(MELF)
    for f in ("rc_page", "rc_schedule", "rc_answer", "rc_last_edge", "rc_poll_parse", "rc_brr"):
        if f not in m.syms:
            sys.exit("%s: no %s (a BOARD=2 MEASURE=1 build is needed)" % (MELF, f))
    scratch = (m.addr("din_ring") + 15) & ~15
    p = poll(index=7, slots=16, page=0x08)
    m.poke(scratch, p)
    n = m.call("rc_poll_parse", [scratch])
    rows.append(("rc_poll_parse (at the capture)", n, m.cycles()))
    w = scratch + 64
    words = 576
    m.poke(w, b"\0" * (2 * words))
    m.poke(w, struct.pack("<H", 0x8000))              # the edge at the copy's start: the longest scan
    n = m.call("rc_last_edge", [w, words])
    assert m.ret == (words - 1) * 16 + 15, m.ret
    rows.append(("rc_last_edge, 576 words (worst: the edge at the start)", n, m.cycles()))
    m.poke(w, b"\0" * (2 * words))
    m.poke(w + 2 * (words - 5), struct.pack("<H", 0x0100))
    n = m.call("rc_last_edge", [w, words])
    rows.append(("rc_last_edge, 576 words (typical: the edge 5 words from the end)", n, m.cycles()))
    sched = scratch + 2048
    n = m.call("rc_schedule", [sched, 1000, 1200, int(50.3 * 65536), scratch])
    rows.append(("rc_schedule (one 64-bit division, 5 events)", n, m.cycles()))
    n = m.call("rc_brr", [int(50.3 * 65536), 0])
    rows.append(("rc_brr", n, m.cycles()))
    ans = scratch + 2100
    n = m.call("rc_answer", [ans, scratch, 0x08, 0x01, 0xBEEF])
    rows.append(("rc_answer (framing + CRC-16)", n, m.cycles()))
    worst_page = (0, "")
    for name, pg in PAGES:
        m.poke(ans, b"\0" * 32)
        n = m.call("rc_page", [pg, scratch + 7, ans + 6])
        c = m.cycles()
        rows.append(("rc_page " + name, n, c))
        if c > worst_page[0]:
            worst_page = (c, name)
    if "pwrev_ack" in m.syms:
        n = m.call("pwrev_ack", [m.addr("pw"), scratch + 7])
        rows.append(("pwrev_ack (POWER with the acknowledge)", n, m.cycles()))
    arm = sum(r[2] for r in rows if r[0].startswith(("rc_last_edge, 576 words (worst", "rc_schedule", "rc_brr",
                                                      "rc_answer", "rc_poll_parse", "pwrev_ack")))
    arm += worst_page[0]
    rows.append(("rc_arm's work, worst (the last edge's worst scan + %s + the rest)" % worst_page[1], 0, arm))

    # the poll ISR in the window (the image)
    e = EmuT(ELF)
    e.mmio[GPIOA_INDR] = 0x40                          # DIN high: the line is ours at the check / the burst
    st, rs, tn = e.addr("rc_state"), e.addr("rc_s"), e.addr("tick_now")
    for nm in ("rc_send", "rc_brr_v"):
        if nm not in e.syms and not any(k.startswith(nm + ".") for k in e.syms):
            pass

    def isr(state, tick, sched, send=0):
        e.wr(st, 1, state)
        e.wr(tn, 4, tick - 1)                          # (the ISR counts it up first)
        e.poke(rs, struct.pack("<5I", *sched))         # dir, check, tx, end, late
        if send:
            e.wr(e.addr("rc_send"), 1, 1)
            e.wr(e.addr("rc_brr_v"), 2, 144)
        n = e.call("poll_isr")
        return n, e.cycles(), e.rd(st, 1)

    S = (110, 119, 120, 191, 118)                      # dir, check, tx, end, late (ticks)
    n, c, s = isr(1, 100, S)
    rows.append(("poll ISR, window armed, before the turn", n, c))
    n, c, s = isr(1, 110, S)
    assert s == 2, s
    rows.append(("poll ISR, the turn (gate closed, transceivers)", n, c))
    n, c, s = isr(2, 115, S)
    rows.append(("poll ISR, turned, nothing due", n, c))
    n, c, s = isr(2, 119, S)
    assert s == 3, s
    rows.append(("poll ISR, the turnaround check (3 us HIGH, read back)", n, c))
    n, c, s = isr(3, 120, S, send=1)
    rows.append(("poll ISR, the burst's start (3 us HIGH, read back, the first byte)", n, c))
    n, c, s = isr(3, 191, S)
    assert s == 0, s
    rows.append(("poll ISR, the window's end (forward, gate re-armed, tracker)", n, c))
    if "rc_uart_isr" in e.syms:
        e.mmio[USART3_CTLR1] = 0x80 | 0x0C | 0x2000    # TXEIE | TE | RE.. | UE
        e.wr(e.addr("rc_left"), 4, 5)
        e.wr(e.addr("rc_next"), 4, e.addr("rc_ans"))
        n = e.call("rc_uart_isr")
        rows.append(("USART3 ISR, a byte (every 10 us at 1 Mbaud)", n, e.cycles()))

    print("Diff-Power ultra receiver: the return channel (%s; main-loop rows: %s)" % (ELF, MELF))
    for name, n, c in rows:
        print("  %-78s %6d cycles  %7.2f us" % (name, c, c / 144.0))


if __name__ == "__main__":
    main()

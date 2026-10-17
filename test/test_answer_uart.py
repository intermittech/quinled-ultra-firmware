# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The answer UART's build switch (board.h ANSWER_UART) on the compiled receiver HAL in tools/rv32emu.py.

    python test/test_answer_uart.py <fw.elf> <board 2|3> <answer uart 3|2|1> [<release fw.elf of the same build>]

The ELF is the LTO=0 build of that board and switch (tools/build_release.sh builds them: BOARD 2 plain, BOARD 3 with
-fno-inline -fno-ipa-cp -fno-ipa-sra). Per variant it checks what the switch moves and what it must not:
  boot     the USART, its clock enable, 1 Mbaud divider, TE + UE and no interrupt enables, the TX pin (BOARD 3: AF
           push-pull, idle HIGH; BOARD 2: an input with a pull, high-Z), the interrupt: VTF slot 1, priority, enable;
           the console: USART2 at 115200 with DMA requests (ANSWER_UART 1, 3) or untouched by the console (2: DMA1 ch7
           never written, the text goes to the RAM ring);
  burst    the divider written, the pin AF during the burst, the lead 0xFF then the 32 bytes in order, TXE -> TC, TC
           releases (BOARD 2 the pin back to an input, BOARD 3 DE low), RE never set, no DMA channel touched, the
           probe (n 0) and the contention abort (DIN low) send nothing and release the pin;
  dividers rc_brr for the three rates at the nominal clock and a few HSI errors against board.h's base;
  no leak  no other USART's registers written, the other candidate TX pins untouched by the burst;
  release  (the optional ELF: the LTO build the release ships) ANSWER_UART 2 keeps con_ring / con_ring_w, the ring the jig
           reads over SWD, which LTO removes when nothing reads it (they are volatile).
UART TXE / TC events are injected: this verifies the software sequencing and the register map, not interrupt latency,
the electrical levels or the baud timing (the same limits as test_r6_rc_hal.py, whose USART3 checks stay)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from hal_check import Hal, dch  # noqa: E402

elf, BOARD, AU = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
release_elf = sys.argv[4] if len(sys.argv) > 4 else None
assert BOARD in (2, 3) and AU in (1, 2, 3) and (AU != 1 or BOARD == 3), "BOARD 2|3, ANSWER_UART 3|2 (1: BOARD 3 only)"

PA, PB = 0x40010800, 0x40010C00
RCC = 0x40021000
PFIC = 0xE000E000
CNT = 0xE000F008
USART = {1: 0x40013800, 2: 0x40004400, 3: 0x40004800}
TXPIN = {1: (PA, 9), 2: (PA, 2), 3: (PB, 10)}
IRQ = {1: 53, 2: 54, 3: 55}
BASE = {1: 72, 2: 144, 3: 144}
U, (TXPORT, TXN) = USART[AU], TXPIN[AU]
LEAD = b"\xff"                                           # core/rc.h RC_LEAD_BYTES
payload = bytes((7 * i + 0xA5) & 255 for i in range(32))
BUF = 0x20004000
checks = failures = 0


def check(ok, label):
    global checks, failures
    checks += 1
    if not ok:
        failures += 1
        print("FAIL: " + label)


class RcHal(Hal):
    def __init__(self, path):
        super().__init__(path)
        self.count = 0
        self.de = 0
        self.blank = 0

    def rd(self, a, n):
        if a == CNT:                                     # SysTick: moves on, for the preamble waits
            self.count += 64
            return self.count
        return super().rd(a, n)

    def wr(self, a, n, v):
        if BOARD == 3:                                   # DE on PB14, BLANK_N on PA1
            if a == PB + 0x10:
                if v & (1 << 14):
                    self.de = 1
                if v & (1 << 30):
                    self.de = 0
            if a == PB + 0x14 and v & (1 << 14):
                self.de = 0
            if a == PA + 0x10 and v & 2:
                self.blank = 1
            if a == PA + 0x14 and v & 2:
                self.blank = 0
        super().wr(a, n, v)

    def cfg(self, port, pin):                            # the pin's CFG nibble
        return (self.rd(port + (0 if pin < 8 else 4), 4) >> ((pin & 7) * 4)) & 15


def burst_cycle(e, brr):
    """one answer on a booted HAL; returns nothing, checks as it goes"""
    e.poke(BUF, payload)
    snap = {k: e.cfg(*TXPIN[k]) for k in TXPIN}
    if BOARD == 3:
        e.mmio[PA + 8] = 0                               # RO high-Z with DE high: PA6 reads LOW through its pull-down
        e.call("rc_hw_dir", [1])
        check(e.de and not e.blank, "answer: BLANK_N LOW, then the return driver on")
    else:
        e.mmio[PA + 8] = 1 << 6                          # DIN HIGH: the segment is free
        e.writes.clear()
        e.call("rc_hw_dir", [1])
        check((PA + 0x14, 1 << 10) in e.writes and (PB + 0x10, 1 << 14) in e.writes,
              "turn: output transceiver listens, input one drives upstream")
    e.writes.clear()
    e.call("rc_hw_send", [BUF, 32, brr], max_steps=20000)
    check(e.ret == 0 and e.rd(U + 8, 2) == brr, "burst: starts with the requested divider (%d)" % brr)
    check(e.cfg(TXPORT, TXN) == 0xB, "burst: the TX pin is alternate-function push-pull")
    check(e.rd(U + 0x0C, 2) == 0x2088, "burst: UE + TE + TXEIE")
    for _ in range(32):
        e.call("rc_uart_isr")                            # TXE: the 32 bytes
    check(bytes(v for a, v in e.writes if a == U + 4) == LEAD + payload, "burst: the lead byte, then the 32 bytes in order")
    check(e.rd(U + 0x0C, 2) == 0x2088 and e.cfg(TXPORT, TXN) == 0xB, "burst: still driving after the last byte entered")
    e.call("rc_uart_isr")                                # the last byte is in the shift register
    check(e.rd(U + 0x0C, 2) == 0x2048 and e.cfg(TXPORT, TXN) == 0xB, "TXE -> TC: the final stop bit is not cut")
    e.call("rc_uart_isr")                                # TC
    if BOARD == 3:
        check(e.rd(U + 0x0C, 2) == 0x2008 and not e.de and e.cfg(TXPORT, TXN) == 0xB,
              "TC: interrupts off, DE low, the pin idles HIGH (AF)")
    else:
        check(e.rd(U + 0x0C, 2) == 0x2008 and e.cfg(TXPORT, TXN) == 8, "TC: interrupts off, the pin high-Z again")
    check(not any(a == U + 0x0C and v & 4 for a, v in e.writes), "RE stays 0 throughout")
    check(not any(0x40020000 <= a < 0x40020400 for a, _ in e.writes), "no DMA register is touched by the burst")
    # nothing of the other candidates is touched: no other USART's register, the other TX pins' modes as at boot
    check(not any(b <= a < b + 0x20 for k, b in USART.items() if k != AU for a, _ in e.writes),
          "no other USART's register is written by the burst")
    check(all(e.cfg(*TXPIN[k]) == snap[k] for k in TXPIN if k != AU), "the other TX pins' modes are as at boot")
    # cut: a burst running is cut by FORWARD / QUIET
    for mode in ((0, 2) if BOARD == 3 else (0,)):         # (BOARD 2's QUIET turns the drivers around only, as before)
        e.call("rc_hw_dir", [1])
        e.call("rc_hw_send", [BUF, 32, brr], max_steps=20000)
        e.call("rc_hw_dir", [mode])
        check(not e.rd(U + 0x0C, 2) & 0xC0, "mode %d cuts a burst still running (no TXE / TC interrupt left)" % mode)
        if BOARD == 3:
            check(not e.de, "mode %d: DE low" % mode)
    # the probe (n 0) and, on BOARD 2, the contention abort send nothing and release
    cases = [(0, True)] if BOARD == 3 else [(0, True), (0, False), (32, False)]
    for n, high in cases:
        e.call("rc_hw_dir", [1])
        e.mmio[PA + 8] = (1 << 6) if high or BOARD == 3 else 0
        e.writes.clear()
        e.call("rc_hw_send", [BUF, n, brr], max_steps=20000)
        released = e.cfg(TXPORT, TXN) == (0xB if BOARD == 3 else 8)
        check(e.ret == (0 if high or BOARD == 3 else 1) and not any(a == U + 4 for a, _ in e.writes) and released
              and not e.rd(U + 0x0C, 2) & 0xC0, "probe / contention n=%d DIN=%d: nothing sent, pin released" % (n, high))


# -- boot
e = RcHal(elf)
e.call("hw_init")
check(e.rd(U + 0x0C, 2) == 0x2008, "boot: the answer USART has UE + TE, no TXE / TC / RX enable")
check(e.rd(U + 8, 2) == BASE[AU], "boot: BRR %d = 1 Mbaud at this PCLK (%d)" % (e.rd(U + 8, 2), BASE[AU]))
check(e.cfg(TXPORT, TXN) == (0xB if BOARD == 3 else 8),
      "boot: TX pin %s%d is %s" % ("A" if TXPORT == PA else "B", TXN, "AF push-pull (idles HIGH)" if BOARD == 3 else "an input with a pull"))
apb1, apb2 = e.mmio.get(RCC + 0x1C, 0), e.mmio.get(RCC + 0x18, 0)
check(bool(apb1 & (1 << 18)) == (AU == 3), "boot: USART3's clock is on only for ANSWER_UART 3")
check(bool(apb2 & (1 << 14)) == (AU == 1), "boot: USART1's clock is on only for ANSWER_UART 1")
check(bool(apb1 & (1 << 17)), "boot: USART2's clock is on (console, or the answer UART)")
check((PFIC + 0x51, IRQ[AU]) in e.writes, "boot: VTF slot 1 = interrupt %d" % IRQ[AU])
check(any(a == PFIC + 0x64 and v & 1 for a, v in e.writes), "boot: VTF slot 1 has the entry address")
check((PFIC + 0x104, 1 << (IRQ[AU] - 32)) in e.writes, "boot: the interrupt is enabled")
check(not any(a == PFIC + 0x104 and v & (1 << (k - 32)) for k in IRQ.values() if k != IRQ[AU] for a, v in e.writes),
      "boot: no other answer-candidate interrupt is enabled")
check((PFIC + 0x400 + IRQ[AU], 0) in e.writes, "boot: priority 0 (preempts the poll tick)")
con_dma = [(a, v) for a, v in e.writes if dch(7) <= a < dch(7) + 0x10]
if AU == 2:
    check(not con_dma and e.rd(USART[2] + 0x14, 2) == 0, "boot: no console TX DMA, no DMAT on the answer UART")
    ring_w = e.addr("con_ring_w")
    e.poke(BUF, b"jig 1\r\n\0")
    e.call("uart_write", [BUF])
    check(e.rd(ring_w, 4) == 7 and e.peek(e.addr("con_ring"), 7) == b"jig 1\r\n",
          "console: the text lands in the RAM ring (SWD)")
    e.call("uart_room")
    check(e.ret == 512, "console: uart_room is the whole ring")
    e.call("uart_busy")
    check(e.ret == 0, "console: uart_busy is never")
else:
    check(e.rd(USART[2] + 8, 2) == 144000000 // 115200 and e.rd(USART[2] + 0x14, 2) & 0x80,
          "boot: the console is USART2 at 115200 with DMA requests")
    check(len(con_dma) >= 1, "boot: the console's DMA channel 7 is set up")

# -- the dividers (core/rc.c rc_brr): the nominal clock and a few HSI errors
for rate in (0, 1, 2):
    e.call("rc_brr", [50 << 16, rate])
    check(e.ret == BASE[AU] << rate, "rc_brr: rate %d nominal = %d" % (rate, BASE[AU] << rate))
    for err in (-0.04, -0.01, 0.01, 0.04):               # the local clock this much fast (the tick shorter, rate_q16 smaller)
        rq = round((50 << 16) / (1 + err))
        e.call("rc_brr", [rq, rate])
        exact = (BASE[AU] << rate) * (1 + err)
        check(abs(e.ret - exact) <= 0.5 + 1e-3, "rc_brr: rate %d clock %+.0f %% -> %d (exact %.2f)" % (rate, err * 100, e.ret, exact))
    e.call("rc_brr", [(50 << 16) * 2, rate])
    check(e.ret == BASE[AU] << rate, "rc_brr: rate %d implausible clock (x2) -> nominal" % rate)

# -- the bursts, at the three dividers, from a fresh boot each
for rate in (0, 1, 2):
    e = RcHal(elf)
    e.call("hw_init")
    burst_cycle(e, BASE[AU] << rate)

if release_elf:
    from rv32emu import Emu  # noqa: E402
    r = Emu(release_elf)
    if AU == 2:
        check("con_ring" in r.syms and "con_ring_w" in r.syms, "release build: the console ring is linked (the jig's SWD read)")
    else:
        check("con_ring" not in r.syms, "release build: no console ring (the console sends on USART2)")

print("answer UART %d on BOARD %d: %d checks, %d failures" % (AU, BOARD, checks, failures))
raise SystemExit(bool(failures))

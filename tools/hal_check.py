# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""hal/hw.c on a register model in tools/rv32emu.py (0.6.4): checks of the HAL's own logic that the host tests (which
replace hw.c) cannot see. Needs an LTO=0 build (every function out of line):
    LTO=0 OUT=build/target-lto0 bash tools/build_target.sh
    python tools/hal_check.py build/target-lto0/fw.elf
The model: timer preload / active registers (ARR, CCR1-4 with ARPE / OCxPE), UG, the update DMA bursts (one burst on
UG; TIM2's on its CH3 request), DMA channel counts. It is a model of the register semantics the code relies on (RM
V2.5), not of timing: bench items stay bench items. Checks:
  h2   pwm_start at boot (tables 0) and as a live restart: no output on at boot (G / W2 right-aligned: PWM mode 2), the
       tables for the new period before the timers run, the first period's compares = entry 0, a trip stays in force;
  h6   the slew ISR's table entry with the update's DMA burst in progress, not started, or done before the interrupt;
  adc  PA4's sample time (SMP4) and the calibration with TSVREFE clear (gpt-hw H5 / H3);
  flash  flash_op: the state during every erase / half-word (the SRAM tick through VTF, HCLK / 2, PCLK2
       kept, SysTick rescaled with its phase, mtvec, the other interrupts off), restored after, the result
       codes (write protection, no EOP, no unlock), the console's dropped bytes, the SRAM tick's trip;
  records  (0.6.5) the boot window for the upper record pages (HCLK / 2, then back), the PVD on at 2.9 V, flash_op
           starting nothing while VDD is below it.
  vectors  (0.6.5) no vector table: the startup sets mtvec to trap_entry (unified mode); every exception code
           and interrupt number reaches its handler (SysTick, TIM4, TIM2, TIM3) or the fault loop.
  ramtext  every call / jump of the SRAM code stays in SRAM (OBJDUMP=<riscv objdump>).
Exit status 1 on a failed check."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rv32emu import Emu  # noqa: E402

T1, T2, T3 = 0x40012C00, 0x40000000, 0x40000400
DMA = 0x40020000
ADC = 0x40012400
TIMS = {T1: 5, T2: 1, T3: 3}                             # timer -> its PWM DMA channel
CH = {"G": (T1, 1), "R": (T1, 2), "B": (T2, 4), "W2": (T3, 3), "W1": (T3, 4)}
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += not ok


def dch(n):
    return DMA + 8 + 20 * (n - 1)


class Hal(Emu):
    def __init__(self, elf):
        super().__init__(elf)
        self.active = {}                                 # timer register -> active (shadow) value
        self.dma_pos = {}                                # DMA channel -> transfers done in its table
        self.cntr_script = None                          # h6: DMA count values for successive reads
        self.cnt_step = 0                                # h6: timer counts per read of CNT
        self.calib = []                                  # adc: CTLR2 at each RSTCAL / CAL

    # -- registers
    def rd(self, a, n):
        if self.cntr_script is not None and a == dch(self.script_ch) + 4:
            v = self.cntr_script[min(self.script_i, len(self.cntr_script) - 1)]
            self.script_i += 1
            if self.script_i == self.script_done:        # the burst completed: the DMA wrote the entry's words
                self.burst_write(self.script_tim)
            return v
        if self.cnt_step and a in (T2 + 0x24, T3 + 0x24):
            v = self.mmio.get(a, 0)
            self.mmio[a] = v + self.cnt_step
            return v
        if a == ADC + 8:
            return self.mmio.get(a, 0) & ~0xC            # RSTCAL / CAL done at once
        return super().rd(a, n)

    def wr(self, a, n, v):
        m, _ = self._mem(a, n)
        if m is not None:
            return super().wr(a, n, v)
        v &= (1 << (8 * n)) - 1
        self.mmio[a] = v
        if a == ADC + 8 and v & 0xC:
            self.calib.append(v)
        for t in TIMS:
            off = a - t
            if off == 0x2C and not self.mmio.get(t, 0) & 0x80:          # ATRLR, no ARPE: active at once
                self.active[a] = v
            if off in (0x34, 0x38, 0x3C, 0x40):
                ch = (off - 0x30) // 4
                cr = self.mmio.get(t + (0x18 if ch <= 2 else 0x1C), 0)
                if not cr & (1 << (3 if ch % 2 else 11)):                # no OCxPE: active at once
                    self.active[a] = v
            if off == 0x14 and v & 1:                                    # UG
                self.update(t)
        for c in (1, 3, 5):
            if a == dch(c) and v & 1:
                self.dma_pos[c] = 0
        self.writes.append((a, v))

    # -- timers
    def update(self, t):
        for off in (0x2C, 0x34, 0x38, 0x3C, 0x40):
            self.active[t + off] = self.mmio.get(t + off, 0)
        self.mmio[t + 0x24] = 0
        if self.mmio.get(t + 0x0C, 0) & 0x100:          # UDE: the update burst
            self.burst(t)

    def burst(self, t):
        c = TIMS[t]
        if not self.mmio.get(dch(c), 0) & 1:
            return
        total = self.mmio.get(dch(c) + 4, 0)
        madr = self.mmio.get(dch(c) + 12, 0)
        padr = self.mmio.get(dch(c) + 8, 0)
        if padr == t + 0x4C:                             # DMAADR: a burst from DBA
            cfg = self.mmio.get(t + 0x48, 0)
            words, base = ((cfg >> 8) & 31) + 1, (cfg & 31) * 4
        else:
            words, base = 1, padr - t
        pos = self.dma_pos.get(c, 0)
        for k in range(words):
            self.mmio[t + base + 4 * k] = self.rd(madr + 2 * ((pos + k) % total), 2)
        self.dma_pos[c] = (pos + words) % total

    def burst_write(self, t):
        self.burst(t)

    def mode(self, name):
        t, ch = CH[name]
        reg = self.mmio.get(t + (0x18 if ch <= 2 else 0x1C), 0)
        return (reg >> (4 if ch % 2 else 12)) & 7

    def on_in_period(self, name, ccr, arr):
        """the channel drives its pin high at some count of a period with this CCR / ARR"""
        m = self.mode(name)
        if m == 4:
            return False
        if m == 6:
            return ccr > 0
        if m == 7:
            return ccr <= arr
        return m == 5


def table(e, name):
    """(entries of the channel's table, stride) as pwm_table() places them"""
    spread = e.rd(e.addr("cur_spread"), 1)
    n = e.rd(e.addr("cur_len"), 1)
    tabs = {"G": ("pwm_tab1", 2, 4), "R": ("pwm_tab1", 3, 4), "B": ("pwm_tab2", 5, 6), "W2": ("pwm_tab3", 4, 6),
            "W1": ("pwm_tab3", 5, 6)} if spread else \
           {"G": ("pwm_tab1", 0, 2), "R": ("pwm_tab1", 1, 2), "B": ("pwm_tab2", 0, 1), "W2": ("pwm_tab3", 0, 2),
            "W1": ("pwm_tab3", 1, 2)}
    sym, first, stride = tabs[name]
    base = e.addr(sym)
    return [e.rd(base + 2 * (first + stride * i), 2) for i in range(n)], spread


def periods(e):
    """each table entry's period (counts)"""
    spread = e.rd(e.addr("cur_spread"), 1)
    n = e.rd(e.addr("cur_len"), 1)
    steps = e.rd(e.addr("cur_steps"), 4)
    if not spread:
        return [steps] * n
    a = e.addr("spread_arr")
    return [e.rd(a + 2 * i, 2) + 1 for i in range(n)]


OFF_PRESET, OFF_STAGGER = 1, 5                          # settings_t (core/config.h): pwm_preset, stagger


def start(e, preset, stagger, v):
    """apply_settings with the default settings at this preset / stagger, the tables holding v before"""
    s = e.addr("set")
    e.call("settings_default", [s])
    e.wr(s + OFF_PRESET, 1, preset)
    e.wr(s + OFF_STAGGER, 1, stagger)
    bv = e.addr("out_now")                              # (0.6.4: out_now = the values the tables hold)
    for i in range(5):
        e.wr(bv + 2 * i, 2, v)
    e.wr(e.addr("run_stagger"), 1, 0xFE)                 # force pwm_start
    e.call("apply_settings")


def h2(elf):
    print("h2: pwm_start (gpt-hw H2): G / W2 off at boot, the tables of the new period before the timers start")
    for preset in (0, 8, 14):
        for stagger in (0, 1, 2, 4):
            e = Hal(elf)
            e.call("hw_init")
            start(e, preset, stagger, 0)
            arr1 = e.active.get(T1 + 0x2C, 0)
            off_all = True
            for name, (t, ch) in CH.items():
                ccr = e.active.get(t + 0x30 + 4 * ch, 0)
                arr = e.active.get(t + 0x2C, 0)
                if e.on_in_period(name, ccr, arr):
                    off_all = False
                    print("    %s on in the first period: mode %d CCR %d ARR %d" % (name, e.mode(name), ccr, arr))
                tab, _ = table(e, name)
                for i, (x, p) in enumerate(zip(tab, periods(e))):
                    if e.on_in_period(name, x, p - 1):
                        off_all = False
                        print("    %s on in entry %d: mode %d CCR %d period %d" % (name, i, e.mode(name), x, p))
                        break
            check(off_all, "boot, preset %2d, stagger %d: every output off (first period and every entry; ARR %d)"
                  % (preset, stagger, arr1))
            # live restart to another period with the tables at 50 %: entries for the new period, entry 0 active
            other = 3 if preset != 3 else 9
            start(e, other, stagger, 32768)
            ok = True
            per = periods(e)
            for name, (t, ch) in CH.items():
                tab, spread = table(e, name)
                ra = e.mode(name) == 7
                for x, p in zip(tab, per):
                    duty = p - x if ra else x
                    if abs(duty - p * 32768 / 65535) > 2.5:
                        ok = False
                        print("    %s: entry %d for period %d (%s)" % (name, x, p, "PWM2" if ra else "PWM1"))
                        break
                # the first period (nominal length, ARR = steps - 1) carries entry 0's pulse: a right-aligned
                # compare is moved by the difference of the periods
                want = tab[0]
                if ra and spread:
                    want = max(0, tab[0] + e.rd(e.addr("cur_steps"), 4) - per[0])
                if e.active.get(t + 0x30 + 4 * ch, -1) != want:
                    ok = False
                    print("    %s: first CCR %d, want %d" % (name, e.active.get(t + 0x30 + 4 * ch, -1), want))
            check(ok, "restart %2d -> %2d, stagger %d, at 50 %%: tables for the new period, the first period = "
                  "entry 0's pulse" % (preset, other, stagger))
            # a trip during the restart stays in force: no write of a mode register may put an output back into a
            # PWM mode, not even for a moment (gpt-bugs H1: 0.6.3 wrote the PWM modes, then forced them low again)
            e.call("pwm_force_off", [1])
            n0 = len(e.writes)
            start(e, preset, stagger, 32768)
            bad = []
            for a, v in e.writes[n0:]:
                for name, (t, ch) in CH.items():
                    if a == t + (0x18 if ch <= 2 else 0x1C) and ((v >> (4 if ch % 2 else 12)) & 7) in (6, 7):
                        bad.append(name)
            check(not bad and all(e.mode(n) == 4 for n in CH),
                  "restart while tripped: every output forced low at every mode write %s" % (bad or ""))


def h6(elf):
    print("h6: the slew ISR's table entry (gpt-hw H6): taken after the update's DMA burst")
    for case in ("in progress", "not started", "done before"):
        e = Hal(elf)
        e.call("hw_init")
        start(e, 8, 4, 32768)                            # 25 kHz, adaptive + spread
        n = e.rd(e.addr("cur_len"), 1)
        tab3 = e.addr("pwm_tab3")
        for i in range(n):
            e.wr(tab3 + 12 * i, 2, 5500 + 10 * i)        # distinct periods
        total = 6 * n
        ent = 5                                          # the burst of this update writes entry 5
        before, after = total - 6 * ent, total - 6 * (ent + 1)
        e.dma_pos[3] = 6 * ent
        e.wr(e.addr("slew_tgt3"), 2, 2000)
        e.mmio[T1 + 0x24] = 1000
        e.script_ch, e.script_tim, e.script_i = 3, T3, 0
        e.cnt_step = 3
        if case == "in progress":
            e.mmio[T3 + 0x24] = 10
            e.cntr_script, e.script_done = [before - 2, before - 4, after, after], 3
        elif case == "not started":
            e.mmio[T3 + 0x24] = 4
            e.cntr_script, e.script_done = [before, before, before, before - 3, after, after], 5
        else:
            e.mmio[T3 + 0x24] = 300                      # the interrupt came late: the burst is done
            e.burst(T3)
            e.cntr_script, e.script_done = [after], 99
        e.call("slew3_isr")
        d = e.rd(e.addr("slew_pend3"), 4)
        d = d - (1 << 32) if d & 0x80000000 else d
        arr = e.mmio.get(T3 + 0x2C, 0)
        check(arr == 5500 + 10 * ent + d, "burst %s: ARR %d = entry %d's period %d + the step %d"
              % (case, arr, ent, 5500 + 10 * ent, d))


def adc(elf):
    print("adc: PA4 sample time, calibration with TSVREFE clear (gpt-hw H5 / H3)")
    e = Hal(elf)
    e.call("hw_adc_init")
    check(((e.mmio.get(ADC + 0x10, 0) >> 12) & 7) == 6, "SMP4 = 110 (71.5 cycles)")
    check(len(e.calib) == 2 and not any(c & (1 << 23) for c in e.calib), "RSTCAL / CAL with TSVREFE clear")
    check(e.mmio.get(ADC + 8, 0) & (1 << 23), "TSVREFE set after the calibration")


# ---------------------------------------------------------------------------------------------- flash (0.6.4)
FLASH, RCC_CFGR0, SYST, PFIC_B, USART2, DMA6_CNTR = 0x40022000, 0x40021004, 0xE000F000, 0xE000E000, 0x40004400, \
    DMA + 8 + 20 * 5 + 4
KEY1, KEY2 = 0x45670123, 0xCDEF89AB
OB_ADDR = 0x1FFFF800                                  # (0.6.8) the user option bytes (RM 32.6)


class FlashHal(Hal):
    """+ the flash controller (operations complete at once; status flags as injected), RCC, SysTick, PFIC VTF / enables,
    mtvec and the interrupt enable CSR 0x800, USART2's status"""
    def __init__(self, elf):
        super().__init__(elf)
        self.csr = {0x800: 0x88, 0x305: self.addr("trap_entry")}   # interrupts on; the app's entry (0.6.5)
        self.ops = []                                    # (kind, address, state) per erase / half-word
        self.no_eop = self.wrp = self.no_unlock = self.no_ob_unlock = False
        self.keys = []
        self.ob = [0xA5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]   # (0.6.8) the option bytes' low bytes (RM 32.6)
        self.mmio[FLASH + 0x10] = 0x8080                 # LOCK, FAST_LOCK
        self.mmio[USART2] = 0x00C0                       # TC, TXE
        self.mmio[RCC_CFGR0] = 0x0002 | (4 << 11)        # SW = PLL, PPRE2 /2
        self.mmio[SYST + 0x10] = 7199
        self.mmio[SYST] = 0x2F
        self.rx_during = 0                               # bytes the USART receives during an operation

    def _i(self, ins, pc, x):
        op, rd, f3, rs1 = ins & 0x7F, (ins >> 7) & 31, (ins >> 12) & 7, (ins >> 15) & 31
        if op == 0x73 and f3 != 0:
            c = ins >> 20
            old = self.csr.get(c, 0)
            v = rs1 if f3 >= 5 else x[rs1]
            if f3 & 3 == 1:
                self.csr[c] = v
            elif f3 & 3 == 2 and rs1:
                self.csr[c] = old | v
            elif f3 & 3 == 3 and rs1:
                self.csr[c] = old & ~v
            if rd:
                x[rd] = old
            return pc + 4, "alu"
        return super()._i(ins, pc, x)

    def state(self):
        return dict(irq_on=bool(self.csr.get(0x800, 0) & 0x08), cfgr=self.mmio.get(RCC_CFGR0, 0),
                    cmp=self.mmio.get(SYST + 0x10, 0), vtfid=self.mmio.get(PFIC_B + 0x50, 0) & 0xFF,
                    vtfaddr=self.mmio.get(PFIC_B + 0x60, 0), mtvec=self.csr.get(0x305, 0),
                    irer1=self.mmio.get(PFIC_B + 0x184, 0), modes=[self.mode(n) for n in CH])

    def rd(self, a, n):
        if a == FLASH + 0x0C:                            # STATR: done; EOP unless injected
            v = self.mmio.get(a, 0)
            return v
        return super().rd(a, n)

    def wr(self, a, n, v):
        if OB_ADDR <= a < OB_ADDR + 16 and n == 2:     # (0.6.8) an option byte: OBPG with OBWRE, unlocked
            ctl = self.mmio.get(FLASH + 0x10, 0)
            if ctl & 0x10 and ctl & 0x200 and not ctl & 0x80:
                self.ops.append(("obprog", a, self.state(), v & 0xFFFF))
                self.ob[(a - OB_ADDR) // 2] = v & 0xFF
                self.flash_done()
            return
        if a == FLASH + 0x08:                            # OBKEYR: the pair sets OBWRE (CTLR bit 9)
            self.keys.append((a, v))
            if len(self.keys) >= 2 and self.keys[-2] == (a, KEY1) and v == KEY2 and not self.no_ob_unlock:
                self.mmio[FLASH + 0x10] = self.mmio.get(FLASH + 0x10, 0) | 0x200
            return
        if 0x08000000 <= a < 0x08010000:
            ctl = self.mmio.get(FLASH + 0x10, 0)
            if ctl & 1 and not ctl & 0x80:               # PG, unlocked: a half-word
                self.ops.append(("prog", a, self.state()))
                self.flash_done()
                if self.wrp:
                    return
            return super().wr(a, n, v)
        if a == FLASH + 0x04 or a == FLASH + 0x24:
            self.keys.append((a, v))
            if len(self.keys) >= 2 and self.keys[-2] == (a, KEY1) and v == KEY2 and not self.no_unlock:
                self.mmio[FLASH + 0x10] = self.mmio.get(FLASH + 0x10, 0) & ~(0x80 if a == FLASH + 0x04 else 0x8000)
            return
        if a == FLASH + 0x0C:                            # STATR: write 1 to clear
            self.mmio[a] = self.mmio.get(a, 0) & ~(v & 0x30)
            return
        if a == FLASH + 0x10:
            if v & 0x40 and v & 0x20 and v & 0x200 and not v & 0x80:   # STRT with OBER (and OBWRE): the option bytes
                self.ops.append(("oberase", OB_ADDR, self.state(), None))
                self.ob = [0xFF] * 8
                self.flash_done()
                v &= ~0x40
            if v & 0x40 and v & 0x20000 and not v & 0x80:   # STRT with PAGE_ER: an erase
                self.ops.append(("erase", self.mmio.get(FLASH + 0x14, 0), self.state()))
                base = self.mmio.get(FLASH + 0x14, 0) - 0x08000000
                self.flash[base:base + 256] = b"\xff" * 256
                self.flash_done()
                v &= ~0x40
            self.mmio[a] = v
            return
        if a == SYST + 0x08 and n == 4:
            self.mmio[a] = v
            return
        super().wr(a, n, v)

    def flash_done(self):
        st = 0 if self.no_eop else 0x20
        if self.wrp:
            st = 0x10
        self.mmio[FLASH + 0x0C] = st
        if self.rx_during:                               # bytes arrive at the halved baud
            self.mmio[DMA6_CNTR] = (self.mmio.get(DMA6_CNTR, 64) - self.rx_during) or 64
            self.mmio[USART2] = self.mmio.get(USART2, 0) | 0x02     # FE
            self.rx_during = 0


def flash(elf):
    print("flash: flash_op (gpt-hw H4 / H8 / H9): the SRAM tick through VTF, HCLK / 2, status, the console")
    src = 0x20004000
    e = FlashHal(elf)
    e.mmio[DMA6_CNTR] = 64
    e.mmio[SYST + 0x08] = 3000
    e.poke(src, bytes([0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0xF0, 0xDE]))
    tick, fault = e.addr("flash_tick"), e.addr("flash_fault")
    e.call("flash_op", [0x0800FF00, src, 4, 0x0800FF00])
    ok = e.ret == 1
    kinds = [o[0] for o in e.ops]
    check(ok and kinds == ["erase", "prog", "prog", "prog", "prog"], "erase + 4 half-words: FLASH_OK (%d), %s"
          % (e.ret, kinds))
    check(e.peek(0x0800FF00, 8) == bytes([0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0xF0, 0xDE]), "the data programmed")
    bad = []
    irq1 = (1 << 12) | (1 << 13) | (1 << 14)           # TIM2 / TIM3 / TIM4
    if "rc_uart_isr" in e.syms:
        irq1 |= 1 << 23                                # the receiver (BOARD=2): its return channel's USART3 too
    for o in e.ops:
        k, a, s = o[0], o[1], o[2]
        if not s["irq_on"]:
            bad.append("%s: interrupts off" % k)
        if (s["cfgr"] >> 4) & 0xF != 0x8 or (s["cfgr"] >> 11) & 7 != 0:
            bad.append("%s: CFGR0 %04x (HPRE /2, PPRE2 /1 wanted)" % (k, s["cfgr"]))
        if s["cmp"] != 72000000 // 20000 - 1:
            bad.append("%s: SysTick CMP %d" % (k, s["cmp"]))
        if s["vtfid"] != 12 or s["vtfaddr"] != tick | 1:
            bad.append("%s: VTF %d %08x" % (k, s["vtfid"], s["vtfaddr"]))
        if s["mtvec"] != fault:
            bad.append("%s: mtvec %08x" % (k, s["mtvec"]))
        if s["irer1"] != irq1:
            bad.append("%s: IRER1 %08x" % (k, s["irer1"]))
        if 4 in s["modes"]:
            bad.append("%s: an output forced low" % k)
    check(not bad, "every erase / half-word: interrupts on, SysTick through VTF slot 0 into SRAM, the other "
          "interrupts off, mtvec to the SRAM fault handler, HCLK / 2 with PCLK2 kept, SysTick 50 us, no output "
          "touched %s" % (bad[:3] or ""))
    after = e.state()
    check(after["cfgr"] == 0x0002 | (4 << 11) and after["cmp"] == 7199 and after["vtfaddr"] == 0 and
          after["mtvec"] == e.addr("trap_entry") and e.mmio.get(PFIC_B + 0x104, 0) == irq1 and after["irq_on"] and
          e.mmio.get(FLASH + 0x10, 0) & 0x8080 == 0x8080,
          "after: the clocks, SysTick, VTF, mtvec, the interrupts and the locks as before (%s)" % after)
    check(e.mmio.get(SYST + 0x08, 0) in (2999, 3000, 3001), "SysTick's phase kept across the two clock changes (%d)"
          % e.mmio.get(SYST + 0x08, 0))
    for name, setup, want in (("write protection", "wrp", 2), ("no EOP", "no_eop", 3), ("no unlock", "no_unlock", 4)):
        f = FlashHal(elf)
        setattr(f, setup, True)
        f.call("flash_op", [0x0800FF00, src, 4, 0])
        n = len(f.ops)
        check(f.ret == want and n <= 1 and f.mmio.get(FLASH + 0x10, 0) & 0x8080 == 0x8080,
              "%s: FLASH_ERR %d (%d), stopped after %d operation(s), locked again" % (name, want, f.ret, n))
    if "uart_rx_r" in FlashHal(elf).syms:
        # the full console (CONSOLE_FULL 1): a byte received during the operation (at half the baud) is dropped, and
        # the line with it
        u = FlashHal(elf)
        u.mmio[DMA6_CNTR] = 60
        u.wr(u.addr("uart_rx_r"), 4, 4)
        u.rx_during = 3
        u.call("flash_op", [0x0800FF00, src, 1, 0])
        check(u.rd(u.addr("uart_rx_drop"), 1) == 1 and u.rd(u.addr("uart_rx_r"), 4) == 7,
              "a byte received meanwhile: dropped (rx pos %d), the line cancelled (%d)"
              % (u.rd(u.addr("uart_rx_r"), 4), u.rd(u.addr("uart_rx_drop"), 1)))
        u2 = FlashHal(elf)
        u2.mmio[DMA6_CNTR] = 60
        u2.wr(u2.addr("uart_rx_r"), 4, 4)
        u2.call("flash_op", [0x0800FF00, src, 1, 0])
        check(u2.rd(u2.addr("uart_rx_drop"), 1) == 0 and u2.rd(u2.addr("uart_rx_r"), 4) == 4,
              "nothing received meanwhile: nothing dropped")
    else:
        # (0.6.7) the release console only sends: hw_init leaves USART2's receiver off and DMA1 ch6 untouched (free)
        r = FlashHal(elf)
        r.call("hw_init")
        ch6 = [(a_, v) for a_, v in r.writes if dch(6) <= a_ < dch(6) + 0x10]
        check(not (r.mmio.get(0x4000440C, 0) & 4) and not ch6,
              "the release console: USART2 RE off (CTLR1 %04x), DMA1 ch6 never written (%d writes)"
              % (r.mmio.get(0x4000440C, 0), len(ch6)))
    # the SRAM tick: the same two-sample trip, through hw_flash_trip; a held sample is skipped
    t = FlashHal(elf)
    t.call("hw_init")
    start(t, 8, 4, 32768)
    isn = t.addr("is")
    t.call("isense_init", [isn, 1, 10000, 8000])
    t.mmio[0x4001244C] = 4095
    t.call("flash_poll_isr")
    one = t.mode("R")
    t.call("flash_poll_isr")
    check(one != 4 and all(t.mode(n) == 4 for n in CH) and t.rd(t.addr("pwm_forced"), 1) == 1,
          "the SRAM tick: two samples over the trip level force every output low (after one: %d)" % one)


def option_bytes(elf):
    print("option bytes (0.6.8): flash_op on HW_OB_ADDR, the bootloader's write protection (boot_protect)")
    src = 0x20004000
    want = [0xA5, 0x5A, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF]   # RDPR, USER, DATA0, DATA1, WRPR0 & 0xFC, WRPR1-3
    e = FlashHal(elf)
    e.poke(src, b"".join(bytes([v, 0]) for v in want))
    e.call("flash_op", [OB_ADDR, src, 8, OB_ADDR])
    kinds = [o[0] for o in e.ops]
    check(e.ret == 1 and kinds == ["oberase"] + ["obprog"] * 8, "the option bytes erased, then 8 half-words: FLASH_OK "
          "(%d), %s" % (e.ret, kinds))
    check([o[1] for o in e.ops[1:]] == [OB_ADDR + 2 * i for i in range(8)] and e.ob == want,
          "programmed in order, as given: RDPR 0xA5 kept, WRPR0 0xFC (%s)" % ["%02X" % v for v in e.ob])
    check((FLASH + 0x08, KEY1) in e.keys and (FLASH + 0x08, KEY2) in e.keys, "OBKEYR unlocked (OBWRE)")
    bad = [o[0] for o in e.ops if not o[2]["irq_on"] or (o[2]["cfgr"] >> 4) & 0xF != 0x8 or o[2]["mtvec"] !=
           e.addr("flash_fault") or 4 in o[2]["modes"]]
    check(not bad, "as every flash operation: the SRAM tick, HCLK / 2, the fault handler, no output touched %s" % bad)
    check(e.mmio.get(FLASH + 0x10, 0) & 0x8280 == 0x8080, "after: LOCK and FAST_LOCK set, OBWRE cleared (CTLR %05x)"
          % e.mmio.get(FLASH + 0x10, 0))
    f = FlashHal(elf)
    f.no_ob_unlock = True
    f.poke(src, b"".join(bytes([v, 0]) for v in want))
    f.call("flash_op", [OB_ADDR, src, 8, OB_ADDR])
    check(f.ret == 4 and not f.ops and f.ob[0] == 0xA5, "OBKEYR refused: FLASH_ERR_KEY (%d), the option bytes untouched"
          % f.ret)
    g = FlashHal(elf)
    g.poke(src, bytes([0x34, 0x12]))
    g.call("flash_op", [0x0800FF00, src, 1, 0])
    check(g.ret == 1 and (FLASH + 0x08, KEY1) not in g.keys and g.ob[4] == 0xFF,
          "a record write never unlocks the option bytes")


def records(elf):
    print("records (0.6.5): the boot window for the upper record pages, the PVD before every flash operation")
    base = 0x0002 | (4 << 11)
    e = FlashHal(elf)
    e.call("hw_records_window", [1])
    c1 = e.mmio.get(RCC_CFGR0, 0)
    e.call("hw_records_window", [0])
    c0 = e.mmio.get(RCC_CFGR0, 0)
    check(c1 == base | (0x8 << 4) and c0 == base, "the window: HCLK / 2 (CFGR0 %04x), then back (%04x)" % (c1, c0))
    h = FlashHal(elf)
    h.call("hw_init")
    pwr = h.mmio.get(0x40007000, 0)
    check(pwr & 0x10 and pwr & 0xE0 == 0xE0, "hw_init: the PVD on at 2.9 V (PWR CTLR %04x)" % pwr)
    f = FlashHal(elf)
    f.mmio[0x40007004] = 0x04                            # PVDO: VDD below the threshold
    f.call("flash_op", [0x08010000, 0x20004000, 4, 0x08010000])
    check(f.ret == 6 and not f.ops and f.mmio.get(RCC_CFGR0, 0) == base and f.mmio.get(FLASH + 0x10, 0) & 0x8080 == 0x8080,
          "VDD below the PVD: FLASH_ERR_VDD (%d), no operation, the clock and the locks untouched" % f.ret)


def vectors(elf):
    """0.6.5: no vector table; the startup's mtvec and trap_entry's dispatch on mcause"""
    print("vectors (0.6.5): mtvec = trap_entry in unified mode, each mcause to its handler, anything else to the loop")
    e = FlashHal(elf)
    e.csr.pop(0x305)
    x = [0] * 32
    pc, n = e.addr("_start"), 0
    while 0x305 not in e.csr and n < 200000:             # through the data copy / bss clear to the mtvec write
        h = e.rd(pc, 2)
        pc, _ = e._c(h, pc, x) if h & 3 != 3 else e._i(e.rd(pc, 4), pc, x)
        x[0] = 0
        n += 1
    entry = e.addr("trap_entry")
    mt = e.csr.get(0x305)
    check(mt == entry and entry & 3 == 0, "the startup: mtvec %s = trap_entry %08x, mode 0 (unified)"
          % ("%08x" % mt if mt is not None else "not written", entry))
    handlers = {e.addr(f): f for f in ("poll_isr", "gate_isr", "slew2_isr", "slew3_isr")}
    want = {0x8000000C: "poll_isr", 0x8000002E: "gate_isr", 0x8000002C: "slew2_isr", 0x8000002D: "slew3_isr"}
    if "rc_uart_isr" in e.syms:                          # (review r6) the receiver: USART3 (55) too, should VTF slot 1
        handlers[e.addr("rc_uart_isr")] = "rc_uart_isr"  # not take it
        want[0x80000037] = "rc_uart_isr"
    bad = []
    # every exception code (0-11: the privileged spec's below the page faults; QingKe V4 has no MMU), every interrupt
    # number of the CH32V203 (0-69; above 63 the low 6 bits alias 0-5)
    for cause in list(range(12)) + [0x80000000 | i for i in range(70)]:
        e.csr[0x342] = cause
        x = [0] * 32
        pc, seen, got = entry, set(), None
        for _ in range(40):
            if pc in handlers:
                got = handlers[pc]
                break
            if pc in seen:                               # the fault loop: the watchdog resets
                break
            seen.add(pc)
            h = e.rd(pc, 2)
            pc, _ = e._c(h, pc, x) if h & 3 != 3 else e._i(e.rd(pc, 4), pc, x)
            x[0] = 0
        if got != want.get(cause) or (got is None and pc not in seen):
            bad.append("%08x -> %s" % (cause, got))
    check(not bad, "SysTick / TIM4 / TIM2 / TIM3%s to poll / gate / slew2 / slew3%s, the other %d causes to the loop %s"
          % (" / USART3" if len(want) > 4 else "", " / rc_uart" if len(want) > 4 else "", 82 - len(want), bad[:4] or ""))


def ramtext(elf):
    """every call / jump from the SRAM code stays in SRAM (nothing is fetched from the flash during an operation)"""
    import re
    import subprocess
    print("ramtext: the SRAM code calls nothing in the flash")
    e = Emu(elf)
    lo, hi = e.addr("__ramtext_start"), e.addr("__ramtext_end")
    objdump = os.environ.get("OBJDUMP", "riscv-none-elf-objdump")
    dis = subprocess.run([objdump, "-d", "-j", ".ramtext", elf], capture_output=True, text=True).stdout
    out = []
    for l in dis.splitlines():
        m = re.search(r"\t(jal|j|call|tail|jalr|beq|bne|blt|bge|bltu|bgeu|beqz|bnez)\s.*?([0-9a-f]{1,8}) <", l)
        if m and not lo <= int(m.group(2), 16) < hi:
            out.append(l.strip())
    funcs = re.findall(r"<([^>]+)>:", dis)
    inside = all(lo <= e.addr(f) < hi for f in ("flash_tick", "flash_fault", "flash_poll_isr", "hw_flash_trip",
                                                  "flash_wait"))
    check(not out and inside, "%s; outside: %s" % (", ".join(funcs), out[:3] or "none"))

if __name__ == "__main__":
    elf = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "build", "target-lto0",
                                                             "fw.elf")
    which = sys.argv[2:] or ["h2", "h6", "adc", "flash", "option_bytes", "records", "vectors", "ramtext"]
    for w in which:
        {"h2": h2, "h6": h6, "adc": adc, "flash": flash, "option_bytes": option_bytes, "records": records,
         "vectors": vectors,
         "ramtext": ramtext}[w](elf)
    print("hal_check: %s (%d failed)" % ("FAILED" if fails else "passed", fails))
    sys.exit(1 if fails else 0)

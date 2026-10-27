# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Chain frames back to back on the COMPILED BOARD=3 image, with the main loop PREEMPTED by the 20 kHz tick (the Diff-Power-UR-Digital-M alone on a
normal Ultra line, the hub at the wire-3 minimum 500 us reset).

test_ur_digital_m_chain_fw.py runs one whole main-loop pass between two ticks, however many cycles it takes. Here each tick gets 7,200 cycles (144 MHz / 20 kHz):
poll_isr first (its own cycles, the chain scanner's copy included), the main loop the rest, resuming where it was preempted (and sleeping at WFI until
the next tick). So a decode that needs 30 k cycles spans as many ticks as the chip would need, and a frame whose start finds the capture still held is
dropped as on the chip.

The line is the hub's normal timing (reset_us between frames, default 500 us; frames back to back while its queue has commands, the
preamble too): 10 DARK frames, CONFIG (V5, 40 px, 60 fps), PIXEL_FX, a burst of 20 NOP frames back to back, a POLL right behind the burst (reset), then
quiet refreshes (9 ms), a POLL after a quiet gap. The clock's rate is poked as measured (the board answers no POLL before it is: test_udchain.c covers
that). The line is shifted against the 50 us tick grid by `phase_us`, in several steps: where the tick falls inside the frame changes the decode's
latency by up to a tick.

Checked: every frame sent is classified (uc.frames == sent), no frame is counted as no Ultra frame (a header read over by the next capture would be) and
both POLLs are answered (slot 0 permits DE from 995 .. 1,080 us after the frame's last edge). Three groups:
  1. a light effect (40 px Rainbow), the RW frame's release at +150 us (the protocol's latest), at reset_us and again MARGIN_US shorter (the capture is free
     well before the classification: a frame that starts while it is held is dropped, and the count shows it), `phases` phases against the tick;
  2. releases at +60 and +100 us after the last edge; the line then remains HIGH.
     Invalid releases below the 60 us protocol minimum are rejected in test_udchain.c;
  3. a heavy effect (Lava Lamp, 600 px) at reset_us with the release at +150 and +60 us, every phase: its main
     loop looks at the line only between two steps, the capture must still be freed before the next frame, and the release HIGH 150 us after an RW frame
     must not start a capture that overwrites the header before it is read.
The runs go in parallel (processes). The slowest classification is printed.

The board is at any position of its line: every valid POLL opens its relay window (main_ur_digital_m.c: DE high from the turnaround,
t0 + 500 us, to forward-again, W + 6 us), and the gate's slot plan (TIM2 CH2) re-arms after 200 us of quiet LOW. The model follows the pins:
DE is high from its first write to its last release (the burst inside a window keeps it), the burst's TC comes 760 words after the UART's
TXE interrupt is enabled, PA6 follows the line while DE is low (RO high-Z while it is high), TIM2 counts the line's falling edges; the gate's
mode writes and DE's edges are returned for test_ur_digital_m_relay_budget.py.

The admitted-write model holds BSY for 16 ms/erase and 100 us/halfword and
executes flash_poll_isr during the stall; these are stress inputs, not a silicon maximum.
Usage: python test_ur_digital_m_chain_budget.py <fw.elf> [reset_us=500] [phases=4]
Build the image: XW=0 LTO=0 BOARD=3 OUT=build/ur_digital_m-opt bash tools/build_target.sh  (or the -fno-inline API image)
"""
import concurrent.futures
import os
import re
import struct
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
sys.path.insert(0, str(FW / "test"))
from mainloop import Emu2  # noqa: E402
from flash_stores import FlashEmu  # noqa: E402
from rv32emu import COST, RET_MAGIC  # noqa: E402
from ur_digital_m_target import offsets  # noqa: E402

TICK_CYCLES = 7200
ISR_OVERHEAD = 60          # entry / exit, the mret
S_PER_US = 36
PERIOD, H0, H1 = 45, 11, 34
MARGIN_US = 100            # the same line with this much less reset must still be classified whole
_protocol=(FW/"src/core/ultra_protocol.h").read_text()
def wire_us(name):
    return int(re.search(r"^#define "+name+r"\s+(\d+)u",_protocol,re.M)[1])
RW_FIRST=wire_us("ULTRA_RW_FIRST_US")
RW_EARLY0=wire_us("ULTRA_RW_EARLY0_US")
RW_LATE=wire_us("ULTRA_RW_LATE_US")
R_SLOTS=1          # the POLL's R (the window's slots; test_ur_digital_m_relay_budget.py sets more)
POLL_INDEX=0       # this board's index in the POLL (255 = RELAY_ONLY)


class Board(FlashEmu, Emu2):
    tracing = False
    flash_model = False
    flash_ticks = 0
    flash_mode = False
    flash_stalls = None
    cycle_clock = 0
    pin_times = None

    def wr(self, a, n, v):
        ctl = self.mmio.get(0x40022010,0)
        super().wr(a,n,v)
        if a == 0x40000018:
            self.mmio[a] = v & 0xFFFF              # TIM2 CHCTLR1 reads back (the gate's mode: upd_forward.h gate_slot_armed, flash_op)
        if self.pin_times is not None and a==0x4001380C and v & 0x80:
            # the answer's burst: USART1 TXEIE (DE is high from the relay window's turnaround, t0 + 500 us, on)
            self.pin_times.append(self.model_us+(self.cycle_clock+1)/144)
        if self.flash_model:
            if a == 0xE000E060:
                self.flash_mode = bool(v)
            if a == 0x40022010 and v & 0x20040 == 0x20040:
                self.flash_ticks = 320             # 16 ms erase stress
            elif 0x08010000 <= a < 0x0801c000 and ctl & 1:
                self.flash_ticks = 2               # 100 us / halfword stress
            else:
                return
            self.flash_stalls.append((self.model_us,self.flash_ticks*50))
    def _c(self, h, pc, x):
        result=super()._c(h,pc,x)
        self.cycle_clock += COST[result[1]]
        if self.tracing: self.trace[pc]=self.trace.get(pc,0)+COST[result[1]]
        return result
    def _i(self, insn, pc, x):
        result=super()._i(insn,pc,x)
        self.cycle_clock += COST[result[1]]
        if self.tracing: self.trace[pc]=self.trace.get(pc,0)+COST[result[1]]
        return result
    systick = 0
    adc_fake = False
    adc_current = 63
    cal_entry = 0
    cal_return = 0

    def rd(self, a, n):
        if self.adc_fake:
            if a == 0x40012400: return 0x14  # regular start + injected completion
            if a == 0x4001244C: return self.adc_current
            if a in (0x4001243C, 0x40012444): return 1303
            if a in (0x40012440, 0x40012448): return 1152
        if a == 0xE000F008:
            self.systick = (self.systick + 40) % 7200
            return self.systick
        return super().rd(a, n)

    def run_budget(self, pc, stop, budget, x):
        """run from pc until `budget` cycles are used or the pc reaches `stop` (after >= 1 step); returns (pc, cycles, stopped)"""
        self.x = x
        c = 0
        n = 0
        while pc != RET_MAGIC:
            if pc == getattr(self, "stream_pc", 0):
                sb = self.rd(self.pre_addr+10,2)
                if sb & 0x8000:
                    self.prefix_errors.add((self.rd(self.pre_addr,4),sb-65536,self.rd(self.pre_addr+4,2)))
            if pc in stop and n > 0:
                return pc, c, True
            if c >= budget:
                return pc, c, False
            if pc == self.cal_entry and 1 <= self.rd(self.cal_phase,1) <= 8:
                self.cal_return=x[1];self.cal_live=0
                self.cal_kind='raw/acquire' if self.rd(self.cal_pending,1) else 'prepare/reduce'
            n += 1
            h = self.rd(pc, 2)
            if h & 3 != 3:
                npc, cls = self._c(h, pc, x)
            else:
                npc, cls = self._i(self.rd(pc, 4), pc, x)
            c += COST[cls]
            if self.cal_return:
                self.cal_live += COST[cls]
                if npc == self.cal_return:
                    self.cal_peaks[self.cal_kind]=max(self.cal_peaks.get(self.cal_kind,0),self.cal_live)
                    self.cal_return=0
            x[0] = 0
            pc = npc
            if self.flash_ticks:
                return pc, c, False
        return pc, c, True


def crc(data, width, poly, init):
    v = init
    m = (1 << width) - 1
    for b in data:
        v ^= b << (width - 8)
        for _ in range(8):
            v = ((v << 1) ^ (poly if v & (1 << (width - 1)) else 0)) & m
    return v


def slot(b, seal=True):
    b = bytearray(b) + bytearray(32 - len(b))
    if seal:
        b[30:32] = crc(b[:30], 16, 0x1021, 0xFFFF).to_bytes(2, "big")
    return bytes(b)


def header(host, flags):
    h = bytearray(b"\x5a\xd2" + bytes([0x30 | flags]) + struct.pack(">I", host & 0xFFFFFFFF))
    h.append(crc(h, 8, 7, 0))
    return bytes(h)


class Line:
    """the hub's line as 36 MHz samples"""

    def __init__(self, rel_us=150):
        self.line = bytearray()
        self.frame_ends = []
        self.frame_starts = []
        self.rel_us = rel_us                                # the host lets go of the pair this long after an RW frame's last edge

    def low_to(self, us):
        n = int(us * S_PER_US) - len(self.line)
        if n > 0:
            self.line.extend(b"\0" * n)

    def frame(self, slot_bytes, rw=0, kind="frame", gap_us=500):
        line = self.line
        start = len(line)
        self.frame_starts.append(start)
        for _ in range(3):
            end = start + ((len(slot_bytes)+8)*8-1) * PERIOD
            f = slot_bytes + header(0, rw)
            last = f[-1] & 1
            end_s = end + (H1 if last else H0)
            f = slot_bytes + header(end_s // S_PER_US, rw)
            if (f[-1] & 1) == last:
                break
        for byte in f:
            for k in range(8):
                h = H1 if (byte >> (7 - k)) & 1 else H0
                line.extend(b"\1" * h + b"\0" * (PERIOD - h))
        t_end = len(line) - (PERIOD - (H1 if f[-1] & 1 else H0))
        self.frame_ends.append((kind, t_end))
        if rw:                                              # the hub releases at +rel_us, the pair floats HIGH until it retakes after its window (one slot)
            line.extend(b"\0" * int(self.rel_us * S_PER_US))
            retake_us = 1000 + R_SLOTS * 500 + 40
            line.extend(b"\1" * (retake_us * S_PER_US - (len(line) - t_end)))
            line.extend(b"\0" * (200 * S_PER_US))
        else:
            line.extend(b"\0" * (gap_us * S_PER_US))

    def words_of(self, a, b):
        out = []
        for w in range(a, b):
            s = self.line[w * 16:(w + 1) * 16]
            if len(s) < 16:
                s = s + b"\0" * (16 - len(s))
            v = 0
            for x in s:
                v = (v << 1) | x
            out.append(v)
        return out


def run_once(elf, O, reset_us, phase_us, rel_us=150, npx=40, effect=4, chip=0, fade=0, calibration=0, profile=0, qualified=1, training=0, flash_pending=0, late_join=0, goldens=0, admitted_flash=0, header_join=0, header_record=None, scenario=None):
    global PERIOD, H0, H1
    PERIOD=(45,60,90,180)[profile]
    H0=(PERIOD+2)//4; H1=PERIOD-H0
    L = Line(rel_us)
    seq = [1]

    def nseq():
        seq[0] += 1
        return seq[0]
    if late_join:
        # Start halfway through unrelated traffic, without a preceding reset.
        # Only later whole frames may establish AUTO.
        for bit in range(157):
            high = H1 if bit & 1 else H0
            L.line.extend(b"\1"*high+b"\0"*(PERIOD-high))
    L.low_to(3000 + phase_us)
    header_checks = []
    if header_join:
        for kind, count in (("bad_crc",3),("bad_pulse",3),("good",3)):
            for k in range(count):
                h=bytearray(header(0,0))
                if kind=="bad_crc": h[-1]^=1
                for byte in h:
                    for shift in range(7,-1,-1):
                        high=H1 if (byte>>shift)&1 else (1 if kind=="bad_pulse" else H0)
                        L.line.extend(b"\1"*high+b"\0"*(PERIOD-high))
                end=len(L.line)-(PERIOD-(H1 if h[-1]&1 else (1 if kind=="bad_pulse" else H0)))
                if kind=="good": L.frame_ends.append(("header_only",end))
                header_checks.append((end/S_PER_US+400,kind,k+1))
                L.line.extend(b"\0"*(reset_us*S_PER_US))
    dark = slot(bytes(30), seal=False)
    for _ in range(10):
        L.frame(dark, 0, "dark", reset_us)
    if training:
        # Real wire traffic, not poked qualification state. Complete the four
        # training patterns over a full second before application commands.
        for i, pattern in enumerate((0x55, 0xaa, 0, 0xff, 0x55)):
            L.frame(slot(bytes(3)+bytes([pattern])*27), 0, "training",
                    250000 if i < 4 else reset_us)
    cfg = bytes([0x05, 0, nseq()]) + bytes([64, 0, chip, 66, npx >> 8, npx & 255, 67, 0, 60])
    L.frame(slot(cfg), 0, "config", reset_us)
    fx = bytearray([0x0B, 15, nseq(), -fade if fade < 0 else effect, 0, 0, 0, 0, 128, 128, 0xFF, 255, 0, 0] + [0] * 6 + [255, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if effect == 16:
        fx[4:8] = struct.pack('>i', -500)               # exercise the four-step catch-up, too
        fx[8:10] = bytes([255, 255])
        fx[28] = 55
    if calibration:
        fx[20] = 8                                  # advancing Fire, safely below the fresh-current guard
    if admitted_flash:
        fx[20] = 0                                  # a physically latched black local scene
    L.frame(slot(fx), 0, "fx", 9000)
    if fade:
        fx[2] = nseq()
        fx[3] = effect if fade < 0 else fade
        if fade == effect:
            fx[10] = 13                               # shared-heat Fire palette fade
        fx[21:23] = struct.pack('>H', 2000)
        L.frame(slot(fx), 0, "fade", 9000)
    if calibration:
        # Allow the in-flight scene, black frame and 48-ms dark block.
        L.frame(slot(bytes([0x06, 0, nseq(), 2])), 0, "calibrate", 80000)
    burst0 = len(L.frame_ends)
    retry = slot(bytes([0x05,0,nseq(),66,0,37])) if admitted_flash else None
    lost_end = None
    for i in range(20):                                     # the hub's queue: frames back to back
        if admitted_flash and i==15:
            L.frame(retry,0,"lost_command",reset_us)
            lost_end=L.frame_ends[-1][1]/S_PER_US
        else:
            L.frame(slot(bytes([0x00, 0, nseq()])), 0, "burst", reset_us)
    burst1 = len(L.frame_ends)
    poll = bytes([0x07, 255, nseq(), POLL_INDEX, R_SLOTS, 6 if calibration else 0x10, 0, 1 if calibration else 0, 1 if calibration else 0])
    L.frame(slot(poll), 1, "poll_b2b")                      # right behind the burst's last frame
    L.low_to(len(L.line) / S_PER_US + 9000)
    for _ in range(4):
        L.frame(slot(bytes([0x00, 0, nseq()])), 0, "nop", 9000)
    poll = bytes([0x07, 255, nseq(), POLL_INDEX, R_SLOTS, 6 if calibration else 0x10, 0, 1 if calibration else 0, 1 if calibration else 0])
    L.frame(slot(poll), 1, "poll_quiet")
    L.low_to(len(L.line) / S_PER_US + 9000)
    for _ in range(3):
        L.frame(slot(bytes([0x00, 0, nseq()])), 0, "nop", 9000)
    golden_count = 0
    if goldens:
        for row in (FW/"test/protocol/golden_traces.h").read_text().splitlines():
            m = re.match(r"\{(\d+)u,(\d+)u,(\d+)u,(\d+)u,(\d+)u,\d+u,\d+u,\{([^}]+)\}", row)
            if m and int(m[1]) == profile:
                clock, period, zero, one = map(int, m.group(2,3,4,5))
                frame = bytes(int(b,16) for b in m[6].split(","))
                # Keep the independent sender origins and wrap bytes exact.
                L.low_to(((len(L.line)//S_PER_US+49)//50*50)+phase_us)
                start = len(L.line)
                bit = 0
                for byte in frame:
                    for shift in range(7,-1,-1):
                        high = one if (byte >> shift) & 1 else zero
                        fall = round((bit*period+high)*36_000_000/clock)
                        end = round((bit+1)*period*36_000_000/clock)
                        L.line.extend(b"\1"*(start+fall-len(L.line)))
                        L.line.extend(b"\0"*(end-fall))
                        bit += 1
                L.frame_ends.append(("golden",start+fall))
                L.line.extend(b"\0"*(reset_us*S_PER_US))
                golden_count += 1
    if admitted_flash:
        # Retry the same valid command after the write. Application state and
        # publication mailboxes must recover without a reset.
        retry_start=len(L.line)/S_PER_US
        for _ in range(3):
            L.frame(retry,0,"retry",9000)
    if header_record:
        L=Line(rel_us)
        L.low_to(3000+phase_us)
        for _ in range(60):
            L.frame(b"",0,"header_only",20000)
        header_loss_end=L.frame_ends[-1][1]/S_PER_US
    if scenario: L=scenario.line
    T_US = len(L.line) // S_PER_US + (900000 if header_record else 2000)
    L.low_to(T_US)
    sent = len(L.frame_ends)

    e = scenario.board(elf) if scenario else Board(elf)
    # (the prefix decoder's entry: LTO / IPA may clone or rename it, or inline it; 0 = no prefix check)
    e.stream_pc = next((v[0] for n, v in sorted(e.syms.items()) if n == "udc_stream" or n.startswith("udc_stream.")), 0)
    e.pre_addr = e.addr("ucp")
    e.prefix_errors = set()
    e.poke(0x08010000, b"\xff" * 0xC000)
    if header_record: e.poke(0x08010000,header_record)
    e.mmio.update({0x40021000: 0x02000002, 0x40021004: 0x08, 0x40004400: 0xC0, 0x40010C08: 1 << 10, 0x40010808: 1 << 3})
    if scenario: scenario.prepare(e)
    base, size = e.sym("main")
    wfi = [q for q in range(base, base + size, 2) if e.rd(q, 4) == 0x10500073]
    stop = set(wfi)
    pc, _ = e.run_to(e.start("main"), stop, max_steps=3000000)
    e.adc_fake = bool(calibration)
    phases_seen = set()
    if calibration:
        from ur_digital_m_cal_target import offsets as cal_offsets
        CO = cal_offsets(elf)
        ca = e.addr('ucal')
        frame_seen = -1
        e.cal_entry=e.addr('cal_work');e.cal_phase=ca+CO['phase'];e.cal_pending=ca+CO['pending'];e.cal_peaks={}
    main_x = list(e.x)
    e.flash_model = bool(admitted_flash)
    e.flash_stalls = []
    e.model_us = 0
    e.pin_times = []
    ux = e.addr("ux")
    # This scheduling benchmark starts with a completed prior qualification
    # interval; acquisition/early-answer refusal is checked independently.
    if qualified:
        e.poke(e.addr("line_lock"),struct.pack('<IIBB',0,0xfffff800,profile,3))
        e.wr(e.addr("dl")+O["dl_line_profile"],1,profile)
        e.wr(e.addr("line_stats")+O["ls_trained"],1,1)
        e.wr(e.addr("line_stats")+O["ls_profile"],1,profile)
        e.wr(ux + O["ux_r_have"], 1, 1)                     # seeded scheduling cases only; acquisition measures its own rate
    ring = e.addr("dl_ring")
    uc = e.addr("uc")
    edges, prev = 0, 0
    tedges = 0
    big_ticks = []
    gate_ev = []
    de_on, de_off = [], []
    de, de_end = 0, None
    de_rises = []
    arm_times = []
    arm_previous = 0
    flash_queued = False
    flash_ops = 0
    frames_at = []                                          # (word, uc.frames) after each tick
    frame_state = []
    wpos = 0
    ticks = 0
    total_words = T_US * S_PER_US // 16
    busy_ticks = longest_busy = cur_busy = max_isr = 0
    worst_trace={}
    flash_dark = True
    flash_pixels = True
    flash_sends = 0
    flash_ram_ticks = 0
    max_flash_isr = 0
    lost_during_flash = False
    lost_applied = False
    header_states = []
    header_dark = True
    loss_first = None
    while wpos < total_words:
        step = 113 if ticks % 2 else 112
        ticks += 1
        new = L.words_of(wpos, wpos + step)
        for i, v in enumerate(new):
            w = wpos + i
            rxv = 0 if de else v
            e.wr(ring + 2 * (w % 512), 2, rxv)
            for j in range(16):
                b = (rxv >> (15 - j)) & 1
                edges += prev and not b
                tedges += prev and not b
                prev = b
            if de and de_end is not None and w >= de_end:
                de_end = None
                e.writes.clear()
                saved = list(e.x)
                # LTO may fold dl_tx_done into its sole caller. Finish the UART
                # through that retained interrupt entry in production images.
                if 'dl_tx_done' in e.syms:
                    done = 'dl_tx_done'
                else:
                    e.mmio[0x4001380C] = 0x2048  # USART1 TE|UE|TCIE, last byte has left
                    done = 'rc_uart_isr'
                x = e.start(done)
                e.x[2] = (main_x[2] - 256) & ~15
                e.run_to(x, set(), max_steps=100000)
                e.x = saved
                for a_, v_ in e.writes:
                    if (a_ == 0x40010C10 and v_ & (1 << 26)) or (a_ == 0x40010C14 and v_ & (1 << 10)):
                        de = 0; de_off.append(w)
                e.writes.clear()
        wpos += step
        rx = wpos % 512
        e.mmio[0x40020020] = 512 - rx if rx else 512
        e.mmio[0x40020034] = 512 - ((rx + 2) % 512)
        e.mmio[0x40000824] = edges & 0xFFFF
        e.mmio[0x40000024] = tedges & 0xFFFF
        e.mmio[0x40010808] = (1 << 3) | ((1 << 6) if (not de and new and (new[-1] & 1)) else 0)   # PA6 = the line now (RO high-Z while DE)
        e.writes.clear()
        e.model_us = wpos*16/S_PER_US
        if admitted_flash:
            if lost_end <= e.model_us < lost_end+50:
                lost_during_flash=bool(e.flash_mode)
            if e.model_us<retry_start and e.rd(ux+O["ux_npx"],2)==37:
                lost_applied=True
        # Every busy flash interval runs the firmware's SRAM interrupt. The
        # normal main PC resumes after BSY, preserving its actual call stack.
        in_flash = bool(e.flash_ticks)
        e.cycle_clock = 0
        x = e.start("flash_poll_isr" if e.flash_mode else "poll_isr")
        e.x[2] = (main_x[2] - 256) & ~15                    # the ISR's frame below the preempted main loop's (as on the chip)
        e.tracing=bool(os.environ.get('PROFILE_ISR')); e.trace={}
        e.run_to(x, set(), max_steps=2000000)
        e.tracing=False
        isr_c = e.cycles()
        if e.flash_mode: max_flash_isr=max(max_flash_isr,isr_c)
        if isr_c>max_isr: worst_trace=dict(e.trace)
        max_isr=max(max_isr,isr_c)
        if isr_c > 4700: big_ticks.append((round(e.model_us), isr_c, de))
        tick_writes = list(e.writes)
        e.writes.clear()
        budget = (TICK_CYCLES//2 if e.flash_mode else TICK_CYCLES) - isr_c - ISR_OVERHEAD
        if in_flash:
            flash_ram_ticks += 1
            flash_dark &= bool(e.rd(ux+O["ux_local"],1) and e.rd(ux+O["ux_dark"],1))
            flash_pixels &= not any(e.peek(e.addr("ux_pool"),npx*3))
            flash_sends += sum(a == 0x40010C10 and bool(v & (1<<10)) for a,v in tick_writes)
            e.flash_ticks -= 1
            used, stopped = 0, False
        else:
            pc, used, stopped = e.run_budget(pc, stop, budget, main_x)
            main_x = list(e.x)
        arm_now = e.rd(e.addr("dl") + O["dl_ch_arm"], 1)
        if arm_now and not arm_previous:
            # Conservative upper bound: publication occurred during this
            # main-loop budget, before the following tick.
            arm_times.append(wpos * 16 / S_PER_US + 50)
        arm_previous = arm_now
        if header_record:
            lit=any(e.peek(e.addr("ux_pool"),npx*3))
            if e.model_us<header_loss_end: header_dark &= not lit
            if lit and loss_first is None: loss_first=e.model_us-header_loss_end
        if header_checks and e.model_us>=header_checks[0][0]:
            _,kind,count=header_checks.pop(0)
            lock=e.addr("line_lock")
            header_states.append(dict(kind=kind,count=count,profile=e.rd(lock+8,1),
                good=e.rd(lock+9,1),mode=e.rd(uc+O["uc_mode"],1),
                trained=e.rd(e.addr("line_stats")+O["ls_trained"],1),
                clock=e.rd(e.addr("clk_have"),1),rate=e.rd(ux+O["ux_r_have"],1),
                answers=len(de_rises),decoded=e.rd(e.addr("uds")+O["uds_decoded"],2),
                pixels=any(e.peek(e.addr("ux_pool"),1800))))
        if flash_pending and not flash_queued and e.rd(ux+O["ux_seg"]+O["seg_p"]+O["fx_effect"],1) == effect:
            ms = e.rd(e.addr("ms_now"),4)
            e.wr(ux+O["ux_save_ms"],4,(ms-2000)&0xffffffff)
            e.wr(e.addr("ufx_last_ms"),4,(ms-30000)&0xffffffff)
            flash_queued = True
        # FlashEmu records controller commands even when no busy delay is
        # modeled. A pending write must stay deferred during this lit/cal job.
        flash_ops += sum(a == 0x40022010 and bool(v & 0x41)
                         for a,v in tick_writes + list(e.writes))
        if calibration:
            phases_seen.add(e.rd(ca + CO['phase'], 1))
            frame_now = e.rd(ux + CO['enc'] + CO['frames'], 4)
            if frame_now != frame_seen:
                frame_seen = frame_now
                bpp = e.rd(ux + O['ux_chip'] + O['chip_bpp'], 1)
                # A separate linear physical load, including the normal scene.
                value = sum(e.peek(e.addr('ux_pool'), (600 if bpp==3 else 500 if bpp==4 else 400)*bpp))
                e.adc_current = round((100 + value*20/255)*65536/e.rd(e.addr('i_q16'),4))
        if not stopped:
            cur_busy += 1
            busy_ticks += 1
            longest_busy = max(longest_busy, cur_busy)
        else:
            cur_busy = 0
        for a, v in tick_writes + list(e.writes):
            if a == 0x40000824 and v == 0:
                edges = 0
            elif a == 0x40000024 and v == 0:
                tedges = 0
            elif a == 0x40000018:
                gate_ev.append((wpos * 16 / S_PER_US, (v >> 12) & 7))
            elif a == 0x40010C10 and v & (1 << 10):
                if not de: de_on.append(wpos)
                de = 1
            elif (a == 0x40010C10 and v & (1 << 26)) or (a == 0x40010C14 and v & (1 << 10)):
                if de: de_off.append(wpos)
                de = 0
                de_end = None
            elif a == 0x4001380C and v & 0x80 and de_end is None:
                de_end = wpos + 760                         # the burst (TXEIE): its TC after 32 + 1 bytes
                de_rises.append(wpos)
        frames_at.append((wpos, struct.unpack_from("<H", e.peek(uc + O["uc_frames"], 2))[0]))
        if scenario: scenario.observe(e)
        if frames_at[-1][1] > len(frame_state):
            frame_state.append(dict(frame=frames_at[-1][1], npx=e.rd(ux+O["ux_npx"],2),
                 effect=e.rd(ux+O["ux_seg"]+O["seg_p"]+O["fx_effect"],1),
                 decoded=e.rd(e.addr("uds")+O["uds_decoded"],2),
                 dec_err=e.rd(e.addr("uds")+O["uds_dec_err"],2)))
    frames = frames_at[-1][1]
    if scenario: return scenario.finish(e,dict(frames=frames,sent=sent,max_isr=max_isr,pin_times=e.pin_times))
    if header_record:
        return dict(profile=profile,phase=phase_us,frames=frames,sent=sent,
            max_isr=max_isr,mode=e.rd(uc+O["uc_mode"],1),header_dark=header_dark,
            loss_first_us=loss_first,loss_lit=any(e.peek(e.addr("ux_pool"),npx*3)),
            lock_profile=e.rd(e.addr("line_lock")+8,1),
            trained=e.rd(e.addr("line_stats")+O["ls_trained"],1),
            clock=e.rd(e.addr("clk_have"),1),rate=e.rd(ux+O["ux_r_have"],1),
            answers=len(de_rises),decoded=e.rd(e.addr("uds")+O["uds_decoded"],2))

    classified_at = []
    last = 0
    for w, f in frames_at:                                  # the tick after which uc.frames went up
        while last < f:
            classified_at.append(w * 16)
            last += 1
    ends = [s for _, s in L.frame_ends]
    delay = [(c - s) / S_PER_US for c, s in zip(classified_at, ends)]
    burst_delay = delay[burst0:burst1] if len(delay) >= burst1 else []
    answers = {}
    pin_answers = {}
    armed_by = {}
    for name in ("poll_b2b", "poll_quiet"):
        t = [s for k, s in L.frame_ends if k == name][0]
        r = [w for w in de_rises if t < w * 16 < t + 2000 * S_PER_US]
        answers[name] = (r[0] * 16 - t) / S_PER_US if r else None
        pins=[at-t/S_PER_US for at in e.pin_times if t/S_PER_US<at<t/S_PER_US+2000]
        pin_answers[name]=pins[0] if pins else None
        armed = [at-t/S_PER_US for at in arm_times if t/S_PER_US < at < t/S_PER_US+900]
        armed_by[name] = armed[0] if armed else None
    worst = max(burst_delay) if burst_delay else 9999.0
    nonultra = struct.unpack_from("<H", e.peek(uc + O["uc_nonultra"], 2))[0]
    by={}
    if worst_trace:
        import bisect
        funcs=sorted((addr,name) for name,(addr,size) in e.syms.items() if size and addr<0x20000000)
        starts=[x[0] for x in funcs]
        for pc,cost in worst_trace.items():
            ix=bisect.bisect_right(starts,pc)-1
            name=funcs[ix][1] if ix>=0 else 'unknown'
            by[name]=by.get(name,0)+cost
    relay = dict(worst_trace=worst_trace, big_ticks=big_ticks, gate_ev=gate_ev, de_on=[w * 16 / S_PER_US for w in de_on], de_off=[w * 16 / S_PER_US for w in de_off],
                 starts=[x / S_PER_US for x in L.frame_starts], ends_us=[(k, s_ / S_PER_US) for k, s_ in L.frame_ends],
                 rel_busy=e.rd(e.addr("relay_busy"),2) if "relay_busy" in e.syms else None)
    return dict(relay=relay,prefix_errors=sorted(e.prefix_errors),frame_state=frame_state,effect_active=e.rd(ux+O["ux_seg"]+O["seg_p"]+O["fx_effect"],1),
                decoded_final=e.rd(e.addr("uds")+O["uds_decoded"],2),
                dec_err_final=e.rd(e.addr("uds")+O["uds_dec_err"],2),
                qualified=qualified,training=training,late_join=late_join,goldens=golden_count,
                header_join=header_join,header_states=header_states,
                trained=e.rd(e.addr("line_stats")+O["ls_trained"],1),
                rate_measured=e.rd(ux+O["ux_r_have"],1),
                isr_functions=by,max_isr=max_isr,sent=sent, frames=frames, worst=worst,
                answers=answers,pin_answers=pin_answers,armed_by=armed_by,
                flash_pending=flash_pending,flash_queued=flash_queued,flash_ops=flash_ops,
                flash_stalls=e.flash_stalls,flash_ram_ticks=flash_ram_ticks,
                max_flash_isr=max_flash_isr,
                flash_dark=flash_dark,flash_pixels=flash_pixels,flash_sends=flash_sends,
                lost_during_flash=lost_during_flash,lost_applied=lost_applied,
                retry_pixels=e.rd(ux+O["ux_npx"],2),
                mailbox_clear=not (e.rd(e.addr("dl")+O["dl_ch_ready"],1) or
                    e.rd(e.addr("dl_ch_ready"),1) or e.rd(e.addr("dl_ready"),1)),
                flash_deferred=not flash_pending or (flash_queued and not flash_ops and e.rd(ux+O["ux_save_due"],1)),
                mode=e.rd(uc+O["uc_mode"],1),longest_busy=longest_busy, busy_ticks=busy_ticks,
                burst_ok=len(burst_delay) == burst1 - burst0, nonultra=nonultra, delays=[round(d) for d in delay], ends=[round(t / S_PER_US) for t in ends],
                cal_ok=not calibration or (4 in phases_seen and not e.rd(ca+CO['error'],1) and max(e.cal_peaks.values(),default=0)<=8000),
                cal_phases=sorted(phases_seen),cal_peaks=e.cal_peaks if calibration else {},
                cal_error=e.rd(ca+CO['error'],1) if calibration else 0)


def job(args):
    """one run in a process of its own: (label, elf, reset_us, phase_us, rel_us, npx, effect) -> (label, result)"""
    label, elf, *case = args
    return label, run_once(elf, offsets(elf), *case)


def main():
    elf = sys.argv[1]
    reset_us = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    nph = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    jobs = []
    for rs in (reset_us, reset_us - MARGIN_US):                         # 1. a light effect, the release at +150 us
        for ph in range(nph):
            jobs.append((f"light, release +150, reset {rs}, phase {ph * 50 // nph}", elf, rs, ph * 50 // nph, 150, 40, 4))
    for rel in (60, 100):                                              # valid earlier releases
        for ph in (0, 37):
            jobs.append((f"light, release +{rel}, reset {reset_us}, phase {ph}", elf, reset_us, ph, rel, 40, 4))
    for ph in range(0, 50, 7):                                          # 3. a heavy effect
        jobs.append((f"Lava Lamp 600 px, release +150, reset {reset_us}, phase {ph}", elf, reset_us, ph, 150, 600, 31))
    for ph in (0, 37):
        jobs.append((f"Lava Lamp 600 px, release +60, reset {reset_us}, phase {ph}", elf, reset_us, ph, 60, 600, 31))
    for chip, np in ((0, 600), (3, 500), (5, 400)):
        for fade in (0, 4, -4, 16):
            for ph in range(nph):
                phase = ph * 50 // nph
                jobs.append((f"Fire chip {chip}, {np} px, fade {fade}, reset {reset_us}, phase {phase}",
                             elf, reset_us, phase, 150, np, 16, chip, fade))
    for chip, np in ((0,600),(3,500),(5,400)):
        for ph in range(nph):
            phase=ph*50//nph
            jobs.append((f"Calibration + Fire chip {chip}, {np} px, reset {reset_us}, phase {phase}",
                         elf,reset_us,phase,150,np,16,chip,0,1))
    print(f"{len(jobs)} runs of the line (light/heavy effects, release +60 .. +150 us, phases against the 50 us tick); image {elf}")
    fails = 0
    workers = max(1, min(len(jobs), os.cpu_count() or 1, 6))                # (this PC: at most 6 jobs at a time)
    with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as pool:
        for label, r in pool.map(job, jobs):
            ok = (r["max_isr"] + 60 <= 7200 and r["frames"] == r["sent"] and r["burst_ok"] and r["nonultra"] == 0 and r['cal_ok'] and
                  all(v is not None and RW_FIRST-RW_EARLY0 <= v <= RW_FIRST+RW_LATE for v in r["answers"].values()))
            a = ", ".join(f"{k} {'NOT answered' if v is None else '%+.0f us' % (v - 1000.0)}" for k, v in r["answers"].items())
            print(f"  {label}: ISR {r['max_isr']}+60/7200 cycles; {r['frames']} of {r['sent']} frames classified, {r['nonultra']} no-Ultra (the slowest {r['worst']:.0f} us after its last edge); "
                  f"answers from their slots: {a}  {'ok' if ok else 'FAIL'}" +
                  (f" calibration phases {r['cal_phases']}, error {r['cal_error']}" if r['cal_phases'] else ''))
            fails += not ok
    print("budget: %s" % ("FAIL, %d runs" % fails if fails else "ok"))
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())

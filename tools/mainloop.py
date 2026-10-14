# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run main() of the compiled firmware in tools/rv32emu.py up to its idle WFI, then measure whole main-loop passes:
the pass with nothing to do (20,000 a second), the pass of a new ms (the per-ms work) and the pass of a new second.
The per-pc histogram of the ms passes is attributed to functions (symbols) and source lines (addr2line).

    python tools/mainloop.py <fw.elf> [--skip MS] [--passes N] [--lit | --effect] [--objdump-bin <dir>] [--top K]

(0.6.6, review round 5) The README's idle figure: --skip 11000 --passes 1000 on the standalone build (build/target;
fw.lst next to fw.elf); the poll tick's own cycles come from tools/measure.py.

The emulator's MMIO reads return 0 unless set here: no frames, no console input, no ADC samples (the idle board with
the line idle). The record pages read as erased flash (0xFF), so the boot finds defaults and no write is due.
The USART TC flag is set (nothing to send: uart_busy() = 0, as on an idle board).
"""
import os
import subprocess
import sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rv32emu import Emu, RET_MAGIC, COST, RAM_BASE, RAM_SIZE  # noqa: E402

F_CPU = 144e6


class Emu2(Emu):
    calls = None                            # Counter of (caller pc, callee entry) when set

    def run_to(self, pc, stop, max_steps=50_000_000, hist=None, x=None):
        """run from pc until the pc is in `stop` (a set) or RET_MAGIC; returns (pc, instructions). x: registers."""
        if x is None:
            x = self.x
        self.x = x
        cnt = {k: 0 for k in COST}
        n = 0
        calls = self.calls
        while pc != RET_MAGIC:
            if pc in stop and n > 0:
                break
            n += 1
            if n > max_steps:
                raise RuntimeError("step limit at %08x" % pc)
            h = self.rd(pc, 2)
            if h & 3 != 3:
                npc, cls = self._c(h, pc, x)
            else:
                npc, cls = self._i(self.rd(pc, 4), pc, x)
            cnt[cls] += 1
            if hist is not None:
                hist[pc] += COST[cls]
                if calls is not None and cls == "jump" and x[1] == pc + (2 if h & 3 != 3 else 4) and npc != x[1]:
                    calls[(pc, npc)] += 1
            x[0] = 0
            pc = npc
        self.last_counts = cnt
        self.ret = x[10]
        self.pc = pc
        return pc, n

    def start(self, fn, args=()):
        x = [0] * 32
        x[2] = RAM_BASE + RAM_SIZE - 16
        x[3] = self.gp
        x[1] = RET_MAGIC
        for i, a in enumerate(args):
            x[10 + i] = a & 0xFFFFFFFF
        self.x = x
        return self.syms[fn][0]


def func_map(emu):
    fs = []
    for nm, (v, sz) in emu.syms.items():
        if sz > 0 and (v < 0x40000 or 0x20000000 <= v < 0x20001000) and not nm.startswith("$"):
            fs.append((v, v + sz, nm))
    fs.sort()
    return fs


def func_of(fs, pc):
    lo, hi = 0, len(fs)
    while lo < hi:
        m = (lo + hi) // 2
        if fs[m][0] <= pc < fs[m][1]:
            return fs[m][2]
        if pc < fs[m][0]:
            hi = m
        else:
            lo = m + 1
    return "?%05x" % pc


def addr2line_plain(elf, pcs, tool):
    if not pcs or not tool:
        return {}
    out = subprocess.run([tool, "-e", elf, "-f"] + ["0x%x" % p for p in pcs], capture_output=True, text=True)
    lines = out.stdout.splitlines()
    res = {}
    for i, p in enumerate(pcs):
        fn = lines[2 * i] if 2 * i < len(lines) else "?"
        loc = lines[2 * i + 1] if 2 * i + 1 < len(lines) else "?"
        loc = loc.replace("\\", "/")
        loc = loc.split("/src/")[-1] if "/src/" in loc else loc
        loc = loc.split(" (")[0]
        res[p] = (fn, loc)
    return res


def report_hist(emu, elf, hist, title, top, a2l, total_c):
    fs = func_map(emu)
    byf = Counter()
    for pc, c in hist.items():
        byf[func_of(fs, pc)] += c
    print("\n%s: %d cycles; by function:" % (title, total_c))
    for nm, c in byf.most_common(top):
        print("  %7d  %5.1f %%  %s" % (c, 100.0 * c / max(total_c, 1), nm))
    pcs = [p for p, c in hist.most_common()]
    m = addr2line_plain(elf, pcs, a2l)
    if m:
        byl = Counter()
        for pc, c in hist.items():
            byl[m.get(pc, ("?", "?"))[1]] += c
        print("by source line (addr2line, the line the code was emitted for; inlined code lands on its own line):")
        for loc, c in byl.most_common(top):
            print("  %7d  %5.1f %%  %s" % (c, 100.0 * c / max(total_c, 1), loc))


def main():
    args = sys.argv[1:]
    elf = args[0]
    passes = 1200
    lit = False
    effect = False
    top = 40
    tooldir = ""
    skip = 0
    i = 1
    while i < len(args):
        if args[i] == "--passes":
            passes = int(args[i + 1]); i += 2
        elif args[i] == "--skip":
            skip = int(args[i + 1]); i += 2
        elif args[i] == "--lit":
            lit = True; i += 1
        elif args[i] == "--effect":
            lit = True; effect = True; i += 1
        elif args[i] == "--top":
            top = int(args[i + 1]); i += 2
        elif args[i] == "--objdump-bin":
            tooldir = args[i + 1]; i += 2
        else:
            raise SystemExit("unknown option " + args[i])
    if not tooldir:
        la = os.environ.get("LOCALAPPDATA", "")
        cands = sorted(d for d in os.listdir(os.path.join(la, "xPacks")) if d.startswith("xpack-riscv-none-elf-gcc-")) if la else []
        if cands:
            tooldir = os.path.join(la, "xPacks", cands[-1], "bin")
    a2l = os.path.join(tooldir, "riscv-none-elf-addr2line.exe") if tooldir else ""
    if a2l and not os.path.exists(a2l):
        a2l = a2l[:-4]
        if not os.path.exists(a2l):
            a2l = ""

    emu = Emu2(elf)
    # the record pages (upper CodeFlash) erased
    emu.poke(0x08010000, b"\xff" * 0xC000)
    # MMIO as an idle board sees it
    RCC = 0x40021000
    emu.mmio[RCC + 0x00] = 0x02000002          # CTLR: HSIRDY, PLLRDY
    emu.mmio[RCC + 0x04] = 0x08                # CFGR0: SWS = PLL
    emu.mmio[0x40004400] = 0x40                # USART2 STATR: TC (nothing on the line)
    emu.mmio[0x40010C08] = 1 << 10             # GPIOB INDR: TP9 (PB10) high: the test pad not grounded
    emu.mmio[0x40010808] = 1 << 3              # GPIOA INDR: TP6 (PA3, the console's RX) high: idle (0.6.7)
    # find the WFI in main
    text = open(os.path.splitext(elf)[0] + ".lst", encoding="utf-8", errors="replace").read().splitlines()
    wfi = None
    in_main = False
    for ln in text:
        if ln.endswith("<main>:"):
            in_main = True
        elif in_main and ln.startswith("0000") and ln.endswith(">:"):
            in_main = False
        if in_main and "\twfi" in ln:
            wfi = int(ln.split(":")[0].strip(), 16)
    assert wfi is not None, "no wfi in main"
    stop = {wfi}
    ms_now = emu.addr("ms_now")
    tick_now = emu.addr("tick_now")

    pc = emu.start("main")
    pc, n_boot = emu.run_to(pc, stop, max_steps=20_000_000)
    assert pc == wfi, "boot did not reach the WFI: pc %08x" % pc
    print("image: %s" % elf)
    print("boot to the first WFI: %d instructions, %d cycles (%.2f ms)" % (n_boot, emu.cycles(), emu.cycles() / F_CPU * 1e3))
    emu.writes.clear()

    import random
    rng = random.Random(7)
    sm_addr = emu.addr("sm")
    dirty = emu.addr("outputs_dirty")

    def tick(ms=1):
        t = emu.rd(tick_now, 4) + 20 * ms
        emu.wr(tick_now, 4, t)
        emu.wr(ms_now, 4, emu.rd(ms_now, 4) + ms)
        if effect:                                   # an effect: new output values every ms (WS path: sm.cur)
            import struct
            emu.poke(sm_addr, struct.pack("<5H", *[rng.randrange(1000, 65535) for _ in range(5)]))
            emu.wr(dirty, 1, 1)

    def one_pass(hist=None):
        p, n = emu.run_to(wfi, stop, max_steps=5_000_000, hist=hist)
        assert p == wfi, "pass left main: %08x" % p
        emu.writes.clear()
        return n, emu.cycles()

    if lit:
        # a steady WS2805-like output: sm.cur = levels, applied through outputs_update once
        sm = emu.addr("sm")
        import struct
        emu.poke(sm, struct.pack("<5H", 40000, 1000, 65535, 20000, 30000))
        emu.wr(emu.addr("outputs_dirty"), 1, 1)
        one_pass()
        tick()
        one_pass()

    # warm-up (no histogram): past the soft start, the LIFE_PREP_S erase-ahead hold-off, ...
    for k in range(skip):
        tick()
        one_pass()
    if skip:
        print("skipped %d ms (uptime now %d ms)" % (skip, emu.rd(ms_now, 4)))
    emu.calls = Counter()

    # the pass with nothing to do (a wake without a new ms: another interrupt, or the same ms)
    idle = []
    hist_idle = Counter()
    for k in range(50):
        idle.append(one_pass(hist_idle))
    n_i, c_i = min(idle, key=lambda t: t[1])
    print("idle pass (no new ms), min / max: %d / %d instructions, %d / %d cycles" %
          (n_i, max(t[0] for t in idle), c_i, max(t[1] for t in idle)))

    # ms passes
    res = []
    hist_ms = Counter()
    hist_sec = Counter()
    for k in range(passes):
        tick()
        h = Counter()
        n, c = one_pass(h)
        res.append((n, c, emu.rd(ms_now, 4)))
        hist_ms.update(h)
    cs = sorted(t[1] for t in res)
    med = cs[len(cs) // 2]
    print("ms passes (%d): median %d cycles, min %d, max %d; mean %.1f" % (passes, med, cs[0], cs[-1], sum(cs) / len(cs)))
    # the distinct levels (every 16 ms the vsense window / isense regression window, every second the second's work)
    lv = Counter(cs)
    print("  cycle levels (count): " + ", ".join("%d (%d)" % (c, k) for c, k in sorted(lv.items(), reverse=True)[:12]))
    # the second's pass: the max
    imax = max(range(len(res)), key=lambda k: res[k][1])
    print("  the heaviest pass: %d instructions, %d cycles at ms %d" % (res[imax][0], res[imax][1], res[imax][2]))
    total = sum(cs)
    report_hist(emu, elf, hist_ms, "all ms passes together", top, a2l, total)
    # callers of the small helpers (who pays for memcpy, the divisions, ...): per ms pass
    fs = func_map(emu)
    byc = defaultdict(Counter)
    for (frm, to), k in emu.calls.items():
        byc[func_of(fs, to)][func_of(fs, frm)] += k
    print("\ncalls per ms pass (callee <- caller: calls / 1000 passes):")
    for callee in sorted(byc, key=lambda c: -sum(byc[c].values())):
        tot = sum(byc[callee].values())
        if tot * 1000 < passes:                                   # less than one call per 1000 passes: skip
            continue
        print("  %-28s %7.1f  <- %s" % (callee, tot * 1000.0 / passes,
              ", ".join("%s %.1f" % (c, k * 1000.0 / passes) for c, k in byc[callee].most_common(4))))
    # cost per second of an idle board, main loop only: 1000 ms passes + the idle passes between (20,000 - 1,000 wakes)
    print("\nmain loop per second (idle board, this image): ms passes %d cycles + %d idle passes x %d = %d cycles = %.2f %% CPU"
          % (sum(cs[:1000]) if len(cs) >= 1000 else total, 19000, c_i, (sum(cs[:1000]) if len(cs) >= 1000 else total) + 19000 * c_i,
             ((sum(cs[:1000]) if len(cs) >= 1000 else total) + 19000 * c_i) / F_CPU * 100))
    report_hist(emu, elf, hist_idle, "idle passes together (50)", 25, a2l, sum(t[1] for t in idle))


if __name__ == "__main__":
    main()

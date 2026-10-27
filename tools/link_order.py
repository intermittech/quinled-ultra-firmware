#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""link_order.py <norelax.map> <norelax.lst> <input.map> <input.lst> [iterations]: the code's order for the final link
(tools/build_target.sh).

RISC-V calls and tail jumps are 8 bytes (auipc + jalr) until the linker relaxes them: to 2 bytes (c.jal / c.j) when the
target is within -2048..+2046, else to 4 (jal / j). The input order leaves most calls far from their targets; this
orders the code's input sections so that more of them are near. The code itself is the same: only the placement
changes (proof: the disassembly normalized by symbol).

How GNU ld 2.45 decides (found from its links, every call of the input order reproduced): it relaxes the sections in
their placement order, giving each its new address as it goes. A call is measured from its address in the layout
before relaxation (every call 8 bytes, nothing relaxed) to its target's new address if the target's section came
earlier, else to the target's address before relaxation; +-4 bytes margin (the section alignment); within reach: 2
bytes, else 4, never looked at again.

The inputs: a link without relaxation (-Wl,--no-relax: each section's size and each call's place before relaxation)
and the input-order link (each section's other code relaxed, each target's place in it). jalr t0 (the -msave-restore
calls) can't become 2 bytes and stays in the sections' other code. An order is scored by the code's size; chains of
the most-connected sections (Pettis-Hansen) start a seeded local search (moves, swaps, block moves). The best order is
printed as linker-script input-section lines for src/mcu/ch32v20x/Link_sections.ld's INCLUDE text_order.ld. Deterministic
for given inputs: the search's moves come from its own xorshift (Rng), not Python's random, so the order (and the
image) doesn't depend on the Python version (the switch moved each image by a few bytes, once).
"""
import bisect
import re
import sys

LO, HI, PAD = -2048, 2046, 4


class Rng:
    """xorshift32 (Marsaglia 2003): the search's moves from this file alone, not from Python's random (whose
    algorithms a Python version may change): the same inputs, the same order, whatever Python runs the build"""

    def __init__(self, seed):
        self.s = seed & 0xFFFFFFFF or 1

    def next(self):
        x = self.s
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self.s = x
        return x

    def below(self, n):
        return self.next() % n                          # (the modulo's bias: no matter for a search)

    def unit(self):
        return self.next() / 4294967296.0
RX1 = re.compile(r'^ (\.\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(.+)$')
RX0 = re.compile(r'^ (\.\S+)$')
RX2 = re.compile(r'^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(.+)$')
RX_IN = re.compile(r'^\s+([0-9a-f]+):\s+([0-9a-f]+)\s+(\S+)\s*(.*)$')


def ident(name, f):
    """a section, by name and file (the temporary LTO output as one name)"""
    base = re.split(r'[/\\]', f)[-1]
    return name, ('LTO' if re.search(r'\.lto\.o$|ltrans', base) else base)


def read_units(path):
    """the code input sections of the output section .text, in address order: (name, file, address, size)"""
    units, pend = [], None
    lines = open(path, encoding='utf-8', errors='replace').read().split('\n')
    i = 0
    while i < len(lines) and not lines[i].startswith('.text '):
        i += 1
    for l in lines[i + 1:]:
        if l and not l.startswith(' '):
            break
        m = RX1.match(l)
        if m:
            name, a, s, f = m.group(1), int(m.group(2), 16), int(m.group(3), 16), m.group(4).strip()
        else:
            m = RX0.match(l)
            if m:
                pend = m.group(1)
                continue
            m = RX2.match(l)
            if not (m and pend):
                continue
            name, a, s, f = pend, int(m.group(1), 16), int(m.group(2), 16), m.group(3).strip()
            pend = None
        if name.startswith(('.rodata', '.srodata')):
            break
        if s and name.startswith('.text'):
            units.append((name, f, a, s))
    units.sort(key=lambda u: u[2])
    return units


def locate(units):
    starts = [u[2] for u in units]

    def unit_of(a):
        k = bisect.bisect_right(starts, a) - 1
        return k if k >= 0 and a < units[k][2] + units[k][3] else None
    return unit_of


def calls_norelax(path, units):
    """each auipc + jalr ra / jr pair to another section: (caller, offset, callee, target offset, relaxable); a jal /
    j written as such (the asm's: never relaxed) is listed too, not relaxable"""
    unit_of = locate(units)
    out, prev = [], None
    for l in open(path, encoding='utf-8', errors='replace'):
        m = RX_IN.match(l)
        if not m:
            continue
        a, mn, ops = int(m.group(1), 16), m.group(3), m.group(4)
        if mn == 'auipc':
            prev = a
            continue
        if mn in ('jal', 'j') and re.match(r'^[0-9a-f]+ <', ops):
            ta = int(ops.split()[0], 16)
            ua, ut = unit_of(a), unit_of(ta)
            if ua is not None and ut is not None and ua != ut:
                out.append((ua, a - units[ua][2], ut, ta - units[ut][2], False))
        if prev == a - 4 and mn in ('jalr', 'jr'):
            arg = ops.split('#')[0].strip()             # jalr 690(ra): rd ra; jalr t0,-12(t0): rd t0
            rd = 'zero' if mn == 'jr' else (arg.split(',')[0] if ',' in arg else 'ra')
            t = re.search(r'#\s*([0-9a-f]+) <', ops)
            if t and rd in ('ra', 'zero'):
                ta = int(t.group(1), 16)
                ua, ut = unit_of(prev), unit_of(ta)
                if ua is not None and ut is not None and ua != ut:
                    out.append((ua, prev - units[ua][2], ut, ta - units[ut][2], True))
        prev = None
    return out


def calls_relaxed(path, units):
    """each jal / j (2 or 4 bytes; not jal t0) to another section: (caller, offset, callee, target offset, bytes)"""
    unit_of = locate(units)
    out = []
    for l in open(path, encoding='utf-8', errors='replace'):
        m = RX_IN.match(l)
        if not m or m.group(3) not in ('jal', 'j', 'c.jal', 'c.j'):
            continue
        ops = m.group(4)
        if m.group(3) == 'jal' and not re.match(r'^[0-9a-f]+ <', ops):
            continue
        t = re.search(r'([0-9a-f]+) <', ops)
        if not t:
            continue
        a, ta = int(m.group(1), 16), int(t.group(1), 16)
        ua, ut = unit_of(a), unit_of(ta)
        if ua is not None and ut is not None and ua != ut:
            out.append((ua, a - units[ua][2], ut, ta - units[ut][2], len(m.group(2)) // 2))
    return out


class Model:
    """the sections (in the no-relax map's order), their sizes before relaxation and their other code relaxed, and
    the calls with their places in both layouts"""

    def __init__(self, nmap, nlst, rmap, rlst):
        self.units = read_units(nmap)
        ru = read_units(rmap)
        rid = {ident(u[0], u[1]): i for i, u in enumerate(ru)}
        self.ri = [rid[ident(u[0], u[1])] for u in self.units]    # (KeyError: the two links differ)
        n = len(self.units)
        cn = calls_norelax(nlst, self.units)
        cr = calls_relaxed(rlst, ru)
        by_n, by_r = {}, {}
        for c in cn:
            by_n.setdefault(c[0], []).append(c)
        for c in cr:
            by_r.setdefault(c[0], []).append(c)
        self.calls = [[] for _ in range(n)]           # per section: (offset before relaxation, callee, its targets'
        self.other = [0] * n                          # offsets before / after relaxation); the other code relaxed
        for u in range(n):
            a = sorted(by_n.get(u, []), key=lambda c: c[1])
            b = sorted(by_r.get(self.ri[u], []), key=lambda c: c[1])
            if len(a) != len(b):
                raise SystemExit(f'link_order.py: {self.units[u][0]}: {len(a)} calls before relaxation, {len(b)} after')
            self.calls[u] = [(x[1], x[2], x[3], y[3]) for x, y in zip(a, b) if x[4]]
            self.other[u] = ru[self.ri[u]][3] - sum(y[4] for x, y in zip(a, b) if x[4])
        self.pre = [u[3] for u in self.units]
        self.real = sum(1 for c in cr if c[4] == 2)

    def size(self, order, count=False, far=None):
        """the code's size (count: the 2-byte calls; far: a list to get the (caller, callee) of each 4-byte one)"""
        n = len(self.units)
        posi = [0] * n
        p0 = [0] * n
        a = 0
        for i, u in enumerate(order):
            posi[u] = i
            p0[u] = a
            a += self.pre[u]
        p1 = [0] * n
        cur = short = 0
        for u in order:
            p1[u] = cur
            sz = self.other[u]
            pu = posi[u]
            for so, g, to_pre, to_rel in self.calls[u]:
                t = p1[g] + to_rel if posi[g] < pu else p0[g] + to_pre
                d = t - (p0[u] + so)
                d += PAD if d >= 0 else -PAD
                if LO <= d <= HI:
                    sz += 2
                    short += 1
                else:
                    sz += 4
                    if far is not None:
                        far.append((u, g))
            cur += sz
        return short if count else cur


def main():
    m = Model(*sys.argv[1:5])
    iters = int(sys.argv[5]) if len(sys.argv) > 5 else 20000
    n = len(m.units)
    cur = list(range(n))
    w = {}
    for u in range(n):
        for c in m.calls[u]:
            k = (min(u, c[1]), max(u, c[1]))
            w[k] = w.get(k, 0) + 1
    chains = {u: [u] for u in cur}
    head = {u: u for u in cur}
    for (a, b), _ in sorted(w.items(), key=lambda kv: (-kv[1], kv[0])):
        ha, hb = head[a], head[b]
        if ha == hb:
            continue
        ca, cb = chains[ha], chains[hb]
        if ca[0] == a and len(ca) > 1:
            ca.reverse()
        if cb[-1] == b and len(cb) > 1:
            cb.reverse()
        nc = ca + cb
        del chains[hb]
        chains[ha] = nc
        for x in nc:
            head[x] = ha
    s0 = m.size(cur)
    best = [x for c in chains.values() for x in c]
    bsize = m.size(best)
    if s0 < bsize:
        best, bsize = cur, s0
    rnd = Rng(7)
    far = []
    m.size(best, far=far)
    for _ in range(iters):
        o = best[:]
        r = rnd.unit()
        if r < 0.5 and far:                              # a call still 4 bytes: its callee next to its caller
            u, g = far[rnd.below(len(far))]
            if rnd.unit() < 0.5:
                u, g = g, u
            o.remove(g)
            j = o.index(u) + (1 if rnd.unit() < 0.5 else 0)
            o.insert(j, g)
        else:
            i, j = rnd.below(n), rnd.below(n)
            if r < 0.7:
                o[i], o[j] = o[j], o[i]
            elif r < 0.9:
                o.insert(j, o.pop(i))
            else:
                ln = 2 + rnd.below(7)
                blk = o[i:i + ln]
                del o[i:i + ln]
                j = rnd.below(len(o) + 1)
                o[j:j] = blk
        fr = []
        sz = m.size(o, far=fr)
        if sz <= bsize:
            best, bsize, far = o, sz, fr
    print(f'/* tools/link_order.py: {n} code sections, {sum(len(c) for c in m.calls)} calls / tail jumps to another '
          f'section that can be 2 bytes: {m.real} in the input order (the model: {m.size(cur, True)}), '
          f'{m.size(best, True)} in this one; the code {s0} -> {bsize} B */')
    for u in best:
        name, f = m.units[u][0], m.units[u][1]
        mem = re.search(r'\(([^)]+)\)$', f)
        if mem:
            print('*:' + mem.group(1) + '(' + name + ')')            # an archive member (libgcc, libc)
        elif ident(name, f)[1] == 'LTO':
            print('*(' + name + ')')                                 # the LTO output (a temporary file name)
        else:
            print('*' + ident(name, f)[1] + '(' + name + ')')        # an object of its own (startup, isr, div64)


if __name__ == '__main__':
    main()

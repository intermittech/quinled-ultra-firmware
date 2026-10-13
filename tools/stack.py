# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Static worst-case stack depth from an ELF's disassembly: python tools/stack.py <objdump> <elf>
(objdump: the toolchain's riscv-none-elf-objdump; its nm next to it bounds each function by its symbol size).
Prints the deepest path from main and from each ISR; with the ISRs nested the worst case is their sum (0.6.2:
main 768 + poll 80 + slew 32 = 880 B of the 2 KB stack; `status` shows the painted high-water mark on the board).
Frame = every sp decrement in the function (addi sp,sp,-N) plus the __riscv_save_N frame it calls (jal t0).
Calls: jal / j / tail to symbols (a tail jump counts as a call at the caller's depth minus its own frame: treated
conservatively as a call). Indirect calls (jalr ra): the console's function pointers write / apply / save_now."""
import re, subprocess, sys
objdump, elf = sys.argv[1], sys.argv[2]
dis = subprocess.run([objdump, "-d", "--no-show-raw-insn", elf], capture_output=True, text=True).stdout
# (0.6.2) each function only up to its symbol size: past the last function in .text objdump decodes .rodata
nm = objdump.replace("objdump", "nm")
ends = {}
for l in subprocess.run([nm, "-S", elf], capture_output=True, text=True).stdout.splitlines():
    f = l.split()
    if len(f) == 4 and f[2] in "tTwW":
        ends[f[3]] = int(f[0], 16) + int(f[1], 16)
frame, calls, ind = {}, {}, {}
fn = None
for l in dis.splitlines():
    m = re.match(r"^[0-9a-f]+ <(.*)>:$", l)
    if m:
        fn = m.group(1); frame[fn] = 0; calls[fn] = set(); ind[fn] = 0
        continue
    if fn is None or not l.strip():
        continue
    ma = re.match(r"^\s*([0-9a-f]+):", l)
    if ma and fn in ends and int(ma.group(1), 16) >= ends[fn]:
        continue
    m = re.search(r"\taddi\s+sp,sp,-(\d+)", l)
    if m:
        frame[fn] += int(m.group(1))
    m = re.search(r"\tjal\s+t0,[0-9a-f]+ <(__riscv_save_\d+)>", l)
    if m:
        calls[fn].add(m.group(1))
        continue
    m = re.search(r"\t(jal|j|tail|call)\s+(ra,)?[0-9a-f]+ <([^+>]+)>", l)
    if m and m.group(3) != fn and not m.group(3).startswith("__riscv_restore"):
        calls[fn].add(m.group(3))
    if re.search(r"\tjalr\s+(ra,)?[a-z0-9]+(\)|$|,)", l) or re.search(r"\tjalr\s", l):
        ind[fn] += 1
# c->write is uart_write everywhere; c->apply / c->save_now only from the command dispatcher (cli_exec, inlined into
# cli_feed by LTO). Override: STACK_ALL_IND=1 gives every indirect call all three.
import os
allt = [f for f in ("uart_write", "cli_apply", "save_now") if f in frame]
for f in frame:
    if ind[f] and f not in ("_start", "memset", "memcpy"):
        disp = f.split(".")[0] in ("cli_exec", "cli_feed") or os.environ.get("STACK_ALL_IND")
        calls[f] |= set(allt if disp else [t for t in allt if t == "uart_write"])
memo = {}
def depth(f, stack=()):
    if f in stack:
        return 0, [f + " (recursion)"]
    if f in memo:
        return memo[f]
    best, path = 0, []
    for c in calls.get(f, ()):
        d, p = depth(c, stack + (f,))
        if d > best:
            best, path = d, p
    memo[f] = (frame.get(f, 0) + best, [f] + path)
    return memo[f]
for root in ("main", "poll_isr", "slew2_isr", "slew3_isr", "gate_isr"):
    if root in frame:
        d, p = depth(root)
        print("%-10s %5d B  %s" % (root, d, " > ".join("%s(%d)" % (x, frame.get(x.split(" ")[0], 0)) for x in p)))
print("indirect-call sites:", sum(1 for f in ind if ind[f]), [f for f in ind if ind[f]][:12])

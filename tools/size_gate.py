# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The size and cycle gate: every development image's free flash and its RAM against the budget table
below, tools/measure.py's rows against a baseline, and the frozen record corpus (tools/record_corpus.py).

    python tools/size_gate.py [--out <dir>] [--no-build] [--measure <baseline>] [--write-measure <baseline>] [--corpus]
    python tools/size_gate.py --mcu ch32v40x [--out <dir>] [--no-build]   (the V407 Digital-M, both compilers)

--out: the builds' folder (default build/size_gate); --no-build: use the builds already there. The builds
(tools/build_target.sh; XW needs WCH_GCC_DIR, as there): the six application images (the Dig2Analog-Plus-5ch app, the
Diff-Power-UR-Analog-M, BOARD=2, and the Diff-Power-UR-Digital-M, BOARD=3, behind the bootloader: UPD_APP=1,
src/mcu/ch32v20x/Link_app.ld; each XW and XW=0). Both receivers require ANSWER_UART=1; with --measure / --write-measure the standalone image and its measurement
twin (MEASURE=1; the app's twin doesn't fit the slot); with --corpus the measurement twins of the main board and the
receiver.
--measure <baseline>: tools/measure.py on the standalone image and its twin, each row's cycles against the baseline
(a row slower by more than CYCLE_TOL fails; rows new or gone are listed, never a failure). --write-measure <file>:
write that baseline (after a change that is meant to be slower, or new rows). --corpus: test/records/* into both
twins. The size gate enforces the required flash slots and SRAM floors; the 512 + 300 byte free-flash target is
reported separately.
Exit 1 when an image is over budget, a row is too slow, or the corpus check fails; 2 when a build fails."""
import os
import re
import struct
import subprocess
import sys

FW = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SLOT = 57088                        # app-slot bytes, including flash loads of SRAM sections
STACK = 2048
RESERVED = 112                     # boot-info/breadcrumb holes in the SRAM layout
RAM = 20480 - STACK - RESERVED
# Receiver wire-3 images require WCH XW. xPack receiver builds are measurements,
# including linker overflow; BOARD 1 retains both compilers as hard gates.
def measure_only(defs, xw):
    return xw == 0 and any(d in defs for d in ("BOARD=2", "BOARD=3"))

def map_sections(path):
    with open(path) as f:
        result = {n: int(s, 16) for n, s in re.findall(
            r"^(\.\w+)\s+0x[0-9a-f]+\s+0x([0-9a-f]+)", f.read(), re.M)}
    if not {'.init', '.text', '.ramtext', '.data', '.bss'} <= result.keys() or not result['.text']:
        raise ValueError('incomplete linker map: ' + str(path))
    return result
# Hard limits only. The references are the measured development checkpoint.
# RAM includes .ramtext, .data, .bss and .noinit; the stack and holes are separate.
APP = ["UPD_APP=1"]
LTO_CHURN = 300
FINAL_FREE = 512
BUDGET = [
    ("app", APP, 1, LTO_CHURN + FINAL_FREE, 18320, (12344, 15436)),
    ("app_xw0", APP, 0, LTO_CHURN + FINAL_FREE, 18320, (10832, 15456)),
    ("ur_analog_m", APP + ["BOARD=2"], 1, LTO_CHURN + FINAL_FREE, 17296, (7200, 15468)),
    ("ur_analog_m_xw0", APP + ["BOARD=2"], 0, LTO_CHURN + FINAL_FREE, 17296, (5552, 15488)),
    ("ur_digital_m", APP + ["BOARD=3"], 1, LTO_CHURN + FINAL_FREE, 16784, (2000, 15664)),
    ("ur_digital_m_xw0", APP + ["BOARD=3"], 0, LTO_CHURN + FINAL_FREE, 16784, (276, 15692)),
]
CYCLE_TOL = (0.02, 8)               # a row may be 2 % or 8 cycles slower (the larger) before it fails
ROW = re.compile(r"^(.*?)\s+(\d+)\s+(\d+)\s+([\d.]+) us(.*)$")


def sections(elf):
    """{name: size} of an ELF32's sections"""
    with open(elf, "rb") as f:
        d = f.read()
    shoff, = struct.unpack_from("<I", d, 32)
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", d, 46)
    secs = [struct.unpack_from("<IIIIIIIIII", d, shoff + i * shentsize) for i in range(shnum)]
    strtab = secs[shstrndx][4]
    names = {}
    for s in secs:
        n = d[strtab + s[0]:d.index(b"\0", strtab + s[0])].decode()
        names[n] = s[5]
    return names


def build(out, name, defs, xw, ldscript=None, measure=False, mcu=None):
    o = os.path.join(out, name)
    # A failed rebuild must never be reported using a previous successful ELF/map.
    for artifact in ('fw.elf', 'fw.map'):
        path = os.path.join(o, artifact)
        if os.path.exists(path): os.remove(path)
    env = dict(os.environ, OUT=o, XW=str(xw), BOARD="1",
               EXTRA_CFLAGS=" ".join("-D" + x for x in defs if not x.startswith("BOARD=")))
    for x in defs:
        if x.startswith("BOARD="):
            env["BOARD"] = x[len("BOARD="):]
    if ldscript:
        env["LDSCRIPT"] = os.path.join(FW, ldscript)
    if measure:
        env["MEASURE"] = "1"
    if mcu:
        env["MCU"] = mcu
    with open(o + ".log", "w") as log:
        r = subprocess.run(["bash", os.path.join(FW, "tools", "build_target.sh")], env=env, stdout=log,
                           stderr=subprocess.STDOUT, cwd=FW)
    if r.returncode != 0:
        with open(o + ".log") as log:
            failure = log.read()
        if (measure_only(defs, xw) and "overflowed by" in failure and
                "region `FLASH'" in failure and "undefined reference" not in failure and
                os.path.exists(os.path.join(o, "fw.map"))):
            return os.path.join(o, "fw.map")
        print("%s: the build FAILED (%s.log)" % (name, o))
        sys.exit(2)
    return os.path.join(o, "fw.elf")


def memory_usage(s):
    flash = sum(s.get(n, 0) for n in (".init", ".text", ".ramtext", ".data"))
    ram = sum(s.get(n, 0) for n in (".ramtext", ".data", ".bss", ".noinit"))
    return flash, ram


def gate_sizes(out, do_build):
    fails = 0
    print("image         free flash (delta)      SRAM used (delta)      free SRAM (floor); hard slot/SRAM gate, free-flash target reported")
    for name, defs, xw, floor, ram_max, (free0, ram0) in BUDGET:
        elf = build(out, name, defs, xw, "src/mcu/ch32v20x/Link_app.ld") if do_build else os.path.join(out, name, "fw.elf")
        if (not do_build and not os.path.exists(elf) and measure_only(defs, xw)
                and os.path.exists(os.path.join(out, name, "fw.map"))):
            elf = os.path.join(out, name, "fw.map")
        flash, ram = memory_usage(map_sections(elf) if elf.endswith(".map") else sections(elf))
        free = SLOT - flash
        measured = measure_only(defs, xw)
        bad = (free < 0 and not measured) or ram > ram_max
        fails += bad
        print("%-13s %6d B (%+5d)       %6d B (%+5d)       %5d B (>= %4d)%s" % (
            name, free, free - free0, ram, ram - ram0, RAM - ram, RAM - ram_max,
            ("  OVER BUDGET" if bad else "") + ("  xPack receiver: flash MEASURE ONLY" if measured else "") +
            ("  TARGET SHORT %d B" % (floor - free) if free < floor else "  target +%d B" % (free - floor))))
    return fails


# CH32V40x (--mcu ch32v40x): the Digital-M behind its 16 KiB bootloader (layout 4). Slot 0x08005000-0x0807FFFF; app
# SRAM the 200 KB of RAM_CODE_MOD = 0 (src/mcu/ch32v40x/link_mem_app.ld), the same 2,048 B stack and 112 B
# reservations, the Digital-M's 1,536 B SRAM floor; both compilers are hard gates here (no flash pressure).
V407_SLOT = 0x80000 - 0x5000
V407_RAM = 0x32000 - STACK - RESERVED
V407_BUDGET = [
    ("v407_ur_digital_m", APP + ["BOARD=3"], 0, 1536),
    ("v407_ur_digital_m_xw1", APP + ["BOARD=3"], 1, 1536),
]


def gate_sizes_v407(out, do_build):
    fails = 0
    print("image                  flash      free flash     SRAM used    free SRAM (floor)")
    for name, defs, xw, sram_floor in V407_BUDGET:
        elf = (build(out, name, defs, xw, "src/mcu/ch32v40x/Link_app.ld", mcu="ch32v40x") if do_build
               else os.path.join(out, name, "fw.elf"))
        flash, ram = memory_usage(sections(elf))
        free, sfree = V407_SLOT - flash, V407_RAM - ram
        bad = free < 0 or sfree < sram_floor
        fails += bad
        print("%-22s %7d B   %7d B     %7d B    %7d B (>= %d)%s" % (name, flash, free, ram, sfree, sram_floor,
                                                                    "  OVER BUDGET" if bad else ""))
    # the whole tick ISR's modeled cycles against the 50 us tick (test/test_poll_budget_v407.py), both images
    elfs = [os.path.join(out, name, "fw.elf") for name, _, _, _ in V407_BUDGET]
    r = subprocess.run([sys.executable, os.path.join(FW, "test", "test_poll_budget_v407.py"),
                        os.path.join(out, "poll-budget")] + elfs, capture_output=True, text=True, timeout=3600)
    worst = re.search(r'"worst": (\d+)', r.stdout)
    print("tick ISR (modeled, + nested answer bytes): %s of %d cycles%s" % (
        worst.group(1) if worst else "?", 200000000 // 20000 - 60, "" if r.returncode == 0 else "  OVER BUDGET"))
    fails += r.returncode != 0
    return fails


def rows(text):
    out = {}
    for line in text.splitlines():
        m = ROW.match(line)
        if m:
            out[m.group(1).strip()] = int(m.group(3))
    return out


def run_measure(out, do_build):
    std = build(out, "standalone", [], 1) if do_build else os.path.join(out, "standalone", "fw.elf")
    mes = build(out, "measure", [], 1, measure=True) if do_build else os.path.join(out, "measure", "fw.elf")
    env = dict(os.environ, FW_ELF=std, FW_ELF_MEASURE=mes)
    r = subprocess.run([sys.executable, os.path.join(FW, "tools", "measure.py")], env=env, capture_output=True,
                       text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        print("measure.py FAILED")
        sys.exit(2)
    return r.stdout


def gate_measure(text, baseline):
    with open(baseline) as f:
        old = rows(f.read())
    new = rows(text)
    fails = 0
    for k in old:
        if k not in new:
            print("  gone: %s" % k)
    for k, c in new.items():
        if k not in old:
            print("  new: %s, %d cycles" % (k, c))
            continue
        lim = old[k] + max(int(old[k] * CYCLE_TOL[0]), CYCLE_TOL[1])
        if c > lim:
            fails += 1
            print("  SLOWER: %s: %d cycles (%d before, the limit %d)" % (k, c, old[k], lim))
        elif c != old[k]:
            print("  %s: %d cycles (%d before)" % (k, c, old[k]))
    print("measure.py: %d rows, %d slower than allowed" % (len(new), fails))
    return fails


def gate_corpus(out, do_build):
    main = os.path.join(out, "measure", "fw.elf")
    if do_build and not os.path.exists(main):
        main = build(out, "measure", [], 1, measure=True)
    rx = build(out, "ur_analog_m_measure", ["BOARD=2"], 1, measure=True) if do_build else os.path.join(out, "ur_analog_m_measure",
                                                                                                "fw.elf")
    rec = os.path.join(FW, "test", "records")
    dirs = sorted(os.path.join(rec, d) for d in os.listdir(rec) if os.path.isdir(os.path.join(rec, d)))
    fails = 0
    for elf in (main, rx):
        r = subprocess.run([sys.executable, os.path.join(FW, "tools", "record_corpus.py"), "check", elf] + dirs,
                           cwd=os.path.join(FW, "tools"), capture_output=True, text=True)
        print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
        fails += r.returncode != 0
    return fails


def main(argv):
    out = os.path.join(FW, "build", "size_gate")
    do_build, baseline, write_to, corpus, mcu = True, None, None, False, "ch32v20x"
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--out":
            out, i = os.path.abspath(argv[i + 1]), i + 1
        elif a == "--no-build":
            do_build = False
        elif a == "--measure":
            baseline, i = argv[i + 1], i + 1
        elif a == "--write-measure":
            write_to, i = argv[i + 1], i + 1
        elif a == "--corpus":
            corpus = True
        elif a == "--mcu":
            mcu, i = argv[i + 1], i + 1
        else:
            raise SystemExit(__doc__)
        i += 1
    os.makedirs(out, exist_ok=True)
    if mcu == "ch32v40x":
        fails = gate_sizes_v407(out, do_build)
        print("size gate (ch32v40x): %s" % ("passed" if not fails else "FAILED (%d)" % fails))
        return 1 if fails else 0
    fails = gate_sizes(out, do_build)
    if baseline or write_to:
        text = run_measure(out, do_build)
        if write_to:
            with open(write_to, "w", newline="\n") as f:
                f.write("".join(line + "\n" for line in text.splitlines() if ROW.match(line) or
                                line.startswith("function / path")))
            print("measure.py baseline written: %s (%d rows)" % (write_to, len(rows(text))))
        if baseline:
            fails += gate_measure(text, baseline)
    if corpus:
        fails += gate_corpus(out, do_build)
    print("size gate: %s" % ("passed" if not fails else "FAILED (%d)" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

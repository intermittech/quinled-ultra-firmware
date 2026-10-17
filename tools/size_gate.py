# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The size and cycle gate (round 12, S10): every release image's free flash and its RAM against the budget table
below, tools/measure.py's rows against a baseline, and the frozen record corpus (tools/record_corpus.py).

    python tools/size_gate.py [--out <dir>] [--no-build] [--measure <baseline>] [--write-measure <baseline>] [--corpus]

--out: the builds' folder (default build/size_gate); --no-build: use the builds already there. The builds
(tools/build_target.sh; XW needs WCH_GCC_DIR, as there): the six release images (the Dig2Analog-Ultra app, the
Ultra-Analog receiver, BOARD=2, and the Ultra-Digital receiver, BOARD=3, behind the bootloader: UPD_APP=1,
vendor/wch/Link_app.ld; each XW and XW=0) and the receivers' answer UART settings (board.h ANSWER_UART: BOARD=2 and 3 with 2,
BOARD=3 with 1; each XW and XW=0); with --measure / --write-measure the standalone image and its measurement
twin (MEASURE=1; the app's twin doesn't fit the slot); with --corpus the measurement twins of the main board and the
receiver.
--measure <baseline>: tools/measure.py on the standalone image and its twin, each row's cycles against the baseline
(a row slower by more than CYCLE_TOL fails; rows new or gone are listed, never a failure). --write-measure <file>:
write that baseline (after a change that is meant to be slower, or new rows). --corpus: test/records/* into both
twins. Exit 1 when an image is over budget, a row is too slow, or the corpus check fails; 2 when a build fails."""
import os
import re
import struct
import subprocess
import sys

FW = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SLOT = 57088                        # the app slot (vendor/wch/Link_app.ld): .init + .text + .ramtext + .data
RAM = 20480 - 2048                  # SRAM less the stack (__stack_size, vendor/wch/Link_sections.ld)

# name, build variables, the least free slot bytes, the most RAM (.data + .bss + .noinit), the 0.6.12 values. The
# floors: room for the next features, and the XW=0 fallback's goal of >= 1 KB (the generic compiler must keep fitting
# every feature); RAM: the round-12 use + 64 B. The Ultra-Digital receiver's rows (the local pixel effects): Quindor's budget
# decision 2026-10-17, "use the flash and RAM, but keep a few KB of flash free and 1 to 2 KB of RAM": the floors are 4 KB of
# the slot (4,096 B) on the XW build and, since the squeeze round (2026-10-17, Quindor), 2 KB (2,048 B) on the XW=0 fallback, and the SRAM total (UD_SRAM below) leaves 1.5 KB free beyond the stack. Measured with
# the real effect engine (36 effects, 31 palettes, the packer): the engine ~19.2 KB of flash, the controller, encoder, glue
# and page ~7 KB: 25.7 KB of the 36 KB the receiver had; the chain mode (docs/ultradigital-firmware.md section 12) +4.1 KB.
# RAM (.data + .bss): the frame buffer 2,000 B (600 RGB / 500 RGBW / 400 RGBCCT pixels),
# the engine's state 1,024 B, the controller 2.1 KB, the ring's guard 128: 10,696 B (limit 12,096 B). Lower a floor or
# raise a RAM limit only as a decision, in this table.
APP = ["UPD_APP=1"]
BUDGET = [
    ("app", APP, 1, 3072, 17068, (3592, 17004)),
    ("app_xw0", APP, 0, 1024, 17068, (1932, 17004)),
    ("rx", APP + ["BOARD=2"], 1, 2560, 17052, (2956, 16988)),
    ("rx_xw0", APP + ["BOARD=2"], 0, 1024, 17052, (1248, 16988)),
    ("ud", APP + ["BOARD=3"], 1, 4096, 12096, (4108, 10696)),
    ("ud_xw0", APP + ["BOARD=3"], 0, 2048, 12096, (2600, 10696)),
    # the answer UART's other settings (board.h ANSWER_UART; the same floors as their boards, so the pivot can't rot out of
    # the budget): USART2 / PA2 on both receivers (the console's TX off, its text in a RAM ring: the buffers it replaces), USART1 / PA9 on the
    # Ultra-Digital receiver
    ("rx_au2", APP + ["BOARD=2", "ANSWER_UART=2"], 1, 2560, 17052, (3316, 16988)),
    ("rx_au2_xw0", APP + ["BOARD=2", "ANSWER_UART=2"], 0, 1024, 17052, (1556, 16988)),
    ("ud_au2", APP + ["BOARD=3", "ANSWER_UART=2"], 1, 4096, 12096, (4464, 10696)),
    ("ud_au2_xw0", APP + ["BOARD=3", "ANSWER_UART=2"], 0, 2048, 12096, (2932, 10696)),
    ("ud_au1", APP + ["BOARD=3", "ANSWER_UART=1"], 1, 4096, 12096, (4108, 10696)),
    ("ud_au1_xw0", APP + ["BOARD=3", "ANSWER_UART=1"], 0, 2048, 12096, (2604, 10696)),
]
UD_SRAM = 20480 - 2048 - 1536       # the Ultra-Digital receiver: .ramtext + .data + .bss + .noinit must fit the SRAM less the stack
                                    # and 1.5 KB kept free (Quindor 2026-10-17)
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


def build(out, name, defs, xw, ldscript=None, measure=False):
    o = os.path.join(out, name)
    env = dict(os.environ, OUT=o, XW=str(xw), EXTRA_CFLAGS=" ".join("-D" + x for x in defs))
    if ldscript:
        env["LDSCRIPT"] = os.path.join(FW, ldscript)
    if measure:
        env["MEASURE"] = "1"
    with open(o + ".log", "w") as log:
        r = subprocess.run(["bash", os.path.join(FW, "tools", "build_target.sh")], env=env, stdout=log,
                           stderr=subprocess.STDOUT, cwd=FW)
    if r.returncode != 0:
        print("%s: the build FAILED (%s.log)" % (name, o))
        sys.exit(2)
    return os.path.join(o, "fw.elf")


def gate_sizes(out, do_build):
    fails = 0
    print("image      free slot (floor, 0.6.12)        RAM (limit, 0.6.12)")
    for name, defs, xw, floor, ram_max, (free0, ram0) in BUDGET:
        elf = build(out, name, defs, xw, "vendor/wch/Link_app.ld") if do_build else os.path.join(out, name, "fw.elf")
        s = sections(elf)
        flash = sum(s.get(n, 0) for n in (".init", ".text", ".ramtext", ".data"))
        ram = sum(s.get(n, 0) for n in (".data", ".bss", ".noinit"))
        free = SLOT - flash
        sram = ram + s.get(".ramtext", 0)
        bad = free < floor or ram > ram_max or (name.startswith("ud") and sram > UD_SRAM)
        fails += bad
        print("%-10s %6d B (>= %5d, %+5d)          %6d B (<= %5d, %+4d)%s%s" % (
            name, free, floor, free - free0, ram, ram_max, ram - ram0,
            ("   SRAM %5d B (<= %d)" % (sram, UD_SRAM)) if name.startswith("ud") else "", "  OVER BUDGET" if bad else ""))
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
    rx = build(out, "rx_measure", ["BOARD=2"], 1, measure=True) if do_build else os.path.join(out, "rx_measure",
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
    do_build, baseline, write_to, corpus = True, None, None, False
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
        else:
            raise SystemExit(__doc__)
        i += 1
    os.makedirs(out, exist_ok=True)
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

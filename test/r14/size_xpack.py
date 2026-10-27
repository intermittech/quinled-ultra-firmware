# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Apply the unmodified size_gate.py budgets to the three xPack release ELFs.

The normal gate also includes WCH XW images. This test explicitly selects only its
xPack rows and reports that scope, without changing the production gate.
"""
from pathlib import Path
import shutil
import sys

fw = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(fw / "tools"))
import size_gate

release = Path(sys.argv[1]) if len(sys.argv) > 1 else fw / "build/release/0.7.1"
out = fw / "build/r14-size-gate"
for name, variant in (("app_xw0", "rev04"), ("ur_analog_m_xw0", "ur_analog_m-rev0"), ("ur_digital_m_xw0", "ur_digital_m-rev0")):
    target = out / name
    target.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(release / f"obj/app-{variant}/fw.elf", target / "fw.elf")
size_gate.BUDGET = [row for row in size_gate.BUDGET if row[2] == 0]
print("Scope: every xPack/XW=0 release variant; original size_gate.py budgets")
raise SystemExit(size_gate.main(["--out", str(out), "--no-build"]))

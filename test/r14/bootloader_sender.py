# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run the real ESPHome sender/bootloader suite after verifying a newline-only header difference.

All writes stay in firmware/build. The read-only source dependency is supplied explicitly.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

p = argparse.ArgumentParser()
p.add_argument("--esphome", required=True)
p.add_argument("--image", required=True)
a = p.parse_args()
fw = Path(__file__).resolve().parents[2]
src = Path(a.esphome).resolve() / "components/dig2analog_ultra"
x = (fw / "src/core/ultra_update.h").read_bytes()
y = (src / "ultra_update.h").read_bytes()
same = x.replace(b"\r\n", b"\n") == y.replace(b"\r\n", b"\n")
print(f"ultra_update.h: firmware={len(x)} bytes ESPHome={len(y)} bytes; byte-identical={x == y}; normalized-identical={same}", flush=True)
if not same:
    raise SystemExit("Substantive header difference: refusing to normalize")
copy = fw / "build/r14-esphome"
shutil.copytree(src, copy / "components/dig2analog_ultra", dirs_exist_ok=True)
(copy / "components/dig2analog_ultra/ultra_update.h").write_bytes(x)
env = dict(os.environ, ULTRA_ESPHOME=str(copy), BL_TEST_IMAGE=str(Path(a.image).resolve()), CL="/fsanitize=address /Zi")
args = ["cmd", "/d", "/c", "bootloader\\test\\build_bl_test.cmd", "build\\r14-bl-sender"]
print(subprocess.list2cmdline(args), flush=True)
raise SystemExit(subprocess.run(args, cwd=fw, env=env, timeout=300).returncode)

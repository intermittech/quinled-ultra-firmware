# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Run each round-11 regression against the pre-round-11 core, requiring runtime failures.

Usage (MSVC developer prompt): python bootloader/test/negative_r11.py <old-commit> <outdir>
The old main's decision flow is adapted only to the new test-model entry point. No new boot policy is supplied.
All generated files stay in outdir; the working tree is never switched or rewritten.
"""
from pathlib import Path
import os
import subprocess
import sys

repo = Path(__file__).resolve().parents[2]
old, out = sys.argv[1], Path(sys.argv[2]).resolve()
out.mkdir(parents=True, exist_ok=True)
core = subprocess.check_output(['git', '-c', f'safe.directory={repo.as_posix()}', 'show',
                                f'{old}:bootloader/bl_core.c'], cwd=repo).decode()
core += '''
/* Adapter: the old bl_main boot flow, including its full-mode-only watchdog. */
bl_action_t bl_boot(bl_app_t *a, upd_handover_t *ho, uint32_t reset, uint32_t flags)
{
    bl_action_t act;
    bl_app_read(a);
    act = bl_decide(a, ho, reset, flags);
    if (act != BL_ACT_JUMP) { bl_plat_watchdog_start(); }
    return act;
}
'''
(out / 'old_core.c').write_text(core)
subprocess.run(['cl', '/nologo', '/O2', '/W4', '/wd4996', f'/I{repo / "bootloader"}',
                f'/I{repo / "src/core"}', f'/Fe:{out / "old_tests.exe"}', f'/Fo:{out}\\',
                str(repo / 'bootloader/test/test_bl.c'), str(out / 'old_core.c'),
                str(repo / 'bootloader/bl_rx.c')], check=True)
for item in ('a', 'b', 'c', 'd', 'D1'):
    result = subprocess.run([str(out / 'old_tests.exe')], env=os.environ | {'BL_TEST_R11': item},
                            text=True, capture_output=True, timeout=30)
    (out / f'{item}.log').write_text(result.stdout + result.stderr)
    if result.returncode != 1 or 'FAIL ' not in result.stdout:
        raise SystemExit(f'{item}: expected assertion failures, got {result.returncode}\n{result.stdout}')
    print(f'{item}: old code rejected: {result.stdout.strip().splitlines()[-1]}')

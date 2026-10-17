# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compiled scene setup and interrupted-fade steps stay below 8,000 cycles.
The regular pixel steps and wire deadlines are measured by measure_steps and chain_budget.
"""
from pathlib import Path
import tempfile
import test_ud_fx_cycles as T
from rv32emu import Emu

with tempfile.TemporaryDirectory() as td:
    elf = str(Path(td) / 'scene.elf')
    T.build(elf)
    e = Emu(elf)
    for running in (0, 16, 18):
        for target in (1, 16, 18, 31):
            e.call('b_setup', [0, 300, 1, 50])
            if running:
                e.call('b_scene_fades', [running])
            e.call('b_scene_commands', [target])
            steps = []
            for _ in range(24):
                e.call('b_render_once', [])
                steps.append(e.cycles())
                e.call('b_preparing', [])
                if not e.ret:
                    break
            else:
                raise AssertionError('preparation did not finish')
            print('running', running, 'target', target, 'setup max', max(steps), 'cycles')
            assert max(steps) <= 8000, steps
            if running:
                e.call('b_scene_at', [300])
                steps = []
                for _ in range(24):
                    e.call('b_render_once', [])
                    steps.append(e.cycles())
                    e.call('b_preparing', [])
                    if not e.ret:
                        break
                else:
                    raise AssertionError('promotion did not finish')
                print('running', running, 'target', target, 'promotion max', max(steps), 'cycles')
                assert max(steps) <= 8000, steps

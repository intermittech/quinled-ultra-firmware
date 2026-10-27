# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The firmware against the other two repositories, every step required (a missing checkout fails, never skips):

  protocol-copy   test/test_protocol_copy.py: the protocol copies and every protocol pin = PROTOCOL_REF
  vectors         test/run_m1a_vectors.py: the pinned codecs and the firmware vector harness (MSVC)
  proto-sims      the protocol's sender/test/run_tests.py --only sim,dline on this tree: test_sim, test_sim_rx (BOARD=2)
                  and dline_sim, the sender library against this receive path, engine and delay line
  esp-*           the ESPHome component's receiver contracts on this tree: the host test (board engine, protocol header
                  and vectors), the stream glide, the wire-3 contract (scheduler on frame_rx / proto / ultra), the
                  telemetry contract (the hub's polls through the BOARD=2 answer path) and the Digital-M test (the hub's
                  digital port replayed into this delay line)

Every step uses a detached worktree of the protocol at PROTOCOL_REF (made in <out>, removed afterwards); the ESPHome
steps run on the ESPHome checkout as it is (its protocol header and vendored sender must equal the pin's).

    python test/run_cross_repo.py <out dir> [--proto <protocol repo>] [--esphome <ESPHome checkout>]

--proto: else $ULTRA_PROTOCOL_REPO, else ../quinled-ultra-protocol (any checkout that has PROTOCOL_REF).
--esphome: else $ULTRA_ESPHOME, else ../quinled-ultra-esphome. ULTRA_ESPHOME_PYTHON: the Python with ESPHome and
ziglang (default: this one). One step at a time, at most 4 parallel compiles inside a step (ULTRA_TEST_JOBS).
Prints "<step> rc=<rc> <seconds>s"; exit 1 if any step failed.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / 'test'))
from test_protocol_copy import PROTOCOL_REF, default_proto      # noqa: E402

ESP_STEPS = ('run_ultra_host_test', 'run_ultra_stream_host_test', 'run_ultra_contract_test',
             'run_ultra_telemetry_contract_test', 'run_ur_digital_m_test')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', type=Path)
    ap.add_argument('--proto', type=Path, default=default_proto())
    ap.add_argument('--esphome', type=Path,
                    default=Path(os.environ.get('ULTRA_ESPHOME', FW.parent / 'quinled-ultra-esphome')))
    a = ap.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = []

    def step(name, cmd, cwd, env=None, timeout=3600):
        t0 = time.time()
        with (out / f'{name}.log').open('w', encoding='utf-8', errors='replace') as log:
            try:
                rc = subprocess.run(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=timeout).returncode
            except (OSError, subprocess.TimeoutExpired) as e:
                log.write(f'\n{e}\n')
                rc = 124
        rows.append((name, rc))
        print(f'{name} rc={rc} {time.time() - t0:.0f}s', flush=True)

    for path, what in ((a.proto, 'the protocol repository (--proto / ULTRA_PROTOCOL_REPO)'),
                       (a.esphome / 'tests', 'the ESPHome checkout (--esphome / ULTRA_ESPHOME)')):
        if not path.exists():
            print(f'FAIL: {what} is missing ({path})')
            return 1
    env = dict(os.environ, ULTRA_TEST_JOBS=os.environ.get('ULTRA_TEST_JOBS', '4'), PYTHONUTF8='1',
               PYTHONIOENCODING='utf-8')
    step('protocol-copy', [sys.executable, 'test/test_protocol_copy.py', '--proto', str(a.proto)], FW)

    pinned = out / 'protocol'
    git = ['git', '-C', str(a.proto)]
    subprocess.run(git + ['worktree', 'remove', '--force', str(pinned)], capture_output=True)
    shutil.rmtree(pinned, ignore_errors=True)
    made = subprocess.run(git + ['worktree', 'add', '--detach', str(pinned), PROTOCOL_REF], capture_output=True, text=True)
    if made.returncode != 0:
        print(f'FAIL: no worktree of {a.proto} at {PROTOCOL_REF[:12]}: {made.stderr.strip()}')
        return 1
    try:
        step('vectors', [sys.executable, 'test/run_m1a_vectors.py', '--spec', str(pinned), '--out', str(out / 'vectors')],
             FW)
        step('proto-sims', [sys.executable, str(pinned / 'sender/test/run_tests.py'), '--only', 'sim,dline',
                            '--firmware', str(FW)], pinned, dict(env, ULTRA_TEST_BUILD=str(out / 'proto-build')))
        py = os.environ.get('ULTRA_ESPHOME_PYTHON', sys.executable)
        esp_env = dict(env, ULTRA_FIRMWARE=str(FW), ULTRA_PROTOCOL=str(pinned))
        for t in ESP_STEPS:
            step('esp-' + t[4:].replace('_test', '').replace('_', '-'), [py, f'tests/{t}.py'], a.esphome, esp_env)
    finally:
        subprocess.run(git + ['worktree', 'remove', '--force', str(pinned)], capture_output=True)

    failed = [n for n, rc in rows if rc]
    print(f'cross-repo: {"FAILED " + ", ".join(failed) if failed else "all passed"} (protocol {PROTOCOL_REF[:12]}, '
          f'ESPHome {a.esphome})')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

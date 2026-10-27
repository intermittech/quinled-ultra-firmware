# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Build and run the ABI-3 bootloader matrix; exits nonzero on any failure."""
from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def build_bootloaders(folder):
    """boards 1-3 with WCH GCC 15 (WCH_GCC_DIR) and xPack GCC 15.2 (RISCV_GCC): <folder>/bl<board>-<wch|xpack>"""
    import os
    wch = Path(os.environ["WCH_GCC_DIR"]) / "bin/riscv32-wch-elf-gcc"
    for board in (1, 2, 3):
        for name, gcc in (("wch", wch.as_posix()), ("xpack", os.environ["RISCV_GCC"])):
            out = folder / f"bl{board}-{name}"
            out.mkdir(parents=True, exist_ok=True)
            env = dict(os.environ, OUT=out.as_posix(), BOARD=str(board), MCU="ch32v20x", RISCV_GCC=gcc)
            subprocess.run(["bash", "bootloader/build_bl.sh"], cwd=ROOT, env=env, check=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--folder", type=Path, default=ROOT / "build/bl-abi3/current")
    p.add_argument("--build", action="store_true")
    p.add_argument("--full-slot", action="store_true")
    p.add_argument("--evidence", type=Path, default=ROOT / "build/bl-abi3/matrix")
    p.add_argument("--apps", type=Path, help="app builds <apps>/b<board>-app-xw1/fw.elf: the boot path through "
                                             "each real app's startup")
    a = p.parse_args()
    if a.build:
        build_bootloaders(a.folder)
    subprocess.run([sys.executable, "bootloader/test/prepare_abi3_senders.py"], cwd=ROOT, check=True)
    for test in ("test_bl_identity.py", "test_bl_descriptor.py"):
        subprocess.run([sys.executable, str(Path(__file__).parent / test), str(a.folder)], cwd=ROOT, check=True)
    for board in (1, 2, 3):
        check = [(a.folder / f"bl{board}-{compiler}/bl.bin").read_bytes() for compiler in ("wch", "xpack")]
        assert check[0] == check[1], "compiler identity changed: exercise both binaries independently"
    evidence = a.evidence
    evidence.mkdir(parents=True, exist_ok=True)
    inputs = json.loads((ROOT / "build/bl-abi3/senders/inputs.json").read_text())
    inputs["bootloaders"] = {
        str(path.relative_to(a.folder)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(a.folder.glob("bl*/*")) if path.suffix in (".elf", ".bin")
    }
    (evidence / "inputs.json").write_text(json.dumps(inputs, indent=2)+"\n")
    jobs = []
    for board in (1, 2, 3):
        elf = a.folder / f"bl{board}-wch/bl.elf"
        jobs.append((f"board{board}-interruptions", [str(elf)]))
        # Released acquisition/training is longer; retain the timeout and all
        # coverage by scheduling each complete profile as its own bounded job.
        for profile in range(4):
            for mode in ("senders", "wire"):
                jobs.append((f"board{board}-{mode}-p{profile}",
                             [str(elf), "--" + mode, "--line-profile", str(profile)]))
        if a.full_slot:
            jobs.append((f"board{board}-full-slot", [str(elf), "--senders", "--full-slot"]))
        if a.apps:
            jobs.append((f"board{board}-app-startup", [str(elf), "--app", str(a.apps / f"b{board}-app-xw1/fw.elf")]))

    def run(job):
        name, args = job
        with (evidence / f"{name}.log").open("w") as log:
            result = subprocess.run([sys.executable, str(Path(__file__).parent / "test_bl_abi3.py"), *args],
                                    cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=1800)
        row = dict(test=name, exit=result.returncode)
        print(json.dumps(row), flush=True)
        return row

    with ThreadPoolExecutor(max_workers=3) as pool:
        rows = list(pool.map(run, jobs))
    (evidence / "results.json").write_text(json.dumps(rows, indent=2)+"\n")
    raise SystemExit(any(row["exit"] for row in rows))


if __name__ == "__main__":
    main()

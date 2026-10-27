# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Build read-only protocol/ESPHome update drivers with the installed MSVC."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build/bl-abi3/senders"
# a protocol checkout that has PIN, and the ESPHome updater sources (defaults: sibling checkouts)
PROTO = Path(os.environ.get("ULTRA_PROTOCOL_REPO", str(ROOT.parent / "quinled-ultra-protocol")))
ESPHOME = Path(os.environ.get("ESPHOME_UPDATE_SOURCE", str(ROOT.parent / "quinled-ultra-esphome/components/ultra_line")))
PIN = "d5f17d2"


def main():
    for path, env in ((PROTO / ".git", "ULTRA_PROTOCOL_REPO"), (ESPHOME / "ultra_sender.h", "ESPHOME_UPDATE_SOURCE")):
        if not path.exists():
            raise SystemExit(f"FAIL: {path} is missing: a sibling checkout or {env} (both required)")
    OUT.mkdir(parents=True, exist_ok=True)
    source = OUT / "protocol"
    source.mkdir(exist_ok=True)
    names = subprocess.check_output(["git", "-C", str(PROTO), "ls-tree", "-r", "--name-only",
                                     PIN, "sender/src"], text=True).splitlines()
    manifest = dict(protocol_commit=subprocess.check_output(
        ["git", "-C", str(PROTO), "rev-parse", PIN], text=True).strip(), files={})
    for name in names:
        if not name.endswith((".c", ".h")):
            continue
        content = subprocess.check_output(["git", "-C", str(PROTO), "show", f"{PIN}:{name}"])
        (source / Path(name).name).write_bytes(content)
        manifest["files"][name] = hashlib.sha256(content).hexdigest()
    vectors = subprocess.check_output(["git", "-C", str(PROTO), "show", f"{PIN}:test-vectors/vectors.json"])
    (OUT / "vectors.json").write_bytes(vectors)
    manifest["files"]["test-vectors/vectors.json"] = hashlib.sha256(vectors).hexdigest()
    for path in sorted(ESPHOME.glob("ultra_sender*.*")):
        manifest["files"]["esphome/"+path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ("abi3_sender_trace.c", "abi3_esphome_trace.cpp"):
        manifest["files"]["drivers/"+name] = hashlib.sha256(
            (Path(__file__).parent / name).read_bytes()).hexdigest()
    (OUT / "inputs.json").write_text(json.dumps(manifest, indent=2)+"\n")
    vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / \
        "Microsoft Visual Studio/Installer/vswhere.exe"
    vs = Path(subprocess.check_output([str(vswhere), "-latest", "-products", "*",
        "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
        "-property", "installationPath"], text=True).strip())
    init = '@echo off\ncall "' + str(vs / "VC/Auxiliary/Build/vcvars64.bat") + '" >nul\n'
    jobs = {
        "c99": '/std:c11 /I"'+str(source)+'" "'+str(ROOT / "bootloader/test/abi3_sender_trace.c")+'" '+
            " ".join('"'+str(p)+'"' for p in source.glob("*.c")),
        "esphome": '/std:c++17 /EHsc /I"'+str(ESPHOME)+'" "'+
            str(ROOT / "bootloader/test/abi3_esphome_trace.cpp")+'" '+
            " ".join('"'+str(p)+'"' for p in ESPHOME.glob("ultra_sender*.c"))
    }
    for name, flags in jobs.items():
        folder = OUT / name
        folder.mkdir(exist_ok=True)
        script = folder / "build.cmd"
        script.write_text(init + "cl /nologo /O2 /W4 /D_CRT_SECURE_NO_WARNINGS /Fe:trace.exe " + flags + "\n")
        with (folder / "build.log").open("w") as log:
            result = subprocess.run(["cmd", "/c", str(script)], cwd=folder, stdout=log, stderr=subprocess.STDOUT)
        assert result.returncode == 0, (name, folder / "build.log")
        print(name, folder / "trace.exe")


if __name__ == "__main__":
    main()

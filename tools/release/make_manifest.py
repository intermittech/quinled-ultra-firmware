# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Update the channel manifests (stable / beta / alpha .json) for a new release.

    python tools/release/make_manifest.py --channels-dir <checkout of the channels branch> --repo OWNER/REPO \
        --tag v0.6.0-beta.1 --summary "one line" --image <image>.json [--image ...]

Rule: each channel points at the newest release that is at least as stable as the channel. A stable release
updates stable, beta and alpha; a beta (or rc) beta and alpha; an alpha only alpha; never to an older version.
Prints the changed files; each needs a signature (sign_release.py) before it is published. The bytes written are
exactly what gets signed and what the ESP32 checks.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qdu  # noqa: E402

NAME = "QuinLED dig2analog-plus-5ch"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channels-dir", required=True)
    ap.add_argument("--repo", required=True, help="OWNER/REPO on GitHub")
    ap.add_argument("--tag", required=True, help="the release tag, e.g. v0.6.0-beta.1")
    ap.add_argument("--summary", default="")
    ap.add_argument("--image", action="append", required=True, help="image .json from mkimage.py (one per revision set)")
    a = ap.parse_args()

    ver = qdu.parse_version(a.tag)
    stage = qdu.stage_of(ver)
    images = []
    for path in a.image:
        m = json.load(open(path, encoding="utf-8"))
        if m["version_word"] != ver:
            raise SystemExit(f"{path}: image version {m['version']} != tag {a.tag}")
        images.append({
            "hw_id": m["hw_id"], "hw_rev_mask": m["hw_rev_mask"], "min_bootloader": m["min_bootloader"],
            "size": m["size"], "sha256": m["sha256"],
            "url": f"https://github.com/{a.repo}/releases/download/{a.tag}/{m['file']}",
        })
    os.makedirs(a.channels_dir, exist_ok=True)
    changed = []
    for ch, rank in qdu.CHANNEL_RANK.items():
        if rank > qdu.CHANNEL_RANK[stage]:
            continue                                    # a beta never reaches the stable channel
        path = os.path.join(a.channels_dir, f"{ch}.json")
        if os.path.exists(path):
            cur = json.load(open(path, encoding="utf-8"))
            if qdu.parse_version(cur["version"]) >= ver:
                continue                                # never move a channel back
        manifest = {
            "name": NAME, "channel": ch, "version": qdu.format_version(ver),
            "release_url": f"https://github.com/{a.repo}/releases/tag/{a.tag}",
            "summary": a.summary, "images": images,
        }
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
        changed.append(path)
    for p in changed:
        print(p)


if __name__ == "__main__":
    main()

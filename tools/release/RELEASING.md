# Releasing board firmware

The firmware repo, intermittech/quinled-ultra-firmware, publishes releases that ESPHome installs over the data line.
Formats: `src/core/ultra_update.h`. The first release has its own checklist: [FIRST-RELEASE.md](FIRST-RELEASE.md).

## Versions and channels

- **Tags:** `vX.Y.Z` (stable), `vX.Y.Z-beta.N`, `vX.Y.Z-rc.N` (counts as beta), `vX.Y.Z-alpha.N`.
  - X, Y, Z <= 255, N <= 63.
  - The descriptor holds them as one number that sorts like semver (alpha < beta < rc < stable).
- **Channel manifests** on the `channels` branch: `stable.json`, `beta.json`, `alpha.json`, each with a `.sig`.
  - A channel points at the newest release at least as stable as itself: a stable release updates all three, a beta
    updates beta and alpha, an alpha only alpha. Never backwards.
- **ESPHome** reads `https://raw.githubusercontent.com/intermittech/quinled-ultra-firmware/channels/<channel>.json`.
  The images come from the release assets.

## What a release contains

`tools/build_release.sh <version>` builds per variant (see the table in the script):

| File | What |
|---|---|
| `quinled-dig2analog-ultra-5ch-<version>-hw1.bin` | update image: descriptor (64 bytes) + the app linked at 0x2100 |
| `...-hw1.json` | its metadata (version, revisions, size, SHA-256) for the manifest |
| `...-hw1-factory.hex` | production: bootloader + app, marked verified + confirmed |
| `bootloader-rev1.bin / .hex` | the bootloader alone, built for board revision 1 |

## Making a release

1. Set `FW_VERSION` in `src/app/main.c` to the version, commit, tag (`git tag -a v0.6.0-beta.1 -m "one line"`) and push
   the tag.
2. `.github/workflows/release.yml` builds and runs the host tests. It also installs the built image through the
   bootloader logic (`BL_TEST_IMAGE`), creates the GitHub release and writes the channel manifests.
3. **alpha / beta / rc:** signed with the CI key and published at once.
4. **Stable:** the release stays a draft with an unsigned `stable.json`. On the signing PC (a Claude session can do
   this on request):
   `python tools/release/sign_release.py stable --tag v0.6.0 --repo intermittech/quinled-ultra-firmware`
   - It downloads `stable.json`, checks it (version, channel, image URLs), shows it and signs it with the offline key.
   - It uploads `stable.json.sig` and starts `publish-stable.yml`.
   - That workflow checks the signature against `tools/release/keys/release_pub.pem`, puts both files on the
     `channels` branch and publishes the release.

## Production

Program `...-factory.hex` with WCH-LinkUtility / wlink (WCH-LinkE on TP1-TP4):

1. Erase the whole chip.
2. Program the hex. Its addresses are 0x0000-based like the build's `fw.hex`; the tools map them to 0x08000000.
3. Set write protection for sectors 0-1 in the option bytes (WRPR0 bits 0 and 1 = 0): the bootloader can then never
   be erased by the app.

**Two cautions:**

- Bytes that are not in the hex (the rest of the descriptor page) must stay erased: don't fill gaps.
- The factory bootloader is built per board revision (`bootloader-rev<N>`). A board revision with its own bootloader
  needs its own factory hex.

## Repo setup (once)

The repository exists (public, so ESPHome downloads without a token). The rest of the setup is part of
[FIRST-RELEASE.md](FIRST-RELEASE.md), step B:
- the xPack checksum in `release.yml`;
- the keys and the CI secret (KEY_MANAGEMENT.md, "Setting up");
- the production public keys in the ESPHome component.

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

`tools/build_release.sh <version>` builds per variant (see the table in the script; today one: `rev04`, the
dig2analog-ultra-5ch rev 0.4 = board revision 3, mask 0x8):

| File | What |
|---|---|
| `quinled-dig2analog-ultra-5ch-<version>-rev04.bin` | update image: descriptor (64 bytes) + the app linked at 0x2100 |
| `...-rev04.json` | its metadata (version, revisions, size, SHA-256) for the manifest |
| `...-rev04-factory.hex` | production: bootloader + app, marked verified + confirmed |
| `bootloader-rev3.bin / .hex` | the bootloader alone, built for board revision 3 (rev 0.4) |

Each app image is built twice, the second time under another path, and must come out byte for byte the same, with the
same code order (`tools/link_order.py` computes it from the build's own links: the same source, the same order). A
difference stops the release. `REPRO=0` skips the second build; `tools/check_repro.sh` checks any other image.

The bootloader's build id comes from `bootloader/BL_BUILD_ID` (one id per bootloader code), and the release stops
unless the bootloader it built has the md5 recorded there. When the bootloader's code changes: a new id (a date never
used before) and the new md5 in that file, in the same commit.

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
3. Let the board boot once. Since 0.6.8 the app write-protects sectors 0-1 itself at its first boot (WRPR0 bits 0
   and 1 = 0; README "New in 0.6.8"), then resets: the bootloader can then never be erased by the app. The jig's
   `wrp = verify` checks FLASH->WPR after that reset (and RDPR 0xA5: no read protection). A power cut during that
   rewrite leaves the chip read-protected: erase it (which lifts the protection) and program it again.

**Two cautions:**

- Bytes that are not in the hex (the rest of the descriptor page) must stay erased: don't fill gaps.
- The factory bootloader is built per board revision (`bootloader-rev<N>`). A board revision with its own bootloader
  needs its own factory hex.

## Repo setup (once)

The repository exists (public, so ESPHome downloads without a token). The rest of the setup is part of
[FIRST-RELEASE.md](FIRST-RELEASE.md), step B:
- the xPack checksum in `release.yml`;
- the repository variable `WCH_GCC15_URL`: where CI fetches WCH GCC 15.2's 7z for the XW app build
  (`tools/toolchain/README.md`; its SHA-256 is checked against `tools/toolchain/wch-gcc15-win64.sha256`);
- the keys and the CI secret (KEY_MANAGEMENT.md, "Setting up");
- the production public keys in the ESPHome component.

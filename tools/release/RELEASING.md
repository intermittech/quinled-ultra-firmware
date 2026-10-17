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
2. `.github/workflows/release.yml` builds and runs the host tests and the size and cycle gate (`tools/size_gate.py
   --measure tools/measure_baseline.txt --corpus`: the images against their budgets, `measure.py`'s rows, the frozen
   record corpus). It also installs the built image through the bootloader logic (`BL_TEST_IMAGE`), creates the
   GitHub release and writes the channel manifests.
   - An image under its floor, a row slower than allowed, or a corpus a build no longer reads stops the release.
     Raising a budget or a baseline (`--write-measure`) is a decision, in its own commit.
   - After a release whose record formats changed: freeze its corpus (`test/records/README.md`).
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
3. Let the board boot. **The bootloader protects sectors 0–1 before any app runs**, then resets and checks loaded WPR, RDPR=0xA5 and all eight option values against the written snapshot.
   Only WRPR0 bits 0–1 change; USER, DATA and WRPR1–3 are retained. Already protected boards are never rewritten.
   The jig's `wrp = verify` still checks FLASH->WPR after the reset and RDPR=0xA5 (no read protection).
4. Protection must succeed before releasing the board from the jig. Low VDD, an invalid option snapshot, a write
   error or a failed read-back keep it in recovery, dark (Quindor: never an unprotected app). **What the jig
   sees:** no app on TP5 (no banner, no live line, no `jig` line), only the bootloader's `QDBL action 00000004` and
   `protection 5053000A` (written, not as written) / `50530008` (options unreadable; any other value: low supply or a
   read-protected chip); the outputs dark but the red feedback channel's 1 Hz, 10 % recovery blink; FLASH->WPR sectors
   0-1 still open. **Recovery:** fix the supply and power up again (the bootloader retries at every boot while the
   sectors are open); still failing: SWD erase and program the factory hex again. A cut during option erase /
   rewrite can leave RDPR read-protected (the bootloader never touches such a chip): SWD erase (which lifts the
   protection) and program the factory hex again. The wiki's Test jig page has the same table.

The app doesn't set the protection; a data-line update never replaces the bootloader (a new bootloader comes only
with the factory hex, over SWD). Run bootloader bench checks 3, 9–14 before the first factory release with this
build; host models do not settle the physical halfword reprogram, partial erase, option-byte reload or watchdog
timing questions.

The bootloader reports the protection through the hand-over's `reserved` word; the app reports protection events
and the jig reads the option / WPR registers. The bootloader's build id, its exact rev3 binary md5 and size are in
`bootloader/BL_BUILD_ID` and `bootloader/README.md`.

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

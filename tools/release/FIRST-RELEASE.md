# The first release

The checklist for the first published release of intermittech/quinled-ultra-firmware, once prototypes are on the bench.
The repository exists; the production keys don't, and they are made only in step B. Procedures referenced:
[RELEASING.md](RELEASING.md) and [KEY_MANAGEMENT.md](KEY_MANAGEMENT.md).

## A. Before: the boards

1. **Flash the prototypes** with a locally built factory hex. Build it with `bash tools/build_release.sh
   0.5.0-alpha.0`, then flash it as described under "Production" in RELEASING.md. Leave the write protection off
   while developing.
2. **Bench checks that the release path depends on** (`bootloader/README.md` "Bench checks", and the firmware
   README's):
   - flash erase / program at HCLK/2;
   - the erased value;
   - status records;
   - the hand-over app ↔ bootloader;
   - VERIFY after a reset;
   - one update session from an ESP32 with the TEST keys (`public_keys: test`, a LAN `source:` with manifests from
     `make_manifest.py` + `sign_release.py`);
   - a power cut during a session (at least by hand; the relay rig before stable).
3. **Turn-on time** with Diff-Power: a confirmed app boots without a listen window.

## B. Once: keys and repository settings

1. **xPack checksum:** fill in `XPACK_GCC_SHA256` in `.github/workflows/release.yml`. It is the SHA-256 of
   `xpack-riscv-none-elf-gcc-<XPACK_GCC>-win32-x64.zip`, from the xPack release page.
2. **Keys:** KEY_MANAGEMENT.md, "Setting up", on the signing PC (Quindor's):
   - `sign_release.py keygen --kind release`: the release key in `%USERPROFILE%\.quinled-release-keys\`. Note the
     passphrase it prints once.
   - The backups: the passphrase in the password manager, `release_key.pem` on a USB stick.
   - `sign_release.py keygen --kind ci --out <temp>`, then
     `gh secret set UPDATE_CI_KEY_PEM -R intermittech/quinled-ultra-firmware < <temp>\ci_key.pem`. Back it up with the
     release key, then delete `<temp>`.
   - Commit `tools/release/keys/release_pub.pem` and `ci_pub.pem` (public keys only: the release workflow fails on a
     committed private key).
3. **ESPHome component:**
   - `sign_release.py pubkey --pub <file>` prints the arrays for `fw_keys.h`. Fill them in and set
     `FW_PRODUCTION_KEYS` to the list of both keys (the release key for all channels, the CI key for beta + alpha).
   - Merge and release the component. `DEFAULT_REPO` in `update.py` already names this repository.
4. **GitHub settings:**
   - Actions may write contents: the workflow pushes the `channels` branch and creates releases. Its permissions are
     in the workflow (`contents: write`).
   - Optional: protect `main`.

The `channels` branch needs no preparation: the first release run creates it.

## C. The first alpha

1. **Version:** set `FW_VERSION` in `src/app/main.c` to `0.5.0-alpha.1`; the release build passes the tag's version
   anyway.
2. **Sync** the development folder into the repository clone (the staging tool's `--diff`, then `--sync`), review,
   and commit.
3. **Tag and push:**
   - `git tag -a v0.5.0-alpha.1 -m "first alpha"`;
   - `git push origin v0.5.0-alpha.1`.
4. **Check the run:**
   - the host tests;
   - the release with the `.bin`, `.json` and `-factory.hex`;
   - `channels/alpha.json` + `alpha.json.sig`;
   - locally, with the manifest and its `.sig` downloaded side by side:
     `sign_release.py verify --file alpha.json --pub tools/release/keys/ci_pub.pem`.
5. **On the bench:**
   - an ESP32 with the released component (production keys), `channel: alpha`;
   - Home Assistant shows 0.5.0-alpha.1;
   - Install;
   - the boards blink "updated";
   - "installed" shows the version;
   - the next power-up has no listen window (the PING confirmed it).

## D. Beta and stable

1. **Beta:** tag `v0.5.0-beta.1`. The beta and alpha channels move to it.
2. **Stable:**
   - tag `v0.5.0`. The release stays a draft with an unsigned `stable.json`.
   - On the signing PC:
     `python tools/release/sign_release.py stable --tag v0.5.0 --repo intermittech/quinled-ultra-firmware`.
   - `publish-stable.yml` checks the signature and publishes; every channel then points at 0.5.0.
3. **Production:** the stable release's `-factory.hex`, flashed with the bootloader write-protected (RELEASING.md,
   "Production").

## Never

- Generate production keys before step B, or anywhere but the signing PC (the CI key is generated there too, then
  only kept as the secret and in the backup).
- Commit a private key. Only the TEST pairs in `keys/test/` are allowed, and the workflow checks.
- Sign a stable manifest without reading what `sign_release.py stable` shows (version, channel, image URLs).

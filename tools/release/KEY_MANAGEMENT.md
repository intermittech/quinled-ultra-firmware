# Signing keys: runbook

Board firmware reaches ESPHome through channel manifests (`stable.json`, `beta.json`, `alpha.json`) on the firmware
repo's `channels` branch. Each manifest carries the SHA-256 of its images. The ESPHome component only trusts a
manifest whose signature (`<manifest>.sig`, ECDSA P-256 / SHA-256) verifies with a public key built into it.

## The two keys

| Key | Signs | Private key lives | Public key |
|---|---|---|---|
| **Release key** | stable (may also sign beta / alpha) | only on the signing PC: `%USERPROFILE%\.quinled-release-keys\` | `tools/release/keys/release_pub.pem` + the ESPHome component |
| **CI key** | alpha, beta | GitHub Actions secret `UPDATE_CI_KEY_PEM` | `tools/release/keys/ci_pub.pem` + the ESPHome component |

**Why two:**

- A stolen CI key, a hijacked workflow or a compromised GitHub account can only reach people who chose beta or alpha.
- Stable needs the offline key: the component accepts the CI key for beta and alpha only.

**How the release key is stored:**

- `release_key.pem` is encrypted (PKCS#8) with a random passphrase.
- The passphrase is in `passphrase.dpapi`, protected by Windows DPAPI for the Windows user that created it.
- So the file alone is useless on another PC or for another user, and signing needs no typing. An automated session
  runs it on request (`sign_release.py stable ...`, see RELEASING.md).
- The folder is readable only by that user (`icacls`, set by `keygen`).

**TEST keys** (`tools/release/keys/test/`) are for development only, and their private keys are in the repo on
purpose. A production ESPHome build never contains their public keys.

## Setting up (once, when the firmware repo exists; not before)

On the signing PC, from the firmware repo:

1. **Release key**
   - `python tools/release/sign_release.py keygen --kind release`
   - It creates `%USERPROFILE%\.quinled-release-keys\` (only you can read it) with `release_key.pem`,
     `passphrase.dpapi` and `release_pub.pem`.
   - It prints the **passphrase once**.
2. **Backup of the release key**, kept offline, in two places:
   - the passphrase in your password manager;
   - `release_key.pem` on a USB stick (or as an attachment in the password manager).
   - `passphrase.dpapi` is not part of the backup: it only works for this Windows user on this PC.
3. **CI key**
   - `python tools/release/sign_release.py keygen --kind ci --out <temporary folder>`
   - `gh secret set UPDATE_CI_KEY_PEM < <temporary folder>\ci_key.pem`
   - Put `ci_key.pem` in the same backup as the release key, then delete the temporary folder.
4. **Publish the public keys**
   - Copy `release_pub.pem` and `ci_pub.pem` to `tools/release/keys/` and commit them.
   - `python tools/release/sign_release.py pubkey --pub <file>` prints the C arrays: put them into the ESPHome
     component (`fw_keys.h`: the arrays and the `FW_PRODUCTION_KEYS` list, empty until then) and release the
     component.
5. **Test the chain** with an alpha tag: the workflow signs, and an ESPHome device on the alpha channel shows the
   update.

## Rotating a key (planned, e.g. every few years or when the signing PC is replaced)

1. Generate the new key as in "Setting up". For the release key, use another folder name, or move the old one away
   first: `keygen` refuses to overwrite a key.
2. Add its public key to `fw_keys.h` **next to** the old one (the component keeps a list) and release the component.
3. Wait until users have updated ESPHome (weeks; announce it in the release notes).
4. Sign with the new key from then on (replace the GitHub secret for the CI key).
5. Later, remove the old public key from `fw_keys.h` in a component release.

**On a new signing PC:**

1. Copy `release_key.pem` from the backup into `%USERPROFILE%\.quinled-release-keys\`.
2. Run `python tools/release/sign_release.py restore`. It asks for the backed-up passphrase, checks that it opens the
   key, stores it with DPAPI for the new Windows user, restricts the folder and writes `release_pub.pem` (it must match
   the repo's).

## Lost key (no backup left)

- **CI key:** generate a new one, set the secret, add its public key to the component; old beta / alpha manifests
  stay valid for old builds. Nothing urgent.
- **Release key:** no new stable firmware reaches ESPHome builds that only know the lost key.
  1. Generate a new release key.
  2. Ship an ESPHome component release with its public key.
  3. Tell users to update ESPHome first; old installs keep their current board firmware.
  4. Meanwhile a release can go out on the beta channel (CI key) for people who need it.

## Compromised key (someone else may have the private key)

1. Generate a new key, put its public key into `fw_keys.h` and **remove the compromised one** in the same component
   release. Publish it at once with a clear note.
2. Re-sign the current channel manifests with the new key.
3. For the CI key: replace the GitHub secret and check the repo's settings and workflows for changes.
4. Check the `channels` branch history for manifests you did not make.
5. Damage is limited: the boards check CRC, hardware id and version themselves, and nothing installs without someone
   pressing *Install*. An ESP32 still on an old component build trusts the compromised key until it is updated:
   say so in the advisory.

# Firmware 0.7.1 first release

Firmware 0.7.1 is a source release for QuinLED Dig2Analog-Plus-5ch and
Diff-Power Ultra Receiver Analog-Medium / Digital-Medium, on CH32V203 and
CH32V407 as applicable. Signed images and channel manifests follow physical
bench qualification. The packaging gate in `tools/build_release.sh` stays closed
until then; the source release does not require a firmware tag or signing keys.

## A. Source release

1. Run the source, host, compiled firmware, bootloader, size and cycle gates.
2. Check the public tree, protocol copies, licences, links and author identity.
3. Publish the reviewed source tree with the matching protocol and board docs.
   Publishing requires the repository owner's approval.

## B. Before signed images

1. Complete the [bootloader bench checks](../../bootloader/README.md) and each
   board's timing, interrupted-flash, protection and update qualification.
   Verify both MCU families, both family orders and downstream update forwarding.
2. Fill in the xPack checksum and `WCH_GCC15_URL` workflow settings described in
   [RELEASING.md](RELEASING.md).
3. Set up production keys only on the signing PC, following
   [KEY_MANAGEMENT.md](KEY_MANAGEMENT.md). Back them up and put the CI key in the
   CI secret. Commit only public keys; install them in the ESPHome component.
4. Review the packaging gate and image identities only after qualification.
   Build reproducible images for every supported target and assembly variant,
   and verify installation, recovery, confirmation and rediscovery on the bench.

## C. Image releases

Follow [RELEASING.md](RELEASING.md) for version tags, manifests, alpha/beta CI
signing and offline stable signing. Choose the next version deliberately;
never publish an image version below the 0.7.1 source release. Read the manifest
before signing it. Production uses a qualified factory image with bootloader
protection verified before the board leaves the jig.

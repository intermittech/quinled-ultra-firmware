# Signing keys

- `test/release/` and `test/ci/`: **TEST key pairs, private keys included on purpose.** For development, host tests
  and bench builds only. Their public keys are never in a production build of the ESPHome component.
- Production public keys go here when the firmware repo is set up (`release_pub.pem`, `ci_pub.pem`), following
  [../KEY_MANAGEMENT.md](../KEY_MANAGEMENT.md). The production private keys are never in any repository:
  - the release key lives in `%USERPROFILE%\.quinled-release-keys\` on the signing PC;
  - the CI key is a GitHub Actions secret.

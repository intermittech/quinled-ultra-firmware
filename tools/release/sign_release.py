# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Signing keys and signatures for the QuinLED dig2analog-ultra-5ch firmware channels (KEY_MANAGEMENT.md).

ECDSA P-256 with SHA-256 over the exact bytes of a channel manifest; the signature (DER) goes next to it as
<manifest>.sig. The ESPHome component checks it with mbedTLS before it trusts the manifest (and through its SHA-256
the image).

    python sign_release.py keygen --kind test    --out tools/release/keys/test      (TEST keys, no protection)
    python sign_release.py keygen --kind ci      --out <tmp dir>                     (CI key: to a GitHub secret)
    python sign_release.py keygen --kind release [--out %USERPROFILE%\\.quinled-release-keys]   (offline key)
    python sign_release.py sign   --file stable.json [--key-dir DIR | --key PEM | --key-env VAR]
    python sign_release.py verify --file stable.json --pub release_pub.pem [--pub ci_pub.pem ...]
    python sign_release.py pubkey --pub release_pub.pem            (C array for the ESPHome component)
    python sign_release.py stable --tag v0.6.0 --repo OWNER/REPO   (sign a draft stable release + publish it)
    python sign_release.py restore                                  (new signing PC: passphrase from the backup)

The release key is stored encrypted (PKCS#8, a random 32-byte passphrase); the passphrase itself is kept with
Windows DPAPI (bound to this Windows user), so signing needs no typing and the key file alone is useless. keygen
prints the passphrase once for the offline backup.
"""
import argparse
import base64
import os
import secrets
import subprocess
import sys

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec

DEFAULT_RELEASE_DIR = os.path.join(os.path.expanduser("~"), ".quinled-release-keys")


# ------------------------------------------------------------------------------------------------ DPAPI (Windows)
def _dpapi(data, protect):
    import ctypes
    from ctypes import wintypes

    class BLOB(ctypes.Structure):
        _fields_ = [("cbData", wintypes.DWORD), ("pbData", ctypes.POINTER(ctypes.c_char))]

    buf = ctypes.create_string_buffer(data, len(data))
    inb = BLOB(len(data), ctypes.cast(buf, ctypes.POINTER(ctypes.c_char)))
    out = BLOB()
    crypt32, kernel32 = ctypes.windll.crypt32, ctypes.windll.kernel32
    flags = 0x1                                           # CRYPTPROTECT_UI_FORBIDDEN
    if protect:
        ok = crypt32.CryptProtectData(ctypes.byref(inb), ctypes.c_wchar_p("quinled release key"), None, None, None,
                                      flags, ctypes.byref(out))
    else:
        ok = crypt32.CryptUnprotectData(ctypes.byref(inb), None, None, None, None, flags, ctypes.byref(out))
    if not ok:
        raise OSError("DPAPI failed (wrong Windows user?)")
    try:
        return ctypes.string_at(out.pbData, out.cbData)
    finally:
        kernel32.LocalFree(out.pbData)


# ------------------------------------------------------------------------------------------------ keys
def _write(path, data, private=False):
    with open(path, "wb") as f:
        f.write(data)
    if private and os.name != "nt":
        os.chmod(path, 0o600)


def _pub_pem(key):
    return key.public_key().public_bytes(serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo)


def keygen(kind, out):
    out = out or (DEFAULT_RELEASE_DIR if kind == "release" else None)
    if not out:
        raise SystemExit("--out is required")
    os.makedirs(out, exist_ok=True)
    key = ec.generate_private_key(ec.SECP256R1())
    if kind == "release":
        if os.path.exists(os.path.join(out, "release_key.pem")):
            raise SystemExit(f"{out} already holds a release key: refusing to overwrite it")
        if os.name != "nt":
            raise SystemExit("the release key setup uses Windows DPAPI: run it on the signing PC")
        passphrase = base64.urlsafe_b64encode(secrets.token_bytes(32))
        pem = key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                serialization.BestAvailableEncryption(passphrase))
        _write(os.path.join(out, "release_key.pem"), pem, private=True)
        _write(os.path.join(out, "passphrase.dpapi"), _dpapi(passphrase, True), private=True)
        _write(os.path.join(out, "release_pub.pem"), _pub_pem(key))
        user = os.environ.get("USERNAME", "")
        if user:
            # only this Windows user (no inherited access for other accounts)
            subprocess.run(["icacls", out, "/inheritance:r", "/grant:r", f"{user}:(OI)(CI)F"], check=True,
                           stdout=subprocess.DEVNULL)
        print(f"release key in {out} (encrypted; passphrase kept with DPAPI for this Windows user)")
        print("BACKUP (store both offline, e.g. password manager + USB stick; this is shown only once):")
        print(f"  passphrase: {passphrase.decode()}")
        print(f"  file:       {os.path.join(out, 'release_key.pem')}")
    else:
        name = "test" if kind == "test" else "ci"
        pem = key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                serialization.NoEncryption())
        _write(os.path.join(out, f"{name}_key.pem"), pem, private=True)
        _write(os.path.join(out, f"{name}_pub.pem"), _pub_pem(key))
        print(f"{name} key pair in {out}")
        if kind == "ci":
            print("upload it:  gh secret set UPDATE_CI_KEY_PEM < ci_key.pem   (then delete ci_key.pem here, keep the")
            print("            backup copy offline); commit ci_pub.pem as tools/release/keys/ci_pub.pem")


def restore(key_dir):
    """after copying release_key.pem from the backup to a new PC / user: store its passphrase with DPAPI again"""
    import getpass
    d = key_dir or DEFAULT_RELEASE_DIR
    pem = open(os.path.join(d, "release_key.pem"), "rb").read()
    passphrase = getpass.getpass("release key passphrase (from the backup): ").strip().encode()
    key = serialization.load_pem_private_key(pem, password=passphrase)      # raises if the passphrase is wrong
    _write(os.path.join(d, "passphrase.dpapi"), _dpapi(passphrase, True), private=True)
    _write(os.path.join(d, "release_pub.pem"), _pub_pem(key))
    user = os.environ.get("USERNAME", "")
    if user:
        subprocess.run(["icacls", d, "/inheritance:r", "/grant:r", f"{user}:(OI)(CI)F"], check=True,
                       stdout=subprocess.DEVNULL)
    print(f"{d}: passphrase stored for this Windows user; release_pub.pem written (compare it with the repo's)")


def load_private(args):
    if args.key_env:
        pem = os.environ.get(args.key_env, "").encode()
        if not pem:
            raise SystemExit(f"environment variable {args.key_env} is empty")
        return serialization.load_pem_private_key(pem, password=None)
    if args.key:
        return serialization.load_pem_private_key(open(args.key, "rb").read(), password=None)
    d = args.key_dir or DEFAULT_RELEASE_DIR
    pem = open(os.path.join(d, "release_key.pem"), "rb").read()
    passphrase = _dpapi(open(os.path.join(d, "passphrase.dpapi"), "rb").read(), False)
    return serialization.load_pem_private_key(pem, password=passphrase)


def sign(args):
    key = load_private(args)
    data = open(args.file, "rb").read()
    sig = key.sign(data, ec.ECDSA(hashes.SHA256()))
    key.public_key().verify(sig, data, ec.ECDSA(hashes.SHA256()))
    _write(args.file + ".sig", sig)
    print(f"{args.file}.sig ({len(sig)} bytes DER)")


def verify(args):
    data = open(args.file, "rb").read()
    sig = open(args.sig or args.file + ".sig", "rb").read()
    for p in args.pub:
        pub = serialization.load_pem_public_key(open(p, "rb").read())
        try:
            pub.verify(sig, data, ec.ECDSA(hashes.SHA256()))
            print(f"OK: signed with {p}")
            return
        except InvalidSignature:
            pass
    raise SystemExit("signature does not match any of the given public keys")


def stable(args):
    """sign a stable release's manifest with the offline key and publish it (RELEASING.md, "Stable")"""
    import json
    import tempfile
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import qdu
    d = tempfile.mkdtemp(prefix="qd-stable-")
    subprocess.run(["gh", "release", "download", args.tag, "--repo", args.repo, "--pattern", "stable.json", "--dir", d],
                   check=True)
    path = os.path.join(d, "stable.json")
    m = json.load(open(path, encoding="utf-8"))
    if m.get("channel") != "stable" or qdu.parse_version(m["version"]) != qdu.parse_version(args.tag) or not m["images"]:
        raise SystemExit(f"{path}: not the stable manifest of {args.tag}")
    for im in m["images"]:
        if f"/{args.repo}/releases/download/{args.tag}/" not in im["url"] or len(im["sha256"]) != 64:
            raise SystemExit(f"{path}: unexpected image entry {im}")
    print(json.dumps(m, indent=2))
    args.file = path
    sign(args)
    subprocess.run(["gh", "release", "upload", args.tag, path + ".sig", "--clobber", "--repo", args.repo], check=True)
    subprocess.run(["gh", "workflow", "run", "publish-stable.yml", "-f", f"tag={args.tag}", "--repo", args.repo],
                   check=True)
    print(f"signed and uploaded; the publish-stable workflow checks it and publishes {args.tag}")


def pubkey(args):
    pub = serialization.load_pem_public_key(open(args.pub, "rb").read())
    der = pub.public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo)
    rows = [", ".join(f"0x{b:02x}" for b in der[i:i + 13]) for i in range(0, len(der), 13)]
    print(f"/* {os.path.basename(args.pub)}: SubjectPublicKeyInfo DER, {len(der)} bytes */")
    print("{\n    " + ",\n    ".join(rows) + ",\n}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    k = sp.add_parser("keygen")
    k.add_argument("--kind", choices=["test", "ci", "release"], required=True)
    k.add_argument("--out")
    s = sp.add_parser("sign")
    s.add_argument("--file", required=True)
    s.add_argument("--key-dir")
    s.add_argument("--key")
    s.add_argument("--key-env")
    v = sp.add_parser("verify")
    v.add_argument("--file", required=True)
    v.add_argument("--sig")
    v.add_argument("--pub", action="append", required=True)
    p = sp.add_parser("pubkey")
    p.add_argument("--pub", required=True)
    st = sp.add_parser("stable", help="sign a draft stable release's manifest (offline key) and publish it")
    st.add_argument("--tag", required=True)
    st.add_argument("--repo", required=True, help="OWNER/REPO")
    st.add_argument("--key-dir")
    st.set_defaults(key=None, key_env=None)
    r = sp.add_parser("restore", help="after restoring release_key.pem from the backup: store its passphrase (DPAPI)")
    r.add_argument("--key-dir")
    a = ap.parse_args()
    if a.cmd == "keygen":
        keygen(a.kind, a.out)
    elif a.cmd == "restore":
        restore(a.key_dir)
    elif a.cmd == "sign":
        sign(a)
    elif a.cmd == "stable":
        stable(a)
    elif a.cmd == "verify":
        verify(a)
    else:
        pubkey(a)


if __name__ == "__main__":
    main()

#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["cryptography"]
# ///
"""Encrypt a raw firmware binary for the TC66 bootloader's update path.

The bootloader accepts an AES-256-ECB encrypted image (it decrypts each 64-byte
chunk on the fly before writing to flash). This pads the raw app to the fixed
image size and encrypts it with the board's firmware-image key.

    encrypt.py app.bin app.enc.bin
    tc66flash.py flash app.enc.bin           # then flash over the bootloader (hold K1)

The app-side 'updat' path (app_update.py) takes a PLAINTEXT image instead and does
not need this step. Encryption is only for the bootloader flash path and recovery.
"""
import sys
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

# Board firmware-image key (AES-256-ECB, no IV/tweak).
KEY = bytes.fromhex("381255645a6b7aaabc6d5877f5c1fbff9805d1ac3c8335661a610c7ff73a638c")
IMAGE_SIZE = 51840          # 810 x 64-byte chunks = the app slot size

def encrypt(raw: bytes) -> bytes:
    if len(raw) > IMAGE_SIZE:
        raise SystemExit(f"image too large: {len(raw)} > {IMAGE_SIZE}")
    padded = raw + b"\x00" * (IMAGE_SIZE - len(raw))
    enc = Cipher(algorithms.AES(KEY), modes.ECB()).encryptor()
    return enc.update(padded) + enc.finalize()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__); sys.exit(2)
    out = encrypt(open(sys.argv[1], "rb").read())
    open(sys.argv[2], "wb").write(out)
    print(f"wrote {sys.argv[2]}: {len(out)} bytes ({len(out)//64} chunks)")

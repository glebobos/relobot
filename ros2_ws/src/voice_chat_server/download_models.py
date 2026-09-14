#!/usr/bin/env python3
"""
Downloads the Piper Optimus Prime voice model into ros2_ws/models.
"""

import os
import sys
import urllib.request

MODELS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models"))
BIOFECTS_MODEL_URL = "https://github.com/biofects/piper-voice/releases/download/v1.0.0/biofects_prime.onnx"
BIOFECTS_CONFIG_URL = "https://github.com/biofects/piper-voice/releases/download/v1.0.0/biofects_prime.onnx.json"


def download_file(url: str, dest_path: str, desc: str = ""):
    if os.path.exists(dest_path) and os.path.getsize(dest_path) > 1000:
        print(f"[OK] {os.path.basename(dest_path)} already exists ({os.path.getsize(dest_path):,} bytes).")
        return

    print(f"Downloading {desc or os.path.basename(dest_path)}...")
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (X11; Linux x86_64)"})
        with urllib.request.urlopen(req) as resp, open(dest_path, "wb") as f:
            total = int(resp.headers.get("Content-Length", 0))
            downloaded = 0
            chunk_size = 1024 * 1024
            while True:
                chunk = resp.read(chunk_size)
                if not chunk:
                    break
                f.write(chunk)
                downloaded += len(chunk)
                if total > 0:
                    percent = downloaded * 100 / total
                    sys.stdout.write(f"\r  {downloaded / (1024*1024):.1f} MB / {total / (1024*1024):.1f} MB ({percent:.1f}%)")
                else:
                    sys.stdout.write(f"\r  {downloaded / (1024*1024):.1f} MB")
                sys.stdout.flush()
        print("\n[Done]")
    except Exception as e:
        print(f"\n[Download Error] {e}")
        if os.path.exists(dest_path):
            os.remove(dest_path)
        raise RuntimeError(f"Failed to download {desc} from {url}")


def setup_models():
    os.makedirs(MODELS_DIR, exist_ok=True)
    prime_dest = os.path.join(MODELS_DIR, "biofects_prime.onnx")
    prime_json_dest = os.path.join(MODELS_DIR, "biofects_prime.onnx.json")

    download_file(BIOFECTS_MODEL_URL, prime_dest, "Piper Optimus Prime voice model (.onnx)")
    download_file(BIOFECTS_CONFIG_URL, prime_json_dest, "Piper Optimus Prime voice config (.json)")

    print(f"\nModel ready in {MODELS_DIR}:")
    for f in sorted(os.listdir(MODELS_DIR)):
        fp = os.path.join(MODELS_DIR, f)
        if os.path.isfile(fp):
            print(f"  {f}: {os.path.getsize(fp):,} bytes")


if __name__ == "__main__":
    setup_models()

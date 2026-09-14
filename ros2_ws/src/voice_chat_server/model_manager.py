#!/usr/bin/env python3
"""
ReloBot Voice Model Manager
Single source of truth for voice model metadata, download URLs, integrity checks, and search paths.
"""

import os
import sys
import urllib.request
from typing import Tuple, Optional

# Single source of truth for Optimus Prime model URLs & filenames
PRIME_MODEL_NAME = "biofects_prime.onnx"
PRIME_CONFIG_NAME = "biofects_prime.onnx.json"
BIOFECTS_MODEL_URL = "https://github.com/biofects/piper-voice/releases/download/v1.0.0/biofects_prime.onnx"
BIOFECTS_CONFIG_URL = "https://github.com/biofects/piper-voice/releases/download/v1.0.0/biofects_prime.onnx.json"
MIN_MODEL_SIZE_BYTES = 10_000_000  # ONNX model is ~60MB

DEFAULT_SEARCH_PATHS = [
    os.getenv("PIPER_MODELS_DIR", "/opt/piper_models"),
    "/opt/piper_models",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "models")),
    "/ros2_ws/models",
]


def resolve_model_paths() -> Optional[Tuple[str, str]]:
    """Locates existing valid voice model (.onnx) and config (.json) in candidate paths."""
    for base_dir in DEFAULT_SEARCH_PATHS:
        model_path = os.path.join(base_dir, PRIME_MODEL_NAME)
        config_path = os.path.join(base_dir, PRIME_CONFIG_NAME)
        if (
            os.path.isfile(model_path)
            and os.path.getsize(model_path) > MIN_MODEL_SIZE_BYTES
            and os.path.isfile(config_path)
            and os.path.getsize(config_path) > 100
        ):
            return model_path, config_path
    return None


def download_file(url: str, dest_path: str, desc: str = ""):
    """Downloads a file with streaming chunks and console progress."""
    if os.path.exists(dest_path) and os.path.getsize(dest_path) > 1000:
        print(f"[ModelManager] {os.path.basename(dest_path)} already exists ({os.path.getsize(dest_path):,} bytes).")
        return

    print(f"[ModelManager] Downloading {desc or os.path.basename(dest_path)}...")
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
        print("\n[ModelManager] Download complete.")
    except Exception as e:
        print(f"\n[ModelManager Download Error] {e}")
        if os.path.exists(dest_path):
            try:
                os.remove(dest_path)
            except OSError:
                pass
        raise RuntimeError(f"Failed to download {desc} from {url}: {e}")


def ensure_model_available(target_dir: Optional[str] = None) -> Tuple[str, str]:
    """Finds existing voice model or downloads it into the target directory."""
    existing = resolve_model_paths()
    if existing:
        return existing

    dest_dir = target_dir or DEFAULT_SEARCH_PATHS[0]
    os.makedirs(dest_dir, exist_ok=True)
    model_dest = os.path.join(dest_dir, PRIME_MODEL_NAME)
    config_dest = os.path.join(dest_dir, PRIME_CONFIG_NAME)

    download_file(BIOFECTS_MODEL_URL, model_dest, "Optimus Prime Piper Voice Model (.onnx)")
    download_file(BIOFECTS_CONFIG_URL, config_dest, "Optimus Prime Voice Config (.json)")

    return model_dest, config_dest


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else None
    m_path, c_path = ensure_model_available(target)
    print(f"[ModelManager] Ready -> Model: {m_path}, Config: {c_path}")

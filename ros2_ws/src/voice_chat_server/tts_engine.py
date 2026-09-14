#!/usr/bin/env python3
"""
ReloBot Piper Neural TTS Engine
High-performance in-memory speech synthesis using ONNX Runtime with Optimus Prime voice model.
"""

import os
import io
import re
import wave
import json
import logging
import threading
from typing import Optional, Generator, Tuple
import numpy as np

from model_manager import ensure_model_available

logger = logging.getLogger("TtsEngine")

# Voice delivery pace tuning (lower = faster, snappier robotic speech)
DEFAULT_LENGTH_SCALE = float(os.getenv("VOICE_LENGTH_SCALE", "0.78"))


def normalize_text_for_speech(text: str) -> str:
    """
    Cleans markdown formatting, links, URLs, bullet points, and code blocks
    so the neural TTS engine reads clean, natural speech without speaking symbols aloud.
    """
    if not text:
        return ""

    # Remove code blocks ```...```
    text = re.sub(r"```[\s\S]*?```", "", text)
    # Remove inline code `...`
    text = re.sub(r"`([^`]+)`", r"\1", text)
    # Replace markdown links [text](url) -> text
    text = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", text)
    # Strip raw HTTP/HTTPS URLs
    text = re.sub(r"https?://\S+", "", text)
    # Remove HTML tags <...>
    text = re.sub(r"<[^>]+>", "", text)
    # Remove bullet markers at line starts: - , * , 1. , etc.
    text = re.sub(r"(?:^|\n)\s*[-*•]\s+", " ", text)
    text = re.sub(r"(?:^|\n)\s*\d+\.\s+", " ", text)
    # Remove bold/italic/strikethrough markers (*, _, ~, #)
    text = re.sub(r"[*_~#]", "", text)
    # Collapse multiple whitespace and newlines to a single space
    text = re.sub(r"\s+", " ", text).strip()

    return text


def trim_leading_silence(data_bytes: bytes, threshold: int = 400) -> bytes:
    """Strips leading silence samples to ensure zero-latency speech onset."""
    data = np.frombuffer(data_bytes, dtype=np.int16)
    non_silent = np.where(np.abs(data) > threshold)[0]
    if len(non_silent) > 0:
        # Keep 10ms ramp-in (220 samples at 22050Hz) to prevent audio clicks
        start_idx = max(0, non_silent[0] - 220)
        return data[start_idx:].tobytes()
    return data_bytes


class OptimusTTS:
    """High-performance Piper TTS engine cached in RAM."""

    def __init__(self, model_path: Optional[str] = None, config_path: Optional[str] = None):
        self._lock = threading.Lock()
        if not model_path:
            self.model_path, self.config_path = ensure_model_available()
        else:
            self.model_path = model_path
            self.config_path = config_path or f"{model_path}.json"

        self.voice = None
        self.sample_rate = 22050
        self._load_voice()

    def _load_voice(self):
        """Loads ONNX inference session and warms up Piper engine."""
        import onnxruntime as ort
        from piper import PiperVoice
        from piper.config import PiperConfig

        logger.info(f"Loading Piper voice model from {self.model_path} into RAM...")
        with open(self.config_path, "r", encoding="utf-8") as f:
            config_dict = json.load(f)
        piper_config = PiperConfig.from_dict(config_dict)

        num_threads = min(6, os.cpu_count() or 4)
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = num_threads
        sess_options.inter_op_num_threads = 1
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

        session = ort.InferenceSession(
            self.model_path,
            sess_options=sess_options,
            providers=["CPUExecutionProvider"]
        )

        self.voice = PiperVoice(config=piper_config, session=session)
        self.sample_rate = self.voice.config.sample_rate

        # Warm up engine with a short token
        try:
            list(self.voice.synthesize("Ready"))
        except Exception:
            pass

        logger.info(f"Piper TTS engine ready ({self.sample_rate}Hz, {num_threads} threads).")

    def synthesize_pcm_chunks(
        self,
        text: str,
        length_scale: Optional[float] = None,
        volume: float = 1.0
    ) -> Generator[bytes, None, None]:
        """
        Yields raw int16 LE PCM audio byte chunks directly in memory.
        Thread-safe for streaming over WebSocket or feeding audio sinks.
        """
        from piper.config import SynthesisConfig
        clean_text = normalize_text_for_speech(text)
        if not clean_text:
            return

        effective_length_scale = length_scale if length_scale is not None else DEFAULT_LENGTH_SCALE
        syn_config = SynthesisConfig(
            length_scale=effective_length_scale,
            volume=volume
        )

        with self._lock:
            is_first = True
            for audio_chunk in self.voice.synthesize(clean_text, syn_config=syn_config):
                raw = audio_chunk.audio_int16_bytes
                if is_first:
                    trimmed = trim_leading_silence(raw)
                    is_first = False
                    to_send = trimmed if trimmed else raw
                else:
                    to_send = raw
                if to_send:
                    yield to_send

    def synthesize_wav_bytes(
        self,
        text: str,
        length_scale: Optional[float] = None,
        volume: float = 1.0
    ) -> bytes:
        """Synthesizes text into complete in-memory WAV file bytes."""
        from piper.config import SynthesisConfig
        clean_text = normalize_text_for_speech(text)
        if not clean_text:
            return b""

        effective_length_scale = length_scale if length_scale is not None else DEFAULT_LENGTH_SCALE
        syn_config = SynthesisConfig(
            length_scale=effective_length_scale,
            volume=volume
        )

        bio = io.BytesIO()
        with self._lock:
            with wave.open(bio, "wb") as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.sample_rate)
                is_first = True
                for chunk in self.voice.synthesize(clean_text, syn_config=syn_config):
                    raw = chunk.audio_int16_bytes
                    if is_first:
                        trimmed = trim_leading_silence(raw)
                        is_first = False
                        to_write = trimmed if trimmed else raw
                    else:
                        to_write = raw
                    if to_write:
                        wav_file.writeframes(to_write)
        return bio.getvalue()

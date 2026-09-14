import os
import io
import json
import wave
import threading
from typing import Optional, Generator, Tuple
import numpy as np

# Search paths for model (prioritizes Docker build directory /opt/piper_models)
DEFAULT_SEARCH_PATHS = [
    os.getenv("PIPER_MODELS_DIR", "/opt/piper_models"),
    "/opt/piper_models",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "models")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "models")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "voice-chat-poc", "models")),
    "/ros2_ws/models",
]

PRIME_MODEL_NAME = "biofects_prime.onnx"
BIOFECTS_MODEL_URL = "https://github.com/biofects/piper-voice/releases/download/v1.0.0/biofects_prime.onnx"
BIOFECTS_CONFIG_URL = "https://github.com/biofects/piper-voice/releases/download/v1.0.0/biofects_prime.onnx.json"


def find_or_download_model() -> Tuple[str, str]:
    """Finds the Optimus Prime ONNX model or downloads it."""
    for base_dir in DEFAULT_SEARCH_PATHS:
        model_p = os.path.join(base_dir, PRIME_MODEL_NAME)
        config_p = f"{model_p}.json"
        if os.path.exists(model_p) and os.path.getsize(model_p) > 1000000:
            return model_p, config_p

    # If not found, download to the first writeable location
    target_dir = DEFAULT_SEARCH_PATHS[0]
    os.makedirs(target_dir, exist_ok=True)
    target_model = os.path.join(target_dir, PRIME_MODEL_NAME)
    target_config = f"{target_model}.json"

    import urllib.request
    print(f"[OptimusTTS] Downloading voice model to {target_model}...")
    headers = {"User-Agent": "Mozilla/5.0"}
    
    # Download model
    req = urllib.request.Request(BIOFECTS_MODEL_URL, headers=headers)
    with urllib.request.urlopen(req) as resp, open(target_model, "wb") as f:
        f.write(resp.read())

    # Download config
    req = urllib.request.Request(BIOFECTS_CONFIG_URL, headers=headers)
    with urllib.request.urlopen(req) as resp, open(target_config, "wb") as f:
        f.write(resp.read())

    print("[OptimusTTS] Voice model downloaded successfully.")
    return target_model, target_config


def trim_leading_silence(data_bytes: bytes, threshold: int = 400) -> bytes:
    """Strips leading silence/padding to ensure instant voice onset."""
    data = np.frombuffer(data_bytes, dtype=np.int16)
    non_silent = np.where(np.abs(data) > threshold)[0]
    if len(non_silent) > 0:
        # Keep 10ms smooth ramp-in (220 samples at 22050Hz)
        start_idx = max(0, non_silent[0] - 220)
        return data[start_idx:].tobytes()
    return b""


class ContinuousAudioStream:
    """
    Maintains a continuous PulseAudio/ALSA PCM stream for sequential sentence chunks.
    """

    def __init__(self, voice, sample_rate: int, player_adapter, length_scale=None, noise_scale=None, volume=1.0):
        from piper.config import SynthesisConfig
        self.voice = voice
        self.sample_rate = sample_rate
        self.player_adapter = player_adapter
        self.syn_config = SynthesisConfig(
            length_scale=length_scale,
            noise_scale=noise_scale,
            volume=volume
        )
        self.stream_proc = self.player_adapter.open_pcm_stream(sample_rate=self.sample_rate)

    def feed_text(self, text: str):
        """Synthesizes text chunk and immediately writes PCM bytes to audio stream."""
        if not self.stream_proc or not self.stream_proc.stdin:
            return
        text = text.strip()
        if not text:
            return
        try:
            for audio_chunk in self.voice.synthesize(text, syn_config=self.syn_config):
                raw = audio_chunk.audio_int16_bytes
                trimmed = trim_leading_silence(raw)
                to_write = trimmed if trimmed else raw
                self.stream_proc.stdin.write(to_write)
                self.stream_proc.stdin.flush()
        except Exception as e:
            print(f"[ContinuousAudioStream Error] {e}")

    def close(self):
        """Closes stream cleanly."""
        if self.stream_proc:
            try:
                if self.stream_proc.stdin:
                    self.stream_proc.stdin.close()
                self.stream_proc.wait(timeout=2)
            except Exception:
                pass
            self.stream_proc = None


class OptimusTTS:
    """High-performance Piper TTS engine with RAM caching and direct streaming."""

    def __init__(self, model_path: Optional[str] = None, config_path: Optional[str] = None):
        self._lock = threading.Lock()
        if not model_path:
            self.model_path, self.config_path = find_or_download_model()
        else:
            self.model_path = model_path
            self.config_path = config_path or f"{model_path}.json"

        self.voice = None
        self.sample_rate = 22050
        self._load_voice()

    def _load_voice(self):
        import onnxruntime as ort
        from piper import PiperVoice
        from piper.config import PiperConfig

        print(f"[OptimusTTS] Loading voice model from {self.model_path} into RAM...")
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

        # Warm up engine
        try:
            list(self.voice.synthesize("Ready"))
        except Exception:
            pass

        print(f"[OptimusTTS] Model cached in RAM ({self.sample_rate}Hz, {num_threads} threads). Ready.")

    def create_stream_session(
        self,
        player_adapter,
        length_scale: Optional[float] = None,
        noise_scale: Optional[float] = None,
        volume: float = 1.0
    ) -> ContinuousAudioStream:
        """Creates a continuous PCM audio streaming session for pipelined speech."""
        return ContinuousAudioStream(
            voice=self.voice,
            sample_rate=self.sample_rate,
            player_adapter=player_adapter,
            length_scale=length_scale,
            noise_scale=noise_scale,
            volume=volume
        )

    def synthesize_pcm_chunks(
        self,
        text: str,
        length_scale: Optional[float] = None,
        volume: float = 1.0
    ) -> Generator[bytes, None, None]:
        """
        Yields raw PCM int16 audio byte chunks directly in memory.
        Ideal for streaming audio over WebSockets to browsers or audio sinks.
        """
        from piper.config import SynthesisConfig
        syn_config = SynthesisConfig(
            length_scale=length_scale,
            volume=volume
        )
        text = text.strip()
        if not text:
            return

        with self._lock:
            for audio_chunk in self.voice.synthesize(text, syn_config=syn_config):
                raw = audio_chunk.audio_int16_bytes
                trimmed = trim_leading_silence(raw)
                yield trimmed if trimmed else raw

    def synthesize_wav_bytes(
        self,
        text: str,
        length_scale: Optional[float] = None,
        volume: float = 1.0
    ) -> bytes:
        """Synthesizes text directly into a complete in-memory WAV file."""
        from piper.config import SynthesisConfig
        syn_config = SynthesisConfig(
            length_scale=length_scale,
            volume=volume
        )
        bio = io.BytesIO()
        with self._lock:
            with wave.open(bio, "wb") as wav_file:
                self.voice.synthesize_wav(text, wav_file, syn_config=syn_config)
        return bio.getvalue()

import os
import subprocess
import shutil
from typing import Optional


class AudioPlayer:
    """Handles audio playback using PulseAudio (paplay), ALSA (aplay), or system fallback."""

    def __init__(self):
        self.paplay_bin = shutil.which("paplay") or "/usr/bin/paplay"
        self.aplay_bin = shutil.which("aplay") or "/usr/bin/aplay"
        self.has_paplay = os.path.exists(self.paplay_bin)
        self.has_aplay = os.path.exists(self.aplay_bin)

    def play(self, wav_path: str, block: bool = True) -> bool:
        """Plays a WAV file through PulseAudio or ALSA."""
        if not os.path.exists(wav_path):
            print(f"[AudioPlayer] File not found: {wav_path}")
            return False

        cmd = None
        if self.has_paplay:
            cmd = [self.paplay_bin, wav_path]
        elif self.has_aplay:
            cmd = [self.aplay_bin, "-q", wav_path]

        if not cmd:
            print(f"[AudioPlayer] No audio player found (paplay/aplay). Audio saved to: {wav_path}")
            return False

        try:
            if block:
                proc = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
                if proc.returncode != 0:
                    print(f"[AudioPlayer Warning] Audio playback returned {proc.returncode}: {proc.stderr.strip()}")
                    return False
                return True
            else:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return True
        except Exception as e:
            print(f"[AudioPlayer Error] Failed to play {wav_path}: {e}")
            return False

    def open_pcm_stream(self, sample_rate: int = 22050, channels: int = 1) -> Optional[subprocess.Popen]:
        """Opens a raw PCM streaming subprocess into paplay/aplay for sub-second audio output."""
        if self.has_paplay:
            cmd = [
                self.paplay_bin,
                "--raw",
                f"--rate={sample_rate}",
                "--format=s16le",
                f"--channels={channels}"
            ]
        elif self.has_aplay:
            cmd = [
                self.aplay_bin,
                "-q",
                "-t", "raw",
                "-f", "S16_LE",
                "-r", str(sample_rate),
                "-c", str(channels)
            ]
        else:
            return None

        try:
            return subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            print(f"[AudioPlayer Stream Error] {e}")
            return None


player = AudioPlayer()

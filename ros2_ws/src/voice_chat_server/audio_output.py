#!/usr/bin/env python3
"""
ReloBot Local Hardware PCM Audio Sink
Streams raw PCM audio chunks to local hardware speakers via PulseAudio (paplay) or ALSA (aplay).
"""

import os
import shutil
import subprocess
import logging
from typing import Optional

logger = logging.getLogger("AudioSink")


class PcmAudioSink:
    """Manages an active raw PCM audio streaming subprocess."""

    def __init__(self, sample_rate: int = 22050, channels: int = 1):
        self.sample_rate = sample_rate
        self.channels = channels
        self.proc: Optional[subprocess.Popen] = None
        self._paplay_bin = shutil.which("paplay")
        self._aplay_bin = shutil.which("aplay")
        self._open_stream()

    def _open_stream(self):
        """Spawns the audio playback subprocess."""
        cmd = None
        if self._paplay_bin:
            cmd = [
                self._paplay_bin,
                "--raw",
                f"--rate={self.sample_rate}",
                "--format=s16le",
                f"--channels={self.channels}"
            ]
        elif self._aplay_bin:
            cmd = [
                self._aplay_bin,
                "-q",
                "-t", "raw",
                "-f", "S16_LE",
                "-r", str(self.sample_rate),
                "-c", str(self.channels)
            ]

        if not cmd:
            logger.warning("No audio player binary (paplay/aplay) found in PATH. Physical audio output disabled.")
            return

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            logger.debug(f"Audio sink process started: {' '.join(cmd)} (PID: {self.proc.pid})")
        except Exception as e:
            logger.warning(f"Failed to spawn audio player subprocess: {e}")
            self.proc = None

    def write(self, pcm_bytes: bytes) -> bool:
        """Writes raw PCM byte chunk to the player subprocess stdin."""
        if not self.proc or not self.proc.stdin or not pcm_bytes:
            return False
        try:
            self.proc.stdin.write(pcm_bytes)
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError) as e:
            logger.warning(f"Audio sink pipe error: {e}")
            self.close()
            return False

    def close(self):
        """Closes the audio stream stdin and waits for playback to finish."""
        if self.proc:
            try:
                if self.proc.stdin:
                    self.proc.stdin.close()
                self.proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
            except Exception:
                pass
            finally:
                self.proc = None

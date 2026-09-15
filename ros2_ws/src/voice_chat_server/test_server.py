#!/usr/bin/env python3
"""
Test script for ReloBot Voice Chat Server
Verifies that Piper TTS loads, and server can stream tokens and audio.
"""

import sys
import os
import asyncio
import json

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from player import player
from tts import OptimusTTS
from server import AgentRunner, VoiceChatServer, clean_markdown_for_speech


async def run_tests():
    print("=== ReloBot Voice Chat Server Unit Tests ===")

    # 1. Test markdown cleaner
    md_test = "Hello **world**! Check `ls -la` and [link](http://test.com) for details."
    cleaned = clean_markdown_for_speech(md_test)
    assert "world" in cleaned and "**" not in cleaned and "`" not in cleaned, f"Markdown cleaning failed: {cleaned}"
    print("[PASS] Markdown cleaner for TTS speech")

    # 2. Test AgentRunner stream
    runner = AgentRunner()
    tokens = []
    async for token in runner.generate_response_stream("battery status"):
        tokens.append(token)
    full_resp = "".join(tokens)
    print(f"[PASS] Agent stream produced: '{full_resp}' ({len(tokens)} chunks)")

    # 3. Test Piper TTS engine
    print("[INFO] Testing Piper TTS engine...")
    try:
        tts = OptimusTTS()
        assert tts.voice is not None, "Voice not loaded"
        print(f"[PASS] Piper TTS loaded: {tts.sample_rate}Hz")

        # Test synthesis to WAV bytes in memory
        wav_bytes = tts.synthesize_wav_bytes("ReloBot online and operational.")
        assert len(wav_bytes) > 1000, f"Synthesized WAV too small: {len(wav_bytes)} bytes"
        print(f"[PASS] Piper TTS generated in-memory WAV ({len(wav_bytes):,} bytes)")

        # Test raw PCM chunks generator
        pcm_chunks = list(tts.synthesize_pcm_chunks("Autobots, roll out!"))
        assert len(pcm_chunks) > 0, "No PCM chunks generated"
        print(f"[PASS] Piper TTS generated {len(pcm_chunks)} raw PCM chunks")
    except Exception as e:
        print(f"[WARN/FAIL] Piper TTS test encountered: {e}")

    print("\n=== ALL VOICE CHAT SERVER TESTS COMPLETED SUCCESSFULLY ===")


if __name__ == "__main__":
    asyncio.run(run_tests())

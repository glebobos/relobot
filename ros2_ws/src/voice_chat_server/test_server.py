#!/usr/bin/env python3
"""
Test Suite for ReloBot Voice Chat Subsystem
Validates Model Manager, Text Normalizer, AGY Agent Runner, Piper TTS Engine, and Audio Output Sink.
"""

import os
import sys
import asyncio
import logging

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from model_manager import resolve_model_paths, ensure_model_available
from tts_engine import OptimusTTS, normalize_text_for_speech
from agent_runner import AgentRunner
from audio_output import PcmAudioSink

logging.basicConfig(level=logging.INFO)


async def run_tests():
    print("=== ReloBot Voice Chat Subsystem Modular Test Suite ===")

    # 1. Test Model Manager
    print("\n[1/5] Testing Model Manager...")
    model_p, config_p = ensure_model_available()
    assert os.path.isfile(model_p), f"Model file missing: {model_p}"
    assert os.path.isfile(config_p), f"Config file missing: {config_p}"
    assert os.path.getsize(model_p) > 10_000_000, "Model file size too small"
    print(f"[PASS] Model verified: {model_p} ({os.path.getsize(model_p):,} bytes)")

    # 2. Test Text Normalizer for Speech
    print("\n[2/5] Testing Speech Text Normalizer...")
    dirty_md = (
        "# Heading\n"
        "Hello **Optimus**! Visit [ReloBot Docs](https://relobot.io/docs) or `ros2 topic list`.\n"
        "- Bullet 1\n"
        "- Bullet 2\n"
        "1. First step\n"
        "```bash\necho test\n```\n"
        "Operational *status* is 100%."
    )
    cleaned = normalize_text_for_speech(dirty_md)
    print(f"[INFO] Cleaned text output: '{cleaned}'")
    assert "**" not in cleaned, "Asterisks remaining"
    assert "https://" not in cleaned, "Raw URL remaining"
    assert "```" not in cleaned, "Code block remaining"
    assert "ReloBot Docs" in cleaned, "Markdown link label was stripped"
    assert "Bullet 1" in cleaned and "Bullet 2" in cleaned, "Bullet content missing"
    print("[PASS] Text normalization verified.")

    # 3. Test Agent Runner (Multi-turn conversation with AGY CLI)
    print("\n[3/5] Testing AGY Agent Runner...")
    runner = AgentRunner()
    if runner.agy_bin:
        print(f"[INFO] AGY Binary: {runner.agy_bin}")
        print("[INFO] Invoking Turn 1: 'Remember secret security token: ALPHA-992'...")
        conv_id = None
        tokens_t1 = []
        async for item in runner.generate_response_stream("Remember secret security token: ALPHA-992"):
            if item.get("event") == "init":
                conv_id = item.get("conversation_id")
            elif item.get("event") == "token":
                tokens_t1.append(item.get("text", ""))

        full_t1 = "".join(tokens_t1).strip()
        print(f"[PASS] Turn 1 completed (conv_id={conv_id}): '{full_t1}'")
        assert conv_id is not None, "Failed to capture conversation ID on Turn 1"
        assert len(full_t1) > 0, "Turn 1 returned empty response"

        print(f"[INFO] Invoking Turn 2 (pinned conv_id={conv_id}): 'What was the secret security token?'...")
        tokens_t2 = []
        t2_conv_id = None
        async for item in runner.generate_response_stream("What was the secret security token?", conversation_id=conv_id):
            if item.get("event") == "init":
                t2_conv_id = item.get("conversation_id")
            elif item.get("event") == "token":
                tokens_t2.append(item.get("text", ""))

        full_t2 = "".join(tokens_t2).strip()
        print(f"[PASS] Turn 2 completed (conv_id={t2_conv_id}): '{full_t2}'")
        assert "ALPHA-992" in full_t2 or "ALPHA" in full_t2 or "992" in full_t2, (
            f"Multi-turn context retention failed: expected 'ALPHA-992' in '{full_t2}'"
        )
        print("[PASS] AGY multi-turn conversation memory verified.")
    else:
        print("[SKIP] AGY binary not available in current environment; skipping CLI test.")

    # 4. Test Piper TTS Engine
    print("\n[4/5] Testing Piper TTS Engine...")
    tts = OptimusTTS(model_path=model_p, config_path=config_p)
    assert tts.voice is not None, "Voice failed to load"
    print(f"[PASS] Piper TTS engine loaded ({tts.sample_rate}Hz)")

    # Test WAV generation
    wav_bytes = tts.synthesize_wav_bytes("ReloBot voice chat subsystem is operational.")
    assert len(wav_bytes) > 1000, "Synthesized WAV file is too small"
    assert wav_bytes[:4] == b"RIFF", "Invalid WAV header (missing RIFF)"
    assert wav_bytes[8:12] == b"WAVE", "Invalid WAV header (missing WAVE)"
    print(f"[PASS] Synthesized valid WAV ({len(wav_bytes):,} bytes)")

    # Test PCM streaming chunks
    pcm_chunks = list(tts.synthesize_pcm_chunks("I am ReloBot. All systems nominal."))
    assert len(pcm_chunks) > 0, "No PCM chunks returned"
    total_pcm = sum(len(c) for c in pcm_chunks)
    assert total_pcm > 1000, "Synthesized PCM bytes too small"
    print(f"[PASS] Synthesized {len(pcm_chunks)} raw PCM chunks ({total_pcm:,} int16 bytes)")

    # 5. Test Audio Output Sink
    print("\n[5/5] Testing PCM Audio Sink lifecycle...")
    sink = PcmAudioSink(sample_rate=tts.sample_rate)
    # Write a test chunk (won't throw even if no physical speaker connected)
    sink.write(pcm_chunks[0])
    sink.close()
    assert sink.proc is None, "Audio sink process not cleaned up properly"
    print("[PASS] Audio sink opened, fed, and cleanly closed.")

    print("\n=======================================================")
    print("=== ALL VOICE CHAT SUBMODULE TESTS PASSED (5/5) ===")
    print("=======================================================")


if __name__ == "__main__":
    asyncio.run(run_tests())

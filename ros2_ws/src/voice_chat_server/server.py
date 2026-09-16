#!/usr/bin/env python3
"""
ReloBot AI Voice Chat Server
Direct bidirectional bridge coordinating Web Interface, Antigravity (AGY) Agent, and Piper Neural TTS.
"""

import os
import sys
import json
import time
import re
import asyncio
import logging
from typing import Optional

# Local imports
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from agent_runner import AgentRunner
from tts_engine import OptimusTTS, normalize_text_for_speech
from audio_output import PcmAudioSink

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger("VoiceChatServer")

PORT = int(os.getenv("VOICE_CHAT_PORT", "8765"))
HOST = os.getenv("VOICE_CHAT_HOST", "0.0.0.0")

# Sentence & natural clause boundary regex
# Matches .!? or \n while ignoring decimal numbers (e.g. 12.4V) and single initials
SENTENCE_END_RE = re.compile(r"(\n+|(?:(?<!\d)(?<!\b[A-Za-z])[.!?]+)(?:\s+(?=[A-ZА-Я0-9])|\s*$))")
CLAUSE_BREAK_RE = re.compile(r"([,;:—–]+)\s*")


class VoiceChatServer:
    """Coordinates AI prompt streaming, real-time token delivery, and concurrent Piper TTS."""

    def __init__(self):
        self.agent = AgentRunner()
        self.tts: Optional[OptimusTTS] = None
        self._init_tts()

    def _init_tts(self):
        try:
            self.tts = OptimusTTS()
            logger.info("Piper TTS engine initialized successfully.")
        except Exception as e:
            logger.warning(f"Piper TTS engine could not be loaded: {e}. Voice synthesis disabled.")
            self.tts = None

    async def handle_connection(self, websocket):
        client_addr = getattr(websocket, 'remote_address', 'unknown')
        logger.info(f"Client connected: {client_addr}")
        active_task: Optional[asyncio.Task] = None

        try:
            async for message_str in websocket:
                try:
                    data = json.loads(message_str)
                except (json.JSONDecodeError, TypeError):
                    continue

                msg_type = data.get("type", "")

                if msg_type == "ping":
                    await websocket.send(json.dumps({"type": "pong"}))
                    continue

                elif msg_type == "status":
                    await websocket.send(json.dumps({
                        "type": "status",
                        "has_agy": self.agent.agy_bin is not None,
                        "agy_path": self.agent.agy_bin,
                        "has_tts": self.tts is not None,
                        "sample_rate": self.tts.sample_rate if self.tts else 22050,
                        "ready": True
                    }))
                    continue

                elif msg_type == "cancel" or msg_type == "stop":
                    if active_task and not active_task.done():
                        active_task.cancel()
                        logger.info(f"Generation cancelled for client {client_addr}.")
                    await websocket.send(json.dumps({"type": "cancelled"}))
                    continue

                elif msg_type == "prompt":
                    prompt = data.get("text", "").strip()
                    msg_id = data.get("msg_id", f"msg_{int(time.time() * 1000)}")
                    conversation_id = data.get("conversation_id")
                    play_robot = data.get("play_robot_audio", True)
                    stream_browser = data.get("stream_browser_audio", True)

                    if not prompt:
                        continue

                    # Cancel any prior active generation for this client
                    if active_task and not active_task.done():
                        active_task.cancel()

                    active_task = asyncio.create_task(
                        self._process_prompt(
                            websocket,
                            prompt,
                            msg_id,
                            conversation_id,
                            play_robot,
                            stream_browser
                        )
                    )

        except Exception as e:
            logger.info(f"Client disconnected ({client_addr}): {e}")
        finally:
            if active_task and not active_task.done():
                active_task.cancel()

    async def _process_prompt(
        self,
        websocket,
        prompt: str,
        msg_id: str,
        conversation_id: Optional[str],
        play_robot: bool,
        stream_browser: bool
    ):
        """Processes a prompt: streams tokens immediately and concurrently synthesizes speech."""
        t_start = time.time()
        logger.info(f"Processing prompt [{msg_id}] (conv={conversation_id or 'new'}): '{prompt}'")

        await websocket.send(json.dumps({
            "type": "start",
            "msg_id": msg_id,
            "conversation_id": conversation_id
        }))

        full_text = ""
        sentence_buffer = ""
        active_conv_id = conversation_id

        # Local hardware speaker audio sink
        audio_sink: Optional[PcmAudioSink] = None
        if play_robot and self.tts:
            try:
                audio_sink = PcmAudioSink(sample_rate=self.tts.sample_rate)
            except Exception as e:
                logger.warning(f"Could not initialize audio sink: {e}")

        # Dedicated background TTS synthesis queue & worker
        tts_queue: asyncio.Queue[Optional[str]] = asyncio.Queue()
        tts_worker_task = asyncio.create_task(
            self._tts_worker(
                tts_queue,
                msg_id,
                websocket,
                audio_sink,
                stream_browser
            )
        )

        try:
            async for event_item in self.agent.generate_response_stream(prompt, conversation_id=conversation_id):
                ev_type = event_item.get("event")

                if ev_type == "init":
                    active_conv_id = event_item.get("conversation_id") or active_conv_id
                    await websocket.send(json.dumps({
                        "type": "init",
                        "msg_id": msg_id,
                        "conversation_id": active_conv_id
                    }))

                elif ev_type == "token":
                    token = event_item.get("text", "")
                    active_conv_id = event_item.get("conversation_id") or active_conv_id
                    full_text += token
                    sentence_buffer += token

                    # Send text token to frontend immediately (ZERO LATENCY)
                    await websocket.send(json.dumps({
                        "type": "token",
                        "msg_id": msg_id,
                        "text": token,
                        "conversation_id": active_conv_id
                    }))

                    # Check for natural sentence or clause boundaries with sufficient word length (>= 8 words)
                    # to ensure audio playback duration masks background synthesis of the next chunk.
                    while True:
                        match = SENTENCE_END_RE.search(sentence_buffer)
                        if match:
                            split_idx = match.end()
                            candidate = sentence_buffer[:split_idx].strip()
                            words = candidate.split()
                            if len(words) >= 8:
                                sentence_buffer = sentence_buffer[split_idx:]
                                if candidate:
                                    tts_queue.put_nowait(candidate)
                                continue

                        # For very long clauses without periods, split on comma/semicolon if >= 12 words
                        words = sentence_buffer.split()
                        if len(words) >= 12:
                            c_match = CLAUSE_BREAK_RE.search(sentence_buffer)
                            if c_match:
                                split_idx = c_match.end()
                                candidate = sentence_buffer[:split_idx].strip()
                                sentence_buffer = sentence_buffer[split_idx:]
                                if candidate:
                                    tts_queue.put_nowait(candidate)
                                continue

                        break

                elif ev_type == "error":
                    err_msg = event_item.get("error", "Unknown AGY error")
                    active_conv_id = event_item.get("conversation_id") or active_conv_id
                    await websocket.send(json.dumps({
                        "type": "error",
                        "msg_id": msg_id,
                        "error": err_msg,
                        "conversation_id": active_conv_id
                    }))

            # Enqueue remaining text in sentence buffer
            remaining = sentence_buffer.strip()
            if remaining:
                tts_queue.put_nowait(remaining)

            # Signal TTS worker to finish
            tts_queue.put_nowait(None)

            # Send done event to browser immediately
            total_dur_ms = (time.time() - t_start) * 1000
            logger.info(
                f"Completed text stream for [{msg_id}] in {total_dur_ms:.1f}ms "
                f"(conv_id={active_conv_id}, chars={len(full_text)})."
            )
            await websocket.send(json.dumps({
                "type": "done",
                "msg_id": msg_id,
                "conversation_id": active_conv_id,
                "full_text": full_text,
                "duration_ms": total_dur_ms
            }))

            # Await TTS worker completion
            await asyncio.wait_for(tts_worker_task, timeout=60.0)

        except asyncio.CancelledError:
            logger.info(f"Prompt processing [{msg_id}] cancelled.")
            tts_worker_task.cancel()
        except Exception as e:
            logger.error(f"Error processing prompt [{msg_id}]: {e}", exc_info=True)
            tts_worker_task.cancel()
            await websocket.send(json.dumps({
                "type": "error",
                "msg_id": msg_id,
                "conversation_id": active_conv_id,
                "error": str(e)
            }))
        finally:
            if audio_sink:
                audio_sink.close()

    async def _tts_worker(
        self,
        queue: asyncio.Queue,
        msg_id: str,
        websocket,
        audio_sink: Optional[PcmAudioSink],
        stream_browser: bool
    ):
        """Synthesizes queued text clauses into raw PCM chunks and streams to browser and speaker."""
        if stream_browser and self.tts:
            try:
                await websocket.send(json.dumps({
                    "type": "audio_start",
                    "msg_id": msg_id,
                    "sample_rate": self.tts.sample_rate
                }))
            except Exception as e:
                logger.warning(f"Failed to send audio_start for [{msg_id}]: {e}")

        while True:
            try:
                clause = await queue.get()
                if clause is None:
                    break

                clean_text = normalize_text_for_speech(clause)
                if not clean_text or not self.tts:
                    continue

                t0 = time.time()
                # Run ONNX inference in threadpool to avoid blocking event loop
                chunks = await asyncio.to_thread(lambda: list(self.tts.synthesize_pcm_chunks(clean_text)))
                synth_ms = (time.time() - t0) * 1000
                total_bytes = sum(len(c) for c in chunks)

                logger.debug(
                    f"Synthesized clause [{msg_id}] in {synth_ms:.1f}ms "
                    f"({len(chunks)} chunks, {total_bytes:,} bytes): '{clean_text[:40]}...'"
                )

                for chunk in chunks:
                    if not chunk:
                        continue
                    # 1. Output to local robot hardware speaker
                    if audio_sink:
                        audio_sink.write(chunk)
                    # 2. Stream binary WebSocket frame to browser
                    if stream_browser:
                        await websocket.send(chunk)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.warning(f"TTS synthesis error for [{msg_id}]: {e}")
            finally:
                if 'clause' in locals() and clause is not None:
                    queue.task_done()

        if stream_browser and self.tts:
            try:
                await websocket.send(json.dumps({
                    "type": "audio_end",
                    "msg_id": msg_id
                }))
            except Exception:
                pass


async def main():
    import websockets
    server_instance = VoiceChatServer()
    logger.info(f"Starting ReloBot Voice Chat WebSocket Server on ws://{HOST}:{PORT}")

    async with websockets.serve(
        server_instance.handle_connection,
        HOST,
        PORT,
        ping_interval=20,
        ping_timeout=20,
        max_size=10 * 1024 * 1024
    ):
        logger.info(f"ReloBot Voice Chat Server listening on {HOST}:{PORT}")
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Voice Chat Server terminated by user.")

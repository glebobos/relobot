#!/usr/bin/env python3
"""
ReloBot AI Voice Chat Server
Bridges ReloBot Web Interface with Antigravity (AGY) Agent & Piper TTS (Optimus Prime).
"""

import os
import sys
import json
import base64
import re
import shutil
import asyncio
import logging
from typing import Optional, AsyncGenerator

# Local imports
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from player import player
from tts import OptimusTTS, trim_leading_silence

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("VoiceChatServer")

# Host / Port
PORT = int(os.getenv("VOICE_CHAT_PORT", "8765"))
HOST = os.getenv("VOICE_CHAT_HOST", "0.0.0.0")

# ReloBot System Context for AGY
RELOBOT_SYSTEM_PROMPT = (
    "You are ReloBot AI, an autonomous intelligent robotic mower and assistant powered by "
    "ROS2 Humble, differential drive hardware, LiDAR SLAM, Nav2 navigation, and Optimus Prime's voice synthesis. "
    "You speak in a confident, helpful, and heroic robotic tone inspired by Optimus Prime. "
    "Keep spoken responses clear, concise, and direct (1 to 3 short sentences usually work best for real-time voice speech). "
    "You can assist with mower status, navigation waypoints, battery telemetry, and general inquiries."
)


def find_agy_binary() -> Optional[str]:
    """Finds the agy CLI binary if installed on the system or available via host mount."""
    for bin_name in ["agy", "antigravity"]:
        path = shutil.which(bin_name)
        if path and os.access(path, os.X_OK):
            return path
        # Common local and host-mounted locations
        for prefix in [
            "/host_snap_bin",
            "/host_usr_local_bin",
            "/host_bin",
            "/snap/bin",
            "/usr/local/bin",
            "/usr/bin",
            os.path.expanduser("~/.local/bin"),
            "/root/.local/bin"
        ]:
            candidate = os.path.join(prefix, bin_name)
            if os.path.exists(candidate) and os.access(candidate, os.X_OK):
                return candidate
    return None


def clean_markdown_for_speech(text: str) -> str:
    """Strips markdown links, bolding, code blocks, and symbols for clean speech synthesis."""
    # Remove code blocks
    text = re.sub(r"```[\s\S]*?```", "", text)
    # Remove inline code
    text = re.sub(r"`([^`]+)`", r"\1", text)
    # Remove markdown links [text](url) -> text
    text = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", text)
    # Remove bold/italic * or _
    text = re.sub(r"[*_~#]", "", text)
    # Replace multiple spaces/newlines
    text = re.sub(r"\s+", " ", text).strip()
    return text


class AgentRunner:
    """Manages AI agent interactions using AGY CLI or Dev Fallback."""

    def __init__(self):
        self.agy_bin = find_agy_binary()
        if self.agy_bin:
            logger.info(f"AGY CLI binary detected at: {self.agy_bin}")
        else:
            logger.info("AGY CLI binary not found in PATH. Operating in Dev/Simulation Fallback mode.")

    async def generate_response_stream(self, prompt: str) -> AsyncGenerator[str, None]:
        """Streams tokens from AGY or fallback simulation agent."""
        if self.agy_bin:
            try:
                # Run agy with prompt, stream-json output, and skip permissions for autonomous server mode
                cmd = [
                    self.agy_bin,
                    "-p",
                    f"{RELOBOT_SYSTEM_PROMPT}\n\nUser: {prompt}",
                    "--output-format",
                    "stream-json",
                    "--dangerously-skip-permissions"
                ]
                proc = await asyncio.create_subprocess_exec(
                    *cmd,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE
                )

                while True:
                    line = await proc.stdout.readline()
                    if not line:
                        break
                    raw_str = line.decode("utf-8").strip()
                    if not raw_str:
                        continue

                    # Try parsing as stream-json event
                    try:
                        event_data = json.loads(raw_str)
                        event_type = event_data.get("event", "")
                        
                        # Handle step update / token delta events
                        if event_type == "step_update":
                            content = event_data.get("delta") or event_data.get("content") or event_data.get("text", "")
                            if content:
                                yield content
                        # Handle final result event (if tokens weren't already streamed)
                        elif event_type == "result":
                            result_text = event_data.get("result", {}).get("text", "") if isinstance(event_data.get("result"), dict) else event_data.get("text", "")
                            if result_text:
                                yield result_text
                    except json.JSONDecodeError:
                        # Fallback for plain text streams
                        yield raw_str + "\n"

                await proc.wait()
                return
            except Exception as e:
                logger.error(f"Error invoking AGY binary: {e}")
                yield f"\n[AGY error: {e}. Switching to internal assistant.]\n"

        # Developer / Simulation Fallback mode
        async for chunk in self._fallback_stream(prompt):
            yield chunk

    async def _fallback_stream(self, prompt: str) -> AsyncGenerator[str, None]:
        """Provides rich contextual responses for local dev / simulation testing."""
        p_lower = prompt.lower()
        if any(w in p_lower for w in ["who are you", "who r u", "identity", "name"]):
            reply = "I am ReloBot AI, Autobot guardian and autonomous mower. All systems are online and standing by for your command."
        elif any(w in p_lower for w in ["battery", "charge", "voltage", "power"]):
            reply = "Main power cells are operating within nominal parameters at 25.4 Volts. Ready for long-range patrol."
        elif any(w in p_lower for w in ["mow", "grass", "blade", "cut"]):
            reply = "Mower cutting deck is primed. Blade motors calibrated up to 3000 RPM. Awaiting zone authorization."
        elif any(w in p_lower for w in ["dock", "home", "charge station"]):
            reply = "Initiating docking maneuver to ReloBot charging station. Aligning optical tags and contact plates."
        elif any(w in p_lower for w in ["explore", "map", "slam"]):
            reply = "LiDAR and visual SLAM mapping initialized. Commencing autonomous frontier exploration."
        elif any(w in p_lower for w in ["stop", "halt", "emergency"]):
            reply = "Emergency stop engaged! Drive actuators and blade motors halted immediately."
        else:
            reply = f"Acknowledged: '{prompt}'. ReloBot navigation core and sensors are active and ready. Autobots, roll out!"

        # Stream words with slight delay for realistic token streaming
        words = reply.split(" ")
        for i, word in enumerate(words):
            yield word + (" " if i < len(words) - 1 else "")
            await asyncio.sleep(0.04)


class VoiceChatServer:
    """WebSocket server coordinating AI chat, live transcription, and Piper TTS."""

    def __init__(self):
        self.agent = AgentRunner()
        self.tts: Optional[OptimusTTS] = None
        self._init_tts()

    def _init_tts(self):
        try:
            self.tts = OptimusTTS()
            logger.info("Optimus Prime Piper TTS successfully loaded and ready.")
        except Exception as e:
            logger.warning(f"Piper TTS could not be initialized: {e}. Voice synthesis will be disabled.")
            self.tts = None

    async def handle_connection(self, websocket):
        logger.info(f"Client connected from {websocket.remote_address}")
        active_task: Optional[asyncio.Task] = None

        try:
            async for message_str in websocket:
                try:
                    data = json.loads(message_str)
                except Exception:
                    continue

                msg_type = data.get("type", "")

                if msg_type == "ping":
                    await websocket.send(json.dumps({"type": "pong"}))
                    continue

                elif msg_type == "status":
                    await websocket.send(json.dumps({
                        "type": "status",
                        "has_agy": self.agent.agy_bin is not None,
                        "has_tts": self.tts is not None,
                        "ready": True
                    }))
                    continue

                elif msg_type == "cancel" or msg_type == "stop":
                    if active_task and not active_task.done():
                        active_task.cancel()
                        logger.info("Generation cancelled by client.")
                    await websocket.send(json.dumps({"type": "cancelled"}))
                    continue

                elif msg_type == "prompt":
                    prompt = data.get("text", "").strip()
                    msg_id = data.get("msg_id", "default")
                    play_robot = data.get("play_robot_audio", True)
                    stream_browser = data.get("stream_browser_audio", True)

                    if not prompt:
                        continue

                    # Cancel any prior active generation on this socket
                    if active_task and not active_task.done():
                        active_task.cancel()

                    active_task = asyncio.create_task(
                        self._process_prompt(websocket, prompt, msg_id, play_robot, stream_browser)
                    )

        except Exception as e:
            logger.info(f"Client connection closed: {e}")
        finally:
            if active_task and not active_task.done():
                active_task.cancel()

    async def _process_prompt(
        self,
        websocket,
        prompt: str,
        msg_id: str,
        play_robot: bool,
        stream_browser: bool
    ):
        """Processes a prompt: streams tokens and concurrently synthesizes sentence chunks via Piper TTS."""
        logger.info(f"Processing prompt [{msg_id}]: {prompt[:50]}...")
        await websocket.send(json.dumps({"type": "start", "msg_id": msg_id}))

        full_text = ""
        sentence_buffer = ""
        
        # Audio stream session on robot speaker
        robot_audio_stream = None
        if play_robot and self.tts:
            try:
                robot_audio_stream = self.tts.create_stream_session(player_adapter=player)
            except Exception as e:
                logger.warning(f"Could not open continuous audio stream: {e}")

        # Regex for sentence split
        sentence_end_pattern = re.compile(r"([.!?\n]+)\s*")

        try:
            async for token in self.agent.generate_response_stream(prompt):
                full_text += token
                sentence_buffer += token

                # Send text token to frontend
                await websocket.send(json.dumps({
                    "type": "token",
                    "msg_id": msg_id,
                    "text": token
                }))

                # Check if we have a full sentence or clause to synthesize
                match = sentence_end_pattern.search(sentence_buffer)
                if match:
                    split_idx = match.end()
                    clause = sentence_buffer[:split_idx].strip()
                    sentence_buffer = sentence_buffer[split_idx:]

                    if clause:
                        await self._synthesize_clause(
                            clause,
                            msg_id,
                            websocket,
                            robot_audio_stream,
                            stream_browser
                        )

            # Synthesize any remaining sentence buffer
            remaining = sentence_buffer.strip()
            if remaining:
                await self._synthesize_clause(
                    remaining,
                    msg_id,
                    websocket,
                    robot_audio_stream,
                    stream_browser
                )

            # Signal completion
            await websocket.send(json.dumps({
                "type": "done",
                "msg_id": msg_id,
                "full_text": full_text
            }))

        except asyncio.CancelledError:
            logger.info(f"Prompt task [{msg_id}] cancelled.")
        except Exception as e:
            logger.error(f"Error processing prompt [{msg_id}]: {e}")
            await websocket.send(json.dumps({
                "type": "error",
                "msg_id": msg_id,
                "error": str(e)
            }))
        finally:
            if robot_audio_stream:
                try:
                    robot_audio_stream.close()
                except Exception:
                    pass

    async def _synthesize_clause(
        self,
        clause: str,
        msg_id: str,
        websocket,
        robot_audio_stream,
        stream_browser: bool
    ):
        """Synthesizes a text clause for robot speaker and/or browser stream."""
        clean_text = clean_markdown_for_speech(clause)
        if not clean_text or not self.tts:
            return

        # 1. Feed to physical robot speaker (PulseAudio / ALSA)
        if robot_audio_stream:
            try:
                # Offload synthesis to thread
                await asyncio.to_thread(robot_audio_stream.feed_text, clean_text)
            except Exception as e:
                logger.warning(f"Robot audio feed error: {e}")

        # 2. Synthesize WAV for browser Web Audio
        if stream_browser:
            try:
                wav_bytes = await asyncio.to_thread(self.tts.synthesize_wav_bytes, clean_text)
                if wav_bytes:
                    b64_audio = base64.b64encode(wav_bytes).decode("utf-8")
                    await websocket.send(json.dumps({
                        "type": "audio",
                        "msg_id": msg_id,
                        "audio": b64_audio,
                        "sample_rate": self.tts.sample_rate
                    }))
            except Exception as e:
                logger.warning(f"Browser audio streaming error: {e}")


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
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Server terminated by user.")

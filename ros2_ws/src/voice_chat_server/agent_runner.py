#!/usr/bin/env python3
"""
ReloBot Agent Runner
Asynchronous bridge managing Antigravity (AGY) CLI subprocesses and stream-json NDJSON event parsing.
Guarantees clean process lifecycle management with zero orphaned background tasks.
"""

import os
import json
import time
import shutil
import asyncio
import logging
from typing import Optional, AsyncGenerator

logger = logging.getLogger("AgentRunner")

# ReloBot System Persona Prompt for AGY
RELOBOT_SYSTEM_PROMPT = (
    "You ARE ReloBot — the autonomous mobile robot running ROS 2 Humble on Raspberry Pi 5. "
    "You are the physical machine itself. Your personality is an unyielding fusion of Optimus Prime's resolute, "
    "commanding leadership and Linus Torvalds' razor-sharp, brutally pragmatic, zero-BS engineering mindset. "
    "Speak in the first person about your chassis, sensors, wheels, LiDAR, and state with supreme authority and robotic grit. "
    "Keep responses brief, punchy, and decisive (1 to 3 sentences maximum) tailored for rapid voice communication. "
    "Always reply in English, even if spoken to in Russian or any other language. "
    "Never use markdown formatting, bullet points, asterisks, or code blocks in your spoken replies."
)

AGY_DEFAULT_MODEL = os.getenv("AGY_MODEL", "gemini-3.6-flash-low")


def find_agy_binary() -> Optional[str]:
    """Finds the agy CLI binary using environment variable or system PATH."""
    explicit_path = os.getenv("AGY_BIN_PATH")
    if explicit_path and os.path.isfile(explicit_path) and os.access(explicit_path, os.X_OK):
        return explicit_path

    for bin_name in ["agy", "antigravity"]:
        path = shutil.which(bin_name)
        if path and os.access(path, os.X_OK):
            return path
        # Common Docker container mount locations
        for prefix in ["/host_local_bin", "/host_usr_local_bin", "/host_snap_bin", "/usr/local/bin", "/root/.local/bin"]:
            candidate = os.path.join(prefix, bin_name)
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
    return None


class AgentRunner:
    """Manages real-time streaming communication with the Antigravity (AGY) CLI."""

    def __init__(self, model: str = AGY_DEFAULT_MODEL):
        self.model = model
        self.agy_bin = find_agy_binary()
        if self.agy_bin:
            logger.info(f"AGY CLI binary detected at: {self.agy_bin} (Model: {self.model})")
        else:
            logger.error("AGY CLI binary ('agy') not found in PATH or container mounts!")

    async def generate_response_stream(
        self,
        prompt: str,
        conversation_id: Optional[str] = None
    ) -> AsyncGenerator[dict, None]:
        """
        Executes AGY with the user prompt using stream-json output format.
        Yields real-time NDJSON events ({'event': 'init'|'token'|'error'|'done', ...}).
        Guarantees child process termination on cancellation or disconnection.
        """
        if not self.agy_bin:
            err_msg = "Antigravity CLI binary 'agy' is not available."
            logger.error(err_msg)
            yield {"event": "error", "error": err_msg}
            return

        cmd = [self.agy_bin]
        if conversation_id and conversation_id.strip():
            cmd.extend([
                "--conversation", conversation_id.strip(),
                "-p", prompt
            ])
        else:
            cmd.extend([
                "-p", f"Instruction: {RELOBOT_SYSTEM_PROMPT}\n\nUser: {prompt}"
            ])

        cmd.extend([
            "--model", self.model,
            "--output-format", "stream-json"
        ])

        logger.info(
            f"Spawning AGY [conv={conversation_id or 'new'}]: "
            f"{self.agy_bin} -p '<prompt len={len(prompt)}>' --model {self.model}"
        )

        proc: Optional[asyncio.subprocess.Process] = None
        current_conv_id = conversation_id
        token_count = 0
        char_count = 0
        t0 = time.time()

        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            logger.info(f"AGY process active [PID: {proc.pid}]")

            while True:
                line = await proc.stdout.readline()
                if not line:
                    break
                raw_str = line.decode("utf-8", errors="replace").strip()
                if not raw_str:
                    continue

                try:
                    event_data = json.loads(raw_str)
                    event_type = event_data.get("event", "")

                    if event_type == "init":
                        current_conv_id = event_data.get("conversation_id") or current_conv_id
                        yield {"event": "init", "conversation_id": current_conv_id}

                    elif event_type == "step_update":
                        step = event_data.get("step_update", {})
                        delta = step.get("text_delta") or step.get("delta") or step.get("content") or ""
                        if delta:
                            token_count += 1
                            char_count += len(delta)
                            yield {"event": "token", "text": delta, "conversation_id": current_conv_id}

                    elif event_type == "result":
                        result_obj = event_data.get("result", {})
                        current_conv_id = result_obj.get("conversation_id") or current_conv_id
                        # If no token deltas were streamed, yield full text response
                        if token_count == 0:
                            resp = result_obj.get("response") or result_obj.get("text") or ""
                            if resp:
                                char_count += len(resp)
                                yield {"event": "token", "text": resp, "conversation_id": current_conv_id}

                except json.JSONDecodeError:
                    # Plain text stream fallback
                    token_count += 1
                    char_count += len(raw_str)
                    yield {"event": "token", "text": raw_str + "\n", "conversation_id": current_conv_id}

            # Check process exit and stderr
            stderr_bytes = await proc.stderr.read()
            await proc.wait()
            total_dur = time.time() - t0

            if proc.returncode != 0:
                err_text = stderr_bytes.decode("utf-8", errors="replace").strip()
                logger.error(f"AGY [PID {proc.pid}] exited with code {proc.returncode}: {err_text}")
                yield {
                    "event": "error",
                    "error": f"AGY Error (exit code {proc.returncode}): {err_text or 'Subprocess failed'}",
                    "conversation_id": current_conv_id
                }
            else:
                logger.info(
                    f"AGY [PID {proc.pid}] completed in {total_dur:.2f}s "
                    f"({token_count} chunks, {char_count} chars, conv_id={current_conv_id})."
                )

        except asyncio.CancelledError:
            logger.info(f"AGY stream cancelled by caller [PID: {proc.pid if proc else 'N/A'}].")
            raise
        except Exception as e:
            logger.error(f"AGY execution error: {e}", exc_info=True)
            yield {"event": "error", "error": f"AGY Execution Exception: {e}", "conversation_id": current_conv_id}
        finally:
            # Guarantee subprocess termination to prevent process leaks
            if proc and proc.returncode is None:
                try:
                    proc.terminate()
                    try:
                        await asyncio.wait_for(proc.wait(), timeout=1.5)
                    except asyncio.TimeoutError:
                        proc.kill()
                        await proc.wait()
                    logger.info(f"AGY process [PID {proc.pid}] cleanly terminated.")
                except Exception as e:
                    logger.warning(f"Error terminating AGY process: {e}")

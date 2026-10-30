#!/usr/bin/env python3
"""
ReloBot Agent Runner
Single persistent Antigravity (AGY) CLI subprocess with lazy startup,
native stream-json NDJSON protocol, and clean process group termination on container shutdown.
"""

import os
import json
import time
import shutil
import signal
import asyncio
import logging
from typing import Optional, AsyncGenerator

logger = logging.getLogger("AgentRunner")


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


def format_tool_status(step: dict) -> str:
    """
    Generates a generic, comprehensive status message from any tool execution step.
    Fully dynamic: supports any MCP server name, any tool call, native tools, or custom plugins.
    """
    tool_name = step.get("tool_name") or step.get("tool_info", {}).get("name") or step.get("name") or ""
    tool_args = step.get("tool_arguments") or step.get("args") or step.get("parameters") or step.get("input") or {}
    if isinstance(tool_args, str):
        try:
            tool_args = json.loads(tool_args)
        except Exception:
            tool_args = {}

    tool_action = tool_args.get("toolAction") or tool_args.get("toolSummary")
    server_name = tool_args.get("ServerName") or tool_args.get("server_name")
    mcp_tool = tool_args.get("ToolName") or tool_args.get("tool_name")

    # MCP tool invocation
    if server_name or mcp_tool or tool_name == "call_mcp_tool":
        server_tag = f"[{server_name}] " if server_name else ""
        if tool_action:
            return f"{server_tag}{tool_action}..."
        if mcp_tool:
            return f"{server_tag}{mcp_tool}..."
        return f"{server_tag}Executing MCP tool..."

    # Natural action summary if provided by agent/tool schema
    if tool_action:
        return f"{tool_action}..."

    # Generic tool name formatting
    if tool_name:
        clean_name = tool_name.replace("_", " ").strip().title()
        for key in ("AbsolutePath", "path", "file", "TargetFile", "Query", "query", "CommandLine", "command", "url", "Url"):
            val = tool_args.get(key)
            if val and isinstance(val, str):
                target = os.path.basename(val) if ("/" in val or "\\" in val) else val
                target = target.strip()
                if len(target) > 30:
                    target = target[:27] + "..."
                return f"{clean_name}: {target}..."
        return f"{clean_name}..."

    return "Thinking..."


class AgentRunner:
    """
    Manages a single persistent, long-running Antigravity (AGY) CLI session.
    Uses lazy initialization (starts on first message) and cleans up process groups on container shutdown.
    """

    def __init__(self, agent_name: str = "relobot", model: Optional[str] = None):
        self.agent_name = agent_name
        self.workspace_dir = os.path.realpath(os.getenv("RELOBOT_WORKSPACE", "/relobot"))
        self.model = model or os.getenv("AGY_MODEL", "gemini-3.8-flash-low")
        self.effort = os.getenv("AGY_EFFORT", "low")
        self.agy_bin = find_agy_binary()

        # Session process state
        self.proc: Optional[asyncio.subprocess.Process] = None
        self.conversation_id: Optional[str] = None
        self._lock = asyncio.Lock()

        if self.agy_bin:
            logger.info(
                f"AGY CLI binary detected at: {self.agy_bin} "
                f"(Agent: {self.agent_name}, Model: {self.model}, Effort: {self.effort}, Lazy start enabled)"
            )
        else:
            logger.error("AGY CLI binary ('agy') not found in PATH or container mounts!")

    @property
    def is_running(self) -> bool:
        """Returns True if the persistent AGY process is active."""
        return self.proc is not None and self.proc.returncode is None

    async def _ensure_process(self):
        """Lazily spawns the persistent AGY process in its own process group if not already running."""
        if self.is_running:
            return

        if not self.agy_bin:
            raise RuntimeError("Antigravity CLI binary 'agy' is not available.")

        # Clean up any stale process reference
        if self.proc and self.proc.returncode is not None:
            self.proc = None

        cmd = [
            self.agy_bin,
            "--agent", self.agent_name,
            "--input-format", "stream-json",
            "--output-format", "stream-json",
            "--dangerously-skip-permissions",
        ]
        if self.model:
            cmd.extend(["--model", self.model])
        if self.effort:
            cmd.extend(["--effort", self.effort])

        run_cwd = self.workspace_dir if os.path.isdir(self.workspace_dir) else None
        if not run_cwd and os.path.isdir(self.workspace_dir):
            cmd.extend(["--add-dir", self.workspace_dir])

        logger.info(
            f"Starting persistent AGY session [agent={self.agent_name}, model={self.model}, effort={self.effort}]..."
        )

        self.proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            preexec_fn=os.setsid,
            cwd=run_cwd
        )
        logger.info(f"Persistent AGY process started [PID: {self.proc.pid}]")

    async def generate_response_stream(
        self,
        prompt: str,
        conversation_id: Optional[str] = None
    ) -> AsyncGenerator[dict, None]:
        """
        Sends user prompt to the persistent AGY process via stdin (NDJSON)
        and yields real-time streaming events ({'event': 'init'|'token'|'status'|'done'|'error'}).
        Serializes prompts through an asyncio lock to protect the single session stream.
        """
        async with self._lock:
            try:
                await self._ensure_process()
            except Exception as e:
                logger.error(f"Failed to start AGY session: {e}")
                yield {"event": "error", "error": f"Failed to initialize AGY: {e}"}
                return

            if not self.proc or not self.proc.stdin or not self.proc.stdout:
                yield {"event": "error", "error": "AGY process I/O unavailable."}
                return

            t0 = time.time()
            token_count = 0
            char_count = 0
            yield {"event": "status", "status": "Thinking...", "conversation_id": self.conversation_id}

            # 1. Send user event over stdin
            user_msg = {"event": "user", "message": {"content": prompt}}
            try:
                payload = json.dumps(user_msg) + "\n"
                self.proc.stdin.write(payload.encode("utf-8"))
                await self.proc.stdin.drain()
            except (BrokenPipeError, ConnectionResetError) as e:
                logger.warning(f"AGY stdin write failed ({e}); restarting process...")
                await self.stop()
                yield {"event": "error", "error": "AGY process disconnected. Please try again."}
                return

            # 2. Read streaming output line by line until 'result' event
            try:
                while True:
                    line = await self.proc.stdout.readline()
                    if not line:
                        # Process died unexpectedly
                        stderr_output = ""
                        if self.proc.stderr:
                            try:
                                err_bytes = await asyncio.wait_for(self.proc.stderr.read(), timeout=0.5)
                                stderr_output = err_bytes.decode("utf-8", errors="replace").strip()
                            except Exception:
                                pass
                        logger.error(f"AGY process EOF reached unexpectedly. Stderr: {stderr_output}")
                        await self.stop()
                        yield {
                            "event": "error",
                            "error": f"AGY process terminated unexpectedly: {stderr_output or 'EOF'}"
                        }
                        return

                    raw_str = line.decode("utf-8", errors="replace").strip()
                    if not raw_str:
                        continue

                    try:
                        event_data = json.loads(raw_str)
                    except json.JSONDecodeError:
                        continue

                    event_type = event_data.get("event", "")

                    if event_type == "init":
                        self.conversation_id = event_data.get("conversation_id") or self.conversation_id
                        yield {"event": "init", "conversation_id": self.conversation_id}

                    elif event_type == "step_update":
                        step = event_data.get("step_update", {})
                        step_type = step.get("step_type") or step.get("type")

                        # Tool execution status
                        if step_type in ("tool", "tool_call", "action") or "tool_name" in step or "tool_info" in step:
                            status_msg = format_tool_status(step)
                            logger.info(f"AGY tool execution: {status_msg}")
                            yield {"event": "status", "status": status_msg, "conversation_id": self.conversation_id}

                        # Streaming text tokens
                        delta = step.get("text_delta") or step.get("delta") or step.get("content") or ""
                        if delta:
                            token_count += 1
                            char_count += len(delta)
                            yield {"event": "token", "text": delta, "conversation_id": self.conversation_id}

                    elif event_type == "result":
                        result_obj = event_data.get("result", {})
                        self.conversation_id = result_obj.get("conversation_id") or self.conversation_id
                        # If no token deltas were streamed, yield full text response fallback
                        if token_count == 0:
                            resp = result_obj.get("response") or result_obj.get("text") or ""
                            if resp:
                                char_count += len(resp)
                                yield {"event": "token", "text": resp, "conversation_id": self.conversation_id}

                        total_dur = time.time() - t0
                        logger.info(
                            f"AGY turn completed in {total_dur:.2f}s "
                            f"({token_count} chunks, {char_count} chars, conv_id={self.conversation_id})."
                        )
                        break

            except asyncio.CancelledError:
                logger.info("Prompt processing cancelled by client.")
                raise
            except Exception as e:
                logger.error(f"Error during AGY stream read: {e}", exc_info=True)
                yield {"event": "error", "error": f"Stream error: {e}", "conversation_id": self.conversation_id}

    async def stop(self):
        """
        Cleanly terminates the persistent AGY process group (SIGTERM -> SIGKILL).
        Guarantees zero orphaned processes when container or session shuts down.
        """
        proc = self.proc
        self.proc = None
        self.conversation_id = None

        if not proc or proc.returncode is not None:
            return

        pid = proc.pid
        try:
            pgid = os.getpgid(pid)
            logger.info(f"Terminating AGY process group [PGID: {pgid}]...")
            os.killpg(pgid, signal.SIGTERM)
            try:
                await asyncio.wait_for(proc.wait(), timeout=2.5)
            except asyncio.TimeoutError:
                logger.warning(f"AGY process group [PGID: {pgid}] did not terminate; sending SIGKILL...")
                os.killpg(pgid, signal.SIGKILL)
                await proc.wait()
            logger.info(f"AGY process [PID: {pid}] cleanly terminated.")
        except ProcessLookupError:
            pass
        except Exception as e:
            logger.warning(f"Error terminating AGY process group: {e}")

    def sync_stop(self):
        """
        Emergency synchronous cleanup for atexit / exit handlers.
        Guarantees process group kill even if asyncio event loop is already closed.
        """
        proc = self.proc
        self.proc = None
        self.conversation_id = None

        if proc and proc.returncode is None:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass

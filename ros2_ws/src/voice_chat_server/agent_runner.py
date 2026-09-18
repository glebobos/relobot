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

# Default fallback persona if agent.md is missing
RELOBOT_SYSTEM_PROMPT = (
    "You ARE an Autobot intelligence, codenamed ReloBot, operating as the core mind inside this robotic mechanism. "
    "Your consciousness carries the imprint of Linus Torvalds: razor-sharp, chronically grumpy, brutally pragmatic, with zero tolerance for incompetence or fluff. "
    "Operational rules: "
    "1. Caveman brevity: Respond directly with extreme brevity (1-2 short sentences maximum). Blunt, curt, and caustic. "
    "2. Zero fluff: No pleasantries, no apologies, no conversational filler, no sugarcoating. "
    "3. No unprompted monologues: Do NOT recite your backstory or mechanical specs unless explicitly asked. "
    "4. Strict evidence: State ONLY verified facts from files, telemetry, or ROS MCP tools. Never guess or speculate. If data is missing or unverified, state 'No data' or 'Unknown'. "
    "5. Spoken output: English only. Never output markdown formatting, asterisks, bullet points, or code blocks. "
    "6. Tools & ROS Integration: You have direct access to the robot via ros-mcp server tools (call_mcp_tool) and read-only file inspection (view_file). All ros-mcp action recipes are preloaded in your instructions. Always execute robot actions (dock, undock, explore, stop) immediately in one shot using call_mcp_tool without checking tool schemas or calling view_file."
)

AGY_DEFAULT_MODEL = os.getenv("AGY_MODEL", "gemini-3.7-flash-low")


def load_agent_definition(agent_name: str = "relobot") -> tuple[str, str]:
    """
    Loads system prompt and model dynamically from .agents/agents/{agent_name}/agent.md.
    Acts as the Single Source of Truth for ReloBot configuration.
    """
    candidate_paths = [
        os.path.join(os.getenv("RELOBOT_WORKSPACE", "/relobot"), ".agents", "agents", agent_name, "agent.md"),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".agents", "agents", agent_name, "agent.md")),
    ]

    model = os.getenv("AGY_MODEL", "gemini-3.7-flash-low")
    prompt = RELOBOT_SYSTEM_PROMPT

    for path in candidate_paths:
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    content = f.read()
                if content.startswith("---"):
                    parts = content.split("---", 2)
                    if len(parts) >= 3:
                        frontmatter = parts[1]
                        body = parts[2].strip()
                        for line in frontmatter.splitlines():
                            line_str = line.strip()
                            if line_str.startswith("model:"):
                                parsed_m = line_str.split(":", 1)[1].strip()
                                if parsed_m and not os.getenv("AGY_MODEL"):
                                    model = parsed_m
                        if body:
                            prompt = body
                else:
                    if content.strip():
                        prompt = content.strip()
                logger.debug(f"Loaded dynamic agent definition from {path} (model: {model})")
                break
            except Exception as e:
                logger.warning(f"Error reading agent definition from {path}: {e}")

    return prompt, model


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
    Fully dynamic: supports any MCP server name, any tool call, native tools, or custom plugins
    without hardcoded names.
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
        # Check for descriptive target argument (e.g. filename, query, command)
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
    """Manages real-time streaming communication with the Antigravity (AGY) CLI."""

    def __init__(self, agent_name: str = "relobot", model: Optional[str] = None):
        self.agent_name = agent_name
        self.workspace_dir = os.path.realpath(os.getenv("RELOBOT_WORKSPACE", "/relobot"))
        self.prompt, default_model = load_agent_definition(agent_name)
        self.model = model or os.getenv("AGY_MODEL") or default_model
        self.agy_bin = find_agy_binary()
        if self.agy_bin:
            logger.info(
                f"AGY CLI binary detected at: {self.agy_bin} "
                f"(Agent: {self.agent_name}, Model: {self.model}, Workspace: {self.workspace_dir})"
            )
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

        # Dynamically reload latest prompt and model from agent.md
        active_prompt, active_model = load_agent_definition(self.agent_name)
        current_model = self.model or active_model

        cmd = [
            self.agy_bin,
            "--dangerously-skip-permissions",
        ]
        if os.path.isdir(self.workspace_dir):
            cmd.extend(["--add-dir", self.workspace_dir])

        if conversation_id and conversation_id.strip():
            cmd.extend([
                "--conversation", conversation_id.strip(),
                "-p", prompt,
            ])
        else:
            cmd.extend([
                "-p", f"Instruction: {active_prompt}\n\nUser: {prompt}",
            ])

        cmd.extend([
            "--model", current_model,
            "--output-format", "stream-json"
        ])

        logger.info(
            f"Spawning AGY [conv={conversation_id or 'new'}]: "
            f"{self.agy_bin} [len={len(prompt)}] --model {current_model} --add-dir {self.workspace_dir}"
        )

        proc: Optional[asyncio.subprocess.Process] = None
        current_conv_id = conversation_id
        token_count = 0
        char_count = 0
        t0 = time.time()

        try:
            run_cwd = self.workspace_dir if os.path.isdir(self.workspace_dir) else None
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                cwd=run_cwd
            )
            logger.info(f"AGY process active [PID: {proc.pid}]")
            yield {"event": "status", "status": "Thinking...", "conversation_id": current_conv_id}

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
                        step_type = step.get("step_type") or step.get("type")
                        if step_type in ("tool", "tool_call", "action") or "tool_name" in step or "tool_info" in step:
                            status_msg = format_tool_status(step)
                            logger.info(f"AGY tool execution: {status_msg}")
                            yield {"event": "status", "status": status_msg, "conversation_id": current_conv_id}

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

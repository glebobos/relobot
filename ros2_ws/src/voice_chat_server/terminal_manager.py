#!/usr/bin/env python3
"""
ReloBot Terminal Manager
Supervises the interactive Antigravity (AGY) Web Terminal (ttyd + tmux)
running with full permissions inside the canonical ReloBot workspace.
"""

import os
import shlex
import shutil
import asyncio
import logging
import subprocess
from typing import Optional

from agent_runner import find_agy_binary

logger = logging.getLogger("TerminalManager")

DEFAULT_TERMINAL_PORT = int(os.getenv("AGY_TERMINAL_PORT", "7681"))
TMUX_SOCKET = "agy-relobot"


class TerminalManager:
    """Manages the background ttyd web terminal lifecycle running interactive AGY CLI."""

    def __init__(self, port: int = DEFAULT_TERMINAL_PORT):
        self.port = port
        self.proc: Optional[asyncio.subprocess.Process] = None
        self._supervisor_task: Optional[asyncio.Task] = None
        self.workspace_dir = self._resolve_workspace()
        self.agy_bin = find_agy_binary()
        self.ttyd_bin = self._find_ttyd_binary()
        self._setup_environment()
        self.launcher_script = self._write_launcher_script()

    def _resolve_workspace(self) -> str:
        """
        Resolves and strictly validates the workspace path.
        Checks RELOBOT_WORKSPACE or defaults to canonical /relobot.
        Raises RuntimeError if the directory does not exist.
        """
        candidate = os.getenv("RELOBOT_WORKSPACE", "/relobot")
        if not os.path.isdir(candidate):
            raise RuntimeError(
                f"Configured workspace directory '{candidate}' does not exist or is not a directory. "
                "Ensure ..:/relobot is mounted or RELOBOT_WORKSPACE is set to a valid directory."
            )
        return os.path.realpath(candidate)

    def _find_ttyd_binary(self) -> Optional[str]:
        """Locates ttyd binary in system PATH or standard binary paths."""
        bin_path = shutil.which("ttyd")
        if bin_path and os.access(bin_path, os.X_OK):
            return bin_path

        for candidate in ["/usr/local/bin/ttyd", "/usr/bin/ttyd", "/tmp/ttyd"]:
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
        return None

    def _setup_environment(self):
        """Prepares workspace symlinks, Git safe directories, and isolated tmux options."""
        # 1. If host path is provided, create a symlink to the canonical workspace
        host_dir = os.getenv("RELOBOT_HOST_DIR")
        if host_dir and host_dir != self.workspace_dir:
            try:
                parent = os.path.dirname(host_dir)
                os.makedirs(parent, exist_ok=True)
                if not os.path.exists(host_dir):
                    os.symlink(self.workspace_dir, host_dir)
                    logger.info("Created host path symlink: %s -> %s", host_dir, self.workspace_dir)
            except Exception as e:
                logger.warning("Could not symlink host path %s: %s", host_dir, e)

        # 2. Configure Git safe directory idempotently without duplicating entries
        dirs_to_register = [self.workspace_dir]
        if host_dir and host_dir != self.workspace_dir:
            dirs_to_register.append(host_dir)

        existing_safe = []
        try:
            res = subprocess.run(
                ["git", "config", "--global", "--get-all", "safe.directory"],
                capture_output=True,
                text=True,
                check=False
            )
            if res.returncode == 0:
                existing_safe = [line.strip() for line in res.stdout.splitlines() if line.strip()]
        except Exception:
            pass

        for directory in dirs_to_register:
            if directory not in existing_safe:
                res = subprocess.run(
                    ["git", "config", "--global", "--add", "safe.directory", directory],
                    capture_output=True,
                    text=True,
                    check=False
                )
                if res.returncode != 0:
                    logger.warning("Failed to configure git safe.directory for %s: %s", directory, res.stderr.strip())

        # 3. Configure zero-delay escape time on an isolated tmux socket without touching user ~/.tmux.conf
        try:
            subprocess.run(
                ["tmux", "-L", TMUX_SOCKET, "start-server"],
                capture_output=True,
                check=False
            )
            subprocess.run(
                ["tmux", "-L", TMUX_SOCKET, "set-option", "-s", "escape-time", "0"],
                capture_output=True,
                check=False
            )
            subprocess.run(
                ["tmux", "-L", TMUX_SOCKET, "set-option", "-s", "focus-events", "off"],
                capture_output=True,
                check=False
            )
        except Exception as e:
            logger.warning("Could not set isolated tmux options: %s", e)

    @property
    def is_available(self) -> bool:
        """Returns True if both ttyd and agy binaries are detected."""
        return bool(self.ttyd_bin and self.agy_bin)

    @property
    def is_running(self) -> bool:
        """Returns True if the ttyd server process and supervisor are currently active."""
        if not self._supervisor_task or self._supervisor_task.done():
            return False
        return self.proc is not None and self.proc.returncode is None

    def _write_launcher_script(self) -> str:
        """
        Writes a clean, dedicated launcher script avoiding nested quote escaping issues.
        Includes cooldown on abnormal exit to prevent crash loops.
        """
        script_path = "/tmp/relobot_agy_launcher.sh"
        quoted_agy = shlex.quote(self.agy_bin) if self.agy_bin else "agy"
        quoted_workspace = shlex.quote(self.workspace_dir)

        # Default to fast, low-thinking model (gemini-3.7-flash-low) to guarantee instant responses
        agy_model = os.getenv("AGY_MODEL", "gemini-3.7-flash-low")
        agy_effort = os.getenv("AGY_EFFORT", "low")

        model_flag = f" --model {shlex.quote(agy_model)}"
        effort_flag = f" --effort {shlex.quote(agy_effort)}"

        content = (
            "#!/usr/bin/env bash\n"
            "set -u\n"
            "export PATH=\"/usr/local/bin:/root/.local/bin:/host_local_bin:${PATH:-}\"\n"
            f"cd {quoted_workspace}\n"
            "while true; do\n"
            "    echo -e '\\033[1;36m[ReloBot] Starting Antigravity (AGY) Interactive CLI...\\033[0m'\n"
            f"    echo -e '\\033[1;30mModel: {agy_model} | Effort: {agy_effort} | Workspace: {self.workspace_dir}\\033[0m\\n'\n"
            f"    {quoted_agy} --dangerously-skip-permissions{model_flag}{effort_flag}\n"
            "    exit_status=$?\n"
            "    if [ $exit_status -ne 0 ]; then\n"
            "        echo -e \"\\n\\033[1;31m[AGY exited with error code $exit_status. Cooling down 2s...]\\033[0m\"\n"
            "        sleep 2\n"
            "    fi\n"
            "    echo -e \"\\n\\033[1;33m[Press Enter to restart AGY, or type 'bash' to drop to shell...]\\033[0m\"\n"
            "    read -r -t 10 choice || choice=\"\"\n"
            "    if [ \"$choice\" = \"bash\" ] || [ \"$choice\" = \"sh\" ]; then\n"
            "        echo -e \"\\033[1;32m[Entering Bash Shell. Type 'exit' to return to AGY.]\\033[0m\"\n"
            "        /bin/bash\n"
            "    fi\n"
            "done\n"
        )

        with open(script_path, "w", encoding="utf-8") as f:
            f.write(content)
        os.chmod(script_path, 0o755)
        return script_path

    async def _supervise_ttyd(self):
        """Supervises the ttyd subprocess with exponential backoff and crash capping."""
        consecutive_crashes = 0
        max_crashes = 5
        base_backoff = 1.0

        while True:
            try:
                ttyd_cmd = [
                    self.ttyd_bin,
                    "-p", str(self.port),
                    "-b", "/agy-terminal",
                    "-W",
                    "-w", self.workspace_dir,
                    "-t", "fontSize=14",
                    "-t", "cursorBlink=true",
                    "-t", 'theme={"background": "#0b0f19", "foreground": "#e2e8f0", "cursor": "#00ff88", "selectionBackground": "#00ff8833"}',
                    "tmux", "-L", TMUX_SOCKET, "new-session", "-A", "-s", "agy-terminal",
                    self.launcher_script,
                ]

                logger.info(
                    "Spawning AGY Web Terminal on port %s (Workspace: %s, Binary: %s)...",
                    self.port, self.workspace_dir, self.agy_bin
                )

                start_time = asyncio.get_running_loop().time()
                self.proc = await asyncio.create_subprocess_exec(
                    *ttyd_cmd,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL,
                )
                logger.info("AGY Web Terminal started [PID: %s]", self.proc.pid)

                # Await process exit to reliably detect termination and reap zombies
                exit_code = await self.proc.wait()
                duration = asyncio.get_running_loop().time() - start_time
                logger.warning("AGY Web Terminal process exited with code %s after %.1fs", exit_code, duration)

                # Reset crash counter if it ran stably for >= 30 seconds
                if duration >= 30.0:
                    consecutive_crashes = 0

                consecutive_crashes += 1
                if consecutive_crashes > max_crashes:
                    logger.error("AGY Web Terminal crashed %s times consecutively; stopping supervisor.", consecutive_crashes)
                    break

                backoff = min(base_backoff * (2 ** (consecutive_crashes - 1)), 30.0)
                logger.info("Restarting AGY Web Terminal in %.1fs (attempt %s/%s)...", backoff, consecutive_crashes, max_crashes)
                await asyncio.sleep(backoff)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error("Unexpected error in AGY Web Terminal supervisor: %s", e)
                await asyncio.sleep(2.0)

    async def start(self):
        """Starts the terminal supervisor task."""
        if not self.is_available:
            logger.warning(
                "Cannot start AGY Terminal. ttyd: %s, agy: %s",
                self.ttyd_bin or 'MISSING', self.agy_bin or 'MISSING'
            )
            return

        if self.is_running:
            logger.info("AGY Terminal supervisor is already running.")
            return

        self._supervisor_task = asyncio.create_task(self._supervise_ttyd())

    async def stop(self):
        """Gracefully terminates the supervisor and reaps the ttyd and tmux session."""
        if self._supervisor_task and not self._supervisor_task.done():
            self._supervisor_task.cancel()
            try:
                await self._supervisor_task
            except asyncio.CancelledError:
                pass
            self._supervisor_task = None

        if self.proc and self.proc.returncode is None:
            logger.info("Stopping AGY Web Terminal [PID: %s]...", self.proc.pid)
            try:
                self.proc.terminate()
                await asyncio.wait_for(self.proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                logger.warning("AGY Web Terminal did not terminate within 3s; killing [PID: %s]...", self.proc.pid)
                self.proc.kill()
                await self.proc.wait()
            except Exception as e:
                logger.warning("Error terminating AGY Web Terminal process: %s", e)
                self.proc.kill()
                await self.proc.wait()

            logger.info("AGY Web Terminal stopped [exitcode: %s].", self.proc.returncode)
            self.proc = None

        # Clean up dedicated tmux session on isolated socket
        try:
            subprocess.run(
                ["tmux", "-L", TMUX_SOCKET, "kill-session", "-t", "agy-terminal"],
                capture_output=True,
                check=False
            )
        except Exception:
            pass

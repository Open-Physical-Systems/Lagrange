# SPDX-License-Identifier: Apache-2.0
"""The lagrange command: help, the tools, and the server and viewer run together."""

import os
import subprocess
from pathlib import Path

CLI = str(Path(__file__).resolve().parents[1] / "lagrange")


def run(*args: str, timeout: float = 120) -> subprocess.CompletedProcess[str]:
    return subprocess.run([CLI, *args], capture_output=True, text=True, timeout=timeout)


def test_help_names_the_samples() -> None:
    result = run("--help")
    assert result.returncode == 0
    assert "lagrange viewer" in result.stdout and "sedan" in result.stdout


def test_samples_are_listed() -> None:
    result = run("samples")
    assert result.returncode == 0
    for name in ("amr", "sedan", "quad", "lexus", "x500", "swerve"):
        assert name in result.stdout


def test_tools_go_to_the_server_cli() -> None:
    result = run("info", "--version")
    assert result.returncode == 0
    assert result.stdout.startswith("lagrange ")


def test_an_unknown_word_is_refused() -> None:
    result = run("no_such_sample")
    assert result.returncode != 0
    assert "no sample or file" in result.stderr


def test_server_and_viewer_run_together_and_stop_together(tmp_path: Path) -> None:
    # The viewer renders a few frames offscreen and exits; the server must stop with it.
    env = {**os.environ, "XDG_RUNTIME_DIR": str(tmp_path)}
    result = subprocess.run(
        [CLI, "--no-ros", "--", "--headless", "--frames", "5", "--wait-ms", "60000"],
        capture_output=True,
        text=True,
        timeout=150,
        env=env,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert '"snapshots": 1' in result.stdout, result.stdout
    assert not list((tmp_path / "lagrange").glob("run-*.sock")), "the socket is cleaned up"

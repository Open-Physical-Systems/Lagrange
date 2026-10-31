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
    for name in ("amr", "sedan", "quad", "lexus", "x500", "swerve", "people"):
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


def test_spawn_and_delete_need_their_arguments() -> None:
    assert "spawn needs a robot file" in run("spawn").stderr
    assert "no file" in run("spawn", "/no/such/robot.urdf").stderr
    assert "delete needs one entity name" in run("delete").stderr


def test_robots_are_spawned_into_an_empty_run_and_deleted(tmp_path: Path) -> None:
    # A headless empty server with ROS on a domain of its own; the CLI spawns and deletes.
    robot = Path(__file__).resolve().parents[3] / "assets/robots/amr_diffdrive/amr_diffdrive.urdf"
    env = {
        **os.environ,
        "XDG_RUNTIME_DIR": str(tmp_path),
        "ROS_DOMAIN_ID": str(100 + os.getpid() % 100),
    }
    server = subprocess.Popen(
        [CLI, "--server-only", "empty", "--listen", f"unix:{tmp_path}/s.sock"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    try:

        def cli(*args: str) -> subprocess.CompletedProcess[str]:
            return subprocess.run(
                [CLI, *args], capture_output=True, text=True, timeout=120, env=env
            )

        first = cli("spawn", str(robot), "--at", "1,2")
        assert first.returncode == 0, first.stderr
        assert first.stdout.strip() == "/amr_diffdrive"
        mate = cli("spawn", str(robot), "--name", "mate", "--at", "0,4,0,1.57")
        assert mate.returncode == 0, mate.stderr
        assert mate.stdout.strip() == "/mate"
        again = cli("spawn", str(robot), "--name", "mate")
        assert again.returncode == 1 and "already named mate" in again.stderr
        assert cli("delete", "/mate").returncode == 0
        gone = cli("delete", "/mate")
        assert gone.returncode == 1 and "no robot named mate" in gone.stderr
    finally:
        server.terminate()
        log = server.communicate(timeout=30)[0]
    assert "spawned amr_diffdrive" in log and "deleted mate" in log, log

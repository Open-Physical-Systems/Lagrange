"""The Builder workspace, headless, without a server (RFC 0016).

WP0: the workspace opens with --workspace builder and no --connect, and the launcher opens it
with Ctrl B. WP-G fills in the scripted steps of docs/plan/builder-phase1-plan.md §2.2.
"""

from __future__ import annotations

import json
import os
import subprocess

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def run(tmp_path, *extra: str) -> tuple[dict, subprocess.CompletedProcess]:
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}
    done = subprocess.run(
        [
            viewer(),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--size",
            "1600x1000",
            "--frames",
            "6",
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return (json.loads(lines[-1]) if lines else {}), done


def test_builder_opens_without_a_server(tmp_path):
    shot = tmp_path / "builder.png"
    report, done = run(tmp_path, "--workspace", "builder", "--screenshot", str(shot))
    assert done.returncode == 0, done.stderr
    assert report.get("connected") is False
    assert report.get("workspace") == "Builder"
    assert shot.stat().st_size > 0


def test_ctrl_b_on_the_launcher_opens_the_builder(tmp_path):
    shot = tmp_path / "launcher_builder.png"
    report, done = run(tmp_path, "--launcher", "--input", "2:key:ctrl+b", "--screenshot", str(shot))
    assert done.returncode == 0, done.stderr
    assert report.get("workspace") == "Builder"
    assert report.get("launcher") is False
    assert report.get("connected") is False
    assert shot.stat().st_size > 0

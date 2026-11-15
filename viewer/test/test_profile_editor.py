"""The Domains workspace's Profile panel: YAML typed, checked by the server, saved (SPEC 03)."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)

# At --size 1600x1000: an empty part of the Profile panel's text, below the last line.
PROFILE_TEXT = "1400,600"


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


@pytest.fixture
def run_view(tmp_path):
    """Starts the server with the sample AMR and gives a function running a headless viewer."""
    try:
        assets = get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive"
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    socket = f"/tmp/lagrange_profile_{os.getpid()}.sock"
    server = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            "--robot",
            f"{assets}/amr_diffdrive.urdf",
            "--world",
            "none",
            "--no-ros",
            "--no-demo",
            "--listen",
            f"unix:{socket}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(100):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}

    def run(script: str, frames: int) -> dict:
        done = subprocess.run(
            [
                tool("lagrange_viewer", "lagrange_viewer"),
                "--headless",
                "--gpu",
                os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
                "--connect",
                f"unix:{socket}",
                "--size",
                "1600x1000",
                "--frames",
                str(frames),
                "--workspace",
                "domains",
                "--input",
                script,
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
            env=environment,
        )
        assert done.returncode == 0, done.stderr
        return json.loads(done.stdout.strip().splitlines()[-1])

    yield run
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=20)
    Path(socket).unlink(missing_ok=True)


def test_typed_yaml_is_checked_by_the_server(run_view) -> None:
    report = run_view(f"30:click:{PROFILE_TEXT};40:text:bogus_key: 1", 240)
    assert report["profile_changes"] == 1
    assert report["profile"] == "invalid"


def test_ctrl_s_saves_the_typed_profile_on_this_machine(run_view, tmp_path) -> None:
    script = f"30:click:{PROFILE_TEXT};40:text:# a note ;80:key:ctrl+s;90:key:enter"
    report = run_view(script, 160)
    assert report["profile"] == "valid"
    saved = tmp_path / "cfg/lagrange/profiles/amr_diffdrive.yaml"
    text = saved.read_text()
    assert text.startswith("# a note # Lagrange simulation profile")
    assert "rigid_body: {enabled: true" in text

"""The Domains workspace's secondary tabs against a real server (SPEC 03): Modules, Coupling,
Diff and Schema, for the sample AMR and the sample quadcopter."""

from __future__ import annotations

import json
import os
import re
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

# At --size 1600x1000: an empty part of the Profile panel's text, below the last line, and
# the Diff tab beside the Profile tab.
PROFILE_TEXT = "1400,600"
DIFF_TAB = "1245,115"


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


def robot_args(robot: str) -> list[str]:
    assets = get_package_share_directory("lagrange_assets") + "/robots"
    if robot == "amr":
        return ["--robot", f"{assets}/amr_diffdrive/amr_diffdrive.urdf"]
    return [
        "--robot",
        f"{assets}/quad_x500/quad_x500.usda",
        "--air-density",
        "1.225",
        "--geo-origin",
        "47.397742,8.545594,488",
    ]


@pytest.fixture(params=["amr", "quad"])
def run_view(request, tmp_path):
    """Starts the server with a sample robot and gives a function running a headless viewer."""
    try:
        get_package_prefix("lagrange_server")
        args = robot_args(request.param)
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server and lagrange_assets")
    socket = f"/tmp/lagrange_domtabs_{os.getpid()}_{request.param}.sock"
    server = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            *args,
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

    def run(frames: int, *extra: str) -> dict:
        command = [
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
            *extra,
        ]
        shots = os.environ.get("LAGRANGE_TEST_SHOTS")
        if shots:
            name = "-".join(re.sub(r"[^a-z0-9]+", "", part.lower()) for part in extra[:2])
            command += ["--screenshot", f"{shots}/{request.param}-{name or 'default'}.png"]
        done = subprocess.run(
            command,
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


def test_first_tabs_are_in_front(run_view) -> None:
    report = run_view(120)
    windows = report["windows"]
    for title in ("Domains", "Wiring", "Schedule", "Profile"):
        assert title in windows, windows
    assert not any(w.startswith("Modules") for w in windows)


def test_modules_tab_lists_the_running_modules(run_view) -> None:
    report = run_view(180, "--panel", "###DomainsModules")
    modules = [w for w in report["windows"] if w.startswith("Modules")]
    assert modules, report["windows"]
    count = int(modules[0].split("·")[1])
    assert count >= 2  # at least physics and sensors run for both samples


def test_coupling_and_schema_tabs_open(run_view) -> None:
    assert "Coupling" in run_view(120, "--panel", "Coupling")["windows"]
    assert "Schema" in run_view(120, "--panel", "Schema")["windows"]


def test_diff_tab_counts_an_edit(run_view) -> None:
    clean = run_view(120, "--panel", "###ProfileDiff")
    assert "Diff" in clean["windows"], clean["windows"]
    # Type a comment into the profile, then click the Diff tab: one change.
    report = run_view(
        260, "--input", f"30:click:{PROFILE_TEXT};40:text:# a note ;120:click:{DIFF_TAB}"
    )
    assert report["profile_changes"] == 1
    assert "Diff · 1" in report["windows"], report["windows"]

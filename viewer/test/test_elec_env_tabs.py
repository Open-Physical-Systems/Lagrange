"""The Electrical and Environment workspaces' secondary tabs against a real server (SPEC 04 and
07): Probes, + scope and History beside Network, Scopes and Inspector for the sample AMR with
its power overlay; Viewport · fields and Field info beside Slice 2D and Overlays for the
quadcopter with EMI by the power line, in rain and fog."""

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

# At --size 1600x1000: the first signal row of the "+ scope" tab (below its filter line).
FIRST_SIGNAL_ROW = "620,700"


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


def server_args(run: str) -> list[str]:
    assets = get_package_share_directory("lagrange_assets")
    if run == "electrical":
        return []  # the sample AMR with its power-system overlay in the sample warehouse
    return [
        "--robot",
        f"{assets}/robots/quad_x500/quad_x500_emi.usda",
        "--world",
        f"{assets}/worlds/power_line/power_line.usda",
        "--air-density",
        "1.225",
        "--geo-origin",
        "47.397742,8.545594,488",
        # 25 mm/h of rain (in m/s) and 400 m of visibility, so the weather runs.
        "--weather",
        "temperature=290,pressure=101325,relative_humidity=0.8,visibility=400,rain_rate=0.0000069",
    ]


def start(run: str, tmp_path: Path):
    try:
        get_package_prefix("lagrange_server")
        args = server_args(run)
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server and lagrange_assets")
    socket = f"/tmp/lagrange_elecenv_{os.getpid()}_{run}.sock"
    Path(socket).unlink(missing_ok=True)
    server = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            *args,
            "--no-ros",
            "--listen",
            f"unix:{socket}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(150):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}

    def view(frames: int, *extra: str) -> dict:
        command = [
            tool("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--validation",
            "--connect",
            f"unix:{socket}",
            "--size",
            "1600x1000",
            "--frames",
            str(frames),
            "--workspace",
            run,
            *extra,
        ]
        shots = os.environ.get("LAGRANGE_TEST_SHOTS")
        if shots:
            name = "-".join(re.sub(r"[^a-z0-9]+", "", part.lower()) for part in extra[:2])
            command += ["--screenshot", f"{shots}/{run}-{name or 'default'}.png"]
        done = subprocess.run(
            command, capture_output=True, text=True, timeout=180, check=False, env=environment
        )
        assert done.returncode == 0, done.stderr
        report = json.loads(done.stdout.strip().splitlines()[-1])
        assert report["validation_errors"] == 0, done.stderr
        return report

    def stop() -> None:
        server.send_signal(signal.SIGTERM)
        try:
            server.wait(timeout=20)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait(5)
        Path(socket).unlink(missing_ok=True)

    return view, stop


@pytest.fixture
def electrical(tmp_path):
    view, stop = start("electrical", tmp_path)
    yield view
    stop()


@pytest.fixture
def environment(tmp_path):
    view, stop = start("environment", tmp_path)
    yield view
    stop()


def probes(windows: list[str]) -> int:
    found = [w for w in windows if w.startswith("Probes")]
    assert found, windows
    return int(found[0].split("·")[1])


def test_electrical_first_tabs_in_front(electrical) -> None:
    windows = electrical(150)["windows"]
    for title in ("Network", "Schematic", "Scopes", "Inspector"):
        assert title in windows, windows
    for hidden in ("Probes", "+ scope", "History"):
        assert not any(w.startswith(hidden) for w in windows), windows


def test_probes_count_the_scope_traces(electrical) -> None:
    # The scopes start with the bus voltage, the motors' q currents and the battery's power.
    assert probes(electrical(180, "--panel", "###ElectricalProbes")["windows"]) >= 3


def test_add_scope_adds_a_probe(electrical) -> None:
    report = electrical(
        260, "--panel", "###ElectricalAddScope", "--input", f"150:click:{FIRST_SIGNAL_ROW}"
    )
    # A signal added brings the scopes in front; a click on nothing would leave the picker.
    assert "Scopes" in report["windows"], report["windows"]
    assert not any(w.startswith("+ scope") for w in report["windows"]), report["windows"]


def test_history_tab_opens(electrical) -> None:
    assert "History" in electrical(150, "--panel", "###ElectricalHistory")["windows"]


def test_environment_first_tabs_in_front(environment) -> None:
    windows = environment(150)["windows"]
    for title in ("Environment", "Viewport · fields", "Overlays"):
        assert title in windows, windows
    assert "Slice 2D" not in windows and "Field info" not in windows, windows


def test_field_tabs_open(environment) -> None:
    assert "Field info" in environment(180, "--panel", "###FieldInfo")["windows"]
    assert "Slice 2D" in environment(180, "--panel", "###FieldSlice")["windows"]

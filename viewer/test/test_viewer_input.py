"""The Electrical workspace's buttons, dragging and forms, pressed as a user does (--input)."""

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

# Where things are at --size 1600x1000 (docs/design/ui/04-electrical): tabs over the middle dock.
SCHEMATIC_TAB = "343,115"
OVERLAY_TAB = "455,115"
NETLIST_TAB = "511,115"
SCENE_WORKSPACE = "40,48"
AUTO_LAYOUT = "1045,114"
PACK_NODE = (420, 340)
SAVE = "332,570"
LIGHTS_BRANCH = "1000,330"
FIELD_FIRST_BELOW_NAME = "520,197"


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


@pytest.fixture
def run_view(tmp_path):
    """Starts the server on the sample netlist and gives a function running a headless viewer."""
    try:
        assets = get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive"
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    socket = f"/tmp/lagrange_input_{os.getpid()}.sock"
    server = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            "--robot",
            f"{assets}/amr_diffdrive.urdf",
            "--overlay",
            f"{assets}/amr_diffdrive_power.cir",
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

    def run(script: str, frames: int, *extra: str) -> dict:
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
                "electrical",
                "--input",
                script,
                *extra,
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


def test_dragging_a_node_is_kept_and_auto_layout_undoes_it(run_view, tmp_path) -> None:
    x, y = PACK_NODE
    layout = tmp_path / "cfg/lagrange/schematic_layouts.tsv"
    run_view(
        f"30:move:{x},{y};32:down;34:move:{x},{y + 40};36:move:{x},{y + 80};38:up",
        60,
    )
    assert "pack" in layout.read_text()
    # A new viewer reads it back, and Auto-layout takes it away.
    run_view(f"30:click:{AUTO_LAYOUT}", 60)
    assert "pack" not in layout.read_text()


def test_save_writes_what_the_form_changed(run_view, tmp_path) -> None:
    saved = tmp_path / "lagrange_circuit.cir"
    script = ";".join(
        [
            f"30:click:{NETLIST_TAB}",
            f"50:click:{LIGHTS_BRANCH}",
            f"70:click:{FIELD_FIRST_BELOW_NAME}",
            "75:text:33",
            f"90:click:{SAVE}",
        ]
    )
    run_view(script, 120, "--screenshot", str(tmp_path / "netlist.png"))
    text = saved.read_text()
    assert "V_pack" in text
    assert "R_lights dc 0 33" in text, text


def test_leaving_the_overlay_gives_the_scene_its_shading_back(run_view) -> None:
    current = 4  # Shading::Current in RenderOptions.hpp
    assert run_view(f"30:click:{OVERLAY_TAB}", 60)["shading"] == current
    assert run_view(f"30:click:{OVERLAY_TAB};50:click:{SCENE_WORKSPACE}", 90)["shading"] != current
    assert run_view(f"30:click:{OVERLAY_TAB};50:click:{SCHEMATIC_TAB}", 90)["shading"] != current


# The Signals workspace (docs/design/ui/05-signals): the first signal row and the scope.
FIRST_SIGNAL_ROW = (100, 250)
SCOPE_PLOTS = (800, 500)


def test_a_signal_dragged_onto_the_scope_is_plotted(run_view) -> None:
    still = run_view("", 30, "--workspace", "signals")
    assert still["samples"] == 0, "nothing plotted until a signal is chosen"
    x, y = FIRST_SIGNAL_ROW
    u, v = SCOPE_PLOTS
    script = (
        f"10:move:{x},{y};11:down:{x},{y};12:move:{x + 10},{y + 5};13:move:{x + 200},{y + 50};"
        f"14:move:{u - 100},{v};15:move:{u},{v};17:up:{u},{v}"
    )
    dragged = run_view(script, 45, "--workspace", "signals")
    assert dragged["samples"] > 0, "the dropped signal streams into the scope"

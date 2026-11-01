"""What the viewer writes as a netlist gives the formula's voltage in ngspice and imports."""

from __future__ import annotations

import os
import shutil
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


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


def test_the_written_netlist_gives_the_same_bus(tmp_path: Path) -> None:
    if shutil.which("ngspice") is None:
        pytest.skip("needs ngspice")
    try:
        assets = get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive"
        server_tool = tool("lagrange_server", "lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    socket = f"/tmp/lagrange_roundtrip_{os.getpid()}.sock"
    written = tmp_path / "written.cir"
    server = subprocess.Popen(
        [
            server_tool,
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
    try:
        for _ in range(100):
            if Path(socket).exists():
                break
            time.sleep(0.1)
        done = subprocess.run(
            [
                tool("lagrange_viewer", "lagrange_viewer"),
                "--headless",
                "--connect",
                f"unix:{socket}",
                "--frames",
                "120",
                "--workspace",
                "electrical",
                "--export-netlist",
                str(written),
            ],
            capture_output=True,
            text=True,
            timeout=90,
            check=False,
        )
    finally:
        server.send_signal(signal.SIGTERM)
        server.wait(timeout=20)
        Path(socket).unlink(missing_ok=True)
    assert done.returncode == 0, done.stderr
    text = written.read_text()
    assert "*lagrange motor left_wheel_joint" in text

    expected = 51.2 * 19.86 / (19.86 + 0.1)
    run = tmp_path / "print.cir"
    run.write_text(
        text.replace(".end\n", ".control\nrun\nprint v(dc)[length(v(dc))-1]\n.endc\n.end\n")
    )
    spice = subprocess.run(
        ["ngspice", "-b", str(run)], capture_output=True, text=True, check=True
    ).stdout
    line = next(line for line in spice.splitlines() if line.startswith("v(dc)"))
    assert float(line.split("=")[1]) == pytest.approx(expected, rel=1e-3)

    # And the importer reads it back on the same robot.
    usd = tmp_path / "again.usda"
    subprocess.run(
        [
            tool("lagrange_server", "lagrange_import"),
            f"{assets}/amr_diffdrive.urdf",
            str(usd),
            "--overlay",
            str(written),
        ],
        check=True,
        capture_output=True,
    )
    body = usd.read_text()
    assert "BatteryAPI_1" in body and "elec:bus:capacitance = 0.0047" in body

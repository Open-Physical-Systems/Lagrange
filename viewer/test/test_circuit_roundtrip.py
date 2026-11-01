"""What the viewer writes as a netlist gives the formula's voltage in ngspice and imports."""

from __future__ import annotations

import math
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

CONVERTER = """A battery on a sloped curve, half charged, and a rail behind a DC-DC
V_pack in 0 DC 48
R_pack in dc 0.1
C_dc dc 0 4.7m IC=48
V_rail rail 0 DC 12
B_rail_in dc 0 I = -I(V_rail) * 12 / 0.9 / max(V(dc), 0.001) * ((V(dc) >= 0) ? 1 : 0)
R_lamp rail 0 6
*lagrange battery V_pack capacity=72000 cutoff=30 max_charge=10 soc=0.5 ocv_soc=0,1 ocv=44,52
*lagrange bus dc undervoltage=25 overvoltage=60
*lagrange bus rail undervoltage=9 overvoltage=15
*lagrange converter dcdc source=V_rail in=dc max_current=5 efficiency=0.9 input_sink=B_rail_in
.tran 1m 0.2 uic
.end
"""
PARTS = """A fuse and a resistor between buses, then a diode
V_pack in 0 DC 48
R_pack in dc 0.1
C_dc dc 0 4.7m IC=48
R_f1 dc main 0.05
R_harness main far 0.05
D_or far low D_or_model
.model D_or_model D(Is=1e-12)
R_lamp low 0 19.8
*lagrange battery V_pack capacity=72000 cutoff=30 max_charge=10
*lagrange bus dc undervoltage=25 overvoltage=60
*lagrange bus main undervoltage=25 overvoltage=60
*lagrange bus far undervoltage=25 overvoltage=60
*lagrange bus low undervoltage=25 overvoltage=60
*lagrange fuse R_f1 i2t=100000
*lagrange diode D_or drop=0.7 r=0.02
.tran 1m 0.2 uic
.end
"""

# 24 W on the rail is 26.67 W from the battery through 0.1 ohm.
CONVERTER_BUS = (48.0 + math.sqrt(48.0**2 - 4.0 * (24.0 / 0.9) * 0.1)) / 2.0


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


def written_by_the_viewer(robot: str, overlay: str, written: Path) -> str:
    socket = f"/tmp/lagrange_roundtrip_{os.getpid()}.sock"
    server = subprocess.Popen(
        [
            tool("lagrange_server", "lagrange_server"),
            "--robot",
            robot,
            "--overlay",
            overlay,
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
    return written.read_text()


def bus_in_ngspice(text: str, folder: Path) -> float:
    run = folder / "print.cir"
    run.write_text(
        text.replace(".end\n", ".control\nrun\nprint v(dc)[length(v(dc))-1]\n.endc\n.end\n")
    )
    spice = subprocess.run(
        ["ngspice", "-b", str(run)], capture_output=True, text=True, check=True
    ).stdout
    line = next(line for line in spice.splitlines() if line.startswith("v(dc)"))
    return float(line.split("=")[1])


@pytest.mark.parametrize(
    ("given", "expected", "needs"),
    [
        (None, 51.2 * 19.86 / (19.86 + 0.1), ["*lagrange motor left_wheel_joint"]),
        (CONVERTER, CONVERTER_BUS, ["*lagrange converter dcdc", "ocv_soc=0,1 ocv=44,52"]),
        (PARTS, None, ["*lagrange fuse R_f1 i2t=100000", "*lagrange diode D_or drop=0.7"]),
    ],
)
def test_the_written_netlist_gives_the_same_bus(tmp_path, given, expected, needs) -> None:
    if shutil.which("ngspice") is None:
        pytest.skip("needs ngspice")
    try:
        assets = get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive"
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    robot = f"{assets}/amr_diffdrive.urdf"
    overlay = f"{assets}/amr_diffdrive_power.cir"
    if given is not None:
        overlay = str(tmp_path / "given.cir")
        Path(overlay).write_text(given)
    written = tmp_path / "written.cir"
    text = written_by_the_viewer(robot, overlay, written)
    for line in needs:
        assert line in text, text
    if expected is not None:  # ngspice's diode is not the run's: compare the import only
        assert bus_in_ngspice(text, tmp_path) == pytest.approx(expected, rel=1e-3)

    # And the importer reads it back on the same robot.
    usd = tmp_path / "again.usda"
    subprocess.run(
        [tool("lagrange_server", "lagrange_import"), robot, str(usd), "--overlay", str(written)],
        check=True,
        capture_output=True,
    )
    body = usd.read_text()
    assert "BatteryAPI_1" in body

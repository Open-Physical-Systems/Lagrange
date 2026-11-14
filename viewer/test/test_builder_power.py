"""The Builder's power system and overlay file (RFC 0016 phase 3), headless.

rover_mk1 gets a bus and a battery from the robot's Power section and a motor on each wheel
joint from the joints' Motor section, by clicks on their Add menus (the sample AMR's values);
Test then runs the server with --overlay and the run has the bus, battery and motor signals.
Save writes <robot>.overlay.yaml beside the URDF, which opens again with the robot; *Into the
connected run* sends it as ImportRobot.overlay_content. The test servers run with --no-ros.
Run on the software Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

HERE = Path(__file__).resolve().parent
ROVER = HERE / "builder" / "rover_mk1.urdf"
ASSETS = HERE.parent.parent / "assets"

# Where things are at --size 1600x1000 in the Builder's default layout (12-builder SPEC).
ROBOT_ROW = "30,177"  # the robot in the tree: Properties shows its Power section
ADD_BUS = "1298,450"  # Power → Add bus, while the robot has no power system
FIRST_BUS = "1358,480"  # its menu: "48 V bus · sample AMR"
ADD_BATTERY = "1374,413"  # Power → Add battery, once there is a bus
FIRST_BATTERY = "1420,443"  # its menu: "LFP 16S 20 Ah · sample AMR"
ADD_MOTOR = "1313,772"  # a wheel joint's Motor → Add a motor
FIRST_MOTOR = "1360,802"  # its menu: "wheel drive, 20:1 · sample AMR"
TEST_BUTTON = "1430,48"  # "Test · empty world"
TEST_CHEVRON = "1515,48"
INTO_THE_RUN = "1500,169"
TEST_INTO_RUN = "1440,48"

# The power system by clicks: bus, battery, a motor on each wheel joint (4 frames a click).
BUILD_POWER = (
    f"3:click:{ROBOT_ROW};7:click:{ADD_BUS};11:click:{FIRST_BUS};"
    f"15:click:{ADD_BATTERY};19:click:{FIRST_BATTERY};"
    f"22:select:left_wheel_joint;25:click:{ADD_MOTOR};29:click:{FIRST_MOTOR};"
    f"32:select:right_wheel_joint;35:click:{ADD_MOTOR};39:click:{FIRST_MOTOR}"
)

# rover_mk1's power system as a file: the sample AMR's bus, battery and wheel drives.
OVERLAY = """\
schema_version: 1
apply:
  - create: power/dc_48v
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.0047
        elec:bus:undervoltageTrip: 42.0
        elec:bus:overvoltageTrip: 60.0
  - create: power/battery
    schemas:
      BatteryAPI_1:
        elec:battery:chemistry: lfp
        elec:battery:seriesCells: 16
        elec:battery:capacity: 72000.0
        elec:battery:ocvSoc: [0.0, 0.1, 0.5, 0.9, 1.0]
        elec:battery:ocvVoltage: [2.80, 3.20, 3.28, 3.33, 3.45]
        elec:battery:r0: 0.0015
        elec:battery:cutoffVoltage: 44.8
        elec:battery:maxChargeCurrent: 10.0
      PortAPI_1:dc_out:
        elec:port:dc_out:connectedTo: power/dc_48v
  - prim: left_wheel_joint
    create_child: left_drive
    schemas:
      ElectricalMotorAPI_1:
        elec:windingResistance: 0.12
        elec:inductance: 0.00031
        elec:torqueConstant: 0.085
        elec:gearRatio: 20.0
        elec:maxCurrent: 8.0
        elec:joint: left_wheel_joint
        elec:powerPort: power/dc_48v
  - prim: base_link
    create_child: gnss
    schemas:
      SensorAPI_1:
        sensor:rate: 10.0
      GnssAPI_1:
        sensor:gnss:horizontalNoiseStd: 0.3
"""


def program(package: str, name: str) -> str:
    try:
        path = Path(get_package_prefix(package)) / "lib" / package / name
    except PackageNotFoundError:
        pytest.skip(f"needs {package} installed")
    if not path.exists():
        pytest.skip(f"needs {name}")
    return str(path)


def run(tmp_path: Path, *extra: str, frames: int = 8) -> dict:
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_STATE_HOME": str(tmp_path / "state"),
        "LAGRANGE_BUILDER_ROS_DOMAIN_ID": "off",
    }
    done = subprocess.run(
        [
            program("lagrange_viewer", "lagrange_viewer"),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--size",
            "1600x1000",
            "--workspace",
            "builder",
            "--frames",
            str(frames),
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=150,
        check=False,
        env=environment,
    )
    assert done.returncode == 0, done.stderr
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return json.loads(lines[-1])


def powered_rover(tmp_path: Path) -> Path:
    """rover_mk1 with OVERLAY beside it, in tmp_path."""
    urdf = tmp_path / "rover_mk1.urdf"
    shutil.copy(ROVER, urdf)
    (tmp_path / "rover_mk1.overlay.yaml").write_text(OVERLAY)
    return urdf


def test_a_power_system_built_by_clicks_runs_in_the_test(tmp_path):
    program("lagrange_server", "lagrange_server")
    report = run(
        tmp_path,
        "--builder-open",
        str(ROVER),
        "--input",
        BUILD_POWER + f";43:click:{TEST_BUTTON};46:wait:connected",
        frames=70,
    )
    builder = report["builder"]
    assert builder["overlay"] == {
        "ElectricalBusAPI_1": ["dc_bus"],
        "BatteryAPI_1": ["battery"],
        "ElectricalMotorAPI_1": ["left_wheel_joint_motor", "right_wheel_joint_motor"],
    }
    assert builder["history"]["labels"][1:] == [
        "Add bus dc_bus",
        "Add battery battery",
        "Add motor left_wheel_joint_motor on left_wheel_joint",
        "Add motor right_wheel_joint_motor on right_wheel_joint",
    ]
    assert builder["errors"] == 0
    assert builder["mode"] == "testing", builder["loader"]
    signals = set(builder["electrical_signals"])
    assert "bus/dc_bus/voltage" in signals, signals
    assert "battery/battery/soc" in signals
    for joint in ("left_wheel_joint", "right_wheel_joint"):
        assert f"motor/{joint}_motor/current_q" in signals
        assert f"motor/{joint}_motor/bus_current" in signals


def test_a_motor_on_a_fixed_joint_and_a_bus_without_a_battery_are_problems(tmp_path):
    urdf = powered_rover(tmp_path)
    bad = OVERLAY.replace("prim: left_wheel_joint", "prim: caster_joint").replace(
        "elec:joint: left_wheel_joint", "elec:joint: caster_joint"
    )
    bad = bad.replace("connectedTo: power/dc_48v", "connectedTo: power/elsewhere")
    (tmp_path / "rover_mk1.overlay.yaml").write_text(bad)
    report = run(tmp_path, "--builder-open", str(urdf), "--input", f"3:click:{TEST_BUTTON}")
    builder = report["builder"]
    # Test is refused while there are errors: nothing started.
    assert builder["mode"] == "build"
    assert builder["errors"] >= 3


def test_save_writes_the_overlay_beside_the_urdf_and_it_opens_again(tmp_path):
    urdf = powered_rover(tmp_path)
    out = tmp_path / "saved" / "rover_mk1.urdf"
    out.parent.mkdir()
    report = run(tmp_path, "--builder-open", str(urdf), "--builder-save", str(out))
    assert report["builder"]["sensors"] == 5  # four in the URDF, the GNSS from the overlay
    written = out.with_name("rover_mk1.overlay.yaml")
    assert yaml.safe_load(written.read_text()) == yaml.safe_load(OVERLAY)
    # The GNSS is not a URDF sensor.
    assert 'type="gnss"' not in out.read_text()
    again = run(tmp_path, "--builder-open", str(out))
    assert again["builder"]["overlay"]["ElectricalMotorAPI_1"] == ["left_drive"]
    assert again["builder"]["sensors"] == 5
    assert again["builder"]["errors"] == 0


def test_an_overlay_that_does_not_read_is_left_as_it_is(tmp_path):
    urdf = powered_rover(tmp_path)
    overlay = tmp_path / "rover_mk1.overlay.yaml"
    overlay.write_text("schema_version: 7\napply: []\n")
    report = run(tmp_path, "--builder-open", str(urdf), "--builder-save", str(urdf))
    files = " ".join(report["builder"]["files"])
    assert "did not read" in files
    assert overlay.read_text() == "schema_version: 7\napply: []\n"


@pytest.fixture
def warehouse(tmp_path):
    """The sample AMR (no overlay) in the warehouse on a socket of its own."""
    socket = f"/tmp/lagrange_power_into_run_{os.getpid()}.sock"
    log = tmp_path / "server.log"
    with log.open("w") as out:
        server = subprocess.Popen(
            [
                program("lagrange_server", "lagrange_server"),
                "--robot",
                str(ASSETS / "robots" / "amr_diffdrive" / "amr_diffdrive.urdf"),
                "--world",
                str(ASSETS / "worlds" / "warehouse_small" / "warehouse_small.urdf"),
                "--no-ros",
                "--listen",
                f"unix:{socket}",
            ],
            stdout=out,
            stderr=subprocess.STDOUT,
        )
    for _ in range(200):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    yield f"unix:{socket}"
    server.send_signal(signal.SIGTERM)
    server.wait(timeout=20)
    Path(socket).unlink(missing_ok=True)


def test_into_the_connected_run_sends_the_overlay(tmp_path, warehouse):
    urdf = powered_rover(tmp_path)
    report = run(
        tmp_path,
        "--connect",
        warehouse,
        "--builder-open",
        str(urdf),
        "--input",
        f"4:click:{TEST_CHEVRON};8:click:{INTO_THE_RUN};12:click:{TEST_INTO_RUN};14:wait:connected",
        frames=40,
    )
    builder = report["builder"]
    assert builder["mode"] == "testing", builder["loader"]
    assert builder["test_into_run"] is True
    signals = set(builder["electrical_signals"])
    # Beside the run's own robot, this one is passive: its prims, and so its signals, carry its
    # name in the run as a prefix (as --add-robot's do).
    prefix = builder["test_robot"] + "_"
    assert f"bus/{prefix}dc_48v/voltage" in signals, signals
    assert f"battery/{prefix}battery/soc" in signals
    assert f"motor/{prefix}left_drive/current_q" in signals

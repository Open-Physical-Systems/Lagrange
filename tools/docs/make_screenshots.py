#!/usr/bin/env python3
"""Regenerates the docs' viewer screenshots (docs/imgs/*.png) from real runs, headless.

    source install/setup.bash && python3 tools/docs/make_screenshots.py [NAME...] [--out DIR]

Each shot starts a simulator when it needs one (on a socket of its own, in this shell's
ROS_DOMAIN_ID), runs `lagrange_viewer --headless --screenshot` against it, and writes the picture
as a 256-colour PNG (the viewer's flat colours survive it; files stay small). The Builder shots
drive the viewer with --input, at the coordinates of viewer/test/test_builder_*.py, and need no
server of their own: the Builder's Test starts one. NAME is a file name without .png; without
NAME, every shot. Uses the real GPU when there is one (`--gpu` from $LAGRANGE_DOCS_GPU).
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
BUILDER_TESTS = ROOT / "viewer" / "test" / "builder"
MAX_BYTES = 400_000


@dataclass
class Shot:
    name: str
    size: tuple[int, int]
    viewer: list[str]  # the viewer's arguments beside --headless, --size, --screenshot
    server: list[str] | None = None  # lagrange_server's arguments; ["@run", PROFILE, ...]
    warmup: float = 0.0  # s the run plays before the viewer connects
    frames: int = 60
    write: Callable[[Path], list[str]] | None = None  # files in the work directory; viewer args
    crop: tuple[int, int, int, int] | None = None  # left, top, width, height
    env: dict[str, str] = field(default_factory=dict)
    needs: Path | None = None  # a file not in the repository: skipped without it


def ros_run(package: str, program: str) -> list[str]:
    return ["ros2", "run", package, program]


def wait_for(path: Path, process: subprocess.Popen[bytes], log: Path) -> None:
    for _ in range(3000):
        if path.exists():
            return
        if process.poll() is not None:
            raise SystemExit(log.read_text()[-2000:])
        time.sleep(0.02)
    raise SystemExit(f"no {path}")


def stop(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def save(raw: Path, out: Path, crop: tuple[int, int, int, int] | None) -> None:
    image = Image.open(raw).convert("RGB")
    if crop:
        left, top, width, height = crop
        image = image.crop((left, top, left + width, top + height))
    image.quantize(colors=256, dither=Image.Dither.NONE).save(out, optimize=True)
    if out.stat().st_size > MAX_BYTES:  # a busy 3D view: dithered, still 256 colours
        image.quantize(colors=256).save(out, optimize=True)


def take(shot: Shot, out_dir: Path) -> None:
    if shot.needs is not None and not shot.needs.exists():
        print(f"{shot.name}: skipped, needs {shot.needs}", flush=True)
        return
    print(shot.name, flush=True)
    work = Path(tempfile.mkdtemp(prefix="lagrange_docs_"))
    # A socket's path holds at most 107 bytes: a short directory of its own.
    sockets = Path(tempfile.mkdtemp(prefix="lgd_", dir="/tmp"))
    env = {
        **os.environ,
        "HOME": str(work),  # no saved layouts, settings or recent files
        "XDG_CONFIG_HOME": str(work / "cfg"),
        "XDG_STATE_HOME": str(work / "state"),
        **shot.env,
    }
    server: subprocess.Popen[bytes] | None = None
    try:
        extra = shot.write(work) if shot.write else []
        connect: list[str] = []
        if shot.server is not None:
            socket = sockets / "run.sock"
            args = shot.server
            if args and args[0] == "@run":
                command = [*ros_run("lagrange_server", "lagrange"), "run", *args[1:]]
            else:
                command = [*ros_run("lagrange_server", "lagrange_server"), *args]
            log = work / "server.log"
            server = subprocess.Popen(
                [*command, "--listen", f"unix:{socket}"],
                cwd=ROOT,
                env=env,
                stdout=log.open("wb"),
                stderr=subprocess.STDOUT,
            )
            wait_for(socket, server, log)
            time.sleep(shot.warmup)
            connect = ["--connect", f"unix:{socket}", "--wait-ms", "60000"]
        raw = work / "shot.png"
        gpu = os.environ.get("LAGRANGE_DOCS_GPU")
        done = subprocess.run(
            [
                *ros_run("lagrange_viewer", "lagrange_viewer"),
                "--headless",
                *(["--gpu", gpu] if gpu else []),
                "--size",
                f"{shot.size[0]}x{shot.size[1]}",
                "--frames",
                str(shot.frames),
                *connect,
                *shot.viewer,
                *extra,
                "--screenshot",
                str(raw),
            ],
            cwd=work,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=300,
            check=False,
        )
        if done.returncode != 0 or not raw.exists():
            raise SystemExit(f"{shot.name}: the viewer failed\n{done.stderr.decode()[-2000:]}")
        out_dir.mkdir(parents=True, exist_ok=True)
        save(raw, out_dir / f"{shot.name}.png", shot.crop)
    finally:
        if server is not None:
            stop(server)
        shutil.rmtree(sockets, ignore_errors=True)
        shutil.rmtree(work, ignore_errors=True)


# --- the Builder: where things are at 1600 x 1000 (viewer/test/test_builder_*.py) ---------------

ADD_MATERIAL, ADD_MATERIAL_PLA = "120,905", "160,932"
BOX_TILE, MESH_TILE = "40,835", "225,835"
MESH_PATH, ADD_MESH_LINK = "400,920", "280,971"
CHILD_JOINT_TEXT, CHILD_LINK_ROW = "130,221", "80,221"
REVOLUTE, LOWER_LIMIT = "1425,272", "1470,517"
CATALOG_TAB, CATALOG_FILTER, FIRST_ROW_ADD = "860,763", "950,793", "1205,845"
RANGE_MAX = "1470,546"
TEST_BUTTON = "1430,48"
SPLIT_CHOICE, PROPERTIES_MIDDLE, SPLIT_BUTTON = "1415,708", "1415,800", "1383,603"
PAN_VALUE, TILT_VALUE = "650,879", "650,913"  # Joints: rover_mk1's pan and tilt values
ROBOT_ROW = "30,177"
ADD_BUS, FIRST_BUS = "1298,450", "1358,480"
ADD_BATTERY, FIRST_BATTERY = "1374,413", "1420,443"
ADD_MOTOR, FIRST_MOTOR = "1313,772", "1360,802"

# A box on base_link by a revolute joint with its four limits (test_builder_acceptance.py).
JOINT_STEPS = [
    f"3:click:{ADD_MATERIAL}",
    f"6:click:{ADD_MATERIAL_PLA}",
    f"9:click:{BOX_TILE}",
    f"14:click:{CHILD_JOINT_TEXT}",
    f"18:click:{REVOLUTE}",
    f"22:click:{LOWER_LIMIT}",
    "24:text:-90",
    "26:key:tab",
    "28:text:90",
    "30:key:tab",
    "32:text:2",
    "34:key:tab",
    "36:text:3",
    "38:key:enter",
]
# Then a rangefinder from the catalog on the new link, its range/max set to 20 m.
CATALOG_STEPS = [
    f"42:click:{CHILD_LINK_ROW}",
    f"45:click:{CATALOG_TAB}",
    f"48:click:{CATALOG_FILTER}",
    "50:text:rangefinder",
    f"54:click:{FIRST_ROW_ADD}",
    f"60:click:{RANGE_MAX}",
    "62:key:ctrl+a",
    "64:text:20",
    "66:key:enter",
]
TEST_STEPS = [f"72:click:{TEST_BUTTON}", "74:wait:connected"]

BUILDER_ENV = {"LAGRANGE_BUILDER_ROS_DOMAIN_ID": "off"}


def builder(
    name: str,
    frames: int,
    robot: Callable[[Path], Path],
    steps: list[str],
    env: dict[str, str] | None = None,
) -> Shot:
    def write(work: Path) -> list[str]:
        return ["--builder-open", str(robot(work)), "--input", ";".join(steps)]

    return Shot(
        name,
        (1600, 1000),
        ["--workspace", "builder"],
        frames=frames,
        write=write,
        env={**BUILDER_ENV, **(env or {})},
    )


def copy_of(source: Path, overlay: str | None = None) -> Callable[[Path], Path]:
    def make(work: Path) -> Path:
        folder = work / source.stem
        folder.mkdir()
        robot = folder / source.name
        shutil.copy(source, robot)
        if overlay is not None:
            (folder / f"{source.stem}.overlay.yaml").write_text(overlay)
        if source.name == "empty.urdf":  # the collision shot's L-shaped mesh beside it
            write_ell(folder / "ell.obj")
        return robot

    return make


def write_ell(path: Path) -> None:
    """An L-shaped prism, 0.2 by 0.2 by 0.05 m with a 0.08 m arm (test_builder_acceptance.py)."""
    outline = [(0, 0), (0.2, 0), (0.2, 0.08), (0.08, 0.08), (0.08, 0.2), (0, 0.2)]
    n = len(outline)
    vertices = [(x, y, 0.0) for x, y in outline] + [(x, y, 0.05) for x, y in outline]
    fan = [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5)]
    faces = [(a, c, b) for a, b, c in fan] + [(a + n, b + n, c + n) for a, b, c in fan]
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, j + n), (i, j + n, i + n)]
    lines = [f"v {x:g} {y:g} {z:g}" for x, y, z in vertices]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")


# rover_mk1 with the sample AMR's battery on a 48 V bus, a converter to a 12 V bus and a 20 W
# computer on it (test_builder_power.py's RAIL).
RAIL = """\
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
  - create: power/dc_12v
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: 0.001
        elec:bus:undervoltageTrip: 10.0
        elec:bus:overvoltageTrip: 14.0
  - create: power/dcdc
    schemas:
      ElectricalConverterAPI_1:
        elec:conv:outputVoltage: 12.0
        elec:conv:efficiency: 0.9
        elec:conv:maxOutputCurrent: 5.0
        elec:conv:input: power/dc_48v
        elec:conv:output: power/dc_12v
  - create: power/computer
    schemas:
      ElectricalLoadAPI_1:
        elec:load:power: 20.0
        elec:powerPort: power/dc_12v
"""
RAIL_SCOPE = (
    "bus/dc_12v/voltage,bus/dc_48v/voltage,load/computer/current,converter/dcdc/input_current"
)

ROVER = BUILDER_TESTS / "rover_mk1.urdf"
EMPTY = BUILDER_TESTS / "empty.urdf"

AMR_PROFILE = ["@run", "profiles/examples/amr_sample_urdf.yaml"]
# The 40 km Zurich sample, built by tools/ci/build_sample_world.sh (network; not committed).
ZURICH = Path(
    os.environ.get(
        "LAGRANGE_ZURICH_WORLD",
        Path.home() / ".cache/lagrange/worlds/zurich_40km/zurich_40km.usda",
    )
)

SHOTS = [
    # docs/guide/tutorial-amr.md: the AMR in the warehouse with its lidar returns.
    Shot(
        "tutorial-scene",
        (1600, 900),
        ["--workspace", "scene", "--overlays", "grid,axes,sensors"],
        server=AMR_PROFILE,
        warmup=7.0,
    ),
    # The commanded speed and the left wheel's speed, some seconds of them, in the scope.
    Shot(
        "tutorial-signals",
        (1600, 900),
        [
            "--workspace",
            "signals",
            "--scope",
            "base/command/linear_velocity,joint/left_wheel_joint/velocity",
        ],
        server=AMR_PROFILE,
        warmup=4.0,
        frames=2400,
    ),
    # docs/guide/profiles.md: the Domains workspace of the same run.
    Shot(
        "tutorial-domains", (1600, 900), ["--workspace", "domains"], server=AMR_PROFILE, warmup=5.0
    ),
    # README.md: the sedan in the city block with its roof lidar's returns.
    Shot(
        "viewer-sedan",
        (1280, 720),
        ["--workspace", "scene", "--follow", "22", "--overlays", "grid,axes,sensors"],
        server=[
            "--robot",
            "assets/robots/sedan/sedan.usda",
            "--world",
            "assets/worlds/city_block/city_block.usda",
        ],
        warmup=10.0,
    ),
    # README.md, docs/guide/tutorial-huge-worlds.md: the sample AMR in the 40 km Zurich world.
    Shot(
        "zurich-40km",
        (1280, 720),
        ["--workspace", "scene", "--follow", "500,25", "--shading", "lit"],  # haze hides 500 m
        server=["--world", str(ZURICH), "--geo-origin", "47.397742,8.545594,0"],
        warmup=30.0,
        frames=120,
        needs=ZURICH,
    ),
    # docs/physics/emi-and-radio.md: the quadcopter's battery leads, |B| sliced through it.
    Shot(
        "motor-leads-emi",
        (1600, 900),
        ["--workspace", "environment", "--slice", "emi/flux_density:xz", "--panel", "Slice 2D"],
        server=[
            "--robot",
            "assets/robots/quad_x500/quad_x500.usda",
            "--world",
            "none",
            "--air-density",
            "1.225",
            "--geo-origin",
            "47.397742,8.545594,488",
            "--emi-motor-leads",
            "0.005,-0.06,0,-0.06",
        ],
        warmup=12.0,
        frames=240,
    ),
    # docs/guide/builder.md
    builder(
        "builder-overview",
        24,
        copy_of(ROVER),
        [
            f"3:click:{PAN_VALUE}",
            "5:key:ctrl+a",
            "6:text:35",
            "8:key:enter",
            f"10:click:{TILT_VALUE}",
            "12:key:ctrl+a",
            "13:text:-12",
            "15:key:enter",
            "18:select:front_range",
        ],
    ),
    builder(
        "builder-test",
        110,
        copy_of(EMPTY),
        [*JOINT_STEPS, *CATALOG_STEPS, *TEST_STEPS],
        # The test server on ROS: Drive lists the topics with their command lines.
        env={"LAGRANGE_BUILDER_ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0")},
    ),
    builder("builder-joint", 42, copy_of(EMPTY), JOINT_STEPS),
    builder("builder-catalog", 70, copy_of(EMPTY), [*JOINT_STEPS, *CATALOG_STEPS]),
    builder(
        "builder-power",
        48,
        copy_of(ROVER),
        [
            f"3:click:{ROBOT_ROW}",
            f"7:click:{ADD_BUS}",
            f"11:click:{FIRST_BUS}",
            f"15:click:{ADD_BATTERY}",
            f"19:click:{FIRST_BATTERY}",
            "22:select:left_wheel_joint",
            f"25:click:{ADD_MOTOR}",
            f"29:click:{FIRST_MOTOR}",
            "32:select:right_wheel_joint",
            f"35:click:{ADD_MOTOR}",
            f"39:click:{FIRST_MOTOR}",
            f"42:click:{ROBOT_ROW}",
        ],
    ),
    Shot(
        "builder-power-rail",
        (1600, 1000),
        ["--workspace", "builder", "--scope", RAIL_SCOPE],
        frames=1500,
        write=lambda work: [
            "--builder-open",
            str(copy_of(ROVER, RAIL)(work)),
            "--input",
            f"3:click:{TEST_BUTTON};6:wait:connected;10:key:ctrl+4",
        ],
        env=BUILDER_ENV,
    ),
]


def collision_shot() -> Shot:
    def write(work: Path) -> list[str]:
        robot = copy_of(EMPTY)(work)
        mesh = robot.parent / "ell.obj"
        steps = [
            f"3:click:{ADD_MATERIAL}",
            f"6:click:{ADD_MATERIAL_PLA}",
            f"9:click:{MESH_TILE}",
            f"12:click:{MESH_PATH}",
            f"14:text:{mesh}",
            "16:key:enter",
            f"19:click:{ADD_MESH_LINK}",
            f"23:click:{SPLIT_CHOICE}",
            f"26:move:{PROPERTIES_MIDDLE}",
            "27:wheel:-10",
            f"30:click:{SPLIT_BUTTON}",
            "34:wait:idle",
            f"36:move:{PROPERTIES_MIDDLE}",
            "37:wheel:-8",  # down to the four choices and the parts
        ]
        return ["--builder-open", str(robot), "--input", ";".join(steps)]

    return Shot(
        "builder-collision",
        (1600, 1000),
        ["--workspace", "builder"],
        frames=46,
        write=write,
        env=BUILDER_ENV,
    )


SHOTS.append(collision_shot())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("names", nargs="*", help="shots to take (default: all)")
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "imgs")
    args = parser.parse_args()
    known = {shot.name for shot in SHOTS}
    unknown = set(args.names) - known
    if unknown:
        print(f"unknown: {', '.join(sorted(unknown))}; known: {', '.join(sorted(known))}")
        return 2
    for shot in SHOTS:
        if not args.names or shot.name in args.names:
            take(shot, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

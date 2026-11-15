#!/usr/bin/env python3
"""Records docs/videos/: each clip is a real run of the simulator seen through the headless viewer.

    source install/setup.bash && python3 tools/videos/make_videos.py [CLIP...] [--out DIR] [--keep]

Needs an ffmpeg: on the PATH, in $FFMPEG, or `uvx --from imageio-ffmpeg`. Each clip starts a server
and a viewer, sends its commands while the viewer writes frames, and puts each command under the
picture at the moment it was sent. Files stay under 1 MB (the repository's limit).
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

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
SCENES = Path(__file__).resolve().parent / "scenes"
FPS = 15
SIZE = (1280, 720)
MAX_BYTES = 1_000_000


VIEWPORT = (1920, 1080, (550, 127, 1310, 847))  # viewer size, crop: the middle of the 3D view
SCOPE = (1151, 916, (316, 160, 836, 880))  # the scope, at one to one
BIRD = (1920, 1080, (268, 126, 928, 618))  # the lidar's bird's-eye view
ENVIRONMENT_SLICE = (1920, 1080, (655, 200, 1295, 920))  # Environment: middle of Slice 2D
VIEWPORT_HALF = (1920, 1080, (610, 127, 1250, 847))  # the middle of the 3D view, 640 wide
# The Sensors workspace's Camera panel at 1920 x 1080 (Chrome.cpp: sensor list 260 px,
# Degradation 320 px, lidar | camera above IMU | radar): a square image (CAMERA_SQUARE) fills its
# height, 522 px from y = 124; a 4:3 one (CAMERA_PANEL) its width, 660 px from x = 933.
CAMERA_PANEL = (1920, 1080, (933, 124, 1593, 624))
CAMERA_SQUARE = (1920, 1080, (996, 124, 1526, 648))
VIEWPORT_THIRD = (1920, 1080, (716, 127, 1142, 847))  # the middle of the 3D view, 426 wide
# The Sensors workspace at 1920 x 1080: the Radar view (bottom right of the 2 x 2 grid, under the
# camera) and the camera's image without its tool row.
RADAR_PANEL = (1920, 1080, (1060, 700, 1593, 1050))
CAMERA_IMAGE = (1920, 1080, (933, 104, 1593, 500))
LEFT = (0, 0, 760, 720)
RIGHT = (760, 0, 520, 720)
RIGHT_TOP = (760, 0, 520, 360)
RIGHT_BOTTOM = (760, 360, 520, 360)
HALF_L = (0, 0, 640, 720)
HALF_R = (640, 0, 640, 720)
THIRD_L = (0, 0, 426, 720)
THIRD_M = (427, 0, 426, 720)
THIRD_R = (854, 0, 426, 720)


@dataclass
class Step:
    at: float  # s after the first frame
    caption: str  # shown from this moment
    command: list[str] | None = None
    run: int = 0  # the run it goes to


@dataclass
class Pane:
    label: str
    box: tuple[int, int, int, int]  # in the picture
    view: tuple[int, int, tuple[int, int, int, int]] = VIEWPORT
    viewer: list[str] = field(default_factory=list)
    run: int = 0
    text: str | None = None  # a file to show instead of a viewer
    of: int | None = None  # another part of pane `of`'s frames: no viewer of its own


@dataclass
class Clip:
    name: str
    title: str
    seconds: float
    # Server arguments of each run; ["@cli", ...]: the lagrange command; ["@run", PROFILE, ...]:
    # lagrange run with a profile.
    runs: list[list[str]]
    panes: list[Pane]
    steps: list[Step] = field(default_factory=list)
    write: Callable[[Path], None] | None = None  # files the runs need, in the working directory


def asset(relative: str) -> str:
    return str(ROOT / "assets" / relative)


def scene(relative: str) -> str:
    return str(SCENES / relative)


def lagrange_set(*values: str) -> list[str]:
    return ["lagrange", "set", *values]


def joints(**commands: dict[str, float]) -> list[str]:
    """A /joint_commands message: position={"elbow": 0.5}, velocity=..., effort=..."""
    names = sorted({name for values in commands.values() for name in values})
    fields = [f"name: {names}".replace("'", '"')]
    for kind, values in commands.items():
        numbers = ", ".join(str(values[n]) if n in values else ".nan" for n in names)
        fields.append(f"{kind}: [{numbers}]")
    message = "{" + ", ".join(fields) + "}"
    return ["ros2", "topic", "pub", "--once", "-w", "1", "/joint_commands",
            "sensor_msgs/msg/JointState", message]  # fmt: skip


def scope(*signals: str) -> list[str]:
    return ["--workspace", "Signals", "--scope", ",".join(signals)]


def view(x: float, y: float, z: float, distance: float, yaw: float, pitch: float) -> list[str]:
    return ["--workspace", "Scene", "--camera", f"{x},{y},{z},{distance},{yaw},{pitch}"]


def tof_robot(work: Path) -> None:
    """The sample AMR with the time-of-flight sensor of the plugin guide looking ahead."""
    urdf = Path(asset("robots/amr_diffdrive/amr_diffdrive.urdf")).read_text()
    sensor = """<gazebo reference="base_link"><sensor name="front_tof" type="tof">
<pose>0.3 0 0.15 0 0 0</pose><update_rate>50</update_rate>
<tof><range><min>0.02</min><max>12</max></range><fov>0.1</fov></tof></sensor></gazebo>"""
    base = Path(asset("robots/amr_diffdrive"))
    urdf = urdf.replace("</robot>", sensor + "</robot>").replace('filename="', f'filename="{base}/')
    (work / "amr_tof.urdf").write_text(urdf)


# A lidar head 1 m up, bolted to the world (the rig of server/test/test_optics_import.cpp), in a
# room open at the back so that the view can stand behind it: a wall 7 m ahead, side walls, a
# crate and a pillar that throw shadows.
def fixed_part(name: str, geometry: str, xyz: str, rgba: str) -> str:
    """A link welded to the world: geometry as collision and visual, at xyz."""
    return (
        f'<link name="{name}"><collision><geometry>{geometry}</geometry></collision>'
        f"<visual><geometry>{geometry}</geometry>"
        f'<material name="{name}"><color rgba="{rgba}"/></material></visual></link>'
        f'<joint name="{name}_fix" type="fixed"><parent link="world"/><child link="{name}"/>'
        f'<origin xyz="{xyz}"/></joint>\n'
    )


GREY = "0.5 0.55 0.6 1"
LIDAR_ROOM = "".join(
    [
        fixed_part("front", '<box size="0.2 14 4"/>', "7 0 2", GREY),
        fixed_part("left", '<box size="8 0.2 4"/>', "3 7 2", GREY),
        fixed_part("right", '<box size="8 0.2 4"/>', "3 -7 2", GREY),
        fixed_part("crate", '<box size="1 1 1"/>', "4 -1.5 0.5", "0.6 0.45 0.3 1"),
        fixed_part("pillar", '<cylinder radius="0.25" length="4"/>', "5 1.8 2", "0.7 0.7 0.72 1"),
    ]
)
LIDAR_HEAD = (
    '<link name="head"><inertial><mass value="5"/>'
    '<inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>'
    '<visual><geometry><cylinder radius="0.08" length="0.1"/></geometry>'
    '<material name="head"><color rgba="0.9 0.5 0.15 1"/></material></visual></link>'
    '<joint name="mount" type="fixed"><parent link="world"/><child link="head"/>'
    '<origin xyz="0 0 1"/></joint>\n'
)

# The three opt-in scan patterns of sensors.md "Lidar scan patterns (as built)", in the URDF form
# that server/src/Robot.cpp lidarPattern() reads: <ray><pattern>, <scan><vertical><angles>,
# <ray><rosette>.
LIDAR_PATTERNS = {
    # 16 channels, uneven: dense near the horizon. Illustrative, not a product's table (deg:
    # -25 -15 -10 -7 -5 -3.5 -2.5 -1.5 -0.75 0 0.75 1.5 3 5 9 15).
    "lidar_channels.urdf": (
        "A Velodyne-style channel table: one elevation per channel",
        "<scan><horizontal><samples>1800</samples><min_angle>-3.14159</min_angle>"
        "<max_angle>3.14159</max_angle></horizontal><vertical><angles>-0.4363 -0.2618 -0.1745"
        " -0.1222 -0.0873 -0.0611 -0.0436 -0.0262 -0.0131 0 0.0131 0.0262 0.0524 0.0873 0.1571"
        " 0.2618</angles></vertical></scan>",
    ),
    # A focal-plane array, ±25° by ±18.75°: every beam at once.
    "lidar_flash.urdf": (
        "A flash lidar: a pinhole's pixels, every beam at once",
        "<pattern>flash</pattern><scan><horizontal><samples>160</samples>"
        "<min_angle>-0.4363</min_angle><max_angle>0.4363</max_angle></horizontal><vertical>"
        "<samples>120</samples><min_angle>-0.3272</min_angle><max_angle>0.3272</max_angle>"
        "</vertical></scan>",
    ),
    # The rosette's defaults (Livox Mid-40 field and point rate; the prisms' speeds from Livox's
    # paper), samples = point rate / update rate as sensors.md asks for the real picture.
    "lidar_rosette.urdf": (
        "A Livox-style rosette: two Risley prisms, a pattern that never repeats",
        "<pattern>rosette</pattern><scan><horizontal><samples>10000</samples></horizontal>"
        "</scan><rosette><fov>0.6702</fov><point_rate>100000</point_rate></rosette>",
    ),
}


def lidar_patterns(work: Path) -> None:
    """The lidar rig three times, one scan pattern each."""
    for name, (doc, ray) in LIDAR_PATTERNS.items():
        sensor = (
            '<gazebo reference="head"><sensor name="lidar" type="lidar">'
            f"<update_rate>10</update_rate><ray>{ray}<range><min>0.1</min><max>40</max></range>"
            "</ray></sensor></gazebo>\n"
        )
        (work / name).write_text(
            f'<?xml version="1.0"?>\n<!-- {doc} -->\n<robot name="lidar_rig"><link name="world"/>\n'
            f"{LIDAR_HEAD}{LIDAR_ROOM}{sensor}</robot>\n"
        )


# Cameras on the sample AMR, 0.28 m ahead of and 0.30 m over its axle, looking ahead; it drives
# its figure-eight demo. Same name in each file, so the Camera panel's tab reads the same.
AMR_CAMERAS = {
    "amr_pinhole.urdf": """<sensor name="front_camera" type="camera">
<pose>0.28 0 0.30 0 0 0</pose>
<update_rate>15</update_rate><camera><horizontal_fov>1.5708</horizontal_fov>
<image><width>640</width><height>640</height></image><clip><near>0.05</near><far>100</far></clip>
</camera></sensor>""",
    # sensors.md "Fisheye cameras (as built)": <lens><type>equidistant</type>, <horizontal_fov> the
    # lens's angle across the width (220°), <cutoff_angle> the half-angle it sees to (110°).
    "amr_fisheye.urdf": """<sensor name="front_camera" type="camera">
<pose>0.28 0 0.30 0 0 0</pose>
<update_rate>15</update_rate><camera><horizontal_fov>3.8397</horizontal_fov>
<image><width>640</width><height>640</height></image><clip><near>0.05</near><far>100</far></clip>
<lens><type>equidistant</type><cutoff_angle>1.9199</cutoff_angle></lens></camera></sensor>""",
    # A camera with the event camera's field and pixels (DAVIS346 defaults, sensors.md §9).
    "amr_camera_346.urdf": """<sensor name="front_camera" type="camera">
<pose>0.28 0 0.30 0 0 0</pose>
<update_rate>15</update_rate><camera><horizontal_fov>1.2357</horizontal_fov>
<image><width>346</width><height>260</height></image></camera></sensor>""",
    # Kind event_camera (sensors.md §9 "event_camera"): its image is a step's events, grey 128
    # plus 64 per ON and minus 64 per OFF. 50 Hz: 20 ms of events in each image.
    "amr_dvs.urdf": """<sensor name="front_camera" type="event_camera">
<pose>0.28 0 0.30 0 0 0</pose>
<update_rate>50</update_rate><event_camera><width>346</width><height>260</height>
<horizontal_fov>1.2357</horizontal_fov></event_camera></sensor>""",
}


def amr_cameras(work: Path) -> None:
    """The sample AMR with one camera each: pinhole, fisheye, the event camera and its twin."""
    urdf = Path(asset("robots/amr_diffdrive/amr_diffdrive.urdf")).read_text()
    for name, sensor in AMR_CAMERAS.items():
        block = f'<gazebo reference="base_link">{sensor}</gazebo>'
        (work / name).write_text(urdf.replace("</robot>", block + "</robot>"))


def ahmed_tunnel(work: Path, cells: int = 2000000, frames: int = 0) -> None:
    """An Ahmed body through lagrange tunnel, in work/tunnel."""
    maker = asset("robots/ahmed_body/make_ahmed_body.py")
    subprocess.run([sys.executable, maker, "25", str(work / "ahmed")], check=True)
    command = ["lagrange", "tunnel", str(work / "ahmed.urdf"), "--speed", "40", "--ground",
               "--clearance", "0.05", "--cells", str(cells), "--passes", "2", "--mean-passes", "2",
               "--frames", str(frames), "--out", str(work / "tunnel")]  # fmt: skip
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


AMR = asset("robots/amr_diffdrive/amr_diffdrive.urdf")
WAREHOUSE = asset("worlds/warehouse_small/warehouse_small.usda")
TOWN = asset("worlds/town/town.usda")
QUAD = asset("robots/quad_x500/quad_x500.usda")
GEO = ["--air-density", "1.225"]
TURN = joints(velocity={"turn": 6.28})
ARM = asset("robots/arm6/arm6.urdf")
BUILDING = asset("worlds/drone_building/drone_building.urdf")
DOWNWASH = str(ROOT / "profiles/examples/quad_x500_downwash.yaml")
LIVE_AIR = str(ROOT / "profiles/examples/quad_x500_live_air.yaml")
AMR_SAFETY = asset("robots/amr_diffdrive/amr_diffdrive_safety.urdf")
SEDAN_4D = asset("robots/sedan/sedan_4d_radar.usda")
# `lagrange drive`'s start (tools/cli/lagrange): in the lane at x = 30 m, heading east.
DRIVE = ["--spawn", "30,-1.75,0.6,0", "--no-demo", "--traffic", "vehicles=24,pedestrians=36,seed=1"]


def cmd_vel(x: float, z: float, seconds: float) -> list[str]:
    """The multicopter's /cmd_vel at 20 Hz for `seconds`: forward x, up z (m/s); then it holds."""
    return ["ros2", "topic", "pub", "-r", "20", "-t", str(round(20 * seconds)), "-w", "1",
            "/cmd_vel", "geometry_msgs/msg/TwistStamped",
            f"{{twist: {{linear: {{x: {x}, z: {z}}}}}}}"]  # fmt: skip


def twist(x: float, wz: float, seconds: float) -> list[str]:
    """A ground robot's /cmd_vel at 20 Hz for `seconds`: forward x (m/s), turn rate wz (rad/s)."""
    return ["ros2", "topic", "pub", "-r", "20", "-t", str(round(20 * seconds)), "-w", "1",
            "/cmd_vel", "geometry_msgs/msg/TwistStamped",
            f"{{twist: {{linear: {{x: {x}}}, angular: {{z: {wz}}}}}}}"]  # fmt: skip


# simulation_interfaces' SetSimulationState to STATE_PLAYING: runs started with --paused, played
# together so that side-by-side runs move in step.
PLAY = ["ros2", "service", "call", "/set_simulation_state",
        "simulation_interfaces/srv/SetSimulationState", "{state: {state: 1}}"]  # fmt: skip


# The multicopter's /cmd_vel: 0.5 m/s down for 3 s (60 messages at 20 Hz), then it holds.
DESCEND = ["ros2", "topic", "pub", "-r", "20", "-t", "60", "-w", "1", "/cmd_vel",
           "geometry_msgs/msg/TwistStamped", "{twist: {linear: {z: -0.5}}}"]  # fmt: skip
FISHEYE_LINE = (
    "<lens><type>equidistant</type><cutoff_angle>1.9199</cutoff_angle></lens>    "
    "(Kannala-Brandt, as OpenCV's fisheye; up to 360°)"
)
EVENT_LINE = (
    '<sensor type="event_camera">    DAVIS346 pixels, thresholds and noise as v2e (sensors.md §9)'
)

CLIPS: list[Clip] = [
    Clip(
        "wind_tunnel_flow",
        "lagrange tunnel: streamlines, a slice and surface pressure on an Ahmed body, as a world",
        8,
        [["--robot", "none", "--world", "tunnel/flow_view.usda", "--paused"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                view(1.2, 0, 0.5, 3.0, -145, 22),
            ),
        ],
        [Step(0, "lagrange tunnel ahmed.urdf --speed 40 --ground --clearance 0.05")],
        ahmed_tunnel,
    ),
    Clip(
        "realistic_water",
        "Water that looks like water: waves, glitter, reflections, foam",
        14,
        [
            [
                "--robot",
                "none",
                "--world",
                scene("crates_deep.sdf"),
                "--water",
                "0,0,8,3000,3000,8",
                "--site",
                "wind_speed=2",
            ]
        ],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                view(0, 0, 8.5, 11, -145, 10),
            )
        ],
        [
            Step(
                0,
                "lagrange --water 0,0,8,3000,3000,8    A wooden crate floats, a steel one has sunk",
            )
        ],
    ),
    Clip(
        "realistic_water_wind",
        "The wind makes the sea rough",
        16,
        [
            [
                "--robot",
                "none",
                "--world",
                scene("crates_deep.sdf"),
                "--water",
                "0,0,8,3000,3000,8",
                "--site",
                "wind_speed=1",
            ]
        ],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                view(0, 0, 8.5, 14, -125, 13),
            )
        ],
        [
            Step(0, "Calm"),
            Step(5, "lagrange set wind_speed=8", lagrange_set("wind_speed=8")),
            Step(10, "lagrange set wind_speed=14", lagrange_set("wind_speed=14")),
        ],
    ),
    Clip(
        "realistic_materials",
        "Materials: chrome, steel, gold, lacquer, plastic, rubber, concrete, wood, glass",
        12,
        [["--robot", "none", "--world", scene("materials.urdf")]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                view(1, -1, 0.6, 9, -95, 14),
            )
        ],
        [Step(0, '<gazebo reference="ball"><visual_material preset="chrome"/></gazebo>')],
    ),
    Clip(
        "town_junction",
        "The town: lights change, people press a button and cross, cars stop and go",
        40,
        [["--robot", "none", "--world", TOWN, "--traffic", "vehicles=24,pedestrians=36,seed=1"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--camera", "0,0,1,34,135,30"],
            )
        ],
        [Step(0, "lagrange town")],
    ),
    Clip(
        "town_street",
        "The town at street level: a lone crossing with its signal",
        30,
        [["--robot", "none", "--world", TOWN, "--traffic", "vehicles=24,pedestrians=36,seed=1"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--camera", "-50,0,1,14,30,8"],
            )
        ],
        [Step(0, "lagrange town")],
    ),
    Clip(
        "town_summer",
        "Summer",
        14,
        [["--robot", "none", "--world", TOWN, "--traffic", "vehicles=24,pedestrians=36,seed=1"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--season", "summer", "--camera", "0,-55,1,22,95,6"],
            )
        ],
        [Step(0, "lagrange town --season summer")],
    ),
    Clip(
        "town_fall",
        "Fall: the leaves turn",
        14,
        [["--robot", "none", "--world", TOWN, "--traffic", "vehicles=24,pedestrians=36,seed=1"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--season", "fall", "--camera", "0,-55,1,22,95,6"],
            )
        ],
        [Step(0, "lagrange town --season fall")],
    ),
    Clip(
        "town_winter",
        "Winter: snow on the street and the trees",
        14,
        [["--robot", "none", "--world", TOWN, "--traffic", "vehicles=24,pedestrians=36,seed=1"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--season", "winter", "--camera", "0,-55,1,22,95,6"],
            )
        ],
        [Step(0, "lagrange town --season winter")],
    ),
    Clip(
        "town_rainy",
        "The rainy season: wet streets, puddles, rain",
        14,
        [["--robot", "none", "--world", TOWN, "--traffic", "vehicles=24,pedestrians=36,seed=1"]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--season", "rainy", "--camera", "0,-55,1,22,95,6"],
            )
        ],
        [Step(0, "lagrange town --season rainy")],
    ),
    Clip(
        "realistic_warehouse",
        "A robot in the sample warehouse, with sun, shadows and sky",
        12,
        [["--robot", AMR, "--world", WAREHOUSE]],
        [
            Pane(
                "",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                ["--workspace", "Scene", "--follow", "4,20"],
            )
        ],
        [Step(0, "lagrange amr_diffdrive.urdf warehouse_small.usda")],
    ),
    Clip(
        "gravity",
        "Gravity, changed while it runs",
        26,
        [["--robot", scene("pendulum.urdf"), "--world", "none", "--joint-control"]],
        [
            Pane("3D view", LEFT, viewer=view(0.5, 0, 1.3, 6, -90, 8)),
            Pane("joint angle (rad)", RIGHT, SCOPE, scope("joint/swing/position")),
        ],
        [
            Step(0, "Earth: 9.81 m/s²"),
            Step(9, "lagrange set gravity=1.62    (the Moon)", lagrange_set("gravity=1.62")),
            Step(18, "lagrange set gravity=24.8    (Jupiter)", lagrange_set("gravity=24.8")),
        ],
    ),
    Clip(
        "wind",
        "Wind, changed while it runs",
        26,
        [["--robot", QUAD, "--world", "none", *GEO, "--site", "wind_speed=0"]],
        [
            Pane("3D view", LEFT, viewer=view(0, 0, 1.9, 2.2, -90, 6)),
            Pane(
                "position along the wind (m), tilt",
                RIGHT,
                SCOPE,
                scope("body/base_link/pose.x", "body/base_link/pose.qy"),
            ),
        ],
        [
            Step(
                0,
                "Still air: the quadcopter holds its hover. Its controller will lean into wind.",
            ),
            Step(
                7,
                "lagrange set wind_speed=6 wind_from=270",
                lagrange_set("wind_speed=6", "wind_from=270"),
            ),
            Step(14, "lagrange set wind_speed=14", lagrange_set("wind_speed=14")),
            Step(21, "lagrange set wind_speed=0", lagrange_set("wind_speed=0")),
        ],
    ),
    Clip(
        "drone_downwash",
        "Rotor downwash: the air under a hovering quadcopter, live",
        24,
        [["--robot", QUAD, "--world", "none", *GEO, "--site", "wind_speed=0"]],
        [
            Pane(
                "3D view: airflow particles",
                HALF_L,
                VIEWPORT_HALF,
                ["--overlays", "airflow", *view(0, 0, 1.0, 6.0, -60, 12)],
            ),
            Pane(
                "air speed through the drone (XZ)",
                HALF_R,
                ENVIRONMENT_SLICE,
                ["--workspace", "Environment", "--slice", "aero/wind_speed:xz"],
            ),
        ],
        [
            Step(0, "Taking off: the rotors' jet reaches the ground and spreads as outwash"),
            Step(
                12,
                "lagrange set wind_speed=4 wind_from=270    (the wake leans downwind)",
                lagrange_set("wind_speed=4", "wind_from=270"),
            ),
        ],
    ),
    Clip(
        "drone_low_altitude",
        "Downwash near the ground: the outwash grows and the rotors need less speed",
        28,
        [["--robot", QUAD, "--world", "none", *GEO]],
        [
            Pane(
                "3D view: airflow particles",
                LEFT,
                viewer=["--overlays", "airflow", *view(0, 0, 0.6, 5.0, -60, 10)],
            ),
            Pane(
                "height (m), rotor speed (rad/s)",
                RIGHT,
                SCOPE,
                scope("body/base_link/pose.z", "joint/rotor_1_joint/velocity"),
            ),
        ],
        [
            Step(0, "Hovering 2 m up: the jet has spread before it reaches the ground"),
            Step(9, "Down to about half a metre (/cmd_vel, 0.5 m/s down for 3 s)", DESCEND),
            Step(
                15,
                "Low: the outwash ring spreads wide; ground effect at 3.9 rotor radii is small"
                " (rotor speed 0.2% lower)",
            ),
        ],
    ),
    Clip(
        "drone_building",
        "Downwash beside and over a 12 m building",
        44,
        [["@run", DOWNWASH, "--robot", QUAD, "--world", BUILDING]],
        [
            Pane(
                "3D view: airflow particles",
                LEFT,
                viewer=["--overlays", "airflow", "--follow", "5,25"],
            ),
            Pane(
                "air down through rotors 1 (by the wall) and 2 (m/s), their speeds",
                RIGHT,
                SCOPE,
                scope(
                    "rotor/rotor_1_joint/air_velocity.z",
                    "rotor/rotor_2_joint/air_velocity.z",
                    "joint/rotor_1_joint/velocity",
                    "joint/rotor_2_joint/velocity",
                ),
            ),
        ],
        [
            Step(
                0,
                "Rotor tips 0.3 m from the wall: more air down through the rotors by it, so to"
                " stay level the controller spins them faster",
            ),
            Step(12, "Up past the roof (/cmd_vel 1.5 m/s up for 8 s)", cmd_vel(0.0, 1.5, 8.0)),
            Step(22, "Over the roof (1 m/s for 4 s)", cmd_vel(1.0, 0.0, 4.0)),
            Step(29, "On the roof the wake spreads as on the ground, and stops at its edges"),
        ],
    ),
    Clip(
        "drone_live_air",
        "Live air: a lattice Boltzmann box round the drone, down to a low hover",
        30,
        [["@run", LIVE_AIR, "--robot", QUAD, "--world", "none"]],
        [
            Pane(
                "3D view: particles in the live air",
                HALF_L,
                VIEWPORT_HALF,
                ["--overlays", "airflow", *view(0, 0, 1.0, 6.0, -60, 12)],
            ),
            Pane(
                "air speed through the drone (XZ)",
                HALF_R,
                ENVIRONMENT_SLICE,
                ["--workspace", "Environment", "--slice", "aero/wind_speed:xz"],
            ),
        ],
        [
            Step(0, "downwash_model: lbm, the air computed every step in a 3 m box, 4 cm cells"),
            Step(10, "Down to about half a metre (/cmd_vel, 0.5 m/s down for 3 s)", DESCEND),
            Step(17, "Low: the jets hit the ground, meet under the drone and rise (the fountain)"),
        ],
    ),
    Clip(
        "plugin",
        "A plugin: a sensor Lagrange does not have, as a shared library",
        20,
        [["--robot", "amr_tof.urdf", "--world", WAREHOUSE, "--plugin", "lagrange_tof"]],
        [
            Pane("3D view", LEFT, viewer=["--workspace", "Scene", "--follow", "9,65"]),
            Pane(
                "sensor/front_tof/range (m), from the plugin",
                RIGHT,
                SCOPE,
                scope("sensor/front_tof/range"),
            ),
        ],
        [
            Step(
                0,
                "lagrange robot.urdf world.usda --plugin lagrange_tof    "
                "(core/plugin_abi/samples/tof_sensor.c)",
            )
        ],
        tof_robot,
    ),
    Clip(
        "sensor_delay",
        "Sensor delay: what the robot gets, late",
        18,
        [
            [
                "--robot",
                AMR,
                "--world",
                WAREHOUSE,
                "--sensor-timing",
                "imu=0.4,0.02",
                "--sensor-timing",
                "front_lidar=0.4,0.02",
            ]
        ],
        [
            Pane(
                "lidar returns, as they arrive (0.4 s late)",
                LEFT,
                viewer=["--workspace", "Scene", "--follow", "9,60", "--overlays", "sensors"],
            ),
            Pane(
                "yaw rate: on time and delayed (rad/s)",
                RIGHT,
                SCOPE,
                scope("sensor/imu/angular_rate.z", "sensor/imu/delayed/angular_rate.z"),
            ),
        ],
        [
            Step(
                0,
                "--sensor-timing imu=0.4,0.02 --sensor-timing front_lidar=0.4,0.02    "
                "(latency, jitter in s)",
            )
        ],
    ),
    Clip(
        "water",
        "Water: buoyancy and drag",
        16,
        [["--robot", "none", "--world", scene("crates.sdf"), "--water", "0,0,1,12,8,1"]],
        [
            Pane("3D view", LEFT, viewer=view(0, 0, 1.2, 8, -90, 12)),
            Pane(
                "height of each crate (m)",
                RIGHT,
                SCOPE,
                scope("body/link/pose.z", "body/steel_link/pose.z"),
            ),
        ],
        [
            Step(
                0,
                "--water 0,0,1,12,8,1    Wood floats at its draft; steel sinks",
            )
        ],
    ),
    Clip(
        "lidar_materials",
        "Lidar reflection: intensity from material, angle and range",
        14,
        [["--robot", AMR, "--world", scene("material_walls.urdf")]],
        [
            Pane(
                "the walls: retroreflector, white paint, asphalt, black rubber",
                HALF_L,
                viewer=view(0, 0, 0.5, 14, -90, 55),
            ),
            Pane(
                "what the lidar sees, bright = strong return",
                HALF_R,
                BIRD,
                ["--workspace", "Sensors"],
            ),
        ],
        [Step(0, '<gazebo reference="wall"><sensor_material preset="retroreflector"/></gazebo>')],
    ),
    Clip(
        "lidar_water",
        "Lidar reflection: water sends the beams away",
        14,
        [["--robot", AMR, "--world", WAREHOUSE, "--spawn", "0,0", "--water", "2,0,0.02,6,6,0.02"]],
        [
            Pane(
                "3D view: returns are crosses",
                HALF_L,
                viewer=["--workspace", "Scene", "--follow", "8,45", "--overlays", "sensors"],
            ),
            Pane("what the lidar sees", HALF_R, BIRD, ["--workspace", "Sensors"]),
        ],
        [
            Step(
                0,
                "At a slant the surface is a mirror: no return, also not from the ground below it",
            )
        ],
    ),
    Clip(
        "lidar_motion_distortion",
        "Lidar motion distortion: columns fire from where the sensor is",
        14,
        [
            ["--robot", scene("spin_lidar.urdf"), "--world", "none", "--joint-control"],
            ["--robot", scene("spin_lidar_off.urdf"), "--world", "none", "--joint-control"],
        ],
        [
            Pane(
                "distortion on (the default): the square room skews",
                HALF_L,
                BIRD,
                ["--workspace", "Sensors"],
            ),
            Pane(
                "distortion off: one instant, a clean square",
                HALF_R,
                BIRD,
                ["--workspace", "Sensors"],
                run=1,
            ),
        ],
        [
            Step(0, "A 5 Hz lidar in a square room, starting up"),
            Step(1, "Turning at 1 rev/s", TURN),
            Step(1, "Turning at 1 rev/s", TURN, run=1),
        ],
    ),
    Clip(
        "joint_control",
        "Joint control: position, velocity or effort per joint",
        20,
        [["--robot", ARM, "--world", "none", "--joint-control"]],
        [
            Pane("3D view", LEFT, viewer=view(0, 0, 0.5, 3, -60, 20)),
            Pane(
                "joint positions (rad)",
                RIGHT,
                SCOPE,
                scope("joint/shoulder_pan_joint/position", "joint/elbow_joint/position"),
            ),
        ],
        [
            Step(0, "ros2 topic pub /joint_commands sensor_msgs/msg/JointState ..."),
            Step(
                3,
                "position: lift -1.2, elbow 1.4, wrist -1.0",
                joints(
                    position={
                        "shoulder_pan_joint": 0.0,
                        "shoulder_lift_joint": -1.2,
                        "elbow_joint": 1.4,
                        "wrist_1_joint": -1.0,
                        "wrist_2_joint": 0.0,
                        "wrist_3_joint": 0.0,
                    }
                ),
            ),
            Step(
                9,
                "velocity: shoulder pan at 0.6 rad/s",
                joints(velocity={"shoulder_pan_joint": 0.6}),
            ),
            Step(
                15, "position: shoulder pan back to 0", joints(position={"shoulder_pan_joint": 0.0})
            ),
        ],
    ),
    Clip(
        "motor_modes",
        "Motor modes: a DC motor on volts",
        20,
        [
            [
                "--robot",
                scene("motor.urdf"),
                "--overlay",
                scene("motor.overlay.yaml"),
                "--world",
                "none",
                "--joint-control",
            ]
        ],
        [
            Pane("3D view", LEFT, viewer=view(0, 0, 0.8, 3.5, -90, 10)),
            Pane(
                "speed (rad/s), current (A)",
                RIGHT,
                SCOPE,
                scope("joint/spin/velocity", "motor/motor/current_q"),
            ),
        ],
        [
            Step(0, "elec:controlMode: voltage"),
            Step(3, "6 V: no-load speed 12 rad/s", joints(effort={"spin": 6.0})),
            Step(9, "12 V: 24 rad/s", joints(effort={"spin": 12.0})),
            Step(15, "0 V: the motor brakes it", joints(effort={"spin": 0.0})),
        ],
    ),
    Clip(
        "spawn",
        "Spawning: robots and people while it runs",
        20,
        [["--robot", "none", "--world", "none"]],
        [
            Pane(
                "3D view",
                (0, 0, 1280, 720),
                (1920, 1080, (289, 127, 1569, 847)),
                view(0, 0, 0.5, 10, -90, 25),
            )
        ],
        [
            Step(0, "lagrange empty"),
            Step(
                3, "lagrange spawn amr_diffdrive.urdf", ["lagrange", "spawn", AMR, "--at", "-2,0"]
            ),
            Step(
                8,
                "lagrange spawn adult.usda --at 2,0",
                ["lagrange", "spawn", asset("humans/adult.usda"), "--at", "2,0"],
            ),
            Step(
                13,
                "lagrange spawn amr_diffdrive.urdf --name mate --at 0,3",
                ["lagrange", "spawn", AMR, "--name", "mate", "--at", "0,3"],
            ),
            Step(17, "lagrange delete mate", ["lagrange", "delete", "mate"]),
        ],
    ),
    Clip(
        "run_file",
        "The run file: one file for a whole run",
        14,
        [["@cli", scene("run.yaml")]],
        [
            Pane("run.yaml", (0, 0, 520, 720), text=scene("run.yaml")),
            Pane(
                "lagrange run.yaml",
                (520, 0, 760, 720),
                VIEWPORT,
                ["--workspace", "Scene", "--follow", "9,35"],
            ),
        ],
        [Step(0, "lagrange run.yaml")],
    ),
    # v1.3 (docs/release/v1.3.0.md). Scripted, not yet recorded: see docs/videos/README.md.
    Clip(
        "radar_4d",
        "A 4D imaging radar: detections by range, azimuth, elevation and Doppler,"
        " in 3D and in its view",
        26,
        [["--robot", SEDAN_4D, "--world", TOWN, *DRIVE]],
        [
            # Behind the car, the follow camera keeping it in the middle; Overlays → Radar
            # detections: boxes sized by RCS, coloured by radial velocity (blue closing).
            Pane(
                "3D view: Overlays → Radar detections",
                LEFT,
                viewer=[
                    "--overlays",
                    "radar",
                    "--follow",
                    "12,24",
                    *view(30, -1.75, 0, 12, 180, 24),
                ],
            ),
            # The Sensors workspace with front_4d_radar picked in its list (the fifth entry):
            # the Radar view (fan) and the camera above it, from one viewer.
            Pane(
                "Sensors → Radar: front_4d_radar, range-azimuth",
                RIGHT_TOP,
                RADAR_PANEL,
                ["--workspace", "Sensors", "--input", "20:click:80,385"],
            ),
            Pane("front_camera", RIGHT_BOTTOM, CAMERA_IMAGE, of=1),
        ],
        [
            Step(
                0,
                "sedan_4d_radar.usda: kind imaging_radar, 64 x 16 virtual channels, 77 GHz,"
                " Doppler folding, ground ghosts",
            ),
            Step(6, "Standing: the walls and parked things are white, at 0 m/s"),
            # The car's /cmd_vel: forward speed (m/s), as `lagrange drive` takes it.
            Step(
                11,
                "/cmd_vel 4 m/s for 9 s: all that stands now closes on it, blue",
                cmd_vel(4.0, 0.0, 9.0),
            ),
        ],
    ),
    Clip(
        "sensor_faults",
        "Sensor faults: a gyro biased, then stuck, while the truth goes on",
        30,
        [
            [
                "--robot",
                AMR,
                "--world",
                WAREHOUSE,
                "--sensor-fault",
                "imu:angular_rate=bias,value=0.5,at=8,for=6,component=2",
                "--sensor-fault",
                "imu:angular_rate=stuck,at=21,for=6",
            ]
        ],
        [
            Pane("3D view", LEFT, viewer=["--workspace", "Scene", "--follow", "3.5,65"]),
            Pane(
                "IMU yaw rate; true yaw rate; fault on",
                RIGHT,
                SCOPE,
                # sensor/<n>/fault/active: modules/sensors/src/Faults.cpp; twist.wz: the Wrench
                # of m/s's component names (server/src/Server.cpp componentNames).
                scope(
                    "sensor/imu/angular_rate.z",
                    "body/base_link/twist.wz",
                    "sensor/imu/fault/active",
                ),
            ),
        ],
        [
            # Fault times are simulated time, about 1.5 s ahead of the clip's. The figure-eight
            # turns the other way about every 12 s (at 11.5 and 24.5 s of simulated time): each
            # fault holds one change of direction, which the stuck gyro then misses.
            Step(0, "--sensor-fault imu:angular_rate=bias,value=0.5,at=8,for=6,component=2"),
            Step(6.5, "Biased: the gyro reads 0.5 rad/s more than the robot turns"),
            Step(15, "--sensor-fault imu:angular_rate=stuck,at=21,for=6"),
            Step(
                19.5, "Stuck: the gyro holds its last reading while the robot turns the other way"
            ),
            Step(26, "Ground truth (/ground_truth/odom, /tf, the bodies' signals) is not faulted"),
        ],
    ),
    Clip(
        "lidar_patterns",
        "Lidar scan patterns: a channel table, a flash lidar, a Livox-style rosette",
        14,
        [
            ["--robot", "lidar_channels.urdf", "--world", "none"],
            ["--robot", "lidar_flash.urdf", "--world", "none"],
            ["--robot", "lidar_rosette.urdf", "--world", "none"],
        ],
        [
            Pane(
                "channels: 16 rings, dense at the horizon",
                THIRD_L,
                VIEWPORT_THIRD,
                ["--overlays", "sensors", *view(3.5, 0, 1.2, 11, 180, 14)],
            ),
            Pane(
                "flash: 160 x 120, all at once",
                THIRD_M,
                VIEWPORT_THIRD,
                ["--overlays", "sensors", *view(3.5, 0, 1.2, 11, 180, 14)],
                run=1,
            ),
            Pane(
                "rosette: 10 000 points a scan",
                THIRD_R,
                VIEWPORT_THIRD,
                ["--overlays", "sensors", *view(3.5, 0, 1.2, 11, 180, 14)],
                run=2,
            ),
        ],
        [
            Step(
                0,
                "<scan><vertical><angles>    <pattern>flash</pattern>    "
                "<pattern>rosette</pattern>    (URDF, SDFormat, USD: RFC 0015)",
            ),
            Step(7, "The rosette never repeats: each 0.1 s scan goes on where the last stopped"),
        ],
        lidar_patterns,
    ),
    Clip(
        "fisheye",
        "A fisheye camera: 220° on one image",
        16,
        [
            ["--robot", "amr_pinhole.urdf", "--world", WAREHOUSE, "--paused"],
            ["--robot", "amr_fisheye.urdf", "--world", WAREHOUSE, "--paused"],
        ],
        [
            Pane("pinhole, 90°", HALF_L, CAMERA_SQUARE, ["--workspace", "Sensors"]),
            Pane(
                "equidistant fisheye, 220°",
                HALF_R,
                CAMERA_SQUARE,
                ["--workspace", "Sensors"],
                run=1,
            ),
        ],
        [
            Step(0, FISHEYE_LINE, PLAY),
            Step(0, FISHEYE_LINE, PLAY, run=1),
        ],
        amr_cameras,
    ),
    Clip(
        "event_camera",
        "An event camera: only what changes",
        16,
        [
            ["--robot", "amr_camera_346.urdf", "--world", WAREHOUSE, "--paused"],
            ["--robot", "amr_dvs.urdf", "--world", WAREHOUSE, "--paused"],
        ],
        [
            Pane("camera, 346 x 260", HALF_L, CAMERA_PANEL, ["--workspace", "Sensors"]),
            Pane(
                "event camera: ON brighter, OFF darker",
                HALF_R,
                CAMERA_PANEL,
                ["--workspace", "Sensors"],
                run=1,
            ),
        ],
        [
            Step(0, EVENT_LINE, PLAY),
            Step(0, EVENT_LINE, PLAY, run=1),
            Step(9.5, "Along a plain wall: no edges cross the pixels, so hardly any events"),
        ],
        amr_cameras,
    ),
    Clip(
        "amr_sensor_kinds",
        "Built-in kinds on an AMR: a safety scanner's fields, wheel torque",
        26,
        # Driven by /cmd_vel from the usual start (facing +x): a quarter turn left, then up to the
        # north rack (face at y = 3.0) into the 1.5 m warning field, short of the 0.6 m protective
        # one, and back.
        [["--robot", AMR_SAFETY, "--world", WAREHOUSE, "--no-demo"]],
        [
            # From the south, looking north at the rack the robot drives up to.
            Pane("3D view", LEFT, viewer=view(0, 1.6, 0, 6, -90, 45)),
            Pane(
                "safety: nearest return (m); warning, protective; left wheel torque (N·m)",
                RIGHT,
                SCOPE,
                # modules/sensors/src/kinds/SafetyScanner.cpp, ForceTorque.cpp (joint_torque).
                scope(
                    "sensor/safety/min_distance",
                    "sensor/safety/warning",
                    "sensor/safety/protective",
                    "sensor/left_wheel_torque/torque",
                ),
            ),
        ],
        [
            Step(
                0,
                "amr_diffdrive_safety.urdf: bumpers, cliff sensors, wheel odometry, a safety"
                " scanner, wheel torque",
            ),
            Step(3, "/cmd_vel: a quarter turn left", twist(0.0, 0.6, 2.6)),
            Step(7, "/cmd_vel 0.3 m/s towards the rack", twist(0.3, 0.0, 6.0)),
            Step(
                10,
                '<sensor type="safety_scanner">: warning 1.5 m, protective 0.6 m ahead'
                " (nanoScan3 style)",
            ),
            Step(17, "/cmd_vel -0.3 m/s: back out of the warning field", twist(-0.3, 0.0, 5.0)),
        ],
    ),
]


def font(size: int) -> ImageFont.FreeTypeFont:
    for path in ROOT.glob("viewer/assets/fonts/*.ttf"):
        if "Mono" not in path.name and "Regular" in path.name:
            return ImageFont.truetype(str(path), size)
    return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)


def text_pane(path: str, size: tuple[int, int]) -> Image.Image:
    image = Image.new("RGB", size, (12, 16, 24))
    draw = ImageDraw.Draw(image)
    mono = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 13)
    draw.multiline_text(
        (12, 34), Path(path).read_text(), font=mono, fill=(210, 220, 230), spacing=7
    )
    return image


def picture(clip: Clip, frames: list[Image.Image | None], line: str, out: Path) -> None:
    """The panes, and a band under them: what the clip is, and what was just done."""
    whole = Image.new("RGB", (SIZE[0], SIZE[1] + 64), (16, 18, 22))
    draw = ImageDraw.Draw(whole)
    for pane, frame in zip(clip.panes, frames, strict=True):
        x, y, width, height = pane.box
        if pane.text is not None:
            whole.paste(text_pane(pane.text, (width, height)), (x, y))
        elif frame is not None:
            part = frame.crop(pane.view[2])
            scale = min(width / part.width, height / part.height)
            size = (round(part.width * scale), round(part.height * scale))
            whole.paste(part.resize(size, Image.Resampling.LANCZOS),
                        (x + (width - size[0]) // 2, y + (height - size[1]) // 2))  # fmt: skip
        width_of_label = int(draw.textlength(pane.label, font=font(15)))
        if not pane.label:
            continue
        draw.rectangle((x + 8, y + 8, x + 16 + width_of_label, y + 30),
                       fill=(12, 16, 24))  # fmt: skip
        draw.text((x + 12, y + 10), pane.label, font=font(15), fill=(235, 235, 235))
    draw.text((16, SIZE[1] + 6), clip.title, font=font(20), fill=(235, 235, 235))
    draw.text((16, SIZE[1] + 34), line, font=font(17), fill=(120, 200, 255))
    whole.save(out)


def ffmpeg() -> list[str]:
    if os.environ.get("FFMPEG"):
        return [os.environ["FFMPEG"]]
    if found := shutil.which("ffmpeg"):
        return [found]
    uvx = shutil.which("uvx") or str(Path.home() / ".local/bin/uvx")
    code = "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())"
    path = subprocess.check_output(
        [uvx, "--from", "imageio-ffmpeg", "python", "-c", code], text=True
    )
    return [path.strip()]


def encode(frames: Path, fps: int, out: Path) -> None:
    """H.264 that any browser plays, at the best quality that stays under the limit."""
    for crf in range(24, 46, 3):
        subprocess.run(
            [*ffmpeg(), "-y", "-loglevel", "error", "-framerate", str(fps), "-i",
             str(frames / "c_%05d.png"), "-c:v", "libx264", "-preset", "veryslow", "-crf", str(crf),
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-vf",
             "pad=ceil(iw/2)*2:ceil(ih/2)*2", str(out)],
            check=True,
        )  # fmt: skip
        if out.stat().st_size <= MAX_BYTES:
            print(f"  {out.name}: {out.stat().st_size / 1e6:.2f} MB, crf {crf}")
            return
    raise SystemExit(f"{out} stays over {MAX_BYTES} bytes")


def stop(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def wait_for(path: Path, what: subprocess.Popen[bytes], log: Path | None = None) -> None:
    for _ in range(3000):
        if path.exists():
            return
        if what.poll() is not None:
            raise SystemExit(log.read_text()[-2000:] if log else f"no {path}")
        time.sleep(0.02)
    raise SystemExit(f"no {path}")


def record(clip: Clip, out_dir: Path, keep: bool) -> None:
    print(clip.name, flush=True)
    work = Path(tempfile.mkdtemp(prefix=f"lagrange_video_{clip.name}_"))
    # Sockets in a short directory of their own: a socket's path holds at most 107 bytes, and
    # $TMPDIR may be deep.
    sockets = Path(tempfile.mkdtemp(prefix="lgv_", dir="/tmp"))
    if clip.write:
        clip.write(work)
    envs: list[dict[str, str]] = []
    servers: list[subprocess.Popen[bytes]] = []
    viewers: list[subprocess.Popen[bytes]] = []
    sent: list[tuple[float, str]] = []
    count = int((clip.seconds + 6) * FPS)
    try:
        for r, args in enumerate(clip.runs):
            # The first run in this shell's ROS_DOMAIN_ID (80 when unset), the others 60 apart
            # (at most 232 for Fast DDS), so that a shared machine's other runs stay apart.
            base = int(os.environ.get("ROS_DOMAIN_ID", "80"))
            env = dict(os.environ, ROS_DOMAIN_ID=str((base + 60 * r) % 233))
            envs.append(env)
            socket = sockets / f"run{r}.sock"
            if args[0] == "@cli":
                command = ["lagrange", *args[1:], "--server-only", "--listen", f"unix:{socket}"]
            elif args[0] == "@run":  # a profile: lagrange run PROFILE ...
                command = ["ros2", "run", "lagrange_server", "lagrange", "run", *args[1:],
                           "--listen", f"unix:{socket}"]  # fmt: skip
            else:
                command = ["ros2", "run", "lagrange_server", "lagrange_server",
                           "--listen", f"unix:{socket}", *args]  # fmt: skip
            log = work / f"run{r}.log"
            servers.append(subprocess.Popen(command, env=env, cwd=work, stdout=log.open("wb"),
                                            stderr=subprocess.STDOUT))  # fmt: skip
            wait_for(socket, servers[-1], log)
        for p, pane in enumerate(clip.panes):
            if pane.text is not None or pane.of is not None:
                continue
            viewers.append(subprocess.Popen(
                ["ros2", "run", "lagrange_viewer", "lagrange_viewer",
                 "--connect", f"unix:{sockets}/run{pane.run}.sock", "--headless",
                 "--size", f"{pane.view[0]}x{pane.view[1]}", "--wait-ms", "60000",
                 "--record-frames", str(work / f"pane{p}"), "--record-fps", str(FPS),
                 "--frames", str(count), *pane.viewer],
                env=envs[pane.run], cwd=work, stdout=subprocess.DEVNULL,
            ))  # fmt: skip
        live = [p for p, pane in enumerate(clip.panes) if pane.text is None and pane.of is None]
        for p, viewer in zip(live, viewers, strict=True):
            wait_for(work / f"pane{p}" / "frame_00000.png", viewer)
        start = time.time()
        # Each command runs beside the clip, its caption from the moment it was sent: a
        # `ros2 topic pub -t N` that streams for seconds does not hold back the steps after it.
        commands: list[tuple[list[str], subprocess.Popen[bytes]]] = []
        for step in sorted(clip.steps, key=lambda s: s.at):
            time.sleep(max(step.at - (time.time() - start), 0.0))
            if step.command:
                commands.append((step.command, subprocess.Popen(
                    step.command, env=envs[step.run], cwd=work,
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                )))  # fmt: skip
            sent.append((time.time() - start, step.caption))
        for command, process in commands:
            _, error = process.communicate(timeout=60)
            if process.returncode != 0:
                raise SystemExit(f"{command}: {error.decode()[-500:]}")
        for viewer in viewers:
            if viewer.wait(clip.seconds * 4 + 120) != 0:
                raise SystemExit("a viewer failed")
    finally:
        for process in [*viewers, *servers]:
            stop(process)
        shutil.rmtree(sockets, ignore_errors=True)
    # By the clock: each pane's latest frame at each moment, so the panes stay together.
    stamped = {p: sorted(((f.stat().st_mtime, f) for f in (work / f"pane{p}").glob("frame_*.png")),
                         key=lambda item: item[0]) for p in live}  # fmt: skip
    finished = work / "finished"
    finished.mkdir()
    next_frame = dict.fromkeys(live, 0)
    for index in range(int(clip.seconds * FPS)):
        now = start + index / FPS
        images: list[Image.Image | None] = []
        for p, pane in enumerate(clip.panes):
            if pane.of is not None:
                images.append(images[pane.of])
                continue
            if p not in stamped:
                images.append(None)
                continue
            frames = stamped[p]
            while next_frame[p] + 1 < len(frames) and frames[next_frame[p] + 1][0] <= now:
                next_frame[p] += 1
            images.append(Image.open(frames[next_frame[p]][1]).convert("RGB"))
        line = next((text for at, text in reversed(sent) if at <= index / FPS), "")
        picture(clip, images, line, finished / f"c_{index:05d}.png")
    out_dir.mkdir(parents=True, exist_ok=True)
    encode(finished, FPS, out_dir / f"{clip.name}.mp4")
    if keep:
        print(f"  frames in {finished}")
    else:
        shutil.rmtree(work)


def wind_tunnel_wake(out_dir: Path, keep: bool) -> None:
    """The wake moving: lagrange tunnel's own frames under the band the other clips have."""
    print("wind_tunnel_wake", flush=True)
    work = Path(tempfile.mkdtemp(prefix="lagrange_video_wake_"))
    ahmed_tunnel(work, frames=200)
    finished = work / "finished"
    finished.mkdir()
    title = (
        "lagrange tunnel --frames: an Ahmed body, 40 m/s. "
        "Top: speed, 0 to 56 m/s. Below: Cp, -1 to 1"
    )
    line = (
        "Coarse: for where the air goes. Cd 1.35 against a measured 0.29 to 0.30. "
        "The stripes are a known defect"
    )
    for index, frame in enumerate(sorted((work / "tunnel" / "frames").glob("*.png"))):
        image = Image.open(frame).convert("RGB")
        scale = min(SIZE[0] / image.width, SIZE[1] / image.height)
        size = (round(image.width * scale), round(image.height * scale))
        whole = Image.new("RGB", (SIZE[0], size[1] + 64), (16, 18, 22))
        whole.paste(image.resize(size, Image.Resampling.NEAREST), ((SIZE[0] - size[0]) // 2, 0))
        draw = ImageDraw.Draw(whole)
        draw.text((16, size[1] + 6), title, font=font(20), fill=(235, 235, 235))
        draw.text((16, size[1] + 34), line, font=font(17), fill=(120, 200, 255))
        whole.save(finished / f"c_{index:05d}.png")
    out_dir.mkdir(parents=True, exist_ok=True)
    encode(finished, FPS, out_dir / "wind_tunnel_wake.mp4")
    if keep:
        print(f"  frames in {finished}")
    else:
        shutil.rmtree(work)


# --- the robot builder: the viewer drives itself (--input), no server of the clip's own ----------


@dataclass
class Script:
    """A headless viewer's --input, written as a person would act, one viewer frame per video frame.

    With --record-fps above the frame rate every frame the viewer counts is written, so an event
    at frame N is the clip's frame N: the captions and the drawn pointer follow the same count.
    Frames held by a wait (the test server starting) are neither counted nor written.
    """

    frame: int = 3
    events: list[str] = field(default_factory=list)
    pointer: list[tuple[int, float, float, bool]] = field(default_factory=list)  # frame, x, y, down
    captions: list[tuple[int, str]] = field(default_factory=list)
    at: tuple[float, float] = (800.0, 500.0)

    def caption(self, text: str) -> None:
        self.captions.append((self.frame, text))

    def pause(self, frames: int) -> None:
        self.frame += frames

    def move(self, x: float, y: float, glide: int = 8) -> None:
        """Glide the pointer there over `glide` frames."""
        x0, y0 = self.at
        for i in range(1, glide + 1):
            px, py = x0 + (x - x0) * i / glide, y0 + (y - y0) * i / glide
            self.events.append(f"{self.frame}:move:{px:.0f},{py:.0f}")
            self.pointer.append((self.frame, px, py, False))
            self.frame += 1
        self.at = (x, y)

    def click(self, x: float, y: float, glide: int = 8, after: int = 6) -> None:
        self.move(x, y, glide)
        self.events.append(f"{self.frame}:click:{x:.0f},{y:.0f}")
        self.pointer.append((self.frame + 1, x, y, True))
        self.pointer.append((self.frame + 2, x, y, False))
        self.frame += 3 + after

    def drag(self, start: tuple[float, float], to: tuple[float, float], frames: int) -> None:
        self.move(*start)
        self.events.append(f"{self.frame}:down")
        self.pointer.append((self.frame, *start, True))
        self.frame += 2
        for i in range(1, frames + 1):
            px = start[0] + (to[0] - start[0]) * i / frames
            py = start[1] + (to[1] - start[1]) * i / frames
            self.events.append(f"{self.frame}:move:{px:.0f},{py:.0f}")
            self.pointer.append((self.frame, px, py, True))
            self.frame += 1
        self.events.append(f"{self.frame}:up")
        self.pointer.append((self.frame, *to, False))
        self.at = to
        self.frame += 4

    def type(self, text: str, per: int = 2) -> None:
        """One character every `per` frames, as typed; all at once with per = 0."""
        if per == 0:
            self.events.append(f"{self.frame}:text:{text}")
            self.frame += 1
            return
        for character in text:
            self.events.append(f"{self.frame}:text:{character}")
            self.frame += per

    def key(self, name: str, after: int = 3) -> None:
        self.events.append(f"{self.frame}:key:{name}")
        self.frame += 2 + after

    def select(self, name: str, after: int = 4) -> None:
        """Select a link, joint or sensor by name, as a click on its row in the tree does."""
        self.events.append(f"{self.frame}:select:{name}")
        self.frame += 1 + after

    def wait(self, what: str) -> None:
        self.events.append(f"{self.frame}:wait:{what}")
        self.frame += 1


# Where things are at 1600 x 1000 in the Builder's default layout with a new robot open
# (viewer/test/test_builder_acceptance.py and the other test_builder_*.py).
B_VIEW = (760, 450)
B_ADD_MATERIAL, B_PLA = (120, 905), (160, 932)
B_BOX, B_CYLINDER, B_MESH = (40, 835), (101, 835), (225, 835)
B_MESH_PATH, B_MESH_ADD = (400, 920), (280, 971)
B_BOX_JOINT = (115, 221)  # "fixed · box" on box_link's row: selects its joint
B_REVOLUTE = (1425, 272)
B_AXIS_Y, B_LOWER = (1475, 448), (1470, 517)
B_SLIDER = (562, 812)  # the first moving joint's slider at zero, in Joints
B_CATALOG, B_FILTER, B_FIRST_ADD = (860, 763), (950, 793), (1205, 845)
B_SHAPE_TRANSLATE_X = (1400, 401)  # a mesh link's Properties: its shape's translate
B_SAME_AS_MESH = (1415, 648)
B_TEST = (1430, 48)


def builder_script(mesh: Path) -> Script:
    s = Script()
    s.caption("lagrange viewer --workspace builder: a new robot is one link, a box")
    s.pause(30)
    s.move(*B_VIEW, glide=1)
    s.events.append(f"{s.frame}:wheel:3")  # closer in
    s.pause(12)
    s.caption("Add: a material for the next parts (PLA, 1240 kg/m³), the mass follows")
    s.click(*B_ADD_MATERIAL)
    s.click(*B_PLA, after=12)
    s.caption("Add → Box: a link on the selected one, by a joint")
    s.click(*B_BOX, after=18)
    s.caption("Add → Cylinder, on the box")
    s.click(*B_CYLINDER, after=18)
    s.caption("The box's joint: revolute about y, limits -90 … 90°, 2 N·m, 3 rad/s")
    s.click(*B_BOX_JOINT, after=8)
    s.click(*B_REVOLUTE, after=8)
    s.click(*B_AXIS_Y, after=2)
    s.key("ctrl+a")
    s.type("1")
    s.key("tab")
    s.key("ctrl+a")
    s.type("0")
    s.key("enter", after=6)
    s.click(*B_LOWER, after=2)
    for value, last in (("-90", False), ("90", False), ("2", False), ("3", True)):
        s.type(value)
        s.key("enter" if last else "tab", after=4)
    s.pause(8)
    s.caption("Joints: drag its slider, the view poses the robot (no physics, no undo step)")
    s.drag(B_SLIDER, (B_SLIDER[0] + 30, B_SLIDER[1]), 14)
    s.pause(6)
    s.drag((B_SLIDER[0] + 30, B_SLIDER[1]), (B_SLIDER[0] - 26, B_SLIDER[1]), 22)
    s.pause(6)
    s.drag((B_SLIDER[0] - 26, B_SLIDER[1]), B_SLIDER, 10)
    s.pause(10)
    s.caption("Sensor catalog: a lidar on the cylinder")
    s.select("cylinder_link")
    s.click(*B_CATALOG, after=4)
    s.click(*B_FILTER, after=2)
    s.type("lidar")
    s.move(B_FIRST_ADD[0] - 200, B_FIRST_ADD[1])
    s.click(*B_FIRST_ADD, after=20)
    s.caption("… and a rangefinder on the box, its range, rate and noise in Properties")
    s.select("box_link")
    s.click(*B_FILTER, after=4)
    s.key("ctrl+a")
    s.type("rangefinder")
    s.move(B_FIRST_ADD[0] - 200, B_FIRST_ADD[1])
    s.click(*B_FIRST_ADD, after=24)
    s.caption("Add → Mesh…: an L-shaped part from an OBJ file, on base_link")
    s.select("base_link")
    s.click(*B_MESH, after=4)
    s.click(*B_MESH_PATH, after=2)
    s.type(str(mesh), per=0)
    s.pause(6)
    s.key("enter", after=4)
    s.click(*B_MESH_ADD, after=10)
    s.click(*B_SHAPE_TRANSLATE_X, after=2)
    for value, last in (("0.15", False), ("-0.1", False), ("-0.1", True)):
        s.key("ctrl+a")
        s.type(value)
        s.key("enter" if last else "tab", after=4)
    s.pause(6)
    s.caption(
        "Collision: same as the mesh collides as its convex hull (outlined), the notch filled"
    )
    s.click(*B_SAME_AS_MESH, after=40)
    s.caption(
        "Test · empty world: its own lagrange_server runs the robot; the free joint tips over"
    )
    s.click(*B_TEST, after=0)
    s.wait("connected")
    s.pause(30)
    s.caption("Joints, while testing: the slider commands the joint, the simulated robot moves")
    s.drag(B_SLIDER, (B_SLIDER[0] + 30, B_SLIDER[1]), 14)
    s.pause(36)
    s.drag((B_SLIDER[0] + 30, B_SLIDER[1]), (B_SLIDER[0] - 26, B_SLIDER[1]), 20)
    s.pause(36)
    s.caption("Back to Build (Esc): the test server stops, the document is as it was")
    s.key("escape", after=0)
    s.wait("build")
    s.pause(40)
    return s


def pointer_arrow(draw: ImageDraw.ImageDraw, x: float, y: float, down: bool) -> None:
    """A mouse pointer at (x, y) of the picture; a ring while the button is down."""
    if down:
        draw.ellipse((x - 14, y - 14, x + 14, y + 14), outline=(255, 160, 80), width=3)
    shape = [(0, 0), (0, 17), (4, 13), (7, 20), (10, 19), (7, 12), (12, 12)]
    points = [(x + px, y + py) for px, py in shape]
    draw.polygon(points, fill=(250, 250, 250), outline=(10, 10, 10))


def builder_clip(out_dir: Path, keep: bool) -> None:
    """The Builder: a robot made from nothing, tested in its own simulator, and back."""
    print("builder", flush=True)
    work = Path(tempfile.mkdtemp(prefix="lagrange_video_builder_"))
    mesh = work / "l_bracket.obj"
    # An L-shaped prism, 0.2 by 0.2 by 0.05 m with a 0.08 m arm: closed and concave
    # (viewer/test/test_builder_acceptance.py write_ell).
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
    mesh.write_text("\n".join(lines) + "\n")
    robot = work / "my_robot.urdf"
    shutil.copy(ROOT / "viewer" / "test" / "builder" / "empty.urdf", robot)
    script = builder_script(mesh)
    frames = work / "frames"
    env = dict(os.environ, HOME=str(work), XDG_CONFIG_HOME=str(work / "cfg"),
               XDG_STATE_HOME=str(work / "state"))  # fmt: skip
    done = subprocess.run(
        ["ros2", "run", "lagrange_viewer", "lagrange_viewer", "--headless", "--size", "1600x1000",
         "--workspace", "builder", "--builder-open", str(robot), "--input",
         ";".join(script.events), "--record-frames", str(frames), "--record-fps", "1000",
         "--frames", str(script.frame)],
        env=env, cwd=work, capture_output=True, text=True, check=False,
    )  # fmt: skip
    if done.returncode != 0:
        raise SystemExit(f"the viewer failed: {done.stderr[-2000:]}")
    finished = work / "finished"
    finished.mkdir()
    title = "The robot builder: make a robot in the viewer, test it, save a URDF"
    scale = SIZE[0] / 1600
    pointer = sorted(script.pointer)
    p = 0
    where = (800.0, 500.0, False)
    for index, frame in enumerate(sorted(frames.glob("frame_*.png"))):
        while p < len(pointer) and pointer[p][0] <= index:
            where = pointer[p][1:]
            p += 1
        image = Image.open(frame).convert("RGB")
        size = (SIZE[0], round(image.height * scale))
        whole = Image.new("RGB", (SIZE[0], size[1] + 64), (16, 18, 22))
        whole.paste(image.resize(size, Image.Resampling.LANCZOS), (0, 0))
        draw = ImageDraw.Draw(whole)
        pointer_arrow(draw, where[0] * scale, where[1] * scale, where[2])
        line = next((text for at, text in reversed(script.captions) if at <= index), "")
        draw.text((16, size[1] + 6), title, font=font(20), fill=(235, 235, 235))
        draw.text((16, size[1] + 34), line, font=font(17), fill=(120, 200, 255))
        whole.save(finished / f"c_{index:05d}.png")
        frame.unlink()  # disk: the composed frame is all that is needed now
    out_dir.mkdir(parents=True, exist_ok=True)
    encode(finished, FPS, out_dir / "builder.mp4")
    if keep:
        print(f"  frames in {finished}")
    else:
        shutil.rmtree(work)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clips", nargs="*", help="names; all by default")
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "videos")
    parser.add_argument("--keep", action="store_true", help="keep the frames")
    args = parser.parse_args()
    known = {clip.name: clip for clip in CLIPS}
    for name in args.clips:
        if name not in known and name not in ("wind_tunnel_wake", "builder"):
            raise SystemExit(f"no clip {name}; one of {', '.join(known)}")
    for clip in CLIPS:
        if not args.clips or clip.name in args.clips:
            record(clip, args.out, args.keep)
    if not args.clips or "wind_tunnel_wake" in args.clips:
        wind_tunnel_wake(args.out, args.keep)
    if not args.clips or "builder" in args.clips:
        builder_clip(args.out, args.keep)
    return 0


if __name__ == "__main__":
    sys.exit(main())

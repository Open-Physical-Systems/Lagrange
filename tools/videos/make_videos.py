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
LEFT = (0, 0, 760, 720)
RIGHT = (760, 0, 520, 720)
HALF_L = (0, 0, 640, 720)
HALF_R = (640, 0, 640, 720)


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


@dataclass
class Clip:
    name: str
    title: str
    seconds: float
    runs: list[list[str]]  # server arguments of each run; ["@cli", ...]: the lagrange command
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


AMR = asset("robots/amr_diffdrive/amr_diffdrive.urdf")
WAREHOUSE = asset("worlds/warehouse_small/warehouse_small.usda")
QUAD = asset("robots/quad_x500/quad_x500.usda")
GEO = ["--air-density", "1.225"]
TURN = joints(velocity={"turn": 6.28})
ARM = asset("robots/arm6/arm6.urdf")

CLIPS: list[Clip] = [
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
                viewer=["--workspace", "Scene", "--follow", "9,60"],
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
                viewer=["--workspace", "Scene", "--follow", "8,45"],
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
    if clip.write:
        clip.write(work)
    envs: list[dict[str, str]] = []
    servers: list[subprocess.Popen[bytes]] = []
    viewers: list[subprocess.Popen[bytes]] = []
    sent: list[tuple[float, str]] = []
    count = int((clip.seconds + 6) * FPS)
    try:
        for r, args in enumerate(clip.runs):
            env = dict(os.environ, ROS_DOMAIN_ID=str(80 + r))
            envs.append(env)
            socket = work / f"run{r}.sock"
            if args[0] == "@cli":
                command = ["lagrange", *args[1:], "--server-only", "--listen", f"unix:{socket}"]
            else:
                command = ["ros2", "run", "lagrange_server", "lagrange_server",
                           "--listen", f"unix:{socket}", *args]  # fmt: skip
            log = work / f"run{r}.log"
            servers.append(subprocess.Popen(command, env=env, cwd=work, stdout=log.open("wb"),
                                            stderr=subprocess.STDOUT))  # fmt: skip
            wait_for(socket, servers[-1], log)
        for p, pane in enumerate(clip.panes):
            if pane.text is not None:
                continue
            viewers.append(subprocess.Popen(
                ["ros2", "run", "lagrange_viewer", "lagrange_viewer",
                 "--connect", f"unix:{work}/run{pane.run}.sock", "--headless",
                 "--size", f"{pane.view[0]}x{pane.view[1]}", "--wait-ms", "60000",
                 "--record-frames", str(work / f"pane{p}"), "--record-fps", str(FPS),
                 "--frames", str(count), *pane.viewer],
                env=envs[pane.run], cwd=work, stdout=subprocess.DEVNULL,
            ))  # fmt: skip
        live = [p for p, pane in enumerate(clip.panes) if pane.text is None]
        for p, viewer in zip(live, viewers, strict=True):
            wait_for(work / f"pane{p}" / "frame_00000.png", viewer)
        start = time.time()
        for step in sorted(clip.steps, key=lambda s: s.at):
            time.sleep(max(step.at - (time.time() - start), 0.0))
            if step.command:
                done = subprocess.run(
                    step.command, env=envs[step.run], cwd=work, timeout=30,
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                )  # fmt: skip
                if done.returncode != 0:
                    raise SystemExit(f"{step.command}: {done.stderr.decode()[-500:]}")
            sent.append((time.time() - start, step.caption))
        for viewer in viewers:
            if viewer.wait(clip.seconds * 4 + 120) != 0:
                raise SystemExit("a viewer failed")
    finally:
        for process in [*viewers, *servers]:
            stop(process)
    # By the clock: each pane's latest frame at each moment, so the panes stay together.
    stamped = {p: sorted(((f.stat().st_mtime, f) for f in (work / f"pane{p}").glob("frame_*.png")),
                         key=lambda item: item[0]) for p in live}  # fmt: skip
    finished = work / "finished"
    finished.mkdir()
    next_frame = dict.fromkeys(live, 0)
    for index in range(int(clip.seconds * FPS)):
        now = start + index / FPS
        images: list[Image.Image | None] = []
        for p in range(len(clip.panes)):
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clips", nargs="*", help="names; all by default")
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "videos")
    parser.add_argument("--keep", action="store_true", help="keep the frames")
    args = parser.parse_args()
    known = {clip.name: clip for clip in CLIPS}
    for name in args.clips:
        if name not in known:
            raise SystemExit(f"no clip {name}; one of {', '.join(known)}")
    for clip in CLIPS:
        if not args.clips or clip.name in args.clips:
            record(clip, args.out, args.keep)
    return 0


if __name__ == "__main__":
    sys.exit(main())

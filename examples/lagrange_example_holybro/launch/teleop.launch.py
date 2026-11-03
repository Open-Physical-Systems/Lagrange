# Your own drone, the Holybro X500 V2 (PX4's model), over the open field. It takes off by itself
# and hovers 2 m up; teleop_twist_keyboard flies it over /cmd_vel, and the viewer follows it.
#
#   ros2 launch lagrange_example_holybro teleop.launch.py
#   ros2 launch lagrange_example_holybro teleop.launch.py teleop:=false   # teleop in your terminal
#   ros2 launch lagrange_example_holybro teleop.launch.py viewer:=false once:=true
#
# The meshes come from PX4's repository, fetched once: ros2 run lagrange_assets fetch_assets
# holybro_x500_v2. The rotors run only when the site's air density is given, GNSS only with its
# geo origin (here PX4's default home in Zurich).
import os
import shlex
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The viewer finds the server on this socket, one per example so that they can run side by side.
ENDPOINT = "unix:/tmp/lagrange_example_holybro.sock"
ROBOT = "holybro_x500_v2"


def asset_dir() -> Path:
    """Where fetch_assets puts the meshes too large for the repository."""
    if "LAGRANGE_ASSET_DIR" in os.environ:
        return Path(os.environ["LAGRANGE_ASSET_DIR"])
    data = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
    return data / "lagrange/assets"


def terminal(command: str) -> list[str] | None:
    """A new terminal window running `command`, so teleop has a keyboard to read."""
    if shutil.which("ptyxis"):
        return ["ptyxis", "--standalone", "--new-window", "-x", command]
    if shutil.which("gnome-terminal"):
        return ["gnome-terminal", "--wait", "--", "bash", "-c", command]
    if shutil.which("konsole"):
        return ["konsole", "-e", "bash", "-c", command]
    if shutil.which("xterm"):
        return ["xterm", "-e", "bash", "-c", command]
    return None


def teleop(context) -> list:
    """teleop_twist_keyboard in its own window, publishing TwistStamped as the server takes it."""
    ros = get_package_prefix("teleop_twist_keyboard")
    # A terminal server may not pass this process's environment on, so say what matters again.
    keep = {k: v for k, v in os.environ.items() if k.startswith(("ROS_", "RMW_"))}
    exports = "".join(f"export {k}={shlex.quote(v)}; " for k, v in keep.items())
    command = (
        f"source {shlex.quote(ros)}/setup.bash; {exports}"
        "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true"
    )
    window = terminal(f"bash -c {shlex.quote(command)}")
    if window is None:
        print(
            "[holybro] no terminal program found (ptyxis, gnome-terminal, konsole, xterm); run "
            "teleop yourself: ros2 run teleop_twist_keyboard teleop_twist_keyboard "
            "--ros-args -p stamped:=true"
        )
        return []
    return [ExecuteProcess(cmd=window, output="screen")]


def setup(context):
    if not (asset_dir() / ROBOT).is_dir():
        raise RuntimeError(
            f"the {ROBOT} meshes are not in {asset_dir()}: "
            f"run `ros2 run lagrange_assets fetch_assets {ROBOT}` once"
        )
    assets = get_package_share_directory("lagrange_assets")
    flag = {k: LaunchConfiguration(k).perform(context) == "true" for k in ("viewer", "once")}
    actions = [
        # Lagrange itself: a plain ROS 2 executable with command-line options.
        Node(
            package="lagrange_server",
            executable="lagrange_server",
            arguments=[
                "--robot",
                f"{assets}/robots/{ROBOT}/{ROBOT}.urdf",
                # Motors, the 4S battery, rotors with APC 10x4.7 coefficients, and the sensors.
                "--overlay",
                f"{assets}/robots/{ROBOT}/{ROBOT}.overlay.yaml",
                "--world",
                f"{assets}/worlds/open_field/open_field.usda",
                "--air-density",
                "1.225",
                "--geo-origin",
                "47.397742,8.545594,488",
                "--listen",
                ENDPOINT,
            ],
            output="screen",
        ),
        # Your code: an ordinary ROS 2 node, here one that reports the flight. When it finishes,
        # so does the launch.
        Node(
            package="lagrange_example_holybro",
            executable="watch",
            parameters=[{"once": flag["once"]}],
            output="screen",
            on_exit=Shutdown(),
        ),
    ]
    if flag["viewer"]:
        # Not a ROS node: a plain process, so no --ros-args are appended. The follow camera stays
        # 4 m behind the drone, 20 degrees above it. (--scope SIGNAL,... would open the
        # Signals workspace with those plotted instead of the 3D view.)
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "lagrange_viewer",
                    "lagrange_viewer",
                    "--connect",
                    ENDPOINT,
                    "--follow",
                    "4,20",
                ],
                output="screen",
            )
        )
    if LaunchConfiguration("teleop").perform(context) == "true" and not flag["once"]:
        actions += teleop(context)
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="true", description="open the viewer"),
            DeclareLaunchArgument(
                "teleop",
                default_value="true",
                description="open teleop_twist_keyboard in a new terminal window",
            ),
            DeclareLaunchArgument(
                "once", default_value="false", description="stop when the node has its result"
            ),
            OpaqueFunction(function=setup),
        ]
    )

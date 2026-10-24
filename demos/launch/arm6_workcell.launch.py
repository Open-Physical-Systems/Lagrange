"""The sample arm in its workcell (docs/design/robot-classes.md §4): the simulator with arm6, its
power and thermal overlay and the controller manager in lockstep, and the controllers spawned.
Then run `ros2 run lagrange_demos pick_place`.

Arguments: viewer:=true|false, electrical:=on|off (off: the ideal torque source),
ambient:=K (the site's air temperature; thermal runs only with it), rtf:=X,
record:=FILE.mcap, motors:=sized|undersized (arm6_undersized.overlay.yaml: shoulder and elbow
motors one size too small, for the derating scenario).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_socket() -> str:
    # Same rule as lagrange::proto::defaultUnixSocketPath().
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime:
        return f"unix:{runtime}/lagrange/default.sock"
    return f"unix:/tmp/lagrange-{os.getuid()}/default.sock"


def setup(context):
    assets = get_package_share_directory("lagrange_assets")
    demos = get_package_share_directory("lagrange_demos")
    controllers = f"{demos}/config/arm6_controllers.yaml"
    motors = LaunchConfiguration("motors").perform(context)
    overlay = "_undersized" if motors == "undersized" else ""
    endpoint = default_socket()
    server = [
        "--robot",
        f"{assets}/robots/arm6/arm6.urdf",
        "--world",
        f"{assets}/worlds/workcell_arm6/workcell_arm6.urdf",
        "--overlay",
        f"{assets}/robots/arm6/arm6{overlay}.overlay.yaml",
        "--ros2-control",
        controllers,
        "--rtf",
        LaunchConfiguration("rtf").perform(context),
        "--listen",
        endpoint,
    ]
    ambient = LaunchConfiguration("ambient").perform(context)
    if ambient:
        server += ["--ambient-temperature", ambient]
    record = LaunchConfiguration("record").perform(context)
    if record:
        server += ["--record", record]
    if LaunchConfiguration("electrical").perform(context) == "off":
        server += ["--domain", "electrical=off"]
    actions = [
        Node(
            package="lagrange_server",
            executable="lagrange_server",
            arguments=server,
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "arm_controller",
                "gripper_controller",
                "--param-file",
                controllers,
                "--controller-manager-timeout",
                "60",
            ],
            output="screen",
        ),
    ]
    if LaunchConfiguration("viewer").perform(context) == "true":
        # Not a ROS node: a plain process, so no --ros-args are appended.
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "run", "lagrange_viewer", "lagrange_viewer", "--connect", endpoint],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="false"),
            DeclareLaunchArgument("electrical", default_value="on"),
            DeclareLaunchArgument("ambient", default_value="298.15"),
            DeclareLaunchArgument("rtf", default_value="1.0"),
            DeclareLaunchArgument("record", default_value=""),
            DeclareLaunchArgument("motors", default_value="sized"),
            OpaqueFunction(function=setup),
        ]
    )

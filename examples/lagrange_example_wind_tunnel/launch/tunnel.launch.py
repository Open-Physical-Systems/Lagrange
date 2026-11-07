# A wind tunnel: the solver runs first, then the flow opens in the viewer, with streamlines, a slice
# of speed and the pressure on the body's skin, with their colour scales. flow.vti opens in
# ParaView.
#
#   ros2 launch lagrange_example_wind_tunnel tunnel.launch.py speed:=15
#   ros2 launch lagrange_example_wind_tunnel tunnel.launch.py viewer:=false once:=true
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ENDPOINT = "unix:/tmp/lagrange_example_wind_tunnel.sock"


def setup(context):
    robot = get_package_share_directory("lagrange_example_wind_tunnel") + "/robot/pod.urdf"
    tool = get_package_prefix("lagrange_server") + "/lib/lagrange_server/lagrange_tunnel"
    out = tempfile.mkdtemp(prefix="lagrange_tunnel_")
    once = LaunchConfiguration("once").perform(context) == "true"
    tunnel = ExecuteProcess(
        cmd=[
            tool,
            robot,
            "--speed",
            LaunchConfiguration("speed").perform(context),
            "--cells",
            LaunchConfiguration("cells").perform(context),
            "--yaw",
            LaunchConfiguration("yaw").perform(context),
            "--out",
            out,
        ],
        output="screen",
    )
    after = [
        Node(
            package="lagrange_example_wind_tunnel",
            executable="read_report",
            arguments=[f"{out}/report.json"],
            output="screen",
            on_exit=Shutdown() if once else None,
        )
    ]
    if not once:
        # The flow as a world: nothing moves in it, so it needs no robot.
        after.append(
            Node(
                package="lagrange_server",
                executable="lagrange_server",
                arguments=[
                    "--robot",
                    "none",
                    "--world",
                    f"{out}/flow_view.usda",
                    "--listen",
                    ENDPOINT,
                ],
                output="screen",
            )
        )
        if LaunchConfiguration("viewer").perform(context) == "true":
            after.append(
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "lagrange_viewer",
                        "lagrange_viewer",
                        "--connect",
                        ENDPOINT,
                    ],
                    output="screen",
                )
            )
    return [tunnel, RegisterEventHandler(OnProcessExit(target_action=tunnel, on_exit=after))]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="true", description="open the viewer"),
            DeclareLaunchArgument("once", default_value="false", description="stop at the result"),
            DeclareLaunchArgument("speed", default_value="15", description="wind, m/s"),
            DeclareLaunchArgument(
                "yaw", default_value="0", description="the robot turned, degrees"
            ),
            DeclareLaunchArgument("cells", default_value="150000", description="grid size"),
            OpaqueFunction(function=setup),
        ]
    )

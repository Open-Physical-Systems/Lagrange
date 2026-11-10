# A quadcopter's battery leads disturb its compass (RFC 0013): a wire from the battery, under the
# frame and back from its middle, to each motor, carrying its bus current, its return 5 mm above.
# The quadcopter takes off and hovers; a node reads the compass's heading error and the
# disturbance's share of the Earth's field, ArduPilot's "compassmot" percentage.
#
#   ros2 launch lagrange_example_compass compass.launch.py
#   ros2 launch lagrange_example_compass compass.launch.py viewer:=false once:=true
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ENDPOINT = "unix:/tmp/lagrange_example_compass.sock"


def setup(context):
    assets = get_package_share_directory("lagrange_assets")
    share = get_package_share_directory("lagrange_example_compass")
    actions = [
        # The leads are laid by --emi-motor-leads SPACING,X,Y,Z (the battery's place on the
        # frame); the compass needs the site (--geo-origin) for the Earth's field.
        Node(
            package="lagrange_server",
            executable="lagrange_server",
            arguments=[
                "--robot",
                f"{assets}/robots/quad_x500/quad_x500.usda",
                "--world",
                "none",
                "--air-density",
                "1.225",
                "--geo-origin",
                "47.397742,8.545594,488",
                "--emi-motor-leads",
                "0.005,-0.06,0,-0.06",
                "--ros-topics",
                f"{share}/config/topics.yaml",
                "--listen",
                ENDPOINT,
            ],
            output="screen",
        ),
        Node(
            package="lagrange_example_compass",
            executable="read_compass",
            parameters=[{"once": LaunchConfiguration("once").perform(context) == "true"}],
            output="screen",
            on_exit=Shutdown(),
        ),
    ]
    if LaunchConfiguration("viewer").perform(context) == "true":
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "lagrange_viewer",
                    "lagrange_viewer",
                    "--connect",
                    ENDPOINT,
                    "--workspace",
                    "environment",
                    "--slice",
                    "emi/flux_density:xz",
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="true", description="open the viewer"),
            DeclareLaunchArgument(
                "once", default_value="false", description="stop after the first result"
            ),
            OpaqueFunction(function=setup),
        ]
    )

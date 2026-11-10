# Heat inside the parts (RFC 0012): two LEDs and their driver on a circuit board, on a finned
# aluminium heat sink, and a node that reads the LED's, the board's hottest and the sink's
# temperatures as they rise. The viewer's Thermal shading shows where the heat goes.
#
#   ros2 launch lagrange_example_heat heat.launch.py
#   ros2 launch lagrange_example_heat heat.launch.py viewer:=false once:=true
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ENDPOINT = "unix:/tmp/lagrange_example_heat.sock"


def setup(context):
    assets = get_package_share_directory("lagrange_assets")
    share = get_package_share_directory("lagrange_example_heat")
    actions = [
        # The materials and heat sources are in the URDF (<thermal_material>, <heat_source>); the
        # thermal domain needs the site's air temperature, and runs 20 times faster than real time
        # so that the parts warm up in a minute.
        Node(
            package="lagrange_server",
            executable="lagrange_server",
            arguments=[
                "--robot",
                f"{assets}/robots/led_heatsink/led_heatsink.urdf",
                "--world",
                "none",
                "--ambient-temperature",
                "298.15",
                "--ros-topics",
                f"{share}/config/topics.yaml",
                "--rtf",
                "20",
                "--listen",
                ENDPOINT,
            ],
            output="screen",
        ),
        Node(
            package="lagrange_example_heat",
            executable="read_heat",
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
                    "--shading",
                    "thermal",
                    "--camera",
                    "0,0,0.02,0.15,30,40",
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

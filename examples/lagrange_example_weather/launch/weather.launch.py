# The same warehouse in fog: visibility is a launch argument, and the lidar's returns follow it.
#
#   ros2 launch lagrange_example_weather weather.launch.py visibility:=8
#   ros2 launch lagrange_example_weather weather.launch.py visibility:=1000 viewer:=false once:=true
#
# Weather runs only when the site is given: temperature (K), pressure (Pa), relative humidity.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The viewer finds the server on this socket, one per example so that they can run side by side.
ENDPOINT = "unix:/tmp/lagrange_example_weather.sock"


def setup(context):
    assets = get_package_share_directory("lagrange_assets")
    visibility = LaunchConfiguration("visibility").perform(context)
    actions = [
        # Lagrange itself: a plain ROS 2 executable with command-line options.
        Node(
            package="lagrange_server",
            executable="lagrange_server",
            arguments=[
                "--robot",
                f"{assets}/robots/amr_diffdrive/amr_diffdrive.urdf",
                "--world",
                f"{assets}/worlds/warehouse_small/warehouse_small.usda",
                "--no-demo",
                "--weather",
                f"temperature=288,pressure=101325,relative_humidity=0.9,visibility={visibility}",
                "--listen",
                ENDPOINT,
            ],
            output="screen",
        ),
        # Your code: an ordinary ROS 2 node. When it finishes, so does the launch.
        Node(
            package="lagrange_example_weather",
            executable="read_range",
            parameters=[{"once": LaunchConfiguration("once").perform(context) == "true"}],
            output="screen",
            on_exit=Shutdown(),
        ),
    ]
    if LaunchConfiguration("viewer").perform(context) == "true":
        # Not a ROS node: a plain process, so no --ros-args are appended.
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "run", "lagrange_viewer", "lagrange_viewer", "--connect", ENDPOINT],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="true", description="open the viewer"),
            DeclareLaunchArgument(
                "once", default_value="false", description="stop when the node has its result"
            ),
            DeclareLaunchArgument(
                "visibility", default_value="8", description="metres you can see in the fog"
            ),
            OpaqueFunction(function=setup),
        ]
    )

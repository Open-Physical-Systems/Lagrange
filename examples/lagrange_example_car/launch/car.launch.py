# The sample sedan in the town: traffic, people, working lights. It waits for /cmd_vel.
#
#   ros2 launch lagrange_example_car car.launch.py
#   ros2 launch lagrange_example_car car.launch.py viewer:=false once:=true
#
# --spawn X,Y,Z,YAW is where it starts; --traffic how many others there are.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The viewer finds the server on this socket, one per example so that they can run side by side.
ENDPOINT = "unix:/tmp/lagrange_example_car.sock"


def setup(context):
    assets = get_package_share_directory("lagrange_assets")
    actions = [
        # Lagrange itself: a plain ROS 2 executable with command-line options.
        Node(
            package="lagrange_server",
            executable="lagrange_server",
            arguments=[
                "--robot",
                f"{assets}/robots/sedan/sedan.usda",
                "--world",
                f"{assets}/worlds/town/town.usda",
                "--spawn",
                "30,-1.75,0.6,0",
                "--no-demo",
                "--traffic",
                "vehicles=8,pedestrians=8,seed=1",
                "--listen",
                ENDPOINT,
            ],
            output="screen",
        ),
        # Your code: an ordinary ROS 2 node. When it finishes, so does the launch.
        Node(
            package="lagrange_example_car",
            executable="drive_car",
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
            OpaqueFunction(function=setup),
        ]
    )

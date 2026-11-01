# Training from a launch file. Here Lagrange runs inside the program, one simulation per process
# (lagrange.envs), stepped as fast as it can go, with no ROS and no viewer.
#
#   ros2 launch lagrange_example_rl train.launch.py workers:=4
#
# viewer and once are taken as in the other examples; training always ends by itself and has no
# viewer to open.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("workers", default_value="4", description="one simulation each"),
            DeclareLaunchArgument("viewer", default_value="false", description="not used"),
            DeclareLaunchArgument("once", default_value="true", description="not used"),
            ExecuteProcess(
                cmd=["ros2", "run", "lagrange_example_rl", "train", LaunchConfiguration("workers")],
                output="screen",
                on_exit=Shutdown(),
            ),
        ]
    )

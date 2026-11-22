# Training from a launch file. Here Lagrange runs inside the program, one simulation per process
# (lagrange.envs), stepped as fast as it can go, with no ROS and no viewer.
#
#   ros2 launch lagrange_example_rl train.launch.py workers:=4
#   ros2 launch lagrange_example_rl train.launch.py task:=quad_hover steps:=1000000
#   ros2 launch lagrange_example_rl train.launch.py task:=amr_goal steps:=300000
#
# task arm (the default) learns an arm's gains by the cross-entropy method, numpy only. The other
# tasks (lagrange.tasks: quad_hover, quad_waypoint, amr_goal) train by PPO with Stable-Baselines3
# and torch, run from the Python they are installed in (a venv); without them it says so in its
# RESULT line and ends. viewer and once are taken as in the other examples; training always ends
# by itself and has no viewer to open.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration


def training(context):
    task = LaunchConfiguration("task").perform(context)
    workers = LaunchConfiguration("workers").perform(context)
    if task == "arm":
        command = ["ros2", "run", "lagrange_example_rl", "train", workers]
    else:
        steps = LaunchConfiguration("steps").perform(context)
        command = ["ros2", "run", "lagrange_example_rl", "ppo", task, "--workers", workers]
        command += ["--steps", steps]
    return [ExecuteProcess(cmd=command, output="screen", on_exit=Shutdown())]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task",
                default_value="arm",
                description="arm (cross-entropy, numpy), or PPO on quad_hover, quad_waypoint, "
                "amr_goal",
            ),
            DeclareLaunchArgument("workers", default_value="4", description="one simulation each"),
            DeclareLaunchArgument(
                "steps", default_value="1000000", description="PPO: environment steps"
            ),
            DeclareLaunchArgument("viewer", default_value="false", description="not used"),
            DeclareLaunchArgument("once", default_value="true", description="not used"),
            OpaqueFunction(function=training),
        ]
    )

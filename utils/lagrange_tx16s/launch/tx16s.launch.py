# The TX16S-style radio: a window that flies a robot over /cmd_vel from a real RadioMaster TX16S
# (USB, EdgeTX's USB Joystick mode) or the keyboard.
#
#   ros2 launch lagrange_tx16s tx16s.launch.py [mode:=2] [topic:=/cmd_vel]
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="2", description="stick mode: 1 or 2"),
            DeclareLaunchArgument("topic", default_value="/cmd_vel", description="where to send"),
            Node(
                package="lagrange_tx16s",
                executable="lagrange_tx16s",
                parameters=[
                    {
                        "mode": LaunchConfiguration("mode"),
                        "topic": LaunchConfiguration("topic"),
                    }
                ],
                output="screen",
            ),
        ]
    )

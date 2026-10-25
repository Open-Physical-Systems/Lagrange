# The sample sedan in the city block for Autoware (docs/guide/autoware.md):
#
#   ros2 launch lagrange_autoware sedan_autoware.launch.py [world:=FILE] [viewer:=false]
#
# lagrange_server runs the car with ROS time (/clock) and its sensors renamed into Autoware's
# /sensing namespace; lagrange_autoware_interface turns Autoware's /control/command/* into the
# server's /ackermann_cmd and publishes /vehicle/status/* and (ground truth standing in for
# localization) /localization/kinematic_state. Autoware itself is started separately, with its
# own vehicle and sensor kit descriptions and use_sim_time.
import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Lagrange topic → Autoware topic. The /sensing names follow Autoware's sample sensor kit;
# TODO(verify) them against the sensor kit you launch Autoware with.
SENSOR_REMAPS = [
    ("/points", "/sensing/lidar/top/pointcloud_raw"),
    ("/imu", "/sensing/imu/imu_data"),
    ("/front_camera/image_raw", "/sensing/camera/front/image_raw"),
    ("/front_camera/camera_info", "/sensing/camera/front/camera_info"),
]


def default_socket() -> str:
    # Same rule as lagrange::proto::defaultUnixSocketPath(), where the server listens by default.
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime:
        return f"unix:{runtime}/lagrange/default.sock"
    return f"unix:/tmp/lagrange-{os.getuid()}/default.sock"


def generate_launch_description() -> LaunchDescription:
    assets = get_package_share_path("lagrange_assets")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot", default_value=str(assets / "robots" / "sedan" / "sedan.usda")
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(assets / "worlds" / "city_block" / "city_block.usda"),
            ),
            DeclareLaunchArgument("viewer", default_value="false"),
            DeclareLaunchArgument(
                "kinematic_state",
                default_value="true",
                description="publish ground truth as /localization/kinematic_state",
            ),
            Node(
                package="lagrange_server",
                executable="lagrange_server",
                arguments=[
                    "--robot",
                    LaunchConfiguration("robot"),
                    "--world",
                    LaunchConfiguration("world"),
                    "--no-demo",
                ],
                remappings=SENSOR_REMAPS,
                output="screen",
            ),
            Node(
                package="lagrange_autoware",
                executable="lagrange_autoware_interface",
                parameters=[
                    {
                        "use_sim_time": True,
                        "steer_joints": ["front_left_steer", "front_right_steer"],
                        "publish_kinematic_state": LaunchConfiguration("kinematic_state"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="lagrange_viewer",
                executable="lagrange_viewer",
                arguments=["--connect", default_socket()],
                condition=IfCondition(LaunchConfiguration("viewer")),
                output="screen",
            ),
        ]
    )

# The sample AMR in the small warehouse, driven by Nav2 (roadmap 0.2-F6).
#
#   ros2 launch lagrange_demos amr_warehouse.launch.py            # server + viewer + Nav2
#   ros2 launch lagrange_demos amr_warehouse.launch.py viewer:=false
#   ros2 run lagrange_demos patrol                                # then send it on a patrol
#
# The server publishes /odom and odom -> base_link from the simulated ground truth, so there is
# no localization: map -> odom is a fixed identity (the map is exported from the same world with
# `lagrange_server --export-map`).
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_NODES = [
    "map_server",
    "planner_server",
    "controller_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def default_socket() -> str:
    # Same rule as lagrange::proto::defaultUnixSocketPath().
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime:
        return f"unix:{runtime}/lagrange/default.sock"
    return f"unix:/tmp/lagrange-{os.getuid()}/default.sock"


def generate_launch_description() -> LaunchDescription:
    demos = get_package_share_path("lagrange_demos")
    assets = get_package_share_path("lagrange_assets")
    robot = assets / "robots" / "amr_diffdrive" / "amr_diffdrive.urdf"
    world = assets / "worlds" / "warehouse_small" / "warehouse_small.urdf"
    params = str(demos / "config" / "nav2_params.yaml")
    sim_time = {"use_sim_time": True}
    endpoint = LaunchConfiguration("endpoint")

    nav2 = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                params,
                sim_time,
                {"yaml_filename": str(demos / "maps" / "warehouse_small.yaml")},
            ],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params, sim_time],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params, sim_time],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params, sim_time],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params, sim_time],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params, sim_time],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[sim_time, {"autostart": True, "node_names": NAV2_NODES}],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("viewer", default_value="true", description="Open the viewer"),
            DeclareLaunchArgument(
                "endpoint",
                default_value=default_socket(),
                description="Where the server listens for viewers",
            ),
            DeclareLaunchArgument(
                "rtf", default_value="1.0", description="Real-time factor (0 = as fast as possible)"
            ),
            Node(
                package="lagrange_server",
                executable="lagrange_server",
                output="screen",
                arguments=[
                    "--robot",
                    str(robot),
                    "--world",
                    str(world),
                    "--listen",
                    endpoint,
                    "--rtf",
                    LaunchConfiguration("rtf"),
                ],
            ),
            Node(
                package="lagrange_viewer",
                executable="lagrange_viewer",
                name="lagrange_viewer",
                arguments=["--connect", endpoint],
                condition=IfCondition(LaunchConfiguration("viewer")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[sim_time, {"robot_description": Path(robot).read_text()}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                parameters=[sim_time],
            ),
            *nav2,
        ]
    )

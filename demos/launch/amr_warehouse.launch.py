# The sample AMR in the small warehouse, driven by Nav2 (roadmap 0.2-F6).
#
#   ros2 launch lagrange_demos amr_warehouse.launch.py            # server + viewer + Nav2
#   ros2 launch lagrange_demos amr_warehouse.launch.py viewer:=false
#   ros2 launch lagrange_demos amr_warehouse.launch.py control:=builtin
#   ros2 run lagrange_demos patrol                                # then send it on a patrol
#
# control:=ros2_control (default): the server runs a ros2_control controller manager in
# lockstep; joint_state_broadcaster and diff_drive_controller are spawned as on a real robot,
# Nav2's /cmd_vel goes to the diff drive, which publishes wheel odometry, and AMCL localizes
# against the map (exported from the same world with `lagrange_server --export-map`).
#
# control:=builtin: the server's own drive controller takes /cmd_vel and publishes ground-truth
# /odom, so map -> odom is a fixed identity and no localization runs.
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessIO
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


# Seconds a lifecycle manager waits for each node's bond. Nav2's default of 4 s is too short
# while a dozen nodes start at once on a busy machine: AMCL then never activates.
BOND_TIMEOUT_S = 30.0


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

    controllers = str(demos / "config" / "ros2_controllers.yaml")
    server_args = [
        "--robot",
        str(robot),
        "--world",
        str(world),
        "--listen",
        endpoint,
        "--rtf",
        LaunchConfiguration("rtf"),
        "--lidar",
        LaunchConfiguration("lidar"),
        # Nav2 takes a while to start; the robot must still be at the start pose (the map's
        # origin) when it does, not out on the demo figure-eight.
        "--no-demo",
    ]
    with_controllers = LaunchConfigurationEquals("control", "ros2_control")

    # The standard spawners load the controllers into the server's /controller_manager, as on a
    # real robot. Nav2 uses /cmd_vel and /odom; the diff drive is remapped onto them.
    spawners = [
        # The standard spawner, retried in a fresh process if it cannot reach the controller
        # manager (spawn_controllers.py).
        Node(
            package="lagrange_demos",
            executable="spawn_controllers",
            arguments=[
                "joint_state_broadcaster",
                "diff_drive_controller",
                "--",
                "--param-file",
                controllers,
                "--controller-ros-args",
                "-r /diff_drive_controller/cmd_vel:=/cmd_vel -r /diff_drive_controller/odom:=/odom",
            ],
            output="screen",
            condition=with_controllers,
        ),
    ]
    # Wheel odometry drifts, so AMCL keeps map -> odom right from the lidar and the map.
    localization = [
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params, sim_time],
            condition=with_controllers,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                sim_time,
                {"autostart": True, "node_names": ["amcl"], "bond_timeout": BOND_TIMEOUT_S},
            ],
            condition=with_controllers,
        ),
    ]

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
            parameters=[
                sim_time,
                {"autostart": True, "node_names": NAV2_NODES, "bond_timeout": BOND_TIMEOUT_S},
            ],
        ),
    ]

    server_with_controllers = Node(
        package="lagrange_server",
        executable="lagrange_server",
        output="screen",
        arguments=[*server_args, "--ros2-control", controllers],
        condition=with_controllers,
    )

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
            DeclareLaunchArgument(
                "lidar",
                default_value="auto",
                description="Where lidars cast: auto, cpu or gpu (lagrange_server --lidar)",
            ),
            DeclareLaunchArgument(
                "control",
                default_value="ros2_control",
                choices=["ros2_control", "builtin"],
                description="Who drives the wheels: ros2_control's diff_drive_controller, or the "
                "server's built-in controller",
            ),
            # The simulator: with ros2_control, or with its built-in drive controller.
            server_with_controllers,
            # The spawners start once that server is listening: a service client made while the
            # controller manager is still starting can lose every reply (Fast DDS discovery),
            # and the spawner then retries for ever.
            RegisterEventHandler(
                OnProcessIO(
                    target_action=server_with_controllers,
                    on_stderr=lambda event: spawners if b"listening on" in event.text else None,
                )
            ),
            Node(
                package="lagrange_server",
                executable="lagrange_server",
                output="screen",
                arguments=server_args,
                condition=LaunchConfigurationEquals("control", "builtin"),
            ),
            *localization,
            # Not a ROS node: started as a plain process, so no --ros-args are appended.
            ExecuteProcess(
                cmd=["ros2", "run", "lagrange_viewer", "lagrange_viewer", "--connect", endpoint],
                name="lagrange_viewer",
                output="screen",
                condition=IfCondition(LaunchConfiguration("viewer")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[sim_time, {"robot_description": Path(robot).read_text()}],
            ),
            # Ground-truth odometry needs no localization: the map frame is the odom frame.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                parameters=[sim_time],
                condition=LaunchConfigurationEquals("control", "builtin"),
            ),
            *nav2,
        ]
    )

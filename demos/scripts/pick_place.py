#!/usr/bin/env python3
"""Pick-and-place cycles of the sample arm with MoveIt 2 (docs/design/robot-classes.md §4).

Plans each move with MoveItCpp (moveit_py, no move_group needed) to the named poses of
config/arm6_moveit/arm6.srdf and executes it through the simulator's joint_trajectory_controller:
home, open, pick approach, pick, close, pick approach, place approach, place, open, place
approach. At full speed (the default) a repeated cycle loads the motors; with thermal on the
windings heat until derating limits torque and the trajectory controller aborts a move on its
path tolerance. Exit status: 0 when every cycle completed, 3 when a move failed (and which).

Run after `ros2 launch lagrange_demos arm6_workcell.launch.py`:
    ros2 run lagrange_demos pick_place [--cycles N] [--speed 0..1]
"""

import argparse
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit.core.robot_state import RobotState  # noqa: F401  (loads moveit.core for moveit_py)
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder

CYCLE = [
    ("arm", "pick_approach"),
    ("gripper", "open"),
    ("arm", "pick"),
    ("gripper", "closed"),
    ("arm", "pick_approach"),
    ("arm", "place_approach"),
    ("arm", "place"),
    ("gripper", "open"),
    ("arm", "place_approach"),
]


def moveit_config() -> dict:
    """The MoveIt configuration of the sample arm, from lagrange_demos and lagrange_assets."""
    assets = get_package_share_directory("lagrange_assets")
    config = (
        MoveItConfigsBuilder("arm6", package_name="lagrange_demos")
        .robot_description(file_path=f"{assets}/robots/arm6/arm6.urdf")
        .robot_description_semantic(file_path="config/arm6_moveit/arm6.srdf")
        .robot_description_kinematics(file_path="config/arm6_moveit/kinematics.yaml")
        .joint_limits(file_path="config/arm6_moveit/joint_limits.yaml")
        .trajectory_execution(file_path="config/arm6_moveit/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path="config/arm6_moveit/moveit_cpp.yaml")
        .to_moveit_configs()
    )
    params = config.to_dict()
    params["use_sim_time"] = True
    return params


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cycles", type=int, default=3, help="pick-and-place cycles (default 3)")
    parser.add_argument(
        "--speed", type=float, default=1.0, help="velocity and acceleration scaling, 0..1"
    )
    args = parser.parse_args()
    rclpy.init()
    moveit = MoveItPy(node_name="pick_place", config_dict=moveit_config())
    components = {name: moveit.get_planning_component(name) for name in ("arm", "gripper")}
    params = PlanRequestParameters(moveit, "pick_place")
    params.max_velocity_scaling_factor = args.speed
    params.max_acceleration_scaling_factor = args.speed
    # MoveIt's action clients find the trajectory controllers' servers asynchronously; the
    # first move waits for that (and a move that still finds none is tried once more).
    time.sleep(2.0)

    def move(group: str, pose: str) -> str:
        """Plans and executes one move; returns "" on success, else what went wrong."""
        component = components[group]
        component.set_start_state_to_current_state()
        component.set_goal_state(configuration_name=pose)
        plan = component.plan(single_plan_parameters=params)
        if not plan:
            return f"planning to {pose} failed"
        for attempt in range(2):
            status = moveit.execute(plan.trajectory, controllers=[])
            text = str(getattr(status, "status", status))
            if text.upper() in ("SUCCEEDED", "TRUE"):
                return ""
            if attempt == 0 and text.upper() == "ABORTED":
                time.sleep(1.0)  # perhaps not connected yet; a tolerance abort fails again
                component.set_start_state_to_current_state()
                plan = component.plan(single_plan_parameters=params)
                if not plan:
                    break
        return f"{group} to {pose}: {text}"

    result = 0
    if problem := move("arm", "home"):
        print(f"pick_place: {problem}", file=sys.stderr)
        result = 3
    for cycle in range(1, args.cycles + 1 if result == 0 else 1):
        started = time.monotonic()
        for group, pose in CYCLE:
            if problem := move(group, pose):
                print(f"pick_place: cycle {cycle}: {problem}", file=sys.stderr)
                result = 3
                break
        if result:
            break
        print(f"cycle {cycle} done in {time.monotonic() - started:.1f} s", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    # moveit_py's teardown of MoveItCpp crashes at exit in this MoveIt release; nothing is left
    # to clean up, so leave without it and keep the exit status.
    os._exit(result)


if __name__ == "__main__":
    sys.exit(main())

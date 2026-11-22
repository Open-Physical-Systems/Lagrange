#!/usr/bin/env python3
"""Flies a trained policy (ppo.py's policy.npz) through a run of targets and records it.

    fly_policy.py quad_hover POLICY.npz OUT.mcap [--hold 4]
    fly_policy.py amr_goal POLICY.npz OUT.mcap
    lagrange_server --replay OUT.mcap        # then watch it in the viewer

Numpy only: the policy runs without torch. The quadcopter holds each of five targets in turn
for --hold seconds (a square 1 m to the side, up and down, and back to where it started); the
AMR drives to four goals in the warehouse aisle in turn, each within 4 m of the last as in
training (a goal farther than it learnt can send it into a rack). It prints each target and
how close it got, and the recording carries the run for lagrange_server --replay.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from lagrange.tasks import HOVER_HEIGHT, AmrGoToGoal, NumpyPolicy, QuadHover

QUAD_TARGETS = np.array(  # m from the hover point
    [[1.0, 0.0, 0.0], [1.0, 1.0, 0.5], [-1.0, 1.0, 0.5], [-1.0, -1.0, 0.0], [0.0, 0.0, 0.0]]
)
TAKE_OFF_SECONDS = 4.0  # s: the built-in controller takes off at 1 s and settles
AMR_GOALS = np.array([[3.0, 1.5, 0.0], [0.5, -1.5, 0.0], [-2.5, 1.0, 0.0], [0.0, 0.0, 0.0]])


def fly_quad(policy: NumpyPolicy, out: Path, hold: float) -> int:
    # Recorded from the start: the built-in controller takes off and hovers 2 m up (4 s), then
    # the policy flies.
    env = QuadHover(kick_seconds=0.0, warm_up=0.0, episode_seconds=1e9)
    env.sim.record(str(out))
    stand = env._pose()[0].copy()
    env.ground = stand[2]
    env.home = stand + np.array([0.0, 0.0, HOVER_HEIGHT])
    env.sim.step_until(TAKE_OFF_SECONDS)
    env._start_time = env.sim.time
    steps = round(hold / env.step_seconds)
    worst = 0.0
    for offset in QUAD_TARGETS:
        env.target = env.home + offset
        observation = env._observe()
        errors = []
        for _ in range(steps):
            observation, _r, terminated, _t, info = env.step(policy(observation))
            errors.append(info["distance"])
            if terminated:
                print(f"crashed on the way to {np.round(offset, 2)}", flush=True)
                env.sim.stop_recording()
                return 1
        settled = float(np.mean(errors[-round(1.0 / env.step_seconds) :]))
        worst = max(worst, settled)
        print(f"target {np.round(offset, 2)} m: error over the last second {settled:.3f} m")
    env.sim.stop_recording()
    print(f"RESULT: held {len(QUAD_TARGETS)} targets, worst settled error {worst:.3f} m; {out}")
    return 0


def drive_amr(policy: NumpyPolicy, out: Path) -> int:
    env = AmrGoToGoal(lidar=True, episode_seconds=1e9)
    observation, _ = env.reset(seed=0, options={"goal": AMR_GOALS[0]})
    env.sim.record(str(out))
    reached = 0
    for goal in AMR_GOALS:
        env.goal = goal.copy()
        env._distance = float(np.linalg.norm(goal[:2] - env._pose()[0][:2]))
        observation = env._observe()
        for _ in range(300):  # 30 s
            observation, _r, terminated, _t, info = env.step(policy(observation))
            if terminated:
                break
        print(
            f"goal {goal[:2]}: {'reached' if info['success'] else 'missed'} "
            f"({info['distance']:.2f} m)",
            flush=True,
        )
        reached += bool(info["success"])
        if info.get("collision"):
            break
    env.sim.stop_recording()
    print(f"RESULT: reached {reached} of {len(AMR_GOALS)} goals; {out}")
    return 0 if reached == len(AMR_GOALS) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("task", choices=["quad_hover", "amr_goal"])
    parser.add_argument("policy", type=Path)
    parser.add_argument("out", type=Path, help="the recording, .mcap")
    parser.add_argument("--hold", type=float, default=4.0, help="s at each quadcopter target")
    args = parser.parse_args()
    policy = NumpyPolicy(args.policy)
    if args.task == "quad_hover":
        return fly_quad(policy, args.out, args.hold)
    return drive_amr(policy, args.out)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Learns the gains that bring an arm to a pose, by the cross-entropy method.

Numpy only, to show the environment rather than a learning library: LagrangeEnv follows Gymnasium's
interface, so Stable-Baselines3 and the like take it as it is. Gravity is drawn afresh each episode.
"""

import os
import sys

import numpy as np
from ament_index_python.packages import get_package_share_directory
from lagrange.envs import LagrangeEnv, ParallelEnvs

TARGET = np.array([0.5, 0.6, -0.4])  # rad
EFFORT = 60.0  # N m at an action of 1
GENERATIONS = 8
POPULATION = 24
STEPS = 60  # at 20 ms: 1.2 s
ELITE = 0.25  # the share of each generation that the next one is drawn around


def reward(observation: np.ndarray, _action: np.ndarray, _sim: object) -> float:
    return -float(np.abs(observation[0::2] - TARGET).sum())


def make_env() -> LagrangeEnv:
    robot = get_package_share_directory("lagrange_example_rl") + "/robot/three_joints.urdf"
    return LagrangeEnv(
        robot,
        mode="effort",
        action_scale=EFFORT,
        episode_seconds=STEPS * 0.02,
        reward=reward,
        randomize={"gravity": (8.8, 10.8)},
    )


def actions_of(gains: np.ndarray, observations: np.ndarray) -> np.ndarray:
    """A PD law in each environment: gains are stiffness and damping as logarithms."""
    stiffness = np.exp(gains[:, 0:1])
    damping = np.exp(gains[:, 1:2])
    return stiffness * (TARGET - observations[:, 0::2]) - damping * observations[:, 1::2]


def evaluate(envs: ParallelEnvs, gains: np.ndarray, seed: int) -> np.ndarray:
    observations = envs.reset(seed=seed)
    total = np.zeros(len(gains))
    for _ in range(STEPS):
        observations, rewards, _ended = envs.step(actions_of(gains, observations))
        total += rewards
    return total / STEPS


def main() -> int:
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else min(4, os.cpu_count() or 1)
    rng = np.random.default_rng(1)
    envs = ParallelEnvs(make_env, workers)
    mean = np.array([0.0, -2.0])
    spread = np.array([1.5, 1.5])
    first = None
    try:
        for generation in range(GENERATIONS):
            gains = rng.normal(mean, spread, size=(POPULATION, 2))
            scores = np.concatenate(
                [
                    evaluate(envs, np.resize(batch, (workers, 2)), seed=100 * generation + i)[
                        : len(batch)
                    ]
                    for i, batch in enumerate(np.array_split(gains, POPULATION / workers))
                ]
            )
            best = gains[np.argsort(scores)[-int(ELITE * POPULATION) :]]
            mean = best.mean(axis=0)
            spread = np.maximum(best.std(axis=0), 0.1)
            print(f"generation {generation}: mean error {-scores.mean():.3f} rad", flush=True)
            first = first if first is not None else -scores.mean()
        last = -evaluate(envs, np.tile(mean, (workers, 1)), seed=999).mean()
    finally:
        envs.close()
    stiffness, damping = np.exp(mean) * EFFORT
    print(
        f"RESULT: summed joint error {first:.2f} rad down to {last:.2f} rad over {GENERATIONS} "
        f"generations; learned stiffness {stiffness:.0f} N m/rad, damping {damping:.1f} N m s/rad",
        flush=True,
    )
    return 0 if last < first else 1


if __name__ == "__main__":
    sys.exit(main())

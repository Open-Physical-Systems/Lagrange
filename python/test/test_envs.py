"""The Gymnasium interface and parallel environments."""

from __future__ import annotations

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from lagrange.envs import LagrangeEnv, ParallelEnvs


def arm() -> str:
    return get_package_share_directory("lagrange_assets") + "/robots/arm6/arm6.urdf"


def make_arm() -> LagrangeEnv:
    return LagrangeEnv(arm(), mode="position", action_scale=0.5, episode_seconds=0.2)


def rollout(env: LagrangeEnv, seed: int) -> np.ndarray:
    env.reset(seed=seed)
    out = []
    for _ in range(5):
        observation, *_ = env.step(env.action_space.sample())
        out.append(observation)
    return np.stack(out)


def test_one_step_has_the_gymnasium_shape() -> None:
    env = make_arm()
    observation, _info = env.reset(seed=1)
    assert env.observation_space.shape == observation.shape
    assert observation.shape == (2 * len(env.joint_names),)
    result = env.step(np.zeros(len(env.joint_names)))
    assert len(result) == 5
    assert isinstance(result[1], float)


def test_the_same_seed_repeats_and_another_does_not() -> None:
    env = make_arm()
    first = rollout(env, 3)
    np.testing.assert_array_equal(first, rollout(env, 3))
    assert not np.array_equal(first, rollout(env, 4))


def test_an_episode_ends_by_time() -> None:
    env = make_arm()
    env.reset(seed=1)
    ended = [env.step(np.zeros(len(env.joint_names)))[3] for _ in range(10)]
    assert ended == [False] * 9 + [True]


def test_randomize_draws_the_site_at_each_reset() -> None:
    env = LagrangeEnv(arm(), randomize={"gravity": (1.0, 2.0)})
    _, one = env.reset(seed=1)
    _, two = env.reset()
    assert 1.0 <= one["site"]["gravity"] <= 2.0
    assert one["site"]["gravity"] != two["site"]["gravity"]


def test_a_bad_mode_is_refused() -> None:
    with pytest.raises(ValueError, match="mode"):
        LagrangeEnv(arm(), mode="torque")


def test_parallel_environments_step_together() -> None:
    envs = ParallelEnvs(make_arm, 2)
    try:
        observations = envs.reset(seed=1)
        assert observations.shape[0] == 2
        actions = np.zeros((2, observations.shape[1] // 2))
        ended = [envs.step(actions)[2] for _ in range(10)]
        assert ended[-1].all()
    finally:
        envs.close()

"""Reinforcement learning on Lagrange: the Gymnasium interface, and environments in parallel.

    from lagrange.envs import LagrangeEnv, ParallelEnvs

    env = LagrangeEnv("arm.urdf", mode="effort", action_scale=50.0, reward=my_reward)
    observation, info = env.reset(seed=1)
    observation, reward, terminated, truncated, info = env.step(env.action_space.sample())

It follows Gymnasium's Env interface, and is one when `gymnasium` is installed; it needs only numpy
otherwise. Each environment is a whole simulation with every domain the robot's file asks for, one
per process: there are no batches on the GPU, so what parallel runs give is the machine's cores.
"""

from __future__ import annotations

import multiprocessing
from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from lagrange._lagrange import Simulation

try:
    import gymnasium
    from gymnasium.spaces import Box

    _Base: type = gymnasium.Env
except ImportError:
    gymnasium = None

    class Box:  # type: ignore[no-redef]
        """What is used of gymnasium.spaces.Box when Gymnasium is not installed."""

        def __init__(self, low: float, high: float, shape: tuple[int, ...], dtype: Any) -> None:
            self.low = np.full(shape, low, dtype=dtype)
            self.high = np.full(shape, high, dtype=dtype)
            self.shape = shape
            self.dtype = dtype
            self._rng = np.random.default_rng()

        def sample(self) -> np.ndarray:
            return self._rng.uniform(self.low, self.high).astype(self.dtype)

        def seed(self, seed: int | None = None) -> None:
            self._rng = np.random.default_rng(seed)

    _Base = object


Reward = Callable[[np.ndarray, np.ndarray, Simulation], float]
Terminated = Callable[[np.ndarray, Simulation], bool]


class LagrangeEnv(_Base):  # type: ignore[misc]
    """One robot in one simulation.

    Actions are one number per joint in -1..1, scaled by `action_scale` and sent as `mode`
    (effort, position or velocity). Observations are each joint's position and velocity, then every
    component of each signal in `signals`, in that order.

    `randomize` draws site parameters afresh at each reset, by name: {"gravity": (9.0, 10.5)}.
    Reset goes back to the saved first state, which is what makes it cheap and repeatable.
    """

    metadata: dict[str, Any] = {"render_modes": []}  # noqa: RUF012

    def __init__(
        self,
        robot: str,
        world: str | None = None,
        *,
        joints: Sequence[str] | None = None,
        mode: str = "effort",
        action_scale: float = 1.0,
        signals: Sequence[str] = (),
        step_seconds: float = 0.02,
        episode_seconds: float = 10.0,
        reward: Reward | None = None,
        terminated: Terminated | None = None,
        randomize: dict[str, tuple[float, float]] | None = None,
    ) -> None:
        self.sim = Simulation(robot, world=world, joint_control=True)
        self.joint_names = list(joints) if joints is not None else self._driven_joints()
        if not self.joint_names:
            raise ValueError("the robot has no joints to command")
        if mode not in ("effort", "position", "velocity"):
            raise ValueError(f"mode must be effort, position or velocity, not {mode}")
        self.mode = mode
        self.action_scale = action_scale
        self.extra_signals = list(signals)
        # What _observe reads, in order: named once, not at every step.
        self._observed = [
            f"joint/{joint}/{quantity}"
            for joint in self.joint_names
            for quantity in ("position", "velocity")
        ] + self.extra_signals
        self.step_seconds = step_seconds
        self.episode_seconds = episode_seconds
        self.reward_function = reward
        self.terminated_function = terminated
        self.randomize = dict(randomize or {})
        self._first_state = self.sim.save_state()
        self._rng = np.random.default_rng()
        self._start_time = self.sim.time
        size = len(self._observe())
        self.observation_space = Box(-np.inf, np.inf, (size,), np.float64)
        self.action_space = Box(-1.0, 1.0, (len(self.joint_names),), np.float32)

    def _driven_joints(self) -> list[str]:
        """The joints that report a position: hinges and slides, not fixed ones."""
        names = {signal["name"] for signal in self.sim.signals()}
        return [joint for joint in self.sim.joints() if f"joint/{joint}/position" in names]

    def _observe(self) -> np.ndarray:
        read = self.sim.read
        values: list[float] = []
        for name in self._observed:
            values += read(name)
        return np.asarray(values, dtype=np.float64)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        if gymnasium is not None:
            super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
            self.action_space.seed(seed)
        self.sim.restore_state(self._first_state)
        drawn = {name: float(self._rng.uniform(*span)) for name, span in self.randomize.items()}
        for name, value in drawn.items():
            self.sim.set_site(name, value)
        self._start_time = self.sim.time
        return self._observe(), {"site": drawn}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        clipped = np.clip(action, -1.0, 1.0)
        for joint, value in zip(self.joint_names, clipped, strict=True):
            self.sim.command_joint(joint, self.mode, float(value) * self.action_scale)
        self.sim.step_until(self.sim.time + self.step_seconds)
        observation = self._observe()
        reward = (
            float(self.reward_function(observation, clipped, self.sim))
            if self.reward_function is not None
            else 0.0
        )
        terminated = (
            bool(self.terminated_function(observation, self.sim))
            if self.terminated_function is not None
            else False
        )
        truncated = self.sim.time - self._start_time >= self.episode_seconds - 1e-9
        return observation, reward, terminated, truncated, {}


def _worker(connection: Any, make: Callable[[], LagrangeEnv]) -> None:
    env = make()
    while True:
        command, argument = connection.recv()
        if command == "reset":
            connection.send(env.reset(seed=argument))
        elif command == "step":
            result = env.step(argument)
            if result[2] or result[3]:  # start the next episode, as vector environments do
                observation, _info = env.reset()
                result = (observation, *result[1:])
            connection.send(result)
        else:
            connection.close()
            return


class ParallelEnvs:
    """`count` environments, one process each, stepped together. An ended one starts again.

    `make` builds one environment in its process; it must be a function at a module's top level.
    """

    def __init__(self, make: Callable[[], LagrangeEnv], count: int) -> None:
        context = multiprocessing.get_context("spawn")
        self._connections = []
        self._processes = []
        for _ in range(count):
            mine, theirs = context.Pipe()
            process = context.Process(target=_worker, args=(theirs, make), daemon=True)
            process.start()
            self._connections.append(mine)
            self._processes.append(process)

    def __len__(self) -> int:
        return len(self._connections)

    def reset(self, seed: int = 0) -> np.ndarray:
        for i, connection in enumerate(self._connections):
            connection.send(("reset", seed + i))
        return np.stack([connection.recv()[0] for connection in self._connections])

    def step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Observations, rewards and whether each episode ended at this step."""
        for connection, action in zip(self._connections, actions, strict=True):
            connection.send(("step", action))
        results = [connection.recv() for connection in self._connections]
        return (
            np.stack([r[0] for r in results]),
            np.array([r[1] for r in results]),
            np.array([r[2] or r[3] for r in results]),
        )

    def close(self) -> None:
        for connection in self._connections:
            connection.send(("close", None))
        for process in self._processes:
            process.join(timeout=10.0)

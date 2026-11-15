"""Reinforcement learning on Lagrange: the Gymnasium interface, and environments in parallel.

    from lagrange.envs import LagrangeEnv, ParallelEnvs, VectorEnv

    env = LagrangeEnv("arm.urdf", mode="effort", action_scale=50.0, reward=my_reward)
    observation, info = env.reset(seed=1)
    observation, reward, terminated, truncated, info = env.step(env.action_space.sample())

It follows Gymnasium's Env interface, and is one when `gymnasium` is installed; it needs only numpy
otherwise. Each environment is a whole simulation with every domain the robot's file asks for, in a
worker process: there are no batches on the GPU, so what parallel runs give is the machine's cores.

`ParallelEnvs` and `VectorEnv` run `count` environments in worker processes. Actions, observations,
rewards and episode ends pass through shared memory, and a step wakes each worker with a semaphore,
so a step costs no pickling and no pipe; a worker restarts an ended episode itself. `VectorEnv` has
Gymnasium's VectorEnv interface (and is one when Gymnasium is installed), with the same-step
autoreset; `ParallelEnvs` keeps the shorter interface of earlier releases and adds `send`/`recv`,
which step some environments while others still run, so a slow one does not hold the rest.
"""

from __future__ import annotations

import contextlib
import math
import multiprocessing
import os
import time
import traceback
import weakref
from collections.abc import Callable, Sequence
from multiprocessing.shared_memory import SharedMemory
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

try:  # Gymnasium's vector interface with autoreset modes: Gymnasium 1.1 and later
    from gymnasium.vector import AutoresetMode
    from gymnasium.vector import VectorEnv as _GymVectorEnv
    from gymnasium.vector.utils import batch_space

    _VectorBase: type = _GymVectorEnv
    _SAME_STEP: Any = AutoresetMode.SAME_STEP
except ImportError:

    def batch_space(space: Any, n: int) -> Any:
        """A Box of `n` rows of `space`, as Gymnasium's batch_space gives for a Box."""
        batched = Box(0.0, 0.0, (n, *space.shape), space.dtype)
        batched.low = np.broadcast_to(space.low, batched.shape).copy()
        batched.high = np.broadcast_to(space.high, batched.shape).copy()
        return batched

    _VectorBase = object
    _SAME_STEP = "SameStep"


Reward = Callable[[np.ndarray, np.ndarray, Simulation], float]
Terminated = Callable[[np.ndarray, Simulation], bool]
Seeds = int | Sequence[int | None] | None


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
        observation: np.ndarray = self.sim.read_many(self._observed)
        return observation

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
        if len(clipped) != len(self.joint_names):
            raise ValueError(f"{len(self.joint_names)} actions expected, not {len(clipped)}")
        # In float64 before scaling, as float(value) * action_scale is for each joint.
        values = (np.asarray(clipped, dtype=np.float64) * self.action_scale).tolist()
        self.sim.command_joints(self.joint_names, self.mode, values)
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


# --- Worker processes ------------------------------------------------------------------------
#
# Each worker owns a run of consecutive environments. Its control row in shared memory holds the
# command, the command's number, the number of the last command it finished, and a status. The
# main process writes the actions and the row, then releases the worker's semaphore; the worker
# steps its environments, writes observations, rewards and ends, sets "finished" and releases the
# one semaphore every worker shares. Only reset (seeds, options and infos) and errors use the pipe.

_STEP, _RESET, _CLOSE = 1, 2, 3
_COMMAND, _NUMBER, _FINISHED, _STATUS = 0, 1, 2, 3
_OK, _FAILED = 0, 1
_POLL_SECONDS = 0.5  # how often a waiting process looks whether the other side still lives
# How long a waiting process watches shared memory before it sleeps on its semaphore. Waking a
# sleeping process costs tens of microseconds and more on a busy machine; a step of a legged robot
# costs a few hundred, so watching for about one step saves most of the wake-ups.
DEFAULT_SPIN_SECONDS = 0.001


class _Layout:
    """Arrays over one shared-memory block: no pickling at any step."""

    def __init__(self, buffer: Any, envs: int, workers: int, observed: int, acted: int) -> None:
        offset = 0

        def take(shape: tuple[int, ...], dtype: Any) -> np.ndarray:
            nonlocal offset
            array = np.ndarray(shape, dtype=dtype, buffer=buffer, offset=offset)
            offset += array.nbytes
            return array

        self.control = take((workers, 4), np.int64)
        self.actions = take((envs, acted), np.float64)
        self.observations = take((envs, observed), np.float64)
        self.final_observations = take((envs, observed), np.float64)
        self.rewards = take((envs,), np.float64)
        self.terminated = take((envs,), np.bool_)
        self.truncated = take((envs,), np.bool_)

    @staticmethod
    def size(envs: int, workers: int, observed: int, acted: int) -> int:
        return 8 * (4 * workers + envs * (acted + 2 * observed + 1)) + 2 * envs


def _run_worker(
    index: int,
    first: int,
    count: int,
    make: Callable[[], Any],
    connection: Any,
    go: Any,
    finished: Any,
    parent: int,
) -> None:
    """A worker process: builds `count` environments, then serves commands until told to close."""
    try:
        envs = [make() for _ in range(count)]
        connection.send(("spaces", envs[0].observation_space, envs[0].action_space))
        for env in envs[1:]:
            if (
                env.observation_space.shape != envs[0].observation_space.shape
                or env.action_space.shape != envs[0].action_space.shape
            ):
                raise ValueError("make() gave environments with different spaces")
        name, total, workers, observed, acted, spin = connection.recv()
    except Exception:
        with contextlib.suppress(OSError):
            connection.send(("error", traceback.format_exc()))
        return
    memory = SharedMemory(name=name, track=False)
    layout = _Layout(memory.buf, total, workers, observed, acted)
    try:
        _serve(index, first, envs, layout, connection, go, finished, parent, spin)
    finally:
        del layout
        memory.close()
        for env in envs:
            close = getattr(env, "close", None)
            if close is not None:
                with contextlib.suppress(Exception):
                    close()


def _serve(
    index: int,
    first: int,
    envs: list[Any],
    layout: _Layout,
    connection: Any,
    go: Any,
    finished: Any,
    parent: int,
    spin: float,
) -> None:
    control = layout.control[index]
    seen = int(control[_NUMBER])
    rows = range(first, first + len(envs))
    actions, observations, final = layout.actions, layout.observations, layout.final_observations
    rewards, terminated, truncated = layout.rewards, layout.terminated, layout.truncated
    pairs = list(zip(rows, envs, strict=True))
    clock = time.perf_counter
    while True:
        if spin > 0.0:  # the next command often comes at once: watch for it before sleeping
            until = clock() + spin
            while control[_NUMBER] == seen and clock() < until:
                pass
        if not go.acquire(timeout=_POLL_SECONDS):
            if os.getppid() != parent:  # the main process is gone: so is the reason to run
                return
            continue
        seen = int(control[_NUMBER])
        command = int(control[_COMMAND])
        try:
            if command == _STEP:
                for row, env in pairs:
                    observation, reward, end, cut, _info = env.step(actions[row].copy())
                    if end or cut:  # start the next episode, as vector environments do
                        final[row] = observation
                        observation, _info = env.reset()
                    observations[row] = observation
                    rewards[row] = reward
                    terminated[row] = end
                    truncated[row] = cut
            elif command == _RESET:
                seeds, options = connection.recv()
                infos = []
                for (row, env), seed in zip(pairs, seeds, strict=True):
                    observation, info = env.reset(seed=seed, options=options)
                    observations[row] = observation
                    infos.append(info)
                connection.send(("infos", infos))
            else:
                control[_FINISHED] = control[_NUMBER]
                finished.release()
                return
        except Exception:
            with contextlib.suppress(OSError):
                connection.send(("error", traceback.format_exc()))
            control[_STATUS] = _FAILED
            control[_FINISHED] = control[_NUMBER]
            finished.release()
            return
        control[_FINISHED] = control[_NUMBER]
        finished.release()


def _stop(processes: list[Any], memory: SharedMemory | None) -> None:
    """The last resort, also at exit: no worker left running, the shared block removed."""
    for process in processes:
        if process.is_alive():
            process.terminate()
    for process in processes:
        process.join(timeout=2.0)
        if process.is_alive():
            process.kill()
            process.join()
    if memory is not None:
        with contextlib.suppress(Exception):
            memory.unlink()


class WorkerError(RuntimeError):
    """A worker process failed (its environment raised, or the process died). The pool is closed."""


class _Pool:
    """Worker processes and the shared memory they step in."""

    def __init__(
        self,
        make: Callable[[], Any],
        count: int,
        envs_per_worker: int,
        start_timeout: float | None,
        spin_seconds: float = DEFAULT_SPIN_SECONDS,
    ) -> None:
        if count < 1 or envs_per_worker < 1:
            raise ValueError("count and envs_per_worker must be at least 1")
        self.closed = False
        self.spin_seconds = max(0.0, spin_seconds)
        self._memory: SharedMemory | None = None
        context = multiprocessing.get_context("spawn")
        workers = math.ceil(count / envs_per_worker)
        self.firsts = [w * envs_per_worker for w in range(workers)]
        self.sizes = [min(envs_per_worker, count - first) for first in self.firsts]
        self.count = count
        self._connections: list[Any] = []
        self._processes: list[Any] = []
        self._go = [context.Semaphore(0) for _ in range(workers)]
        self._finished = context.Semaphore(0)
        for w in range(workers):
            mine, theirs = context.Pipe()
            process = context.Process(
                target=_run_worker,
                args=(
                    w,
                    self.firsts[w],
                    self.sizes[w],
                    make,
                    theirs,
                    self._go[w],
                    self._finished,
                    os.getpid(),
                ),
                daemon=True,
                name=f"lagrange-env-{w}",
            )
            process.start()
            theirs.close()
            self._connections.append(mine)
            self._processes.append(process)
        self._finalizer = weakref.finalize(self, _stop, self._processes, None)
        try:
            spaces = [self._receive(w, "spaces", start_timeout) for w in range(workers)]
        except BaseException:
            self._finalizer()
            raise
        self.observation_space, self.action_space = spaces[0][0], spaces[0][1]
        for observation_space, action_space in spaces[1:]:
            if (
                observation_space.shape != self.observation_space.shape
                or action_space.shape != self.action_space.shape
            ):
                self._finalizer()
                raise ValueError("make() gave environments with different spaces")
        if len(self.observation_space.shape) != 1 or len(self.action_space.shape) != 1:
            self._finalizer()
            raise ValueError("parallel environments take one-dimensional observations and actions")
        observed, acted = self.observation_space.shape[0], self.action_space.shape[0]
        self._memory = SharedMemory(create=True, size=_Layout.size(count, workers, observed, acted))
        self._finalizer.detach()
        self._finalizer = weakref.finalize(self, _stop, self._processes, self._memory)
        self._layout = _Layout(self._memory.buf, count, workers, observed, acted)
        self._layout.control[:] = 0
        for connection in self._connections:
            connection.send((self._memory.name, count, workers, observed, acted, self.spin_seconds))
        self._numbers = np.zeros(workers, dtype=np.int64)
        self._busy = np.zeros(workers, dtype=bool)
        self._credits = 0  # semaphore releases taken minus finished commands found

    @property
    def workers(self) -> int:
        return len(self._processes)

    def worker_of(self, env: int) -> int:
        return env // self.sizes[0]

    def _check_open(self) -> None:
        if self.closed:
            raise WorkerError("the environments are closed")

    def _receive(self, worker: int, tag: str, timeout: float | None) -> Any:
        """The next message from a worker, which must carry `tag`; WorkerError when it fails."""
        connection, process = self._connections[worker], self._processes[worker]
        waited = 0.0
        while not connection.poll(_POLL_SECONDS):
            waited += _POLL_SECONDS
            if not process.is_alive():
                raise WorkerError(
                    f"environment worker {worker} died (exit code {process.exitcode})"
                )
            if timeout is not None and waited >= timeout:
                raise WorkerError(f"environment worker {worker} did not answer in {timeout} s")
        try:
            message = connection.recv()
        except EOFError as error:
            raise WorkerError(f"environment worker {worker} closed its pipe") from error
        if message[0] == "error":
            raise WorkerError(f"environment worker {worker} failed:\n{message[1]}")
        if message[0] != tag:
            raise WorkerError(f"environment worker {worker} sent {message[0]}, not {tag}")
        return message[1:] if len(message) > 2 else message[1]

    def command(self, workers: Sequence[int], what: int) -> None:
        """Start `what` on these workers (their actions or reset arguments already given)."""
        control = self._layout.control
        for w in workers:
            if self._busy[w]:
                raise RuntimeError(f"environment worker {w} has not finished its last step")
            self._numbers[w] += 1
            control[w, _COMMAND] = what
            control[w, _NUMBER] = self._numbers[w]
            self._busy[w] = True
            self._go[w].release()

    def collect(self, minimum: int, timeout: float | None) -> np.ndarray:
        """Wait until at least `minimum` busy workers have finished; those that have, in order.

        Each worker releases the shared semaphore once per command, after marking it finished,
        so taking one release per finished command keeps the count right across calls.
        """
        self._check_open()
        control = self._layout.control
        found: list[int] = []
        waited = 0.0
        if self.spin_seconds > 0.0 and minimum > 0:  # most steps end soon: watch before sleeping
            clock = time.perf_counter
            until = clock() + self.spin_seconds
            busy, numbers, finished = self._busy, self._numbers, control[:, _FINISHED]
            while clock() < until and np.count_nonzero(busy & (finished == numbers)) < minimum:
                pass
        while len(found) < minimum:
            while self._credits < minimum - len(found):
                if self._finished.acquire(timeout=_POLL_SECONDS):
                    self._credits += 1
                    continue
                waited += _POLL_SECONDS
                self._check_alive()
                if timeout is not None and waited >= timeout:
                    raise TimeoutError(f"the environments did not finish in {timeout} s")
            done = np.flatnonzero(self._busy & (control[:, _FINISHED] == self._numbers))
            self._busy[done] = False
            self._credits -= len(done)
            found += done.tolist()
        failed = [w for w in found if control[w, _STATUS] != _OK]
        if failed:
            message = self._failure(failed[0])
            self.close()
            raise WorkerError(message)
        return np.array(sorted(found), dtype=np.int64)

    def _failure(self, worker: int) -> str:
        try:
            self._receive(worker, "nothing", 5.0)
        except WorkerError as error:
            return str(error)
        return f"environment worker {worker} failed"

    def _check_alive(self) -> None:
        for w, process in enumerate(self._processes):
            if self._busy[w] and not process.is_alive():
                code = process.exitcode
                self.close()
                raise WorkerError(f"environment worker {w} died (exit code {code})")

    def reset(self, seeds: list[int | None], options: dict[str, Any] | None) -> list[Any]:
        self._check_open()
        if self._busy.any():
            self.collect(int(self._busy.sum()), None)  # a step still running ends first
        for w, connection in enumerate(self._connections):
            first = self.firsts[w]
            connection.send((seeds[first : first + self.sizes[w]], options))
        everyone = list(range(self.workers))
        self.command(everyone, _RESET)
        self.collect(self.workers, None)
        infos: list[Any] = []
        for w in everyone:
            try:
                infos += self._receive(w, "infos", 30.0)
            except WorkerError:
                self.close()
                raise
        return infos

    def close(self, timeout: float = 10.0) -> None:
        if self.closed:
            return
        self.closed = True
        if self._memory is not None and hasattr(self, "_layout"):
            control = self._layout.control
            for w, process in enumerate(self._processes):
                if process.is_alive():
                    control[w, _COMMAND] = _CLOSE
                    self._go[w].release()
            for process in self._processes:
                process.join(timeout=timeout)
            del control
            del self._layout
        _stop(self._processes, self._memory)
        for connection in self._connections:
            connection.close()
        if self._memory is not None:
            with contextlib.suppress(BufferError):
                self._memory.close()
        self._finalizer.detach()

    # Views of the results, valid until the next command.

    @property
    def layout(self) -> _Layout:
        return self._layout

    def envs_of(self, workers: np.ndarray) -> np.ndarray:
        if len(workers) == self.workers:
            return np.arange(self.count)
        return np.concatenate(
            [np.arange(self.firsts[w], self.firsts[w] + self.sizes[w]) for w in workers]
        )

    def workers_of(self, envs: np.ndarray) -> list[int]:
        """The workers that own exactly these environments; ValueError when one is split."""
        workers = sorted({self.worker_of(int(e)) for e in envs})
        owned = sum(self.sizes[w] for w in workers)
        if owned != len(set(envs.tolist())) or len(envs) != owned:
            raise ValueError(
                "send() takes all the environments of a worker or none: with envs_per_worker="
                f"{self.sizes[0]}, give whole groups of consecutive environments"
            )
        return workers


def _seeds_of(seed: Seeds, count: int) -> list[int | None]:
    if seed is None:
        return [None] * count
    if isinstance(seed, int):
        return [seed + i for i in range(count)]
    seeds = list(seed)
    if len(seeds) != count:
        raise ValueError(f"{count} seeds expected, not {len(seeds)}")
    return seeds


class ParallelEnvs:
    """`count` environments in worker processes, stepped together. An ended one starts again.

    `make` builds one environment in its process; it must be a function at a module's top level.
    `envs_per_worker` puts that many in each process, stepped one after another (fewer processes
    for cheap environments). A worker that raises or dies raises WorkerError here and closes all.

    step() waits for every environment; step_async() and step_wait() are the same in two halves,
    so the caller can work while they step. send() and recv() step some environments while others
    still run: recv() gives back those finished first, with their indices.

    Waiting processes watch shared memory for `spin_seconds` (default 1 ms) before they sleep on a
    semaphore: faster steps for a little CPU while waiting; 0 never spins.
    """

    def __init__(
        self,
        make: Callable[[], Any],
        count: int,
        *,
        envs_per_worker: int = 1,
        start_timeout: float | None = 300.0,
        step_timeout: float | None = None,
        spin_seconds: float = DEFAULT_SPIN_SECONDS,
    ) -> None:
        self._pool = _Pool(make, count, envs_per_worker, start_timeout, spin_seconds)
        self.step_timeout = step_timeout
        self.observation_space = self._pool.observation_space
        self.action_space = self._pool.action_space
        self.num_envs = count
        observed = self._pool.layout.observations.shape[1]
        self.terminated = np.zeros(count, dtype=bool)
        self.truncated = np.zeros(count, dtype=bool)
        # The observation each environment's last episode ended with (rows where it ended).
        self.final_observations = np.zeros((count, observed))

    def __len__(self) -> int:
        return self.num_envs

    def __enter__(self) -> ParallelEnvs:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    @property
    def closed(self) -> bool:
        return self._pool.closed

    @property
    def processes(self) -> list[Any]:
        """The worker processes (multiprocessing.Process), for monitoring."""
        return list(self._pool._processes)

    def reset(self, seed: Seeds = 0, options: dict[str, Any] | None = None) -> np.ndarray:
        """Every environment's first observation. Environment i gets seed + i (a list: its own;
        None: no new seed), as LagrangeEnv.reset(seed=...) would in one process."""
        self.infos = self._pool.reset(_seeds_of(seed, self.num_envs), options)
        return self._pool.layout.observations.copy()

    def step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Observations, rewards and whether each episode ended at this step."""
        self.step_async(actions)
        return self.step_wait()

    def step_async(self, actions: np.ndarray) -> None:
        """Start a step of every environment; step_wait() gives its results."""
        self.send(actions)

    def step_wait(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The step started by step_async(): observations, rewards and whether each ended."""
        pool = self._pool
        pool.collect(int(pool._busy.sum()), self.step_timeout)
        layout = pool.layout
        self.terminated[:] = layout.terminated
        self.truncated[:] = layout.truncated
        ended = self.terminated | self.truncated
        self.final_observations[ended] = layout.final_observations[ended]
        return layout.observations.copy(), layout.rewards.copy(), ended

    def send(self, actions: np.ndarray, env_ids: Sequence[int] | np.ndarray | None = None) -> None:
        """Start a step of these environments (all by default) with one action row each."""
        pool = self._pool
        pool._check_open()
        if env_ids is None:
            workers: Sequence[int] = range(pool.workers)
            pool.layout.actions[:] = np.asarray(actions, dtype=np.float64).reshape(
                pool.layout.actions.shape
            )
        else:
            ids = np.asarray(env_ids, dtype=np.int64)
            workers = pool.workers_of(ids)
            pool.layout.actions[ids] = np.asarray(actions, dtype=np.float64).reshape((len(ids), -1))
        pool.command(workers, _STEP)

    def recv(
        self, min_envs: int = 1
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Wait until at least `min_envs` environments sent by send() have stepped; gives their
        indices, observations, rewards, terminated and truncated (an ended one has started again:
        its last observation is in final_observations)."""
        pool = self._pool
        sizes = pool.sizes
        finished: list[int] = []
        envs = 0
        while envs < min_envs or not finished:
            if not pool._busy.any():
                if finished:
                    break
                raise RuntimeError("recv() with no step sent")
            more = pool.collect(1, self.step_timeout).tolist()
            finished += more
            envs += sum(sizes[w] for w in more)
        ids = pool.envs_of(np.array(sorted(finished), dtype=np.int64))
        layout = pool.layout
        self.terminated[ids] = layout.terminated[ids]
        self.truncated[ids] = layout.truncated[ids]
        ended = ids[self.terminated[ids] | self.truncated[ids]]
        self.final_observations[ended] = layout.final_observations[ended]
        return (
            ids,
            layout.observations[ids],
            layout.rewards[ids],
            layout.terminated[ids],
            layout.truncated[ids],
        )

    def close(self) -> None:
        """Stop every worker process (none is left behind) and free the shared memory."""
        self._pool.close()


def _merge_infos(infos: dict[str, Any], info: dict[str, Any], i: int, n: int) -> dict[str, Any]:
    """Gymnasium's vector info layout (VectorEnv._add_info): each key an array over the
    environments with a boolean `_key` mask of those that gave it; dicts nest."""
    for key, value in info.items():
        if isinstance(value, dict):
            infos[key] = _merge_infos(infos.get(key, {}), value, i, n)
        else:
            if key not in infos:
                if type(value) in (int, float, bool) or isinstance(value, np.number):
                    infos[key] = np.zeros(n, dtype=type(value))
                elif isinstance(value, np.ndarray):
                    infos[key] = np.zeros((n, *value.shape), dtype=value.dtype)
                else:
                    infos[key] = np.full(n, None, dtype=object)
            infos[key][i] = value
        mask = infos.setdefault(f"_{key}", np.zeros(n, dtype=bool))
        mask[i] = True
    return infos


class VectorEnv(_VectorBase):  # type: ignore[misc]
    """`num_envs` environments in worker processes with Gymnasium's VectorEnv interface.

    reset(seed=, options=) gives (observations, infos); step(actions) gives (observations,
    rewards, terminations, truncations, infos). It resets an ended environment in the same step
    (metadata["autoreset_mode"] is same-step): the observation is the next episode's first, and
    infos["final_obs"] and infos["final_info"] hold the ended one's (masks "_final_obs", ...).
    Per-step infos of the environments are not carried, only their numbers.
    """

    metadata: dict[str, Any] = {"autoreset_mode": _SAME_STEP}  # noqa: RUF012

    def __init__(
        self,
        make: Callable[[], Any],
        num_envs: int,
        *,
        envs_per_worker: int = 1,
        start_timeout: float | None = 300.0,
        step_timeout: float | None = None,
        spin_seconds: float = DEFAULT_SPIN_SECONDS,
    ) -> None:
        self._envs = ParallelEnvs(
            make,
            num_envs,
            envs_per_worker=envs_per_worker,
            start_timeout=start_timeout,
            step_timeout=step_timeout,
            spin_seconds=spin_seconds,
        )
        self.num_envs = num_envs
        self.single_observation_space = self._envs.observation_space
        self.single_action_space = self._envs.action_space
        self.observation_space = batch_space(self.single_observation_space, num_envs)
        self.action_space = batch_space(self.single_action_space, num_envs)
        self.closed = False

    def reset(
        self, *, seed: Seeds = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        if gymnasium is not None and isinstance(seed, int):
            super().reset(seed=seed)
        observations = self._envs.reset(seed, options)
        infos: dict[str, Any] = {}
        for i, info in enumerate(self._envs.infos):
            infos = _merge_infos(infos, info, i, self.num_envs)
        return observations, infos

    def step_async(self, actions: np.ndarray) -> None:
        self._envs.step_async(actions)

    def step_wait(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        observations, rewards, ended = self._envs.step_wait()
        infos: dict[str, Any] = {}
        if ended.any():
            final_obs = np.full(self.num_envs, None, dtype=object)
            final_info = np.full(self.num_envs, None, dtype=object)
            for i in np.flatnonzero(ended):
                final_obs[i] = self._envs.final_observations[i].copy()
                final_info[i] = {}
            infos = {
                "final_obs": final_obs,
                "_final_obs": ended.copy(),
                "final_info": {},
                "_final_info": ended.copy(),
            }
        return (
            observations,
            rewards,
            self._envs.terminated.copy(),
            self._envs.truncated.copy(),
            infos,
        )

    def step(
        self, actions: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        self.step_async(actions)
        return self.step_wait()

    def close_extras(self, **_kwargs: Any) -> None:
        self._envs.close()

    def close(self, **kwargs: Any) -> None:
        if self.closed:
            return
        self.close_extras(**kwargs)
        self.closed = True

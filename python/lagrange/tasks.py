"""Tasks for reinforcement learning: what to observe, how actions act, the reward and the ends.

    from lagrange.tasks import QuadHover, AmrGoToGoal, make
    from lagrange.envs import VectorEnv

    env = QuadHover()
    observation, info = env.reset(seed=1)
    observation, reward, terminated, truncated, info = env.step(env.action_space.sample())

    envs = VectorEnv(functools.partial(make, "QuadHover"), 8)   # one process each

Each task is a LagrangeEnv (the Gymnasium Env interface, a Gymnasium Env when Gymnasium is
installed) on the sample robots, with its own observation, action mapping, reward, termination,
truncation and reset distribution. They pickle by their class and options, so ParallelEnvs and
VectorEnv build them in worker processes (`functools.partial(QuadHover, ...)`, or `make` with
the task's name as above).

QuadHover and QuadWaypoint fly the sample quadcopter by collective thrust and body rates
(Simulation.command_flight("rates", ...)): the rate loop and the mixer of the built-in controller
stay, the position and attitude loops are the policy's. AmrGoToGoal drives the sample AMR's two
wheels by speed in the sample warehouse. Every task's reward terms and ranges are in its class's
docstring and in docs/guide/training-a-policy.md.

Each quadcopter task also has `step_builtin()`, a step flown by Lagrange's own controller toward
the same target (velocity commands to its position loop), and the AMR task a hand-written
go-to-goal law: the reference a learned policy is measured against.
"""

from __future__ import annotations

import math
import os
from typing import Any

import numpy as np

from lagrange import sample_quadcopter, sample_robot, sample_world
from lagrange.envs import Box, LagrangeEnv

GRAVITY = 9.81  # m/s², what the built-in flight controller takes for g (thrust over mass)
QUAD_ROTORS = ("rotor_1_joint", "rotor_2_joint", "rotor_3_joint", "rotor_4_joint")
QUAD_PROFILE = "quad_x500_rl"  # profiles/examples/quad_x500_rl.yaml: no sensor rendering
HOVER_HEIGHT = 2.0  # m above its stand, where the built-in controller hovers before a task
CRASH_PENALTY = -10.0  # reward at a crash, flying away, or a collision (ends the episode)


def rotation(quaternion: np.ndarray) -> np.ndarray:
    """The world-from-body rotation matrix of a quaternion (w, x, y, z) as poses carry it."""
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


class _Task(LagrangeEnv):
    """What every task shares: a first state saved after `warm_up`, a seeded reset that draws
    the task, and a step that acts, advances `step_seconds` and judges."""

    observation_size = 0
    action_size = 0

    def _set_spaces(self) -> None:
        self.observation_space = Box(-np.inf, np.inf, (self.observation_size,), np.float64)
        self.action_space = Box(-1.0, 1.0, (self.action_size,), np.float32)

    def _save_first_state(self) -> None:
        self._first_state = self.sim.save_state()

    # Subclasses: draw the task, act, observe, judge.
    def _draw(self, options: dict[str, Any] | None) -> None:
        raise NotImplementedError

    def _act(self, action: np.ndarray) -> None:
        raise NotImplementedError

    def _observe(self) -> np.ndarray:
        raise NotImplementedError

    def _judge(self, action: np.ndarray) -> tuple[float, bool, dict[str, Any]]:
        raise NotImplementedError

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _observation, info = super().reset(seed=seed, options=options)
        self._draw(options)
        self._start_time = self.sim.time
        return self._observe(), info

    def _advance(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        self.sim.step_until(self.sim.time + self.step_seconds)
        reward, terminated, info = self._judge(action)
        truncated = (
            not terminated and self.sim.time - self._start_time >= self.episode_seconds - 1e-9
        )
        info["success"] = bool(info.get("success", False))
        return self._observe(), reward, terminated, truncated, info

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        clipped = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if clipped.shape != (self.action_size,):
            raise ValueError(f"{self.action_size} actions expected, not {clipped.shape}")
        self._act(clipped)
        return self._advance(clipped)


class _QuadTask(_Task):
    """The sample quadcopter flown by collective thrust and body rates.

    Action (4, each -1..1): collective thrust over mass g·(1 + thrust_range·a0) m/s² (g hovers
    level; with the default 0.5, 0.5 g to 1.5 g), and body roll, pitch and yaw rates
    a1..a3 * max_rates rad/s (default 2, 2, 1). Why thrust and rates rather than each rotor's
    command: the controller's rate loop and mixer stay, so the policy need not learn the
    propellers' torque-to-thrust ratios or the motors' lag, it learns faster, and it outputs what
    a real autopilot takes from an offboard computer (PX4's rate set point, Betaflight's acro
    mode), so the same policy can fly one. Kaufmann et al., "A Benchmark Comparison of Learned
    Control Policies for Agile Quadrotor Flight", ICRA 2022, found thrust and body rates the best
    of the three action spaces they compared.

    Each episode starts from the hover 2 m up that the built-in controller reaches (`warm_up`
    seconds after take-off, saved once), then flies a random thrust and rates for up to
    `kick_seconds` so that no two start alike.
    """

    action_size = 4

    def __init__(
        self,
        *,
        profile: str | None = QUAD_PROFILE,
        step_seconds: float = 0.02,
        episode_seconds: float = 5.0,
        thrust_range: float = 0.5,
        max_rates: tuple[float, float, float] = (2.0, 2.0, 1.0),
        kick_seconds: float = 0.2,
        warm_up: float = 4.0,
        randomize: dict[str, tuple[float, float]] | None = None,
    ) -> None:
        super().__init__(
            sample_quadcopter(),
            None,
            profile=profile,
            joints=QUAD_ROTORS,
            mode="velocity",
            step_seconds=step_seconds,
            episode_seconds=episode_seconds,
            randomize=randomize,
        )
        self.thrust_range = thrust_range
        self.max_rates = np.asarray(max_rates, dtype=np.float64)
        self.kick_seconds = kick_seconds
        self.sim.step_until(warm_up)
        self.home = self._pose()[0].copy()  # m: the hover point
        self.ground = self.home[2] - HOVER_HEIGHT  # m: the base link's height standing
        self._save_first_state()
        self._set_spaces()

    def _pose(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Position (m), velocity (m/s, world), rotation (world from body) and body rates."""
        values = self.sim.read_many(["body/base_link/pose", "body/base_link/twist"])
        rot = rotation(values[3:7])
        return values[0:3], values[7:10], rot, rot.T @ values[10:13]

    def _act(self, action: np.ndarray) -> None:
        thrust = GRAVITY * (1.0 + self.thrust_range * float(action[0]))
        rates = action[1:4] * self.max_rates
        self.sim.command_flight("rates", [thrust, *rates.tolist()])

    def _kick(self) -> None:
        """A random thrust and rates for a random time up to kick_seconds."""
        seconds = float(self._rng.uniform(0.0, self.kick_seconds))
        if seconds > 0.0:
            self._act(self._rng.uniform(-0.5, 0.5, self.action_size))
            self.sim.step_until(self.sim.time + seconds)

    def _down(self, position: np.ndarray, rot: np.ndarray) -> bool:
        """Crashed: below 0.3 m over its stand, or tilted past 80°."""
        return bool(position[2] < self.ground + 0.3 or rot[2, 2] < math.cos(math.radians(80.0)))

    def _builtin_toward(self, target: np.ndarray, speed: float) -> None:
        """Velocity commands to the built-in controller's position loop: towards `target` at
        up to `speed` m/s, slowing within 1 m of it."""
        position, _velocity, rot, _rates = self._pose()
        error = target - position
        distance = float(np.linalg.norm(error))
        wanted = error * min(speed, 1.2 * distance) / max(distance, 1e-9)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        forward = c * wanted[0] + s * wanted[1]
        left = -s * wanted[0] + c * wanted[1]
        self.sim.command_flight("velocity", [forward, left, float(wanted[2]), 0.0])


class QuadHover(_QuadTask):
    """Hold a target point with the sample quadcopter.

    Reset: the target is the hover point plus a uniform offset in x and y within ±1 m and in z
    within -0.5..+1 m, after the random kick (see _QuadTask).

    Observation (18): the target less the position (m, world, 3), the velocity (m/s, world, 3),
    the rotation matrix world from body (row-major, 9) and the body rates (rad/s, 3).

    Reward per step (0.02 s), about -0.3..1:
      + 1 - tanh(|e|)        position, 1 at the target, 0.24 at 1 m (e: error, m)
      - 0.05 |ω|             body rates (rad/s): no wobbling
      - 0.01 |a|²            action size, at most 0.04
      CRASH_PENALTY (-10) instead, and the episode ends, when it comes within 0.3 m of the
      ground, tilts past 80°, or strays 3 m from the target.

    Truncated after episode_seconds (5 s, 250 steps). Success (info["success"] at the last
    step): not ended early, and within 0.2 m of the target.
    """

    observation_size = 18

    def __init__(self, *, target_offset: float = 1.0, **options: Any) -> None:
        self.target_offset = target_offset
        self.target = np.zeros(3)
        super().__init__(**options)
        self.target = self.home.copy()

    def _draw(self, options: dict[str, Any] | None) -> None:
        self.sim.command_flight("hold")  # no command left from the last episode
        span = self.target_offset
        offset = self._rng.uniform([-span, -span, -0.5 * span], [span, span, span])
        self.target = self.home + offset
        if options and "target" in options:
            self.target = np.asarray(options["target"], dtype=np.float64)
        self._kick()

    def _observe(self) -> np.ndarray:
        position, velocity, rot, rates = self._pose()
        return np.concatenate([self.target - position, velocity, rot.ravel(), rates])

    def _judge(self, action: np.ndarray) -> tuple[float, bool, dict[str, Any]]:
        position, _velocity, rot, rates = self._pose()
        distance = float(np.linalg.norm(self.target - position))
        info: dict[str, Any] = {"distance": distance}
        if self._down(position, rot) or distance > 3.0:
            return CRASH_PENALTY, True, info
        reward = 1.0 - math.tanh(distance) - 0.05 * float(np.linalg.norm(rates))
        reward -= 0.01 * float(action @ action)
        info["success"] = distance < 0.2
        return reward, False, info

    def step_builtin(self) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """A step flown by the built-in controller towards the target (the baseline)."""
        self._builtin_toward(self.target, 2.0)
        return self._advance(np.zeros(self.action_size))


class QuadWaypoint(_QuadTask):
    """Fly through a sequence of points with the sample quadcopter.

    Reset: `waypoints` points (default 4), each 1 to 2.5 m from the one before in a random
    direction, within 4 m of the hover point across and 1 m below to 1.5 m above it.

    Observation (21): the current waypoint less the position (m, 3), the next one less the
    position (m, 3; the current one again at the last), the velocity (m/s, 3), the rotation
    matrix (9) and the body rates (rad/s, 3).

    Reward per step (0.02 s):
      + 10 (d_before - d_after)   progress towards the current waypoint (m): 0.2 a step at 1 m/s
      + 5                         when it comes within `reach` (0.3 m) of it: the next is current
      - 0.05 |ω| - 0.01 |a|²      as QuadHover
      CRASH_PENALTY (-10) and the end, as QuadHover, or 4 m from the current waypoint.

    Terminated with success at the last waypoint; truncated after episode_seconds (15 s).
    """

    observation_size = 21

    def __init__(self, *, waypoints: int = 4, reach: float = 0.3, **options: Any) -> None:
        options.setdefault("episode_seconds", 15.0)
        self.count = waypoints
        self.reach = reach
        self.points = np.zeros((waypoints, 3))
        self.current = 0
        self._distance = 0.0
        super().__init__(**options)

    def _draw(self, options: dict[str, Any] | None) -> None:
        self.sim.command_flight("hold")  # no command left from the last episode
        points = []
        here = self.home.copy()
        low = self.home + np.array([-4.0, -4.0, -1.0])
        high = self.home + np.array([4.0, 4.0, 1.5])
        for _ in range(self.count):
            for _attempt in range(100):
                direction = self._rng.normal(size=3)
                direction /= np.linalg.norm(direction)
                point = here + direction * self._rng.uniform(1.0, 2.5)
                if np.all(point >= low) and np.all(point <= high):
                    break
            point = np.clip(point, low, high)
            points.append(point)
            here = point
        self.points = np.array(points)
        if options and "waypoints" in options:
            self.points = np.asarray(options["waypoints"], dtype=np.float64).reshape(-1, 3)
        self.current = 0
        self._kick()
        self._distance = float(np.linalg.norm(self.points[0] - self._pose()[0]))

    def _observe(self) -> np.ndarray:
        position, velocity, rot, rates = self._pose()
        now = self.points[min(self.current, len(self.points) - 1)]
        after = self.points[min(self.current + 1, len(self.points) - 1)]
        return np.concatenate([now - position, after - position, velocity, rot.ravel(), rates])

    def _judge(self, action: np.ndarray) -> tuple[float, bool, dict[str, Any]]:
        position, _velocity, rot, rates = self._pose()
        distance = float(np.linalg.norm(self.points[self.current] - position))
        info: dict[str, Any] = {"distance": distance, "reached": self.current}
        if self._down(position, rot) or distance > 4.0:
            return CRASH_PENALTY, True, info
        reward = 10.0 * (self._distance - distance) - 0.05 * float(np.linalg.norm(rates))
        reward -= 0.01 * float(action @ action)
        self._distance = distance
        if distance < self.reach:
            reward += 5.0
            self.current += 1
            info["reached"] = self.current
            if self.current == len(self.points):
                self.current -= 1  # observations keep the last one
                info["success"] = True
                return reward, True, info
            self._distance = float(np.linalg.norm(self.points[self.current] - position))
        return reward, False, info

    def step_builtin(self) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """A step flown by the built-in controller towards the current waypoint (the baseline)."""
        self._builtin_toward(self.points[self.current], 2.0)
        return self._advance(np.zeros(self.action_size))


# The sample AMR (amr_diffdrive.urdf): wheel radius and half the track, m.
AMR_WHEEL_RADIUS = 0.10
AMR_HALF_TRACK = 0.23
AMR_WHEELS = ("left_wheel_joint", "right_wheel_joint")
# The sample warehouse's free aisle (warehouse_small.urdf: racks at y = ±3.4 m, 0.8 m deep and
# 12 m long, a pallet at x = 5.6 m): goals within x ±4.5 m and y ±2.2 m keep 0.4 m clear of them.
AISLE_X = 4.5
AISLE_Y = 2.2


class AmrGoToGoal(_Task):
    """Drive the sample AMR to a goal in the sample warehouse.

    Action (2, each -1..1): forward speed a0 * max_speed (0.6 m/s) and turn rate a1 *
    max_turn (1.5 rad/s), turned into the two wheels' speeds (radius 0.10 m, track 0.46 m) and
    sent by joint control.

    Reset: from the start at the origin facing +x, a goal 1 to `max_distance` (4) m away in the
    warehouse's free aisle (x ±4.5 m, y ±2.2 m), with a random heading.

    Observation (9, or 9 + `lidar_sectors` with lidar=True): the goal in the robot's frame (m,
    forward and left, 2), its distance (m), the cosine and sine of its bearing (2: the direction
    stays clear when the goal is close), the cosine and sine of the goal heading less the
    robot's (2), the robot's forward speed (m/s) and turn rate (rad/s), and with lidar the
    nearest range of front_lidar in each of `lidar_sectors` (16) equal sectors over its 360°,
    over 5 m and at most 1.

    Reward per step (0.1 s), about -0.4..+0.4 and the bonus:
      + 5 (d_before - d_after)   progress towards the goal (m): 0.3 a step at 0.6 m/s
      - 0.01                     each step: no dawdling
      + 10                       and the episode ends (success) within `reach` (0.3 m)
      CRASH_PENALTY (-10) and the end when its body touches anything but the floor (a wall, a
      rack: body/base_link/contact_force over 1 N).

    Truncated after episode_seconds (20 s, 200 steps). The goal's heading is observed but not
    required: with heading_tolerance (rad) success also needs it.
    """

    action_size = 2

    def __init__(
        self,
        *,
        lidar: bool = False,
        lidar_sectors: int = 16,
        max_speed: float = 0.6,
        max_turn: float = 1.5,
        max_distance: float = 4.0,
        reach: float = 0.3,
        heading_tolerance: float | None = None,
        step_seconds: float = 0.1,
        episode_seconds: float = 20.0,
        randomize: dict[str, tuple[float, float]] | None = None,
    ) -> None:
        self.lidar = lidar
        self.lidar_sectors = lidar_sectors
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.max_distance = max_distance
        self.reach = reach
        self.heading_tolerance = heading_tolerance
        self.goal = np.zeros(3)  # x, y (m), heading (rad)
        self._distance = 0.0
        super().__init__(
            sample_robot(),
            sample_world(),
            joints=AMR_WHEELS,
            mode="velocity",
            step_seconds=step_seconds,
            episode_seconds=episode_seconds,
            randomize=randomize,
        )
        self.sim.step_until(0.5)  # settled on its wheels
        self._save_first_state()
        self.start = self._pose()[0].copy()
        self.observation_size = 9 + (lidar_sectors if lidar else 0)
        self._set_spaces()

    def _pose(self) -> tuple[np.ndarray, float, float, float]:
        """Position (m), heading (rad), forward speed (m/s) and turn rate (rad/s)."""
        values = self.sim.read_many(["body/base_link/pose", "body/base_link/twist"])
        w, x, y, z = values[3:7]
        heading = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        forward = math.cos(heading) * values[7] + math.sin(heading) * values[8]
        return values[0:3], heading, forward, float(values[12])

    def _draw(self, options: dict[str, Any] | None) -> None:
        start = self._pose()[0]
        for _attempt in range(1000):
            goal = self._rng.uniform([-AISLE_X, -AISLE_Y], [AISLE_X, AISLE_Y])
            if 1.0 <= np.linalg.norm(goal - start[:2]) <= self.max_distance:
                break
        self.goal = np.array([goal[0], goal[1], self._rng.uniform(-math.pi, math.pi)])
        if options and "goal" in options:
            self.goal = np.asarray(options["goal"], dtype=np.float64)
        self._distance = float(np.linalg.norm(self.goal[:2] - start[:2]))

    def _act(self, action: np.ndarray) -> None:
        speed = float(action[0]) * self.max_speed
        turn = float(action[1]) * self.max_turn
        left = (speed - turn * AMR_HALF_TRACK) / AMR_WHEEL_RADIUS
        right = (speed + turn * AMR_HALF_TRACK) / AMR_WHEEL_RADIUS
        self.sim.command_joints(list(AMR_WHEELS), "velocity", [left, right])

    def _observe(self) -> np.ndarray:
        position, heading, forward, turn = self._pose()
        dx, dy = self.goal[0] - position[0], self.goal[1] - position[1]
        c, s = math.cos(heading), math.sin(heading)
        off = self.goal[2] - heading
        ahead, left = c * dx + s * dy, -s * dx + c * dy
        bearing = math.atan2(left, ahead)
        parts = [
            np.array(
                [
                    ahead,
                    left,
                    math.hypot(dx, dy),
                    math.cos(bearing),
                    math.sin(bearing),
                    math.cos(off),
                    math.sin(off),
                    forward,
                    turn,
                ]
            )
        ]
        if self.lidar:
            parts.append(self._scan())
        return np.concatenate(parts)

    def _scan(self) -> np.ndarray:
        ranges = np.asarray(self.sim.read("sensor/front_lidar/ranges"), dtype=np.float64)
        ranges = np.where(np.isfinite(ranges) & (ranges > 0.0), ranges, 5.0)
        sectors = np.array_split(ranges, self.lidar_sectors)
        return np.minimum(np.array([sector.min() for sector in sectors]), 5.0) / 5.0

    def _judge(self, action: np.ndarray) -> tuple[float, bool, dict[str, Any]]:
        position, heading, _forward, _turn = self._pose()
        distance = float(np.linalg.norm(self.goal[:2] - position[:2]))
        info: dict[str, Any] = {"distance": distance}
        touch = float(np.linalg.norm(self.sim.read("body/base_link/contact_force")))
        if touch > 1.0:
            info["collision"] = True
            return CRASH_PENALTY, True, info
        reward = 5.0 * (self._distance - distance) - 0.01
        self._distance = distance
        facing = (
            self.heading_tolerance is None
            or abs(math.remainder(self.goal[2] - heading, 2 * math.pi)) < self.heading_tolerance
        )
        if distance < self.reach and facing:
            info["success"] = True
            return reward + 10.0, True, info
        return reward, False, info

    def step_builtin(self) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """A step of a hand-written go-to-goal law (the baseline; Lagrange has no built-in one):
        turn towards the goal, and drive forward as the heading error allows."""
        position, heading, _forward, _turn = self._pose()
        dx, dy = self.goal[0] - position[0], self.goal[1] - position[1]
        bearing = math.remainder(math.atan2(dy, dx) - heading, 2 * math.pi)
        turn = float(np.clip(2.0 * bearing / self.max_turn, -1.0, 1.0))
        speed = float(np.clip(math.hypot(dx, dy), 0.0, 1.0)) * max(0.0, math.cos(bearing)) ** 2
        action = np.array([speed, turn])
        self._act(action)
        return self._advance(action)


class NumpyPolicy:
    """A trained actor run by numpy alone, deterministic: from an .npz with `layers` (the count),
    the weights `w0`, `b0`, ... of each layer (tanh between them, none after the last), and the
    observation normalisation `obs_mean`, `obs_var`, `clip_obs`, `epsilon`. Its action is the
    last layer's output clipped to -1..1, as Stable-Baselines3's MlpPolicy acts with
    deterministic=True (examples/lagrange_example_rl/scripts/ppo.py writes such a file)."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        data = np.load(path)
        self.mean, self.var = data["obs_mean"], data["obs_var"]
        self.clip, self.epsilon = float(data["clip_obs"]), float(data["epsilon"])
        self.layers = [(data[f"w{i}"], data[f"b{i}"]) for i in range(int(data["layers"]))]

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        x = np.clip(
            (observation - self.mean) / np.sqrt(self.var + self.epsilon), -self.clip, self.clip
        )
        for i, (weight, bias) in enumerate(self.layers):
            x = weight @ x + bias
            if i < len(self.layers) - 1:
                x = np.tanh(x)
        result: np.ndarray = np.clip(x, -1.0, 1.0)
        return result


TASKS: dict[str, type[_Task]] = {
    "QuadHover": QuadHover,
    "QuadWaypoint": QuadWaypoint,
    "AmrGoToGoal": AmrGoToGoal,
}


def make(task: str, **options: Any) -> _Task:
    """A task by name ("QuadHover", "QuadWaypoint", "AmrGoToGoal") with its options; at a
    module's top level, so ParallelEnvs and VectorEnv can build it in their workers:
    VectorEnv(functools.partial(make, "QuadHover"), 8)."""
    if task not in TASKS:
        raise ValueError(f"no task {task}: there are {', '.join(TASKS)}")
    return TASKS[task](**options)


def register() -> None:
    """Registers the tasks with Gymnasium as Lagrange/QuadHover-v0, ... (gymnasium.make)."""
    import gymnasium

    for name, cls in TASKS.items():
        identifier = f"Lagrange/{name}-v0"
        if identifier not in gymnasium.registry:
            gymnasium.register(identifier, entry_point=cls)

#!/usr/bin/env python3
"""Trains a policy for a task of lagrange.tasks by PPO (Stable-Baselines3, torch on the CPU).

    ppo.py quad_hover [--workers 8] [--steps 1000000] [--out DIR]
    ppo.py amr_goal --steps 300000
    ppo.py quad_hover --evaluate POLICY.npz       # only the 20 seeded episodes

The environments run in worker processes (lagrange.envs.ParallelEnvs, one simulation each),
behind Stable-Baselines3's VecEnv interface. When it has trained it writes to OUT (default
$XDG_STATE_HOME/lagrange/rl/<task>): the reward curve (curve.csv, curve.png with matplotlib),
the policy as plain arrays (policy.npz: the actor's layers and the observation normalisation,
which lagrange.tasks.NumpyPolicy runs with numpy alone) and the SB3 model (model.zip). Then it
flies 20 seeded episodes with the learned policy, a random one and the built-in controller (the
quadcopter's own, or a hand-written go-to-goal law for the AMR) and prints their success rates
in its RESULT line.

Stable-Baselines3 (MIT) and torch (BSD-3-Clause) are not dependencies of Lagrange: without them
this prints a RESULT line that says so and ends without an error.
"""

from __future__ import annotations

import argparse
import functools
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from lagrange.envs import ParallelEnvs
from lagrange.tasks import NumpyPolicy, make

TASKS = {"quad_hover": "QuadHover", "quad_waypoint": "QuadWaypoint", "amr_goal": "AmrGoToGoal"}
# What each task trains with: steps per environment per update, discount, and the task's options.
SETTINGS: dict[str, dict[str, Any]] = {
    "quad_hover": {"n_steps": 256, "gamma": 0.99, "options": {}},
    "quad_waypoint": {"n_steps": 256, "gamma": 0.99, "options": {}},
    "amr_goal": {"n_steps": 256, "gamma": 0.99, "options": {"lidar": True}},
}
EVALUATION_SEED = 10_000  # episode i of the evaluation is reset with this + i


def export(model: Any, normalize: Any, path: Path) -> None:
    """The actor's weights and the observation statistics as policy.npz."""
    linear = [m for m in model.policy.mlp_extractor.policy_net if hasattr(m, "weight")]
    linear.append(model.policy.action_net)
    arrays = {"layers": np.array(len(linear))}
    for i, layer in enumerate(linear):
        arrays[f"w{i}"] = layer.weight.detach().cpu().numpy().astype(np.float32)
        arrays[f"b{i}"] = layer.bias.detach().cpu().numpy().astype(np.float32)
    arrays["obs_mean"] = normalize.obs_rms.mean
    arrays["obs_var"] = normalize.obs_rms.var
    arrays["clip_obs"] = np.array(normalize.clip_obs)
    arrays["epsilon"] = np.array(normalize.epsilon)
    np.savez_compressed(path, **arrays)


def vec_env_class() -> type:
    """Stable-Baselines3's VecEnv over lagrange.envs.ParallelEnvs."""
    from stable_baselines3.common.vec_env import VecEnv

    class LagrangeVecEnv(VecEnv):
        def __init__(self, make_one: Any, count: int) -> None:
            self.envs = ParallelEnvs(make_one, count)
            super().__init__(count, self.envs.observation_space, self.envs.action_space)
            self._actions = np.zeros((count, *self.envs.action_space.shape))
            self._next_seed = 0

        def reset(self) -> np.ndarray:
            seeds = [s if s is not None else self._next_seed + i for i, s in enumerate(self._seeds)]
            self._next_seed += self.num_envs
            self._reset_seeds()
            return self.envs.reset(seed=seeds).astype(np.float32)

        def step_async(self, actions: np.ndarray) -> None:
            self.envs.step_async(actions)

        def step_wait(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[dict[str, Any]]]:
            observations, rewards, ended = self.envs.step_wait()
            infos: list[dict[str, Any]] = [{} for _ in range(self.num_envs)]
            for i in np.flatnonzero(ended):
                infos[i]["terminal_observation"] = self.envs.final_observations[i].astype(
                    np.float32
                )
                infos[i]["TimeLimit.truncated"] = bool(
                    self.envs.truncated[i] and not self.envs.terminated[i]
                )
            return observations.astype(np.float32), rewards.astype(np.float32), ended, infos

        def close(self) -> None:
            self.envs.close()

        def get_attr(self, name: str, indices: Any = None) -> list[Any]:
            return [getattr(self, name, None)] * len(self._get_indices(indices))

        def set_attr(self, name: str, value: Any, indices: Any = None) -> None:
            raise NotImplementedError("the environments live in worker processes")

        def env_method(self, name: str, *args: Any, indices: Any = None, **kwargs: Any) -> list:
            raise NotImplementedError("the environments live in worker processes")

        def env_is_wrapped(self, wrapper_class: type, indices: Any = None) -> list[bool]:
            return [False] * len(self._get_indices(indices))

    return LagrangeVecEnv


def evaluate(task: str, options: dict[str, Any], policy: Any, episodes: int) -> dict[str, Any]:
    """Success rate and mean return over `episodes` seeded episodes of each of: the policy (None:
    left out), a random policy, and the built-in controller."""
    env = make(TASKS[task], **options)
    results: dict[str, Any] = {}
    rng = np.random.default_rng(EVALUATION_SEED)
    actors = {
        "learned": (lambda o: env.step(policy(o))) if policy is not None else None,
        "random": lambda _o: env.step(rng.uniform(-1.0, 1.0, env.action_size)),
        "builtin": lambda _o: env.step_builtin(),
    }
    for name, act in actors.items():
        if act is None:
            continue
        successes, returns, lengths = 0, [], []
        for i in range(episodes):
            observation, _info = env.reset(seed=EVALUATION_SEED + i)
            total, steps = 0.0, 0
            while True:
                observation, reward, terminated, truncated, info = act(observation)
                total += reward
                steps += 1
                if terminated or truncated:
                    break
            successes += bool(info["success"])
            returns.append(total)
            lengths.append(steps)
        results[name] = {
            "success": successes,
            "episodes": episodes,
            "mean_return": float(np.mean(returns)),
            "mean_steps": float(np.mean(lengths)),
        }
    close = getattr(env, "close", None)
    if close is not None:
        close()
    return results


def plot(curve: list[tuple[int, float, float]], title: str, path: Path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    steps = [c[0] for c in curve]
    returns = [c[1] for c in curve]
    figure, axes = plt.subplots(figsize=(6.4, 3.6), dpi=100)
    axes.plot(steps, returns, color="#2a6fdb", linewidth=1.6)
    axes.set_xlabel("environment steps")
    axes.set_ylabel("mean episode return")
    axes.set_title(title)
    axes.grid(alpha=0.3)
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)
    return True


def summary(results: dict[str, Any]) -> str:
    names = {"learned": "PPO", "random": "random", "builtin": "built-in"}
    return ", ".join(
        f"{names[k]} {v['success']}/{v['episodes']} (return {v['mean_return']:.1f})"
        for k, v in results.items()
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("task", choices=sorted(TASKS), help="what to train")
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--steps", type=int, default=1_000_000, help="environment steps")
    parser.add_argument("--episodes", type=int, default=20, help="evaluation episodes")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--threads", type=int, default=2, help="torch threads")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--evaluate", type=Path, default=None, help="a policy.npz: no training")
    args = parser.parse_args()
    settings = SETTINGS[args.task]
    options = settings["options"]
    state = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
    out = args.out or state / "lagrange" / "rl" / args.task
    out.mkdir(parents=True, exist_ok=True)

    if args.evaluate is not None:
        results = evaluate(args.task, options, NumpyPolicy(args.evaluate), args.episodes)
        (out / "evaluation.json").write_text(json.dumps(results, indent=2))
        print(
            f"RESULT: {TASKS[args.task]} success over {args.episodes} seeded episodes: "
            f"{summary(results)}",
            flush=True,
        )
        return 0

    try:
        import stable_baselines3
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback
        from stable_baselines3.common.vec_env import VecMonitor, VecNormalize
    except ImportError as error:
        print(
            f"RESULT: skipped, {error.name} is not installed: pip install stable-baselines3 "
            "torch (in a venv) to train",
            flush=True,
        )
        return 0
    torch.set_num_threads(args.threads)
    print(
        f"Stable-Baselines3 {stable_baselines3.__version__}, torch {torch.__version__}, "
        f"{args.workers} workers",
        flush=True,
    )

    curve: list[tuple[int, float, float]] = []
    started = time.monotonic()

    class Curve(BaseCallback):
        """The mean return of the last episodes after each rollout."""

        def _on_rollout_end(self) -> None:
            if self.model.ep_info_buffer:
                mean = float(np.mean([e["r"] for e in self.model.ep_info_buffer]))
                curve.append((self.num_timesteps, mean, time.monotonic() - started))
                print(
                    f"{self.num_timesteps:>9} steps  return {mean:8.2f}  "
                    f"{time.monotonic() - started:6.0f} s",
                    flush=True,
                )

        def _on_step(self) -> bool:
            return True

    venv = vec_env_class()(functools.partial(make, TASKS[args.task], **options), args.workers)
    env = VecNormalize(VecMonitor(venv), norm_obs=True, norm_reward=True, gamma=settings["gamma"])
    try:
        model = PPO(
            "MlpPolicy",
            env,
            n_steps=settings["n_steps"],
            batch_size=512,
            n_epochs=10,
            gamma=settings["gamma"],
            gae_lambda=0.95,
            learning_rate=3e-4,
            clip_range=0.2,
            policy_kwargs={"net_arch": [64, 64], "log_std_init": -1.0},
            seed=args.seed,
            device="cpu",
            verbose=0,
        )
        model.learn(total_timesteps=args.steps, callback=Curve())
    finally:
        env.close()
    wall = time.monotonic() - started
    model.save(out / "model.zip")
    export(model, env, out / "policy.npz")
    with (out / "curve.csv").open("w") as f:
        f.write("steps,mean_return,wall_seconds\n")
        f.writelines(f"{s},{r:.4f},{w:.1f}\n" for s, r, w in curve)
    plotted = plot(curve, f"{TASKS[args.task]}: PPO, {args.workers} workers", out / "curve.png")

    results = evaluate(args.task, options, NumpyPolicy(out / "policy.npz"), args.episodes)
    report = {
        "task": TASKS[args.task],
        "steps": int(model.num_timesteps),
        "wall_seconds": round(wall, 1),
        "workers": args.workers,
        "stable_baselines3": stable_baselines3.__version__,
        "torch": torch.__version__,
        "evaluation": results,
    }
    (out / "report.json").write_text(json.dumps(report, indent=2))
    print(
        f"RESULT: {TASKS[args.task]} PPO {model.num_timesteps} steps in {wall:.0f} s with "
        f"{args.workers} workers; success over {args.episodes} seeded episodes: "
        f"{summary(results)}; {out / ('curve.png' if plotted else 'curve.csv')}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

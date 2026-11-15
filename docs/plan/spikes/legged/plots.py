#!/usr/bin/env python3
# ruff: noqa: N812, RUF001  (physics notation in text; models imported as M)
"""Plots for docs/plan/spikes/legged-contact.md from contact_tests.py's and rl_speed.py's output.

plots.py RESULTS_DIR OUT_DIR       (RESULTS_DIR: contact_tests.py's OUT_DIR, with rl_speed.json)
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import contact_tests as C

import models as M

COLORS = {"mujoco": "#2a6fdb", "jolt": "#d9822b"}
STYLES = {0.0005: ":", 0.001: "-", 0.002: "--"}


def stance(res: Path, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 3.4))
    mg = M.QUAD_M * M.G
    for be in C.BACKENDS:
        for dt in C.STEPS:
            d = np.load(res / f"stance_{be}_{dt * 1e3:g}ms.npz")
            ax.plot(
                d["t"],
                100 * (d["total"] - mg) / mg,
                STYLES[dt],
                color=COLORS[be],
                lw=1,
                label=f"{be} {dt * 1e3:g} ms",
            )
    ax.set_xlabel("time (s)")
    ax.set_ylabel("Σ contact force − m·g (% of m·g)")
    ax.set_title("Standing quadruped (kp 100): sum of contact forces against its weight")
    ax.set_ylim(-3, 3)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(out / "stance_force.png", dpi=110)
    plt.close(fig)


def drop(res: Path, out: Path) -> None:
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8, 3.2))
    for be in C.BACKENDS:
        for dt in C.STEPS:
            d = np.load(res / f"drop_{be}_{dt * 1e3:g}ms.npz")
            t0 = d["t"][np.nonzero(d["f"] > 1e-3 * M.BALL_M * M.G)[0][0]]
            m = (d["t"] > t0 - 0.01) & (d["t"] < t0 + 0.08)
            a1.plot(
                1e3 * (d["t"][m] - t0),
                1e3 * (M.BALL_R - d["z"][m]),
                STYLES[dt],
                color=COLORS[be],
                lw=1,
                label=f"{be} {dt * 1e3:g} ms",
            )
            a2.plot(1e3 * (d["t"][m] - t0), d["f"][m], STYLES[dt], color=COLORS[be], lw=1)
    a1.set_xlabel("time from first contact (ms)")
    a1.set_ylabel("penetration (mm)")
    a2.set_xlabel("time from first contact (ms)")
    a2.set_ylabel("contact force (N)")
    a2.set_yscale("symlog", linthresh=10)
    a1.legend(fontsize=7)
    for a in (a1, a2):
        a.grid(alpha=0.3)
    fig.suptitle("1 kg ball dropped from 0.5 m (2.97 m/s at impact)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "drop.png", dpi=110)
    plt.close(fig)


def creep(res: Path, out: Path) -> None:
    rigid = json.loads((res / "rigid.json").read_text())
    slope = json.loads((res / "slope.json").read_text())
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    for be in C.BACKENDS:
        xs, ys = [], []
        for deg in M.RIGID_SLOPES:
            v = rigid.get(f"{deg:g} deg {be} 1 ms")
            if v and v["holds_closed_form"]:
                xs.append(deg)
                ys.append(v["creep_rate_3_6s_mm_s"])
        ax.plot(xs, ys, "o-", color=COLORS[be], label=f"{be}: rigid quadruped (μ 0.5)")
        xs, ys = [], []
        for deg in M.QUAD_SLOPES:
            v = slope.get(f"{deg:g} deg {be} 1 ms")
            if v:
                xs.append(deg)
                ys.append(v["feet_creep_rate_3_6s_mm_s"])
        ax.plot(xs, ys, "s--", color=COLORS[be], label=f"{be}: legged, impedance-held (μ 0.6)")
    ax.set_xlabel("slope (°), all below atan μ: should hold")
    ax.set_ylabel("creep 3–6 s (mm/s)")
    ax.set_title("Creep on slopes that should hold (1 ms; 0.5 and 2 ms agree)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "creep.png", dpi=110)
    plt.close(fig)


def rimless(res: Path, out: Path) -> None:
    cf = C.rimless_closed_form()
    fig, ax = plt.subplots(figsize=(7, 3.2))
    for be in C.BACKENDS:
        d = np.load(res / f"rimless_{be}_1ms.npz")
        ax.plot(d["t"], d["w"], color=COLORS[be], lw=0.8, label=f"{be} 1 ms")
    ax.axhline(
        cf["omega_before_rad_s"], color="k", lw=0.8, ls="--", label="closed form: before impact"
    )
    ax.axhline(cf["omega_after_rad_s"], color="k", lw=0.8, ls=":", label="after impact")
    ax.set_xlim(4, 7)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("hub angular velocity (rad/s)")
    ax.set_title(f"Rimless wheel, {M.RIM_N} spokes, {M.RIM_SLOPE_DEG:g}° slope (steady gait)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="lower right")
    fig.tight_layout()
    fig.savefig(out / "rimless.png", dpi=110)
    plt.close(fig)


def rl(res: Path, out: Path) -> None:
    data = json.loads((res / "rl_speed.json").read_text())
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    for name, color in (
        ("quadruped (example URDF)", "#2a6fdb"),
        ("hexapod (spike MJCF)", "#7a3fb0"),
    ):
        rows = data[name]
        lock = [r for r in rows if not r.get("independent")]
        free = [r for r in rows if r.get("independent")]
        ax.plot(
            [r["processes"] for r in lock],
            [r["policy_steps_per_s"] for r in lock],
            "o-",
            color=color,
            label=f"{name}: ParallelEnvs",
        )
        ax.plot(
            [r["processes"] for r in free],
            [r["policy_steps_per_s"] for r in free],
            "s--",
            color=color,
            label=f"{name}: independent processes",
        )
    ax.set_xlabel("processes")
    ax.set_ylabel("20 ms policy steps/s, all processes")
    ax.set_title(f"lagrange.envs on {data.get('cpu', '')}, shared machine", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "rl_speed.png", dpi=110)
    plt.close(fig)


def main() -> int:
    res, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    for f in (stance, drop, creep, rimless, rl):
        f(res, out)
    print("plots in", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

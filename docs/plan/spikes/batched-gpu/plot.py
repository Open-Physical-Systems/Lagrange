"""Plot of the RFC 0025 spike: OUT.json from mjw_spike.py to a PNG."""

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main() -> None:
    with Path(sys.argv[1]).open() as f:
        r = json.load(f)
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for name, v in r["models"].items():
        ns = [g["n"] for g in v["gpu"]]
        ax[0].loglog(
            ns, [g["env_steps_per_s"] for g in v["gpu"]], "o-", label=f"{name} MuJoCo Warp"
        )
        ax[0].axhline(
            v["cpu_mujoco_steps_per_s"],
            ls="--",
            lw=1,
            color=ax[0].lines[-1].get_color(),
            label=f"{name} CPU MuJoCo, 1 core",
        )
        ax[1].semilogx(ns, [g["rtf_per_env"] for g in v["gpu"]], "o-", label=name)
    ax[0].set(
        xlabel="environments N",
        ylabel="environment steps per second (1 ms steps)",
        title="Throughput",
    )
    ax[1].set(
        xlabel="environments N",
        ylabel="real-time factor of each environment",
        title="Speed of each environment",
    )
    for a in ax:
        a.grid(True, which="both", alpha=0.3)
        a.legend(fontsize=7)
    fig.suptitle(
        f"RFC 0025 spike: MuJoCo Warp {r['mujoco_warp']}, Warp {r['warp']}, {r['device']}"
        " (physics only, CUDA graph)",
        fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=110)


if __name__ == "__main__":
    main()

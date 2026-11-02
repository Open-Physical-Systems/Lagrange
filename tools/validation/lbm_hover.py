#!/usr/bin/env python3
"""The live lattice Boltzmann box against OpenFOAM and Bauersfeld et al. (2024).

Runs `lagrange_lbm_hover` (modules/aerodynamics) with the rotors of a `lagrange foam-hover` case
at each cell size, then compares, for each run: the speed down through the discs, the speed under
the drone's centre against the OpenFOAM case and Bauersfeld's jet (s >= 2.5 l), and the peak
outwash 3 cm above the ground. Writes `lbm_hover.json` and `lbm_hover.png` in OUT.

    python3 tools/validation/lbm_hover.py ~/cfd/hover_open OUT [--cells 0.08,0.05,0.03]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np

RHO = 1.225  # kg/m³, the foam-hover cases'
# Bauersfeld, Muller, Ziegler, Coletti and Scaramuzza (2024), arXiv:2403.13321, eqs. 5, 6, 9.
JET_DECAY, JET_ORIGIN = 10.11, -5.817
HALF = 2.0  # m, the box each side of the drone
ABOVE = 1.0  # m, the box over the drone


def read_internal(path: Path) -> np.ndarray:
    text = path.read_text()
    match = re.compile(r"internalField\s+nonuniform List<\w+>\s*(\d+)\s*\(").search(text)
    assert match, path
    count = int(match.group(1))
    end = text.index(";", match.end())
    numbers = np.array(
        text[match.end() : end].replace("(", " ").replace(")", " ").split(), dtype=float
    )
    return numbers[: 3 * count].reshape(count, 3)


def latest_time(case: Path) -> Path:
    times = [p for p in case.iterdir() if p.is_dir() and re.fullmatch(r"[0-9.]+", p.name)]
    return max(times, key=lambda p: float(p.name))


class Box:
    """A run's mean flow, sampled trilinearly in the case's frame (drone's centre at x = y = 0)."""

    def __init__(self, prefix: Path) -> None:
        self.meta = json.loads(Path(f"{prefix}.json").read_text())
        m = self.meta
        raw = np.fromfile(Path(f"{prefix}.f32"), dtype=np.float32)
        self.u = raw.reshape(m["nz"], m["ny"], m["nx"], 3)
        self.cell = m["cell"]

    def __call__(self, points: np.ndarray) -> np.ndarray:
        n = np.array([self.meta["nx"], self.meta["ny"], self.meta["nz"]])
        g = (points + np.array([HALF, HALF, 0.0])) / self.cell - 0.5
        g = np.clip(g, 0.0, n - 1.000001)
        lo = g.astype(int)
        t = g - lo
        out = np.zeros((len(points), 3))
        for corner in range(8):
            up = np.array([(corner >> k) & 1 for k in range(3)])
            at = np.minimum(lo + up, n - 1)
            w = np.prod(np.where(up, t, 1.0 - t), axis=1)
            out += w[:, None] * self.u[at[:, 2], at[:, 1], at[:, 0]]
        return out


def through_discs(sample, discs, thickness: float) -> list[float]:
    """Mean speed down through each disc: points over the disc, as foam-hover averages its cells."""
    out = []
    for d in discs:
        r = np.linspace(0.0, d["radius"], 8)[1:]
        a = np.linspace(0.0, 2 * math.pi, 24, endpoint=False)
        rr, aa = np.meshgrid(r, a)
        pts = np.column_stack(
            [
                d["hub"][0] + (rr * np.cos(aa)).ravel(),
                d["hub"][1] + (rr * np.sin(aa)).ravel(),
                np.full(rr.size, d["hub"][2]),
            ]
        )
        weights = rr.ravel()  # area weighting
        u = sample(pts)
        out.append(float(-(u[:, 2] * weights).sum() / weights.sum()))
    del thickness
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("case", type=Path, help="a lagrange foam-hover case directory")
    parser.add_argument("out", type=Path)
    parser.add_argument("--cells", default="0.08,0.05,0.03", help="cell sizes, m")
    parser.add_argument("--seconds", type=float, default=3.0, help="air time per run")
    parser.add_argument("--average", type=float, default=1.0, help="mean over the last S")
    parser.add_argument("--bench", default=None, help="lagrange_lbm_hover (default: on PATH)")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    plan = json.loads((args.case / "plan.json").read_text())
    h = plan["height"]
    discs = plan["discs"]
    radius = discs[0]["radius"]
    hover_induced = math.sqrt(discs[0]["thrust"] / (2 * RHO * math.pi * radius * radius))
    l_motor = plan["motor_distance"]
    bench = args.bench or shutil.which("lagrange_lbm_hover")
    if not bench:
        prefix = Path(__file__).resolve().parents[2] / "install/lagrange_aerodynamics/lib"
        bench = str(prefix / "lagrange_aerodynamics/lagrange_lbm_hover")
    rotors = args.out / "rotors.txt"
    rotors.write_text(
        "".join(
            f"{d['hub'][0] + HALF} {d['hub'][1] + HALF} {d['hub'][2]} "
            f"{d['axis'][0]} {d['axis'][1]} {d['axis'][2]} {d['radius']} {d['thrust']}\n"
            for d in discs
        )
    )

    time = latest_time(args.case)
    from scipy.spatial import cKDTree

    centres = read_internal(time / "C")
    velocity = read_internal(time / "U")
    tree = cKDTree(centres)

    def foam(points: np.ndarray) -> np.ndarray:
        _, index = tree.query(points)
        return velocity[index]

    depth = np.linspace(0.04, h - 0.02, 120)
    line = np.column_stack([np.zeros_like(depth), np.zeros_like(depth), depth])
    s = h - depth
    jet = np.where(
        s / l_motor >= 2.5, hover_induced * JET_DECAY / (s / l_motor - JET_ORIGIN), np.nan
    )
    span = np.linspace(-HALF + 0.05, HALF - 0.05, 161)
    gx, gy = np.meshgrid(span, span)
    ground = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, 0.03)])

    def summary(sample) -> dict:
        centre = -sample(line)[:, 2]
        g = sample(ground)
        far = ~np.isnan(jet)
        return {
            "down_through_discs": [round(v, 3) for v in through_discs(sample, discs, 0.0)],
            "centreline": centre,
            "peak_outwash_3cm": float(np.linalg.norm(g[:, :2], axis=1).max()),
            "jet_error_vs_bauersfeld": (
                float(np.nanmean(np.abs(centre[far] - jet[far]) / jet[far])) if far.any() else None
            ),
        }

    reference = summary(foam)
    runs = []
    for cell in [float(c) for c in args.cells.split(",")]:
        prefix = args.out / f"box_{cell:.3f}"
        command = [
            bench,
            str(rotors),
            str(prefix),
            "--box",
            f"{2 * HALF},{2 * HALF},{h + ABOVE}",
            "--cell",
            str(cell),
            "--floor",
            "--seconds",
            str(args.seconds),
            "--average",
            str(args.average),
        ]
        print(" ".join(command), flush=True)
        subprocess.run(command, check=True)
        box = Box(prefix)
        result = summary(box)
        result["meta"] = box.meta
        result["cell"] = cell
        result["centreline_error_vs_openfoam"] = float(
            np.mean(np.abs(result["centreline"] - reference["centreline"]))
            / np.mean(np.abs(reference["centreline"]))
        )
        runs.append(result)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), dpi=110)
    ax = axes[0]
    ax.plot(reference["centreline"], depth, "k", lw=2, label="OpenFOAM (2 cm cells)")
    ax.plot(jet, depth, "k--", label="Bauersfeld et al. 2024 (s ≥ 2.5 l)")
    for r in runs:
        rt = r["meta"]["real_time_factor"]
        ax.plot(
            r["centreline"],
            depth,
            label=f"live box, {r['cell'] * 100:.0f} cm ({rt:.2f}x real time)",
        )
    ax.axhline(h, color="#ff8a3d", lw=1)
    ax.set_xlabel("downward speed under the centre (m/s)")
    ax.set_ylabel("height (m)")
    ax.set_title(f"under the drone, hovering {h:g} m up")
    ax.legend(fontsize=8)
    ax = axes[1]
    labels = ["OpenFOAM"] + [f"{r['cell'] * 100:.0f} cm" for r in runs]
    disc = [np.mean(reference["down_through_discs"])] + [
        np.mean(r["down_through_discs"]) for r in runs
    ]
    out = [reference["peak_outwash_3cm"]] + [r["peak_outwash_3cm"] for r in runs]
    x = np.arange(len(labels))
    ax.bar(x - 0.2, disc, 0.4, label="down through the discs")
    ax.bar(x + 0.2, out, 0.4, label="peak outwash 3 cm up")
    ax.set_xticks(x, labels)
    ax.set_ylabel("m/s")
    ax.legend(fontsize=8)
    ax.set_title("discs and outwash")
    fig.tight_layout()
    fig.savefig(args.out / "lbm_hover.png")

    def clean(r: dict) -> dict:
        return {k: v for k, v in r.items() if k != "centreline"}

    report = {
        "case": str(args.case),
        "height": h,
        "hover_induced": hover_induced,
        "openfoam": clean(reference),
        "runs": [clean(r) for r in runs],
    }
    (args.out / "lbm_hover.json").write_text(json.dumps(report, indent=2))
    print(
        f"OpenFOAM: discs {np.mean(reference['down_through_discs']):.2f} m/s, "
        f"outwash {reference['peak_outwash_3cm']:.2f} m/s, "
        f"jet vs Bauersfeld {reference['jet_error_vs_bauersfeld'] or 0:.0%}"
    )
    for r in runs:
        print(
            f"{r['cell'] * 100:.0f} cm: {r['meta']['real_time_factor']:.2f}x real time, "
            f"discs {np.mean(r['down_through_discs']):.2f} m/s, outwash "
            f"{r['peak_outwash_3cm']:.2f} m/s, centreline vs OpenFOAM "
            f"{r['centreline_error_vs_openfoam']:.0%}, jet vs Bauersfeld "(
                f"{r['jet_error_vs_bauersfeld']:.0%}"
                if r["jet_error_vs_bauersfeld"] is not None
                else "n/a"
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Fits the equivalent circuit's r0 and two RC pairs to the 25 degC UDDS drive cycle.

The pulse fit (fit_ecm.py) sees 10 s pulses only, so it barely constrains the slow RC pair; under
the sustained load of a drive cycle that pair sets a steady offset. This fit keeps the open-
circuit voltage curve and the capacity of ecm_fit.txt (from the C/20 test) and fits the five
resistances and capacitances by least squares to the UDDS cycle (udds_25degC.csv, made by
make_drive_cycles.py) above 20 % state of charge. US06 and LA92 are left for validation. Writes
ecm_fit_drive.txt. Needs NumPy and SciPy, used only here.
"""

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares  # type: ignore[import-untyped]

HERE = Path(__file__).resolve().parent
SOC_LOW = 0.2

Array = NDArray[np.float64]


def read_fit() -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for line in (HERE / "ecm_fit.txt").read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        key, *values = line.split()
        out[key] = [float(v) for v in values]
    return out


def read_cycle(name: str) -> tuple[Array, Array, Array]:
    data = np.loadtxt(HERE / name, delimiter=",", skiprows=1)
    return data[:, 1], data[:, 2], data[:, 3]  # mean current, current at the end, voltage


def simulate(
    params: Array, fit: dict[str, list[float]], mean: Array, now: Array
) -> tuple[Array, Array]:
    r0, r1, c1, r2, c2 = params
    capacity = fit["capacity_Ah"][0] * 3600.0
    soc_grid = np.array(fit["ocv_soc"])
    ocv_grid = np.array(fit["ocv_V"])
    a1 = np.exp(-1.0 / (r1 * c1))
    a2 = np.exp(-1.0 / (r2 * c2))
    soc = 1.0 - np.concatenate([[0.0], np.cumsum(mean[1:])]) / capacity
    v1 = np.zeros_like(mean)
    v2 = np.zeros_like(mean)
    for k in range(1, len(mean)):
        v1[k] = v1[k - 1] * a1 + r1 * mean[k] * (1.0 - a1)
        v2[k] = v2[k - 1] * a2 + r2 * mean[k] * (1.0 - a2)
    return np.interp(soc, soc_grid, ocv_grid) - v1 - v2 - r0 * now, soc


def main() -> None:
    fit = read_fit()
    mean, now, voltage = read_cycle("udds_25degC.csv")
    start = np.array([fit[k][0] for k in ("r0_ohm", "r1_ohm", "c1_F", "r2_ohm", "c2_F")])

    def residual(log_params: Array) -> Array:
        model, soc = simulate(np.exp(log_params), fit, mean, now)
        return (model - voltage)[soc >= SOC_LOW]

    result = least_squares(residual, np.log(start), x_scale=1.0)
    params = np.exp(result.x)
    lines = [
        "# Written by fit_ecm_drive.py from doi:10.17632/b5mj79w5w9.2 (CC BY 4.0), 25 degC: r0 and",
        "# two RC pairs fitted to the UDDS cycle above SoC 0.2; OCV and capacity as ecm_fit.txt.",
    ]
    for name in ("udds_25degC.csv", "us06_25degC.csv", "la92_25degC.csv"):
        m, n, v = read_cycle(name)
        model, soc = simulate(params, fit, m, n)
        error = (model - v)[soc >= 0.35]
        lines.append(f"#   {name}: {1e3 * np.sqrt(np.mean(error**2)):.2f} mV RMS above SoC 0.35")
    for key, value in zip(("r0_ohm", "r1_ohm", "c1_F", "r2_ohm", "c2_F"), params, strict=True):
        lines.append(f"{key} {value:.6g}")
    (HERE / "ecm_fit_drive.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()

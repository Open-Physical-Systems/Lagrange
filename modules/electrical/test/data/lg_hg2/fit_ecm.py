#!/usr/bin/env python3
"""Fits the network's battery model to a published cell dataset (electrical.md §8).

Source: P. Kollmeyer, M. Naguib, M. Skells, "LG 18650HG2 Li-ion Battery Data", Mendeley Data,
V2, 2020, doi:10.17632/b5mj79w5w9.2, CC BY 4.0. Tests at McMaster University on a new 3 Ah LG
HG2 cell. Two of its 25 °C files are used: the C/20 discharge and charge (549_C20DisCh.csv) for
the open-circuit voltage curve, and the HPPC test (549_HPPC.csv) for the resistances.

What it does, and so what was changed in the data:
- the open-circuit voltage is the mean of the C/20 discharge and charge voltages at each state
  of charge (a pseudo-OCV that cancels the C/20 overvoltage and most of the hysteresis);
- the HPPC test's 4-pulse blocks (1, 2, 4 and 6C discharge pulses, charge pulses, rests) are
  cut out; the blocks starting between 20 % and 97 % state of charge are split alternately into
  a fitting and a validation set;
- r0 and two RC pairs, one set for the whole range (BatteryParams has no state-of-charge
  dependence), are fitted by least squares to the terminal voltage of the fitting blocks;
- the validation blocks are written to hppc_25degC_validation.csv (current positive when
  discharging, as the network counts it) and the fit to ecm_fit.txt, both read by
  test_battery_dataset.

Run with the two CSV files downloaded into a directory:
    python3 fit_ecm.py DOWNLOAD_DIR
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares  # type: ignore[import-untyped]

BASE = "https://data.mendeley.com/public-files/datasets/b5mj79w5w9/files"
SOURCES = {
    "549_C20DisCh.csv": (
        f"{BASE}/c2472dec-04f8-4ab5-9bf8-49b616782d53/file_downloaded",
        "52eff1a8d273f353d557af8172139c2f23709499d36130ac24ffbbb820e8f232",
    ),
    "549_HPPC.csv": (
        f"{BASE}/7d99de7c-e149-48d9-a172-ef0f9fc64aed/file_downloaded",
        "70ffb357511dd77e7c6bc14c642a095ddd51d6918b547987247100a3bd2ea29d",
    ),
}
# State-of-charge range the single parameter set is fitted and validated in (see README).
SOC_LOW = 0.2
SOC_HIGH = 0.97
OCV_GRID = np.concatenate([[0.0, 0.01, 0.025], np.arange(0.05, 0.951, 0.05), [0.975, 0.99, 1.0]])

Array = NDArray[np.float64]


def load(path: Path) -> dict[str, NDArray[np.generic]]:
    """The cycler's CSV: time (s), procedure, status, voltage (V), current (A), Ah counter."""
    lines = path.read_text(encoding="latin-1").splitlines()
    header = next(k for k, line in enumerate(lines) if line.startswith("Time Stamp"))
    t, procedure, status, v, i, ah = [], [], [], [], [], []
    for line in lines[header + 2 :]:
        x = line.split(",")
        if len(x) < 12 or not x[3]:
            continue
        hours, minutes, seconds = x[3].split(":")
        t.append(int(hours) * 3600 + int(minutes) * 60 + float(seconds))
        procedure.append(x[7])
        status.append(x[2])
        v.append(float(x[8]))
        i.append(float(x[9]))
        ah.append(float(x[11]))
    return {
        "t": np.array(t),
        "procedure": np.array(procedure),
        "status": np.array(status),
        "v": np.array(v),
        "i": np.array(i),
        "ah": np.array(ah),
    }


def check(directory: Path) -> None:
    for name, (url, digest) in SOURCES.items():
        path = directory / name
        if not path.is_file():
            sys.exit(f"fit_ecm: {path} missing; download it from {url}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            sys.exit(f"fit_ecm: {path} is not the published file (SHA-256 differs)")


def pseudo_ocv(c20: dict[str, NDArray[np.generic]]) -> tuple[float, Array]:
    """Discharge capacity (Ah) and the open-circuit voltage on OCV_GRID (V per cell)."""
    discharge = c20["status"] == "DCH"
    charge = c20["status"] == "CHA"
    ah = c20["ah"].astype(float)
    capacity = float(-ah[discharge].min())
    soc_d = 1.0 + ah[discharge] / capacity
    a = ah[charge]
    soc_c = (a - a.min()) / (a.max() - a.min())

    def on_grid(soc: Array, v: Array) -> Array:
        order = np.argsort(soc)
        return np.asarray(np.interp(OCV_GRID, soc[order], v[order]), dtype=np.float64)

    v = c20["v"].astype(float)
    return capacity, 0.5 * (on_grid(soc_d, v[discharge]) + on_grid(soc_c, v[charge]))


def blocks(hppc: dict[str, NDArray[np.generic]]) -> list[tuple[int, int]]:
    """Index ranges of the 4-pulse blocks."""
    four = (hppc["procedure"] == "HPPC_4pulse").astype(int)
    edges = np.flatnonzero(np.diff(four)) + 1
    starts = ([0] if four[0] else []) + [int(e) for e in edges if four[e]]
    ends = [int(e) for e in edges if not four[e]] + ([len(four)] if four[-1] else [])
    return list(zip(starts, ends, strict=True))


def simulate(p: Array, t: Array, current: Array, soc: Array, ocv: Array) -> Array:
    """The network's battery model (Models.cpp): OCV - r0·i - v1 - v2, each RC pair held over
    the sample interval with its exact solution."""
    r0, r1, tau1, r2, tau2 = p
    v1 = v2 = 0.0
    out = np.empty(len(t))
    for k in range(len(t)):
        if k:
            dt = t[k] - t[k - 1]
            d1, d2 = np.exp(-dt / tau1), np.exp(-dt / tau2)
            v1 = d1 * v1 + (1.0 - d1) * r1 * current[k - 1]
            v2 = d2 * v2 + (1.0 - d2) * r2 * current[k - 1]
        out[k] = np.interp(soc[k], OCV_GRID, ocv) - r0 * current[k] - v1 - v2
    return out


def main() -> None:
    directory = Path(sys.argv[1]) if len(sys.argv) > 1 else Path()
    check(directory)
    capacity, ocv = pseudo_ocv(load(directory / "549_C20DisCh.csv"))
    hppc = load(directory / "549_HPPC.csv")
    t = hppc["t"].astype(float)
    current = -hppc["i"].astype(float)  # positive discharging
    soc = 1.0 + hppc["ah"].astype(float) / capacity
    voltage = hppc["v"].astype(float)
    inside = [b for b in blocks(hppc) if SOC_LOW < soc[b[0]] < SOC_HIGH]
    fitting, validation = inside[0::2], inside[1::2]

    def residual(p: Array) -> Array:
        return np.concatenate(
            [simulate(p, t[s:e], current[s:e], soc[s:e], ocv) - voltage[s:e] for s, e in fitting]
        )

    fit = least_squares(
        residual,
        [0.02, 0.01, 10.0, 0.01, 100.0],
        bounds=([0, 0, 0.1, 0, 1], [0.2, 0.2, 200, 0.2, 5000]),
    )
    r0, r1, tau1, r2, tau2 = (float(x) for x in fit.x)
    report = []
    for s, e in validation:
        error = simulate(fit.x, t[s:e], current[s:e], soc[s:e], ocv) - voltage[s:e]
        report.append(
            {
                "soc": round(float(soc[s]), 4),
                "rms_mV": round(1e3 * float(np.sqrt(np.mean(error**2))), 2),
            }
        )
    here = Path(__file__).parent
    values = {
        "capacity_Ah": [capacity],
        "ocv_soc": list(OCV_GRID),
        "ocv_V": list(ocv),
        "r0_ohm": [r0],
        "r1_ohm": [r1],
        "c1_F": [tau1 / r1],
        "r2_ohm": [r2],
        "c2_F": [tau2 / r2],
        "soc_range": [SOC_LOW, SOC_HIGH],
    }
    with (here / "ecm_fit.txt").open("w") as out:
        out.write("# Written by fit_ecm.py from doi:10.17632/b5mj79w5w9.2 (CC BY 4.0), 25 degC.\n")
        out.write("# One cell; current positive when discharging. Validation RMS errors:\n")
        for entry in report:
            out.write(f"#   block at SoC {entry['soc']}: {entry['rms_mV']} mV\n")
        for key, numbers in values.items():
            out.write(key + " " + " ".join(f"{float(x):.6g}" for x in numbers) + "\n")
    with (here / "hppc_25degC_validation.csv").open("w") as out:
        out.write("block,time_s,current_A,voltage_V,soc\n")
        for n, (s, e) in enumerate(validation):
            for k in range(s, e):
                out.write(f"{n},{t[k] - t[s]:.3f},{current[k]:.5f},{voltage[k]:.5f},{soc[k]:.5f}\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()

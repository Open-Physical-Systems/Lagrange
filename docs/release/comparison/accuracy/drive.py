"""Runs every accuracy case on one engine and measures it the same way (docs/comparison.md).

An engine is a factory `make(case) -> run` where `run` has:
    step_to(t)        advance to simulated time t (s), in steps of scenes.DT
    position()        the free body's (or pendulum bob frame's) world position [x, y, z] (m)
    joint()           the hinge's (position rad, velocity rad/s); pendulum cases only
    contact_force()   the vertical force the ground puts on the box (N), or None if the engine
                      cannot report it; rest case only
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any, Protocol

import scenes


class Run(Protocol):
    def step_to(self, t: float) -> None: ...
    def position(self) -> list[float]: ...
    def joint(self) -> tuple[float, float]: ...
    def contact_force(self) -> float | None: ...


def _period(times: list[float], angles: list[float]) -> tuple[float, int]:
    """Mean period from the upward zero crossings of the angle, linearly interpolated."""
    up = []
    for i in range(1, len(angles)):
        a, b = angles[i - 1], angles[i]
        if a < 0.0 <= b:
            up.append(times[i - 1] + (times[i] - times[i - 1]) * (-a) / (b - a))
    if len(up) < 2:
        return math.nan, 0
    return (up[-1] - up[0]) / (len(up) - 1), len(up) - 1


def _quadratic_fit(ts: list[float], ss: list[float]) -> float:
    """Least-squares s = a0 + a1 t + a2 t²; returns the acceleration 2 a2."""
    n = len(ts)
    sx = [sum(t**k for t in ts) for k in range(5)]
    sy = [sum(s * t**k for t, s in zip(ts, ss, strict=True)) for k in range(3)]
    m = [[sx[i + j] for j in range(3)] + [sy[i]] for i in range(3)]
    for c in range(3):  # Gauss-Jordan
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c], strict=True)]
    del n
    return 2.0 * m[2][3] / m[2][2]


def _grid(end: float, every: float) -> list[float]:
    n = round(end / every)
    return [round((k + 1) * every, 9) for k in range(n)]


def measure(make: Callable[[str], Run], scene_dir: str) -> dict[str, Any]:
    out: dict[str, Any] = {}
    path = f"{scene_dir}/{{}}"

    run = make(path.format("freefall"))
    run.step_to(scenes.FALL_T)
    z = run.position()[2]
    out["freefall_z"] = z
    out["freefall_error"] = z - scenes.CLOSED["freefall_z"]

    run = make(path.format("pendulum_small"))
    ts, th = [], []
    for t in _grid(scenes.PEND_PERIOD_T, scenes.DT):
        run.step_to(t)
        ts.append(t)
        th.append(scenes.PEND_SMALL + run.joint()[0])
    period, cycles = _period(ts, th)
    out["pendulum_period"] = period
    out["pendulum_cycles"] = cycles
    out["pendulum_period_rel_error"] = period / scenes.CLOSED["pendulum_period"] - 1.0
    out["pendulum_small_final_amplitude"] = max(abs(a) for a in th[-2100:])

    run = make(path.format("pendulum_large"))
    e0 = scenes.pendulum_energy(scenes.PEND_LARGE, 0.0)
    worst = 0.0
    e = e0
    for t in _grid(scenes.PEND_ENERGY_T, 0.01):
        run.step_to(t)
        q, w = run.joint()
        e = scenes.pendulum_energy(scenes.PEND_LARGE + q, w)
        worst = max(worst, abs(e - e0) / e0)
    out["energy_rel_drift_end"] = (e - e0) / e0
    out["energy_rel_max_dev"] = worst

    for name, deg in (
        ("slope_slide", scenes.SLOPE_SLIDE_DEG),
        ("slope_static", scenes.SLOPE_STATIC_DEG),
    ):
        run = make(path.format(name))
        th_r = math.radians(deg)
        d = (math.cos(th_r), 0.0, -math.sin(th_r))
        run.step_to(scenes.DT)
        p0 = run.position()
        ts, ss = [], []
        for t in _grid(scenes.SLOPE_T, 0.01):
            run.step_to(t)
            p = run.position()
            ts.append(t)
            ss.append(sum((p[k] - p0[k]) * d[k] for k in range(3)))
        if name == "slope_slide":
            window = [(t, s) for t, s in zip(ts, ss, strict=True) if t >= 0.5]
            a = _quadratic_fit([t for t, _ in window], [s for _, s in window])
            out["slide_acceleration"] = a
            out["slide_acceleration_rel_error"] = a / scenes.CLOSED["slide_acceleration"] - 1.0
            out["slide_distance_2s"] = ss[-1]
        else:
            out["static_displacement_2s"] = ss[-1]

    run = make(path.format("rest"))
    forces = []
    for t in _grid(scenes.REST_T, 0.01):
        run.step_to(t)
        if t > scenes.REST_T - 0.5:
            forces.append(run.contact_force())
    top = scenes.SLAB_Z + 0.1
    out["rest_penetration"] = top + scenes.CUBE / 2 - run.position()[2]
    if all(f is not None for f in forces):
        f = sum(forces) / len(forces)
        out["rest_force"] = f
        out["rest_force_rel_error"] = f / scenes.CLOSED["rest_force"] - 1.0
    else:
        out["rest_force"] = None
    return out

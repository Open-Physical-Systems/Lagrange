#!/usr/bin/env python3
# ruff: noqa: B023, N812, RUF002, RUF003
# B023: every closure below is called within the loop iteration that made it. N812: models as M.
# RUF002/3: physics notation (×, −, ·) in text.
"""RFC 0019 step 1: closed-form contact tests on Lagrange's MuJoCo and Jolt backends.

    source /opt/ros/lyrical/setup.bash; source <lagrange install>/setup.bash
    contact_tests.py MODEL_DIR OUT_DIR [TEST ...]     (MODEL_DIR from models.py)

TEST is any of stance, leg2, drop, slope, box, rigid, rigid_body_friction_1, rimless, rtf,
determinism, ... (the keys of TESTS; default: all). Each
runs on both backends at 0.5, 1 and 2 ms and writes OUT_DIR/<test>.json, plus time series
(.npz) for the plots. Legs are held by an impedance law (kp, kd, gravity-free) evaluated in Python
before every physics step and sent as joint efforts: what RFC 0019 §5's impedance mode would do
in the server. Nothing in Lagrange is changed.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import lagrange

import models as M

G = M.G
BACKENDS = ("mujoco", "jolt")
STEPS = (0.0005, 0.001, 0.002)

PROFILE = """schema_version: 1
stage: {stage}
domains:
  rigid_body:   {{ enabled: true,  backend: {backend}, dt: {dt} }}
  electrical:   {{ enabled: false, substitute: ideal }}
  thermal:      {{ enabled: false, substitute: constant }}
  aerodynamics: {{ enabled: false, substitute: zero }}
  weather:      {{ enabled: false, substitute: clear }}
  emi:          {{ enabled: false, substitute: zero }}
  radiation:    {{ enabled: false, substitute: zero }}
  sensors:      {{ enabled: false, substitute: ground_truth }}
"""

# Impedance gains for the quadruped's joints (N·m/rad, N·m·s/rad).
KP, KD = 100.0, 1.0  # stable at 0.5, 1 and 2 ms on both engines (test_gains)
LEG2_KP, LEG2_KD = 400.0, 3.0


def make(model_dir: Path, out: Path, xml: str, backend: str, dt: float) -> lagrange.Simulation:
    """`backend` is mujoco, jolt, or jolt4 (Jolt with `substeps: 4`)."""
    stage = model_dir / xml
    profile = out / "profiles" / f"{Path(xml).stem}.{backend}.{dt * 1e3:g}ms.yaml"
    profile.parent.mkdir(parents=True, exist_ok=True)
    engine, extra = ("jolt", ", substeps: 4") if backend == "jolt4" else (backend, "")
    profile.write_text(
        PROFILE.format(stage=stage, backend=engine, dt=f"{dt}{extra}"), encoding="utf-8"
    )
    return lagrange.Simulation(str(stage), world=None, profile=str(profile), joint_control=True)


def quat_rotate(q: list[float], v: tuple[float, float, float]) -> np.ndarray:
    w, x, y, z = q
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return r @ np.asarray(v)


def point_of(sim: lagrange.Simulation, body: str, local: tuple[float, float, float]) -> np.ndarray:
    p = sim.read(f"body/{body}/pose")
    return np.asarray(p[:3]) + quat_rotate(p[3:7], local)


class Impedance:
    """tau = kp (q* - q) - kd q', sent as an effort before each step."""

    def __init__(self, sim: lagrange.Simulation, joints: list[str], kp: float, kd: float) -> None:
        self.sim, self.joints, self.kp, self.kd = sim, joints, kp, kd
        self.target = dict.fromkeys(joints, 0.0)

    def apply(self) -> None:
        read = self.sim.read
        for j in self.joints:
            q = read(f"joint/{j}/position")[0]
            qd = read(f"joint/{j}/velocity")[0]
            tau = self.kp * (self.target[j] - q) - self.kd * qd
            self.sim.command_joint(j, "effort", tau)


def stand_angles(height: float) -> tuple[float, float]:
    """Thigh and knee with the foot under the hip, `height` below it (trot.py's leg_angles)."""
    a, b = M.THIGH_L, M.SHIN_L
    knee = -math.acos((height * height - a * a - b * b) / (2 * a * b))
    thigh = -math.atan2(b * math.sin(knee), a + b * math.cos(knee))
    return thigh, knee


STAND_HEIGHT = 0.28  # m, hip to foot centre, as the example's trot.py
QUAD_JOINTS = [f"{leg}_{j}_joint" for leg in M.LEGS for j in ("hip", "thigh", "knee")]


class Quad:
    """The quadruped held by the impedance law, standing up from straight legs over RISE s."""

    RISE = 0.5

    def __init__(self, sim: lagrange.Simulation) -> None:
        self.sim = sim
        self.ctl = Impedance(sim, QUAD_JOINTS, KP, KD)
        self.thigh, self.knee = stand_angles(STAND_HEIGHT)

    def apply(self, t: float) -> None:
        s = min(1.0, t / self.RISE)
        for leg in M.LEGS:
            self.ctl.target[f"{leg}_thigh_joint"] = s * self.thigh
            self.ctl.target[f"{leg}_knee_joint"] = s * self.knee
        self.ctl.apply()

    def feet(self) -> dict[str, np.ndarray]:
        return {leg: point_of(self.sim, f"{leg}_shin", (0, 0, -M.SHIN_L)) for leg in M.LEGS}

    def foot_forces(self) -> dict[str, np.ndarray]:
        return {leg: np.asarray(self.sim.read(f"body/{leg}_shin/contact_force")) for leg in M.LEGS}


def run_steps(sim, dt: float, t_end: float, before=None, after=None) -> float:
    """Steps to t_end one physics step at a time; returns the wall time (s)."""
    n = round(t_end / dt)
    start = time.perf_counter()
    for i in range(round(sim.time / dt), n):
        if before is not None:
            before((i) * dt)
        sim.step_until((i + 1) * dt)
        if after is not None:
            after((i + 1) * dt)
    return time.perf_counter() - start


def loadavg() -> float:
    return float(Path("/proc/loadavg").read_text().split()[0])


# ---------------------------------------------------------------- tests 1 and 2: stance


def com(sim: lagrange.Simulation) -> np.ndarray:
    """The quadruped's centre of mass from the body poses and the model's inertial offsets."""
    total = M.BASE_M * np.asarray(sim.read("body/base_link/pose")[:3])
    for leg in M.LEGS:
        total = total + M.HIP_M * np.asarray(sim.read(f"body/{leg}_hip/pose")[:3])
        total = total + M.THIGH_M * point_of(sim, f"{leg}_thigh", (0, 0, -0.1))
        total = total + M.SHIN_M * point_of(sim, f"{leg}_shin", (0, 0, -0.1))
    return total / M.QUAD_M


def front_share(sim: lagrange.Simulation, feet: dict[str, np.ndarray]) -> float:
    """The front feet's share of the weight, by moments about y (left and right mirror each
    other, so front against rear is statically determinate even with four feet)."""
    xf = 0.5 * (feet["front_left"][0] + feet["front_right"][0])
    xr = 0.5 * (feet["rear_left"][0] + feet["rear_right"][0])
    return float((com(sim)[0] - xr) / (xf - xr))


def test_stance(model_dir: Path, out: Path) -> dict:
    results = {}
    mg = M.QUAD_M * G
    for backend in BACKENDS:
        for dt in STEPS:
            sim = make(model_dir, out, "quad.xml", backend, dt)
            q = Quad(sim)
            series: dict[str, list] = {
                "t": [],
                "sum": [],
                "feet": [],
                "pen": [],
                "base_z": [],
                "split": [],
            }

            def sample(t: float) -> None:
                if t < 1.0:
                    return
                f = q.foot_forces()
                feet = q.feet()
                series["t"].append(t)
                series["feet"].append([f[leg][2] for leg in M.LEGS])
                series["sum"].append(
                    sum(
                        sim.read(f"body/{b}/contact_force")[2]
                        for b in sim.bodies()
                        if b != "worldbody"
                    )
                )
                series["pen"].append([M.FOOT_R - feet[leg][2] for leg in M.LEGS])
                series["base_z"].append(sim.read("body/base_link/pose")[2])
                series["split"].append(front_share(sim, feet))

            wall = run_steps(sim, dt, 4.0, before=q.apply, after=sample)
            t = np.asarray(series["t"])
            m = t >= 2.0  # measured over 2-4 s
            total = np.asarray(series["sum"])[m]
            feet = np.asarray(series["feet"])[m]
            pen = np.asarray(series["pen"])[m]
            each = feet.mean(axis=0)
            share = float(np.mean(np.asarray(series["split"])[m]))
            closed_each = np.array([share, share, 1 - share, 1 - share]) * mg / 2
            results[f"{backend} {dt * 1e3:g} ms"] = {
                "backend": backend,
                "dt": dt,
                "weight_N": mg,
                "sum_mean_N": float(total.mean()),
                "sum_error_pct": float(100 * (total.mean() - mg) / mg),
                "feet_sum_mean_N": float(feet.sum(axis=1).mean()),
                "ripple_std_pct": float(100 * total.std() / mg),
                "ripple_p2p_pct": float(100 * (total.max() - total.min()) / mg),
                "per_foot_N": {leg: float(v) for leg, v in zip(M.LEGS, each, strict=True)},
                "split_max_dev_pct": float(100 * np.max(np.abs(each - mg / 4)) / (mg / 4)),
                "per_foot_closed_N": {
                    leg: float(v) for leg, v in zip(M.LEGS, closed_each, strict=True)
                },
                "per_foot_vs_closed_max_pct": float(
                    100 * np.max(np.abs(each - closed_each) / closed_each)
                ),
                "penetration_mean_mm": float(1e3 * pen.mean()),
                "penetration_max_mm": float(1e3 * pen.max()),
                "base_z_m": float(np.mean(np.asarray(series["base_z"])[m])),
                "wall_s_per_sim_s_with_python_loop": wall / 4.0,
                "load": loadavg(),
            }
            np.savez(
                out / f"stance_{backend}_{dt * 1e3:g}ms.npz",
                t=t,
                total=np.asarray(series["sum"]),
                feet=np.asarray(series["feet"]),
                pen=np.asarray(series["pen"]),
            )
            print(
                f"stance {backend} {dt * 1e3:g} ms:",
                json.dumps(results[f"{backend} {dt * 1e3:g} ms"]),
            )
    return results


# ---------------------------------------------------------------- test 3: static leg torques


def leg2_closed_form(sim: lagrange.Simulation, foot_force: np.ndarray) -> dict[str, float]:
    """Torque each joint must apply on its child to hold still: minus the moment, about the joint
    axis, of every external force on the bodies beyond it (gravity at the centres of mass, the
    floor at the foot). Positions from the measured poses."""
    hip_o = point_of(sim, "thigh", (0, 0, 0))
    knee_o = point_of(sim, "shin", (0, 0, 0))
    thigh_c = point_of(sim, "thigh", (0, 0, -M.LEG2_L / 2))
    shin_c = point_of(sim, "shin", (0, 0, -M.LEG2_L / 2))
    # Contact point: right under the foot sphere's centre (a flat floor).
    foot = point_of(sim, "shin", (0, 0, -M.LEG2_L)) - np.array([0.0, 0.0, M.LEG2_FOOT_R])
    axis_t = quat_rotate(sim.read("body/thigh/pose")[3:7], (0, 1, 0))
    axis_s = quat_rotate(sim.read("body/shin/pose")[3:7], (0, 1, 0))
    gt = np.array([0, 0, -M.LEG2_THIGH_M * G])
    gs = np.array([0, 0, -M.LEG2_SHIN_M * G])

    def moment(o: np.ndarray, forces: list[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
        return sum((np.cross(p - o, f) for p, f in forces), np.zeros(3))

    m_hip = moment(hip_o, [(thigh_c, gt), (shin_c, gs), (foot, foot_force)])
    m_knee = moment(knee_o, [(shin_c, gs), (foot, foot_force)])
    return {"hip_pitch": float(-m_hip @ axis_t), "knee": float(-m_knee @ axis_s)}


def test_leg2(model_dir: Path, out: Path) -> dict:
    results = {}
    weight = (M.LEG2_HIP_M + M.LEG2_THIGH_M + M.LEG2_SHIN_M) * G
    for backend in BACKENDS:
        for dt in STEPS:
            sim = make(model_dir, out, "leg2.xml", backend, dt)
            ctl = Impedance(sim, ["hip_pitch", "knee"], LEG2_KP, LEG2_KD)
            # A joint on a fixed base is held where it starts until commanded: free the rail.
            sim.command_joint("rail", "effort", 0.0)
            acc: dict[str, list] = {"tau": [], "cmd": [], "closed": [], "closed_meas": [], "f": []}

            def sample(t: float) -> None:
                if t < 1.0:
                    return
                f = np.asarray(sim.read("body/shin/contact_force"))
                acc["f"].append(f)
                acc["tau"].append([sim.read(f"joint/{j}/torque")[0] for j in ("hip_pitch", "knee")])
                acc["cmd"].append(
                    [sim.read(f"joint/{j}/torque_command")[0] for j in ("hip_pitch", "knee")]
                )
                c = leg2_closed_form(sim, np.array([0.0, 0.0, weight]))
                cm = leg2_closed_form(sim, f)
                acc["closed"].append([c["hip_pitch"], c["knee"]])
                acc["closed_meas"].append([cm["hip_pitch"], cm["knee"]])

            run_steps(sim, dt, 2.0, before=lambda _t: ctl.apply(), after=sample)
            tau = np.mean(acc["tau"], axis=0)
            closed = np.mean(acc["closed"], axis=0)
            closed_meas = np.mean(acc["closed_meas"], axis=0)
            f = np.mean(acc["f"], axis=0)
            q = [sim.read(f"joint/{j}/position")[0] for j in ("hip_pitch", "knee")]
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {
                "backend": backend,
                "dt": dt,
                "foot_force_N": f.tolist(),
                "foot_force_closed_N": weight,
                "foot_force_error_pct": float(100 * (f[2] - weight) / weight),
                "joint_sag_rad": q,
                "torque_Nm": tau.tolist(),
                "torque_command_Nm": np.mean(acc["cmd"], axis=0).tolist(),
                "closed_form_Nm": closed.tolist(),
                "closed_form_measured_force_Nm": closed_meas.tolist(),
                "error_pct": (100 * (tau - closed) / np.abs(closed)).tolist(),
                "error_vs_measured_force_pct": (
                    100 * (tau - closed_meas) / np.abs(closed_meas)
                ).tolist(),
            }
            print("leg2", key, json.dumps(results[key]))
    return results


# ---------------------------------------------------------------- test 4: drop


def test_drop(model_dir: Path, out: Path) -> dict:
    results = {}
    m, r = M.BALL_M, M.BALL_R
    for backend in (*BACKENDS, "jolt4"):
        for dt in STEPS:
            sim = make(model_dir, out, "ball.xml", backend, dt)
            ts, zs, vs, fs = [], [], [], []

            def sample(t: float) -> None:
                ts.append(t)
                zs.append(sim.read("body/ball/pose")[2])
                vs.append(sim.read("body/ball/twist")[2])
                fs.append(sim.read("body/ball/contact_force")[2])

            wall = run_steps(sim, dt, 1.5, after=sample)
            t, z, v, f = map(np.asarray, (ts, zs, vs, fs))
            # Jolt's contact force is Newton's law on the body, so a falling body reads ~1e-6 N:
            # a contact is a force over 0.1 % of the weight.
            on = np.nonzero(f > 1e-3 * m * G)[0]
            first = on[0]
            # The impact: from the first step with a force to the highest upward speed within
            # 50 ms after it (compression and restitution together).
            window = slice(first, first + round(0.05 / dt))
            last = first + int(np.argmax(v[window]))
            v_in = v[first - 1]
            v_out = v[last]
            impulse = float(np.sum(f[first : last + 1]) * dt)
            duration = (last - first + 1) * dt
            momentum = m * (v_out - v_in) + m * G * duration
            z_final = float(np.mean(z[t > 1.3]))
            settled = np.nonzero((np.abs(z - z_final) > 1e-4) | (np.abs(v) > 1e-3))[0]
            settle_t = float(t[settled[-1]] - t[first]) if settled.size else 0.0
            bounce = float(np.max(z[last:]) - z_final)
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {
                "backend": backend,
                "dt": dt,
                "v_in_mps": float(v_in),
                "v_closed_mps": -math.sqrt(2 * G * (M.BALL_Z0 - r)),
                "v_out_mps": float(v_out),
                "restitution": float(-v_out / v_in),
                "impact_ms": duration * 1e3,
                "impulse_Ns": impulse,
                "momentum_change_plus_mg_T_Ns": float(momentum),
                "impulse_error_pct": float(100 * (impulse - momentum) / momentum),
                "peak_force_N": float(f.max()),
                "peak_penetration_mm": float(1e3 * (r - z.min())),
                "rest_sink_mm": float(1e3 * (r - z_final)),
                "rest_force_N": float(np.mean(f[t > 1.3])),
                "settle_s": settle_t,
                "bounce_height_mm": 1e3 * bounce,
                "wall_s": wall,
            }
            np.savez(out / f"drop_{backend}_{dt * 1e3:g}ms.npz", t=t, z=z, v=v, f=f)
            print("drop", key, json.dumps(results[key]))
    return results


# ---------------------------------------------------------------- test 5: slopes


def test_slope(model_dir: Path, out: Path) -> dict:
    results = {}
    for deg in M.QUAD_SLOPES:
        th = math.radians(deg)
        down = np.array([math.cos(th), 0.0, -math.sin(th)])
        for backend in BACKENDS:
            for dt in STEPS:
                sim = make(model_dir, out, f"quad_slope_{deg:g}.xml", backend, dt)
                q = Quad(sim)
                ts, feet_s, base_s = [], [], []

                def sample(t: float) -> None:
                    if t < 1.0 or round(t / dt) % max(1, round(0.005 / dt)):
                        return
                    ts.append(t)
                    feet_s.append(np.mean(list(q.feet().values()), axis=0) @ down)
                    base_s.append(np.asarray(sim.read("body/base_link/pose")[:3]) @ down)

                run_steps(sim, dt, 6.0, before=q.apply, after=sample)
                t, feet, base = map(np.asarray, (ts, feet_s, base_s))
                creep = np.interp(3.0, t, feet) - np.interp(1.0, t, feet)
                creep_late = np.interp(6.0, t, feet) - np.interp(3.0, t, feet)
                fit = np.polyfit(t[t <= 3.0], base[t <= 3.0], 2)
                key = f"{deg:g} deg {backend} {dt * 1e3:g} ms"
                results[key] = {
                    "slope_deg": deg,
                    "backend": backend,
                    "dt": dt,
                    "holds_closed_form": math.tan(th) < M.FEET_MU,
                    "closed_acceleration": max(0.0, G * (math.sin(th) - M.FEET_MU * math.cos(th))),
                    "feet_creep_mm_1_to_3s": float(1e3 * creep),
                    "feet_creep_rate_3_6s_mm_s": float(1e3 * creep_late / 3.0),
                    "base_moved_mm_1_to_6s": float(1e3 * (base[-1] - base[0])),
                    "base_fit_acceleration": float(2 * fit[0]),
                    "base_z": float(sim.read("body/base_link/pose")[2]),
                }
                np.savez(
                    out / f"slope_{deg:g}_{backend}_{dt * 1e3:g}ms.npz", t=t, feet=feet, base=base
                )
                print("slope", key, json.dumps(results[key]))
    return results


def test_rigid_slope(
    model_dir: Path, out: Path, name: str = "rigid_quad_slope", slopes=M.RIGID_SLOPES
) -> dict:
    """The rigid quadruped (models.rigid_quad_slope) on slopes below and above atan μ: creep
    along the slope over 1-3 s and 3-6 s, and the fitted acceleration over 1-3 s."""
    results = {}
    for deg in slopes:
        th = math.radians(deg)
        down = np.array([math.cos(th), 0.0, -math.sin(th)])
        for backend in BACKENDS:
            for dt in STEPS:
                sim = make(model_dir, out, f"{name}_{deg:g}.xml", backend, dt)
                ts, xs = [], []

                def sample(t: float) -> None:
                    if round(t / dt) % max(1, round(0.005 / dt)) == 0:
                        ts.append(t)
                        xs.append(np.asarray(sim.read("body/base_link/pose")[:3]) @ down)

                run_steps(sim, dt, 6.0, after=sample)
                t, x = np.asarray(ts), np.asarray(xs)

                def at(time_s: float) -> float:
                    return float(np.interp(time_s, t, x))

                early = (t >= 1.0) & (t <= 3.0)
                fit = np.polyfit(t[early], x[early], 2)
                q = sim.read("body/base_link/pose")[3:7]
                key = f"{deg:g} deg {backend} {dt * 1e3:g} ms"
                results[key] = {
                    "slope_deg": deg,
                    "backend": backend,
                    "dt": dt,
                    "holds_closed_form": math.tan(th) < M.RIGID_MU,
                    "closed_acceleration": max(0.0, G * (math.sin(th) - M.RIGID_MU * math.cos(th))),
                    "creep_1_3s_mm": 1e3 * (at(3.0) - at(1.0)),
                    "creep_3_6s_mm": 1e3 * (at(6.0) - at(3.0)),
                    "creep_rate_3_6s_mm_s": 1e3 * (at(6.0) - at(3.0)) / 3.0,
                    "fit_acceleration": float(2 * fit[0]),
                    "upright": float(abs(2 * math.atan2(q[2], q[0]) - th)) < 0.05,
                }
                np.savez(out / f"{name}_{deg:g}_{backend}_{dt * 1e3:g}ms.npz", t=t, x=x)
                print(name, key, json.dumps(results[key]))
    return results


def test_box(model_dir: Path, out: Path) -> dict:
    """The comparison's box on 10° (μ 0.3), for the creep reference, at each step."""
    results = {}
    th = math.radians(10.0)
    down = np.array([math.cos(th), 0.0, -math.sin(th)])
    for backend in BACKENDS:
        for dt in STEPS:
            sim = make(model_dir, out, "box_slope_10.xml", backend, dt)
            p0 = np.asarray(sim.read("body/box/pose")[:3])
            sim.step_until(2.0)
            p1 = np.asarray(sim.read("body/box/pose")[:3])
            sim.step_until(4.0)
            p2 = np.asarray(sim.read("body/box/pose")[:3])
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {
                "backend": backend,
                "dt": dt,
                "creep_0_2s_mm": float(1e3 * (p1 - p0) @ down),
                "creep_2_4s_mm": float(1e3 * (p2 - p1) @ down),
            }
            print("box", key, json.dumps(results[key]))
    return results


# ---------------------------------------------------------------- test 6: rimless wheel


def rimless_closed_form() -> dict:
    a = math.pi / M.RIM_N
    g = math.radians(M.RIM_SLOPE_DEG)
    ml2 = M.RIM_HUB_M * M.RIM_L**2
    j = M.RIM_HUB_I + ml2  # about the foot
    k = (M.RIM_HUB_I + ml2 * math.cos(2 * a)) / j  # McGeer's cos 2α, with the hub's own inertia
    c = 2 * M.RIM_HUB_M * G * M.RIM_L / j
    w1 = math.sqrt(c * (math.cos(g - a) - math.cos(g + a)) / (1 - k * k))
    w0 = k * w1
    # Step time: integrate dφ/ω over the stance, ω² = ω0² + c (cos φ0 - cos φ).
    n = 20000
    phis = np.linspace(g - a, g + a, n + 1)
    om = np.sqrt(w0 * w0 + c * (math.cos(g - a) - np.cos(phis)))
    step_t = float(np.trapezoid(1.0 / om, phis))
    speed = 2 * M.RIM_L * math.sin(a) / step_t
    return {
        "k": k,
        "omega_before_rad_s": w1,
        "omega_after_rad_s": w0,
        "step_s": step_t,
        "speed_mps": speed,
    }


def analyse_rimless(t: np.ndarray, w: np.ndarray, x: np.ndarray, th_hub: np.ndarray) -> dict:
    """Per stance, from the hub's angle: the stance angle φ (hub from the foot, from vertical,
    positive downhill) runs from γ−α to γ+α. Away from the impacts (the middle half of each
    stance) the rigid stance gives ω² = ω0² + c (cos φ0 − cos φ), so each sample estimates the
    speed just after the impact that began the stance (ω0) and just before the one that ends it
    (ω1); medians per stance. The impact ratio is ω0 of a stance over ω1 of the one before. This
    leaves the soft contact's impact window out of the measurement."""
    cf = rimless_closed_form()
    a = math.pi / M.RIM_N
    g = math.radians(M.RIM_SLOPE_DEG)
    c = 2 * M.RIM_HUB_M * G * M.RIM_L / (M.RIM_HUB_I + M.RIM_HUB_M * M.RIM_L**2)
    rel = th_hub + M.RIM_START_TILT - (g - a)
    stance = np.floor(rel / (2 * a)).astype(int)
    phi = rel - stance * 2 * a + (g - a)
    frac = (phi - (g - a)) / (2 * a)
    mid = (frac > 0.25) & (frac < 0.75) & (w > 0.05)
    w0 = np.sqrt(np.maximum(w**2 - c * (math.cos(g - a) - np.cos(phi)), 0.0))
    w1 = np.sqrt(w**2 + c * (np.cos(phi) - math.cos(g + a)))
    per = {}
    for k in np.unique(stance[mid]):
        sel = mid & (stance == k)
        per[int(k)] = (float(np.median(w0[sel])), float(np.median(w1[sel])))
    ratios = [per[k][0] / per[k - 1][1] for k in sorted(per) if k - 1 in per]
    last = [per[k][1] for k in sorted(per)][-10:]
    r_last = ratios[-10:]
    late = t > 6.0
    speed = float(np.polyfit(t[late], x[late], 1)[0])
    out = {
        "stances": len(per),
        "ratio_mean_last10": float(np.mean(r_last)) if r_last else float("nan"),
        "ratio_closed": cf["k"],
        "omega_before_last10": float(np.mean(last)) if last else float("nan"),
        "omega_before_closed": cf["omega_before_rad_s"],
        "speed_mps_6_10s": speed,
        "speed_closed": cf["speed_mps"],
        "travelled_m": float(x[-1] - x[0]),
        "ratios": ratios,
    }
    out["ratio_error_pct"] = 100 * (out["ratio_mean_last10"] - cf["k"]) / cf["k"]
    out["omega_before_error_pct"] = (
        100 * (out["omega_before_last10"] - cf["omega_before_rad_s"]) / cf["omega_before_rad_s"]
    )
    out["speed_error_pct"] = 100 * (speed - cf["speed_mps"]) / cf["speed_mps"]
    return out


def test_rimless(model_dir: Path, out: Path) -> dict:
    results = {"closed_form": rimless_closed_form()}
    th = math.radians(M.RIM_SLOPE_DEG)
    down = np.array([math.cos(th), 0.0, -math.sin(th)])
    for backend in (*BACKENDS, "jolt4"):
        for dt in STEPS:
            sim = make(model_dir, out, "rimless.xml", backend, dt)
            ts, ws, xs, angles = [], [], [], []

            def sample(t: float) -> None:
                ts.append(t)
                ws.append(sim.read("body/hub/twist")[4])
                p = sim.read("body/hub/pose")
                xs.append(np.asarray(p[:3]) @ down)
                angles.append(2.0 * math.atan2(p[5], p[3]))  # about y

            run_steps(sim, dt, 10.0, after=sample)
            t, w, x, th_hub = map(np.asarray, (ts, ws, xs, angles))
            res = analyse_rimless(t, w, x, np.unwrap(th_hub))
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {"backend": backend, "dt": dt, **res}
            np.savez(
                out / f"rimless_{backend}_{dt * 1e3:g}ms.npz", t=t, w=w, x=x, th=np.unwrap(th_hub)
            )
            print("rimless", key, json.dumps(results[key]))
    return results


# ---------------------------------------------------------------- real-time factor, determinism


def test_rtf(model_dir: Path, out: Path) -> dict:
    """The quadruped standing on Lagrange's own joint loops (position commands, no Python in the
    loop): 10 s in one step_until call. Engine cost per simulated second."""
    results = {}
    th, kn = stand_angles(STAND_HEIGHT)
    for backend in BACKENDS:
        for dt in STEPS:
            sim = make(model_dir, out, "quad.xml", backend, dt)
            for leg in M.LEGS:
                sim.command_joint(f"{leg}_thigh_joint", "position", th)
                sim.command_joint(f"{leg}_knee_joint", "position", kn)
                sim.command_joint(f"{leg}_hip_joint", "position", 0.0)
            sim.step_until(0.5)
            load0 = loadavg()
            start = time.perf_counter()
            sim.step_until(10.5)
            wall = time.perf_counter() - start
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {
                "backend": backend,
                "dt": dt,
                "rtf": 10.0 / wall,
                "steps_per_s": 10.0 / dt / wall,
                "base_z": sim.read("body/base_link/pose")[2],
                "load": (load0 + loadavg()) / 2,
            }
            print("rtf", key, json.dumps(results[key]))
    return results


def test_determinism(model_dir: Path, out: Path) -> dict:
    results = {}
    for backend in BACKENDS:
        hashes = []
        for _ in range(2):
            sim = make(model_dir, out, "quad.xml", backend, 0.001)
            q = Quad(sim)
            seq = []
            run_steps(
                sim,
                0.001,
                2.0,
                before=q.apply,
                after=lambda t, s=sim, h=seq: (
                    h.append(s.state_hash()) if round(t * 1000) % 100 == 0 else None
                ),
            )
            hashes.append(seq)
        results[backend] = {"identical": hashes[0] == hashes[1], "checks": len(hashes[0])}
        print("determinism", backend, results[backend])
    return results


def test_gains(model_dir: Path, out: Path) -> dict:
    """The stiffest impedance (kd = kp/100) that still stands still, per engine and step: the
    law runs in Python before each physics step and Lagrange's joint module passes it on at
    1 ms, so this is an explicit, delayed controller, as an RL policy's PD often is."""
    results = {}
    for backend in BACKENDS:
        for dt in STEPS:
            for kp in (25.0, 50.0, 100.0, 150.0, 250.0, 400.0):
                sim = make(model_dir, out, "quad.xml", backend, dt)
                q = Quad(sim)
                q.ctl.kp, q.ctl.kd = kp, kp / 100.0
                zs = []
                key = f"{backend} {dt * 1e3:g} ms kp {kp:g}"
                try:
                    run_steps(
                        sim,
                        dt,
                        3.0,
                        before=q.apply,
                        after=lambda t, s=sim, z=zs: (
                            z.append(s.read("body/base_link/pose")[2]) if t > 2.0 else None
                        ),
                    )
                except RuntimeError as e:
                    results[key] = {
                        "backend": backend,
                        "dt": dt,
                        "kp": kp,
                        "kd": kp / 100.0,
                        "stands_still": False,
                        "diverged": str(e)[:160],
                    }
                    print("gains", key, "diverged")
                    continue
                speeds = [abs(sim.read(f"joint/{j}/velocity")[0]) for j in QUAD_JOINTS]
                stood = bool(np.ptp(zs) < 0.002 and min(zs) > 0.2 and max(speeds) < 0.1)
                results[key] = {
                    "backend": backend,
                    "dt": dt,
                    "kp": kp,
                    "kd": kp / 100.0,
                    "base_z_mean": float(np.mean(zs)),
                    "base_z_ptp_mm": float(1e3 * np.ptp(zs)),
                    "max_joint_speed": float(max(speeds)),
                    "stands_still": stood,
                }
                print("gains", key, json.dumps(results[key]))
    return results


def test_quad_drop(model_dir: Path, out: Path) -> dict:
    """The quadruped, legs held straight (kp 100), dropped with its feet 0.1 m above the floor:
    peak foot penetration and force, how long until the feet carry the weight within 2 %, and
    the energy taken by the landing (kinetic + potential, from the body motion)."""
    results = {}
    mg = M.QUAD_M * G
    for backend in BACKENDS:
        for dt in STEPS:
            sim = make(model_dir, out, "quad_drop.xml", backend, dt)
            ctl = Impedance(sim, QUAD_JOINTS, KP, KD)
            ts, tot, pens, energy = [], [], [], []
            bodies = [b for b in sim.bodies() if b != "worldbody"]
            masses = {"base_link": M.BASE_M}
            for leg in M.LEGS:
                masses[f"{leg}_hip"] = M.HIP_M
                masses[f"{leg}_thigh"] = M.THIGH_M
                masses[f"{leg}_shin"] = M.SHIN_M

            def sample(t: float) -> None:
                ts.append(t)
                tot.append(sum(sim.read(f"body/{b}/contact_force")[2] for b in bodies))
                pens.append(
                    max(
                        M.FOOT_R - point_of(sim, f"{leg}_shin", (0, 0, -M.SHIN_L))[2]
                        for leg in M.LEGS
                    )
                )
                # Translational energy of the bodies' frames (an estimate; the legs barely turn).
                e = 0.0
                for b, m in masses.items():
                    v = sim.read(f"body/{b}/twist")[:3]
                    e += (
                        0.5 * m * (v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
                        + m * G * sim.read(f"body/{b}/pose")[2]
                    )
                energy.append(e)

            run_steps(sim, dt, 1.5, before=lambda _t: ctl.apply(), after=sample)
            t, f, pen, e = map(np.asarray, (ts, tot, pens, energy))
            first = int(np.nonzero(f > 1e-3 * mg)[0][0])
            bad = np.nonzero(np.abs(f[first:] - mg) > 0.02 * mg)[0]
            settle = float(t[first + bad[-1]] - t[first]) if bad.size else 0.0
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {
                "backend": backend,
                "dt": dt,
                "impact_speed_closed_mps": math.sqrt(2 * G * M.QUAD_DROP),
                "peak_force_over_mg": float(f.max() / mg),
                "peak_penetration_mm": float(1e3 * pen.max()),
                "rest_penetration_mm": float(1e3 * np.mean(pen[t > 1.2])),
                "settle_to_2pct_s": settle,
                "energy_lost_J": float(e[first - 1] - e[-1]),
                "drop_energy_J": float(M.QUAD_M * G * M.QUAD_DROP),
            }
            np.savez(out / f"quad_drop_{backend}_{dt * 1e3:g}ms.npz", t=t, f=f, pen=pen)
            print("quad_drop", key, json.dumps(results[key]))
    return results


HEX_JOINTS = [
    f"leg{i}{s}_{j}" for i in range(3) for s in ("l", "r") for j in ("swing", "lift", "knee")
]


def test_hexapod(model_dir: Path, out: Path) -> dict:
    """Six feet on the floor (statically indeterminate twice over): the sum against the weight,
    and the split per foot as each engine makes it."""
    results = {}
    mass = M.HEX_M + 6 * (0.15 + 0.1)
    for backend in BACKENDS:
        for dt in STEPS:
            sim = make(model_dir, out, "hexapod.xml", backend, dt)
            ctl = Impedance(sim, HEX_JOINTS, 30.0, 0.3)
            feet, tot = [], []
            bodies = [b for b in sim.bodies() if b != "worldbody"]

            def sample(t: float) -> None:
                if t >= 1.0:
                    feet.append(
                        [
                            sim.read(f"body/leg{i}{s}_lower/contact_force")[2]
                            for i in range(3)
                            for s in ("l", "r")
                        ]
                    )
                    tot.append(sum(sim.read(f"body/{b}/contact_force")[2] for b in bodies))

            run_steps(sim, dt, 2.0, before=lambda _t: ctl.apply(), after=sample)
            each = np.mean(feet, axis=0)
            key = f"{backend} {dt * 1e3:g} ms"
            results[key] = {
                "backend": backend,
                "dt": dt,
                "weight_N": mass * G,
                "sum_error_pct": float(100 * (np.mean(tot) - mass * G) / (mass * G)),
                "per_foot_N": [float(x) for x in each],
                "front_middle_rear_pair_N": [
                    float(each[0] + each[1]),
                    float(each[2] + each[3]),
                    float(each[4] + each[5]),
                ],
            }
            print("hexapod", key, json.dumps(results[key]))
    return results


TESTS = {
    "quad_drop": test_quad_drop,
    "hexapod": test_hexapod,
    "gains": test_gains,
    "stance": test_stance,
    "leg2": test_leg2,
    "drop": test_drop,
    "slope": test_slope,
    "box": test_box,
    "rigid": test_rigid_slope,
    # The body box at μ 1 on μ 0.5 feet: each collider's own friction (Jolt once took the body's
    # largest for all), written to rigid_body_friction_1.json.
    "rigid_body_friction_1": lambda d, o: test_rigid_slope(
        d, o, "rigid_body1_quad_slope", M.BODY1_SLOPES
    ),
    "rimless": test_rimless,
    "rtf": test_rtf,
    "determinism": test_determinism,
}


def main() -> int:
    model_dir, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    names = sys.argv[3:] or list(TESTS)
    for name in names:
        start = time.perf_counter()
        load = loadavg()
        res = TESTS[name](model_dir, out)
        res["_meta"] = {
            "wall_s": time.perf_counter() - start,
            "load_start": load,
            "load_end": loadavg(),
            "lagrange": getattr(lagrange, "__file__", ""),
        }
        (out / f"{name}.json").write_text(json.dumps(res, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())

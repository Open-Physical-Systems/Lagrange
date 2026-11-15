#!/usr/bin/env python3
# ruff: noqa: E501, RUF002  (generated XML)
"""The test models of the legged contact spike (RFC 0019 step 1), written as MJCF.

    models.py OUT_DIR

Every model carries its own floor (a plane or a slab), so Lagrange adds no ground of its own,
and g = 9.81 m/s² straight down. Friction is the larger of the two geoms' in both backends.

- quad.xml, quad_slope_<deg>.xml: the primitive quadruped of examples/lagrange_example_quadruped
  (same links, masses, inertias, joint axes and limits; 15.4 kg), legs straight at the start.
- leg2.xml: a two-link leg (0.3 m + 0.3 m) under a 5 kg hip that slides on a vertical rail,
  foot and floor frictionless, so the foot force is exactly the weight above it.
- ball.xml: a 1 kg sphere (r 0.05 m) dropped from 0.5 m.
- box_slope_<deg>.xml: the comparison's 1 kg, 0.2 m cube on a slab (reference for creep).
- rimless.xml: a rimless wheel of N spokes (two rims side by side so it cannot fall over), mass
  at the hub, on a slope.
- hexapod.xml: a simple legged model for the RL speed test: a box body, six two-joint legs.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

G = 9.81

# The example quadruped (examples/lagrange_example_quadruped/robot/quadruped.urdf).
BASE_M, BASE_SIZE, BASE_I = 9.0, (0.5, 0.22, 0.1), (0.06, 0.2, 0.24)
HIP_M, HIP_I = 0.5, (0.001, 0.001, 0.001)
THIGH_M, THIGH_I, THIGH_L = 0.8, (0.003, 0.003, 0.0005), 0.2
SHIN_M, SHIN_I, SHIN_L = 0.3, (0.001, 0.001, 0.0002), 0.2
FOOT_R = 0.025
HIP_X, HIP_Y = 0.22, 0.13
QUAD_M = BASE_M + 4 * (HIP_M + THIGH_M + SHIN_M)
LEGS = {"front_left": (1, 1), "front_right": (1, -1), "rear_left": (-1, 1), "rear_right": (-1, -1)}
FEET_MU = 0.6  # feet and slab

# Two-link leg.
LEG2_HIP_M, LEG2_THIGH_M, LEG2_SHIN_M, LEG2_L = 5.0, 0.4, 0.2, 0.3
LEG2_Q = (0.6, -1.2)  # thigh, knee (rad): foot right under the hip
LEG2_FOOT_R = 0.02

# Drop.
BALL_M, BALL_R, BALL_Z0 = 1.0, 0.05, 0.5

# Rimless wheel.
RIM_N = 10  # spokes per rim
RIM_L = 0.5  # m, hub to tip centre
RIM_TIP_R = 0.005
RIM_HUB_M = 10.0
RIM_HUB_I = 0.01  # kg m², about the axle (small next to m l² = 2.5)
RIM_HALF_WIDTH = 0.15
RIM_SLOPE_DEG = 4.0
RIM_MU = 1.5


def _inertia(m: float, i: tuple[float, float, float]) -> str:
    return f'mass="{m}" diaginertia="{i[0]} {i[1]} {i[2]}"'


def _header(name: str, dt: float = 0.001) -> str:
    return f"""<mujoco model="{name}">
  <compiler angle="radian"/>
  <option timestep="{dt}" gravity="0 0 -{G}"/>
"""


def _quad_body(pos: tuple[float, float, float], pitch: float) -> str:
    legs = []
    for leg, (sx, sy) in LEGS.items():
        legs.append(f"""      <body name="{leg}_hip" pos="{sx * HIP_X} {sy * HIP_Y} 0">
        <joint name="{leg}_hip_joint" type="hinge" axis="1 0 0" range="-0.6 0.6" damping="0.05"/>
        <inertial pos="0 0 0" {_inertia(HIP_M, HIP_I)}/>
        <body name="{leg}_thigh" pos="0 0 0">
          <joint name="{leg}_thigh_joint" type="hinge" axis="0 1 0" range="-1.6 1.6" damping="0.05"/>
          <inertial pos="0 0 -0.1" {_inertia(THIGH_M, THIGH_I)}/>
          <body name="{leg}_shin" pos="0 0 -{THIGH_L}">
            <joint name="{leg}_knee_joint" type="hinge" axis="0 1 0" range="-2.6 0" damping="0.05"/>
            <inertial pos="0 0 -0.1" {_inertia(SHIN_M, SHIN_I)}/>
            <geom name="{leg}_foot" type="sphere" size="{FOOT_R}" pos="0 0 -{SHIN_L}" friction="{FEET_MU}" mass="0"/>
          </body>
        </body>
      </body>""")
    bx, by, bz = (s / 2 for s in BASE_SIZE)
    return f"""    <body name="base_link" pos="{pos[0]} {pos[1]} {pos[2]}" euler="0 {pitch} 0">
      <freejoint name="base_free"/>
      <inertial pos="0 0 0" {_inertia(BASE_M, BASE_I)}/>
      <geom name="base" type="box" size="{bx} {by} {bz}" mass="0"/>
{chr(10).join(legs)}
    </body>"""


QUAD_DROP = 0.10  # m, feet above the floor at the start of the drop test


def quad(clearance: float = 0.005, name: str = "quad") -> str:
    # Legs straight: feet 0.4 m + foot radius below the hips, `clearance` above the floor.
    z = THIGH_L + SHIN_L + FOOT_R + clearance
    return (
        _header(name)
        + f"""  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" friction="{FEET_MU}"/>
{_quad_body((0.0, 0.0, z), 0.0)}
  </worldbody>
</mujoco>
"""
    )


SLAB_Z = 5.0
SLAB_HALF = (6.0, 3.0, 0.05)


def _slab(deg: float, mu: float) -> str:
    th = math.radians(deg)
    return f'    <geom name="slab" type="box" size="{SLAB_HALF[0]} {SLAB_HALF[1]} {SLAB_HALF[2]}" pos="0 0 {SLAB_Z}" euler="0 {th} 0" friction="{mu}"/>'


def _on_slab(deg: float, height: float, along: float = 0.0) -> tuple[float, float, float]:
    """A point `height` above the slab's top surface along its normal, `along` metres downhill."""
    th = math.radians(deg)
    n = (math.sin(th), 0.0, math.cos(th))
    t = (math.cos(th), 0.0, -math.sin(th))  # downhill
    h = SLAB_HALF[2] + height
    return (
        along * t[0] + h * n[0],
        0.0,
        SLAB_Z + along * t[2] + h * n[2],
    )


def quad_slope(deg: float) -> str:
    pos = _on_slab(deg, THIGH_L + SHIN_L + FOOT_R + 0.005)
    return (
        _header(f"quad_slope_{deg:g}")
        + f"""  <worldbody>
{_slab(deg, FEET_MU)}
{_quad_body(pos, math.radians(deg))}
  </worldbody>
</mujoco>
"""
    )


def box_slope(deg: float, mu: float = 0.3) -> str:
    h = 0.1
    pos = _on_slab(deg, h + 0.0005)
    th = math.radians(deg)
    i = 1.0 * 0.2 * 0.2 / 6.0
    return (
        _header(f"box_slope_{deg:g}")
        + f"""  <worldbody>
{_slab(deg, mu)}
    <body name="box" pos="{pos[0]} {pos[1]} {pos[2]}" euler="0 {th} 0">
      <freejoint name="box_free"/>
      <inertial pos="0 0 0" mass="1" diaginertia="{i} {i} {i}"/>
      <geom type="box" size="{h} {h} {h}" friction="{mu}" mass="0"/>
    </body>
  </worldbody>
</mujoco>
"""
    )


def leg2() -> str:
    q1, q2 = LEG2_Q
    foot_depth = LEG2_L * math.cos(q1) + LEG2_L * math.cos(q1 + q2)
    z = foot_depth + LEG2_FOOT_R + 0.002
    ti = (LEG2_THIGH_M * LEG2_L**2 / 12, LEG2_THIGH_M * LEG2_L**2 / 12, 1e-4)
    si = (LEG2_SHIN_M * LEG2_L**2 / 12, LEG2_SHIN_M * LEG2_L**2 / 12, 1e-4)
    return (
        _header("leg2")
        + f"""  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" friction="0"/>
    <body name="hip" pos="0 0 {z}">
      <joint name="rail" type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="{LEG2_HIP_M}" diaginertia="0.02 0.02 0.02"/>
      <body name="thigh" pos="0 0 0" euler="0 {q1} 0">
        <joint name="hip_pitch" type="hinge" axis="0 1 0" range="-2 2" damping="0.05"/>
        <inertial pos="0 0 -{LEG2_L / 2}" {_inertia(LEG2_THIGH_M, ti)}/>
        <body name="shin" pos="0 0 -{LEG2_L}" euler="0 {q2} 0">
          <joint name="knee" type="hinge" axis="0 1 0" range="-2.6 2.6" damping="0.05"/>
          <inertial pos="0 0 -{LEG2_L / 2}" {_inertia(LEG2_SHIN_M, si)}/>
          <geom name="foot" type="sphere" size="{LEG2_FOOT_R}" pos="0 0 -{LEG2_L}" friction="0" mass="0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
    )


def ball() -> str:
    i = 0.4 * BALL_M * BALL_R**2
    return (
        _header("ball")
        + f"""  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" friction="1"/>
    <body name="ball" pos="0 0 {BALL_Z0}">
      <freejoint name="ball_free"/>
      <inertial pos="0 0 0" mass="{BALL_M}" diaginertia="{i} {i} {i}"/>
      <geom type="sphere" size="{BALL_R}" friction="1" mass="0"/>
    </body>
  </worldbody>
</mujoco>
"""
    )


RIM_START_TILT = 0.02  # rad: the first spoke just past upright, the hub just downhill of it


def rimless() -> str:
    """Spokes at k·2α + offset from straight down (α = π/N), in the world frame. It starts at rest
    balanced on one upright spoke, tipped RIM_START_TILT downhill, so the first stance starts at
    the top; McGeer's steady gait is an attractor from there when the slope is below α."""
    th = math.radians(RIM_SLOPE_DEG)
    alpha = math.pi / RIM_N
    # Hub over the first tip; the tip's centre one tip radius above the slab (along its normal).
    top = SLAB_Z + SLAB_HALF[2] / math.cos(th)
    hub_z = top + (RIM_TIP_R + 0.0005) / math.cos(th) + RIM_L * math.cos(RIM_START_TILT)
    hub_x = RIM_L * math.sin(RIM_START_TILT)
    spokes = []
    for side in (-1, 1):
        for k in range(RIM_N):
            phi = 2 * k * alpha - RIM_START_TILT
            x, z = RIM_L * math.sin(phi), -RIM_L * math.cos(phi)
            y = side * RIM_HALF_WIDTH
            spokes.append(
                f'      <geom type="capsule" fromto="0 {y} 0 {x:.6f} {y} {z:.6f}" size="0.004" contype="0" conaffinity="0" mass="0"/>\n'
                f'      <geom type="sphere" size="{RIM_TIP_R}" pos="{x:.6f} {y} {z:.6f}" friction="{RIM_MU}" mass="0"/>'
            )
    return (
        _header("rimless")
        + f"""  <worldbody>
    <geom name="slab" type="box" size="30 2 {SLAB_HALF[2]}" pos="0 0 {SLAB_Z}" euler="0 {th} 0" friction="{RIM_MU}"/>
    <body name="hub" pos="{hub_x:.6f} 0 {hub_z:.6f}">
      <freejoint name="hub_free"/>
      <inertial pos="0 0 0" mass="{RIM_HUB_M}" diaginertia="{RIM_HUB_I} {RIM_HUB_I} {RIM_HUB_I}"/>
      <geom type="cylinder" fromto="0 -{RIM_HALF_WIDTH} 0 0 {RIM_HALF_WIDTH} 0" size="0.01" contype="0" conaffinity="0" mass="0"/>
{chr(10).join(spokes)}
    </body>
  </worldbody>
</mujoco>
"""
    )


HEX_M = 4.0
HEX_LEG_X = (0.18, 0.0, -0.18)


def hexapod() -> str:
    legs = []
    for i, x in enumerate(HEX_LEG_X):
        for side, sy in (("l", 1), ("r", -1)):
            n = f"leg{i}{side}"
            legs.append(f"""      <body name="{n}_upper" pos="{x} {sy * 0.1} 0">
        <joint name="{n}_swing" type="hinge" axis="0 0 1" range="-0.8 0.8" damping="0.05"/>
        <joint name="{n}_lift" type="hinge" axis="{sy} 0 0" range="-1.2 1.2" damping="0.05"/>
        <inertial pos="0 {sy * 0.06} 0" mass="0.15" diaginertia="0.0003 0.0003 0.0003"/>
        <body name="{n}_lower" pos="0 {sy * 0.12} 0">
          <joint name="{n}_knee" type="hinge" axis="{sy} 0 0" range="-0.4 1.8" damping="0.05"/>
          <inertial pos="0 0 -0.06" mass="0.1" diaginertia="0.0002 0.0002 0.00005"/>
          <geom name="{n}_foot" type="sphere" size="0.015" pos="0 0 -0.12" friction="1" mass="0"/>
        </body>
      </body>""")
    return (
        _header("hexapod")
        + f"""  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" friction="1"/>
    <body name="body" pos="0 0 0.16">
      <freejoint name="body_free"/>
      <inertial pos="0 0 0" mass="{HEX_M}" diaginertia="0.01 0.03 0.035"/>
      <geom type="box" size="0.22 0.08 0.03" mass="0"/>
{chr(10).join(legs)}
    </body>
  </worldbody>
</mujoco>
"""
    )


RIGID_MU = 0.5  # atan 0.5 = 26.57°
RIGID_SLOPES = (10.0, 20.0, 25.0, 26.0, 28.0, 32.0)
BODY1_SLOPES = (26.0, 28.0, 32.0)  # the body box at μ 1 on μ 0.5 feet: holds, slides, slides
STAND_DEPTH = 0.28  # m, hip to foot centre (the example's stance)


def rigid_quad_slope(deg: float, base_mu: float = RIGID_MU, name: str = "rigid_quad_slope") -> str:
    """The quadruped as one rigid body in its stance (legs welded), so that only the contacts can
    give: the creep test without a controller. Same mass, feet where the standing robot has them
    (under the hips), μ 0.5 on feet and slab, and on the body box unless `base_mu` says otherwise
    (`rigid_body1_quad_slope_*`: the body at MJCF's default 1, which Jolt once gave the whole
    robot, as it took one friction per rigid body). It tips over forward beyond about 33.7°, so
    the slopes stay below that."""
    pos = _on_slab(deg, STAND_DEPTH + FOOT_R + 0.0005)
    feet = []
    for leg, (sx, sy) in LEGS.items():
        x, y = sx * HIP_X, sy * HIP_Y
        feet.append(
            f'      <geom type="capsule" fromto="{x} {y} 0 {x} {y} -{STAND_DEPTH}" size="0.015" contype="0" conaffinity="0" mass="0"/>\n'
            f'      <geom name="{leg}_foot" type="sphere" size="{FOOT_R}" pos="{x} {y} -{STAND_DEPTH}" friction="{RIGID_MU}" mass="0"/>'
        )
    bx, by, bz = (s / 2 for s in BASE_SIZE)
    return (
        _header(f"{name}_{deg:g}")
        + f"""  <worldbody>
{_slab(deg, RIGID_MU)}
    <body name="base_link" pos="{pos[0]} {pos[1]} {pos[2]}" euler="0 {math.radians(deg)} 0">
      <freejoint name="base_free"/>
      <inertial pos="0 0 -0.05" mass="{QUAD_M}" diaginertia="0.2 0.4 0.4"/>
      <geom name="base" type="box" size="{bx} {by} {bz}" friction="{base_mu}" mass="0"/>
{chr(10).join(feet)}
    </body>
  </worldbody>
</mujoco>
"""
    )


QUAD_SLOPES = (0.0, 10.0, 20.0)
BOX_SLOPES = (10.0,)


def main() -> int:
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    files = {
        "quad.xml": quad(),
        "quad_drop.xml": quad(QUAD_DROP, "quad_drop"),
        "leg2.xml": leg2(),
        "ball.xml": ball(),
        "rimless.xml": rimless(),
        "hexapod.xml": hexapod(),
    }
    for d in QUAD_SLOPES:
        files[f"quad_slope_{d:g}.xml"] = quad_slope(d)
    for d in RIGID_SLOPES:
        files[f"rigid_quad_slope_{d:g}.xml"] = rigid_quad_slope(d)
    for d in BODY1_SLOPES:
        files[f"rigid_body1_quad_slope_{d:g}.xml"] = rigid_quad_slope(
            d, 1.0, "rigid_body1_quad_slope"
        )
    for d in BOX_SLOPES:
        files[f"box_slope_{d:g}.xml"] = box_slope(d)
    for name, text in files.items():
        (out / name).write_text(text, encoding="utf-8")
    print(f"wrote {len(files)} models to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

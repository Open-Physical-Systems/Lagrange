#!/usr/bin/env python3
# ruff: noqa: E501  (comparison scripts: generated XML, units in text)
"""The physics accuracy cases of docs/comparison.md, and their closed forms.

One description per case, written out as MJCF (read by Lagrange, with either backend, and by
standalone MuJoCo) and as SDFormat (read by Gazebo). PyBullet builds the same bodies through its
API in run_mujoco_pybullet.py. Every case: g = 9.81 m/s² straight down, a 1 ms step, no damping.

    scenes.py OUT_DIR     writes <case>.xml and <case>.sdf for every case
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

G = 9.81  # m/s², the same in every engine
DT = 0.001  # s

# Free fall: a 0.1 m cube of 1 kg dropped from rest at 10 m, read at 1 s (nothing to land on).
FALL_Z0 = 10.0
FALL_T = 1.0

# Pendulum: a 1 kg sphere of radius 0.02 m on a massless rod of 1 m, hinged about y.
PEND_L = 1.0
PEND_M = 1.0
PEND_R = 0.02
PEND_PIVOT_Z = 2.0
PEND_SMALL = 0.05  # rad, period case
PEND_LARGE = 1.0  # rad, energy case
PEND_PERIOD_T = 20.0  # s simulated for the period
PEND_ENERGY_T = 60.0  # s simulated for the energy drift

# Slope: a 0.2 m cube of 1 kg on a fixed slab tilted by theta, Coulomb friction mu (both surfaces).
SLOPE_MU = 0.3
SLOPE_SLIDE_DEG = 30.0  # tan 30° = 0.577 > 0.3: slides
SLOPE_STATIC_DEG = 10.0  # tan 10° = 0.176 < 0.3: stays
SLOPE_T = 2.0
CUBE = 0.2  # m, side
SLAB_Z = 5.0

# Resting box: a 0.2 m cube of 2 kg on a fixed slab, its contact force after 2 s.
REST_M = 2.0
REST_T = 2.0


def sphere_inertia() -> float:
    return 0.4 * PEND_M * PEND_R**2


def pivot_inertia() -> float:
    return sphere_inertia() + PEND_M * PEND_L**2


def ellipk(k: float) -> float:
    """The complete elliptic integral of the first kind K(k), by the arithmetic-geometric mean."""
    a, b = 1.0, math.sqrt(1.0 - k * k)
    while abs(a - b) > 1e-15:
        a, b = (a + b) / 2.0, math.sqrt(a * b)
    return math.pi / (2.0 * a)


def pendulum_period(amplitude: float) -> float:
    """Exact period of the physical pendulum at this amplitude (no small-angle approximation)."""
    return (
        4.0 * math.sqrt(pivot_inertia() / (PEND_M * G * PEND_L)) * ellipk(math.sin(amplitude / 2))
    )


def pendulum_energy(theta: float, omega: float) -> float:
    """Kinetic plus potential energy, zero at rest at the bottom (J)."""
    return 0.5 * pivot_inertia() * omega**2 + PEND_M * G * PEND_L * (1.0 - math.cos(theta))


def slope_acceleration(deg: float) -> float:
    th = math.radians(deg)
    return max(0.0, G * (math.sin(th) - SLOPE_MU * math.cos(th)))


CLOSED = {
    "freefall_z": FALL_Z0 - 0.5 * G * FALL_T**2,
    "pendulum_period": pendulum_period(PEND_SMALL),
    "slide_acceleration": slope_acceleration(SLOPE_SLIDE_DEG),
    "static_displacement": 0.0,
    "rest_force": REST_M * G,
}


@dataclass
class Case:
    name: str
    mjcf: str
    sdf: str


def _box_inertia(m: float, s: float) -> float:
    return m * s * s / 6.0


def _sdf(name: str, body: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="{name}">
    <physics name="1ms" type="ode">
      <max_step_size>{DT}</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gravity>0 0 -{G}</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
{body}
  </world>
</sdf>
"""


def _sdf_cube(name: str, pose: str, m: float, s: float, mu: float | None = None) -> str:
    i = _box_inertia(m, s)
    friction = (
        f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
        if mu is not None
        else ""
    )
    return f"""    <model name="{name}"><pose>{pose}</pose>
      <link name="{name}">
        <inertial><mass>{m}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>{s} {s} {s}</size></box></geometry>
          {friction}</collision>
      </link>
    </model>"""


def _sdf_slab(pose: str, size: str, mu: float | None) -> str:
    friction = (
        f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
        if mu is not None
        else ""
    )
    return f"""    <model name="slab"><static>true</static><pose>{pose}</pose>
      <link name="slab"><collision name="c"><geometry><box><size>{size}</size></box></geometry>
        {friction}</collision></link>
    </model>"""


def freefall() -> Case:
    h = 0.05
    mjcf = f"""<mujoco model="freefall">
  <compiler angle="radian"/>
  <option timestep="{DT}" gravity="0 0 -{G}"/>
  <worldbody>
    <body name="box" pos="0 0 {FALL_Z0}">
      <freejoint name="box_free"/>
      <geom type="box" size="{h} {h} {h}" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""
    return Case(
        "freefall", mjcf, _sdf("freefall", _sdf_cube("box", f"0 0 {FALL_Z0} 0 0 0", 1.0, 0.1))
    )


def pendulum(name: str, amplitude: float) -> Case:
    """Body frame at the pivot, turned by the amplitude; the joint starts at 0, so the angle from
    the vertical is amplitude + q."""
    i = sphere_inertia()
    mjcf = f"""<mujoco model="{name}">
  <compiler angle="radian"/>
  <option timestep="{DT}" gravity="0 0 -{G}"/>
  <worldbody>
    <body name="bob" pos="0 0 {PEND_PIVOT_Z}" axisangle="0 1 0 {amplitude}">
      <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0" damping="0"/>
      <inertial pos="0 0 -{PEND_L}" mass="{PEND_M}" diaginertia="{i} {i} {i}"/>
      <geom type="sphere" size="{PEND_R}" pos="0 0 -{PEND_L}" contype="0" conaffinity="0" mass="0"/>
    </body>
  </worldbody>
</mujoco>
"""
    body = f"""    <model name="pendulum"><pose>0 0 {PEND_PIVOT_Z} 0 {amplitude} 0</pose>
      <link name="bob">
        <inertial><pose>0 0 -{PEND_L} 0 0 0</pose><mass>{PEND_M}</mass><inertia><ixx>{i}</ixx>
          <iyy>{i}</iyy><izz>{i}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      </link>
      <joint name="hinge" type="revolute"><parent>world</parent><child>bob</child>
        <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0</damping><friction>0</friction></dynamics></axis>
      </joint>
    </model>"""
    return Case(name, mjcf, _sdf(name, body))


def slope(name: str, deg: float) -> Case:
    """A 20 m by 2 m by 0.2 m slab turned about y by deg (so +x runs downhill), and the cube on it,
    turned the same way, its face on the slab's face."""
    th = math.radians(deg)
    half = 0.1  # slab half thickness
    up = half + CUBE / 2  # cube centre above slab centre, along the slab's normal
    cx, cz = up * math.sin(th), SLAB_Z + up * math.cos(th)
    h = CUBE / 2
    mjcf = f"""<mujoco model="{name}">
  <compiler angle="radian"/>
  <option timestep="{DT}" gravity="0 0 -{G}"/>
  <worldbody>
    <geom name="slab" type="box" size="10 1 {half}" pos="0 0 {SLAB_Z}" axisangle="0 1 0 {th}"
          friction="{SLOPE_MU} 0.005 0.0001"/>
    <body name="box" pos="{cx} 0 {cz}" axisangle="0 1 0 {th}">
      <freejoint name="box_free"/>
      <geom type="box" size="{h} {h} {h}" mass="1" friction="{SLOPE_MU} 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>
"""
    body = (
        _sdf_slab(f"0 0 {SLAB_Z} 0 {th} 0", f"20 2 {2 * half}", SLOPE_MU)
        + "\n"
        + _sdf_cube("box", f"{cx} 0 {cz} 0 {th} 0", 1.0, CUBE, SLOPE_MU)
    )
    return Case(name, mjcf, _sdf(name, body))


def rest() -> Case:
    half = 0.1
    h = CUBE / 2
    z = SLAB_Z + half + h
    mjcf = f"""<mujoco model="rest">
  <compiler angle="radian"/>
  <option timestep="{DT}" gravity="0 0 -{G}"/>
  <worldbody>
    <geom name="slab" type="box" size="2 2 {half}" pos="0 0 {SLAB_Z}"/>
    <body name="box" pos="0 0 {z}">
      <freejoint name="box_free"/>
      <geom type="box" size="{h} {h} {h}" mass="{REST_M}"/>
    </body>
  </worldbody>
</mujoco>
"""
    # Gazebo reports contact forces through a contact sensor (gz-sim's Contact system): its
    # messages carry each contact point's wrench.
    i = _box_inertia(REST_M, CUBE)
    body = f"""    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
{_sdf_slab(f"0 0 {SLAB_Z} 0 0 0", f"4 4 {2 * half}", None)}
    <model name="box"><pose>0 0 {z} 0 0 0</pose>
      <link name="box">
        <inertial><mass>{REST_M}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>{CUBE} {CUBE} {CUBE}</size></box></geometry></collision>
        <sensor name="contact" type="contact"><always_on>1</always_on><update_rate>1000</update_rate>
          <contact><collision>c</collision></contact></sensor>
      </link>
    </model>"""
    return Case("rest", mjcf, _sdf("rest", body))


def cases() -> list[Case]:
    return [
        freefall(),
        pendulum("pendulum_small", PEND_SMALL),
        pendulum("pendulum_large", PEND_LARGE),
        slope("slope_slide", SLOPE_SLIDE_DEG),
        slope("slope_static", SLOPE_STATIC_DEG),
        rest(),
    ]


def main() -> int:
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for c in cases():
        (out / f"{c.name}.xml").write_text(c.mjcf, encoding="utf-8")
        (out / f"{c.name}.sdf").write_text(c.sdf, encoding="utf-8")
    for key, value in CLOSED.items():
        print(f"{key} = {value:.9g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""MuJoCo's noslip iterations (a profile key) and joint armature (MJCF, motors), end to end.

The legged contact spike (docs/plan/spikes/legged-contact.md §5, §6) measured MuJoCo's soft
contacts creeping on slopes that should hold, and both levers against it.
"""

from __future__ import annotations

import math
from pathlib import Path

import lagrange
from ament_index_python.packages import get_package_share_directory

SLOPE = math.radians(10.0)
MU = 0.3  # tan 10° = 0.176: the box should hold

PROFILE = """schema_version: 1
stage: {stage}
domains:
  rigid_body:   {{ enabled: true, backend: mujoco, dt: 0.001{extra} }}
  electrical:   {{ enabled: false, substitute: ideal }}
  thermal:      {{ enabled: false, substitute: constant }}
  aerodynamics: {{ enabled: false, substitute: zero }}
  weather:      {{ enabled: false, substitute: clear }}
  emi:          {{ enabled: false, substitute: zero }}
  radiation:    {{ enabled: false, substitute: zero }}
  sensors:      {{ enabled: false, substitute: ground_truth }}
"""


def _box_on_slope(path: Path) -> Path:
    """The comparison's 1 kg, 0.2 m box on a 10° slab of its own (the spike's box_slope_10)."""
    h = 0.05 + 0.1  # slab half thickness + box half size, along the slab's normal
    x, z = h * math.sin(SLOPE), 5.0 + h * math.cos(SLOPE)
    path.write_text(
        f"""<mujoco model="box_slope">
  <compiler angle="radian"/>
  <worldbody>
    <geom name="slab" type="box" size="6 3 0.05" pos="0 0 5" euler="0 {SLOPE} 0" friction="{MU}"/>
    <body name="box" pos="{x} 0 {z}" euler="0 {SLOPE} 0">
      <freejoint name="box_free"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.0066667 0.0066667 0.0066667"/>
      <geom type="box" size="0.1 0.1 0.1" friction="{MU}" mass="0"/>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    return path


def _creep_rate(tmp_path: Path, extra: str) -> float:
    """Down-slope speed of the box from 2 to 4 s (m/s)."""
    stage = _box_on_slope(tmp_path / "box.xml")
    profile = tmp_path / f"box{len(extra)}.yaml"
    profile.write_text(PROFILE.format(stage=stage, extra=extra), encoding="utf-8")
    sim = lagrange.Simulation(str(stage), world=None, profile=str(profile))
    down = (math.cos(SLOPE), 0.0, -math.sin(SLOPE))

    def along() -> float:
        p = sim.read("body/box/pose")
        return sum(a * b for a, b in zip(p[:3], down, strict=True))

    sim.step_until(2.0)
    x0 = along()
    sim.step_until(4.0)
    return (along() - x0) / 2.0


def test_noslip_iterations_stop_a_box_creeping_down_a_slope(tmp_path: Path) -> None:
    soft = _creep_rate(tmp_path, "")
    held = _creep_rate(tmp_path, ", noslip_iterations: 5")
    assert soft > 1e-4, f"MuJoCo's soft contact creeps ({soft * 1e3:.3f} mm/s)"
    assert abs(held) < 1e-5, f"noslip holds it ({held * 1e3:.5f} mm/s)"
    assert _creep_rate(tmp_path, ", noslip_iterations: 0") == soft, "0 is the default, off"


def _effective_inertia(robot: str, joint: str) -> float:
    sim = lagrange.Simulation(robot, world=None)
    sim.step_until(0.01)
    return sim.read(f"joint/{joint}/effective_inertia")[0]


def test_a_motors_rotor_inertia_is_its_joints_armature() -> None:
    """The powered AMR's wheel motors (rotor 0.00012 kg·m², gear 20) add J·N² = 0.048 kg·m² to
    their joints' inertia over the bare URDF's."""
    share = get_package_share_directory("lagrange_assets") + "/robots/amr_diffdrive/"
    bare = _effective_inertia(share + "amr_diffdrive.urdf", "left_wheel_joint")
    powered = _effective_inertia(share + "amr_diffdrive_powered.usda", "left_wheel_joint")
    assert math.isclose(powered - bare, 0.00012 * 20.0**2, rel_tol=1e-6, abs_tol=1e-9)


def test_an_mjcf_joints_armature_reaches_the_physics(tmp_path: Path) -> None:
    """A wheel (I = 0.02 kg·m² about its axle) with a joint armature of 0.005 and a motor's
    0.0001 kg·m² through a gear of 10: 0.02 + 0.005 + 0.0001 · 10²."""
    path = tmp_path / "wheel.xml"
    path.write_text(
        """<mujoco model="wheel">
  <worldbody>
    <geom name="floor" type="plane" size="0 0 1"/>
    <body name="wheel" pos="0 0 1">
      <joint name="axle" type="hinge" axis="0 0 1" armature="0.005"/>
      <inertial pos="0 0 0" mass="2" diaginertia="0.01 0.01 0.02"/>
    </body>
  </worldbody>
  <actuator><motor joint="axle" gear="10" armature="0.0001"/></actuator>
</mujoco>
""",
        encoding="utf-8",
    )
    assert math.isclose(_effective_inertia(str(path), "axle"), 0.035, rel_tol=1e-9)

"""Image signals from Python (sensors.md section 9 "Images of kinds"): Simulation.image."""

from __future__ import annotations

import math
from pathlib import Path

import lagrange
import numpy as np
import pytest

# A box with a wall 2 m ahead, a stereo camera and an event camera of the built-in kinds.
STEREO_BOX = """<?xml version="1.0"?>
<robot name="stereo_box">
  <link name="base_link">
    <inertial><mass value="2.0"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
  </link>
  <link name="wall">
    <inertial><mass value="50.0"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision><geometry><box size="0.1 4 2"/></geometry></collision>
    <visual><geometry><box size="0.1 4 2"/></geometry></visual>
  </link>
  <joint name="wall_joint" type="fixed">
    <parent link="base_link"/><child link="wall"/><origin xyz="2 0 0.5"/>
  </joint>
  <gazebo reference="base_link">
    <sensor name="rs" type="stereo_camera">
      <pose>0.11 0 0.05 0 0 0</pose>
      <update_rate>30</update_rate>
      <stereo_camera><width>64</width><height>48</height><horizontal_fov>1.518</horizontal_fov>
        <baseline>0.05</baseline></stereo_camera>
    </sensor>
    <sensor name="dvs" type="event_camera">
      <update_rate>100</update_rate>
      <event_camera><width>32</width><height>24</height></event_camera>
    </sensor>
  </gazebo>
</robot>
"""


@pytest.fixture(scope="module")
def sim(tmp_path_factory: pytest.TempPathFactory) -> lagrange.Simulation:
    path = Path(tmp_path_factory.mktemp("images")) / "stereo_box.urdf"
    path.write_text(STEREO_BOX)
    try:
        run = lagrange.Simulation(str(path), world=None)
    except RuntimeError as error:
        if "Vulkan" in str(error):
            pytest.skip("no Vulkan device to render the camera kinds")
        raise
    run.step_until(0.2)
    return run


def test_a_kinds_images_read_as_numpy_arrays(sim: lagrange.Simulation) -> None:
    left = sim.image("sensor/rs/left")
    assert left is not None
    assert left.shape == (48, 64, 4)
    assert left.dtype == np.uint8
    right = sim.image("sensor/rs/right")
    assert right is not None and right.shape == (48, 64, 4)
    depth = sim.image("sensor/rs/depth")
    assert depth is not None
    assert depth.shape == (48, 64)
    assert depth.dtype == np.float32
    # The wall's face is 2 - 0.05 - 0.11 = 1.84 m ahead of the lens, on the optical axis.
    centre = depth[24, 32]
    assert math.isfinite(centre)
    assert centre == pytest.approx(1.84, abs=0.05)
    events = sim.image("sensor/dvs/image")
    assert events is not None and events.shape == (24, 32, 4)


def test_a_kinds_calibration_reads_beside_its_images(sim: lagrange.Simulation) -> None:
    fx, fy, cx, cy = sim.read("sensor/rs/left/intrinsics")
    assert fx == pytest.approx(64 / 2 / math.tan(1.518 / 2))
    assert fy == fx
    assert (cx, cy) == (31.5, 23.5)
    assert sim.read("sensor/rs/left/position") == [0.0, 0.0, 0.0]
    assert sim.read("sensor/rs/right/position") == [0.05, 0.0, 0.0]


def test_images_and_numbers_are_told_apart(sim: lagrange.Simulation) -> None:
    with pytest.raises(TypeError):
        sim.read("sensor/rs/depth")
    with pytest.raises(TypeError):
        sim.image("sensor/dvs/count")
    with pytest.raises(KeyError):
        sim.image("sensor/rs/nothing")
    packed = np.asarray(sim.read("sensor/dvs/events")).reshape(-1, 4)  # x, y, t, polarity
    assert packed.shape[1] == 4


def test_a_restored_run_reads_its_images_and_repeats(tmp_path: Path) -> None:
    # QA 2026-11-12 #9: after restore_state the images read at once (not None until the next
    # frame), and the restored run hashes as the uninterrupted one (state_hash leaves the pixels
    # out, a GPU may differ by a bit in them, but not the frames' handles).
    path = tmp_path / "stereo_box.urdf"
    path.write_text(STEREO_BOX)
    try:
        run = lagrange.Simulation(str(path), world=None)
    except RuntimeError as error:
        if "Vulkan" in str(error):
            pytest.skip("no Vulkan device to render the camera kinds")
        raise
    run.step_until(0.2)
    state = run.save_state()
    saved = run.image("sensor/rs/depth")
    assert saved is not None
    run.step_until(0.5)
    first = run.state_hash()
    run.restore_state(state)
    back = run.image("sensor/rs/depth")
    assert back is not None
    assert np.array_equal(back, saved, equal_nan=True)
    run.step_until(0.5)
    assert run.state_hash() == first


def test_a_non_ascii_sensor_name_is_refused_with_a_readable_message(tmp_path: Path) -> None:
    # QA 2026-11-12 #14: the start error quoted one byte of "é" and Python raised
    # UnicodeDecodeError instead of it.
    path = tmp_path / "accent.urdf"
    path.write_text(
        """<robot name="accent"><link name="base_link"><inertial><mass value="1"/>
        <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
        <gazebo reference="base_link"><sensor name="caméra" type="imu">
        <update_rate>100</update_rate></sensor></gazebo></robot>""",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="'é' not allowed"):
        lagrange.Simulation(str(path), world=None)

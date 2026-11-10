# SPDX-License-Identifier: Apache-2.0
"""lagrange foam-cht (RFC 0012): the conjugate heat transfer case it writes from a URDF's thermal
materials and heat sources, and the temperatures it reads back. OpenFOAM itself is not needed."""

import importlib.machinery
import importlib.util
import json
import subprocess
from pathlib import Path

CLI = Path(__file__).resolve().parents[1]
ROBOT = """<robot name="lamp">
<link name="sink"><inertial><mass value="0.04"/>
<inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size="0.06 0.04 0.004"/></geometry></collision></link>
<link name="board"><inertial><mass value="0.005"/>
<inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size="0.05 0.03 0.0016"/></geometry></collision></link>
<joint name="mount" type="fixed"><parent link="sink"/><child link="board"/>
<origin xyz="0 0 0.0028"/></joint>
<gazebo reference="sink"><thermal_material preset="aluminium_6061" emissivity="0.85"/></gazebo>
<gazebo reference="board">
  <thermal_material preset="fr4" conductivity="20" conductivity_along="0.3"
                    contact_conductance="2000"/>
  <heat_source name="led" power="2.0" efficiency="0.35" xyz="0.01 0 0.0008"
               size="0.003 0.003 0.001"/>
</gazebo>
</robot>"""


def tool():
    loader = importlib.machinery.SourceFileLoader(
        "lagrange_foam_cht", str(CLI / "lagrange_foam_cht")
    )
    spec = importlib.util.spec_from_loader("lagrange_foam_cht", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def write_case(tmp_path: Path) -> Path:
    robot = tmp_path / "lamp.urdf"
    robot.write_text(ROBOT)
    case = tmp_path / "case"
    command = [str(CLI / "lagrange"), "foam-cht", str(robot), "--ambient", "298.15",
               "--out", str(case), "--write-only"]  # fmt: skip
    subprocess.run(command, check=True, capture_output=True, timeout=120)
    return case


def test_a_case_has_a_region_per_material_and_the_air(tmp_path: Path) -> None:
    case = write_case(tmp_path)
    regions = (case / "constant/regionProperties").read_text()
    assert "fluid (air)" in regions and "solid (sink board)" in regions
    for name in ("system/blockMeshDict", "system/snappyHexMeshDict", "system/controlDict",
                 "constant/air/thermophysicalProperties", "constant/sink/thermophysicalProperties",
                 "constant/board/thermophysicalProperties", "0.regions/air/T", "0.regions/air/U",
                 "0.regions/air/p_rgh", "0.regions/sink/T", "0.regions/board/T",
                 "constant/triSurface/sink.stl", "constant/triSurface/board.stl",
                 "Allrun"):  # fmt: skip
        assert (case / name).exists(), name
    # The preset's numbers and the board's two conductivities.
    sink = (case / "constant/sink/thermophysicalProperties").read_text()
    assert "kappa 167;" in sink and "Cp 896;" in sink and "rho 2700;" in sink
    board = (case / "constant/board/thermophysicalProperties").read_text()
    assert "constAnIso" in board and "kappa (20 20 0.3)" in board
    # Each solid's point is inside its own shape.
    snappy = (case / "system/snappyHexMeshDict").read_text()
    assert "sink)" in snappy and "board)" in snappy and "air)" in snappy


def test_the_led_heats_its_box_and_the_contact_is_a_layer(tmp_path: Path) -> None:
    case = write_case(tmp_path)
    options = (case / "system/board/fvOptions").read_text()
    assert "scalarSemiImplicitSource" in options
    assert "h (1.3 0)" in options  # 2 W less the 35 % that leaves as light
    assert "rotatedBoxToCell" in (case / "system/board/topoSetDict").read_text()
    board_t = (case / "0.regions/board/T").read_text()
    # 2000 W/(m²·K) as a 0.1 mm layer of 0.2 W/(m·K), on the child's side only.
    assert "kappaLayers (0.2)" in board_t and "thicknessLayers (0.0001)" in board_t
    assert "kappaLayers" not in (case / "0.regions/sink/T").read_text()


def test_it_reads_the_last_temperatures_back(tmp_path: Path) -> None:
    case = write_case(tmp_path)
    post = case / "postProcessing"
    for region, name, value in (("sink", "sink_volAverage", 310.5), ("sink", "sink_max", 311.0),
                                ("sink", "sink_min", 310.0), ("board", "board_volAverage", 320.0),
                                ("board", "board_max", 340.0), ("board", "board_min", 315.0),
                                ("board", "led_volAverage", 338.0)):  # fmt: skip
        path = post / region / name / "0" / "volFieldValue.dat"
        path.parent.mkdir(parents=True)
        path.write_text(f"# Time volAverage(T)\n1 {value - 5}\n2000 {value}\n")
    thermal = json.loads((case / "thermal.json").read_text())
    result = tool().read_results(case, thermal["parts"])
    assert result["parts"]["sink"] == {"mean": 310.5, "max": 311.0, "min": 310.0}
    assert result["parts"]["board"]["max"] == 340.0
    assert result["sources"]["led"] == 338.0

# SPDX-License-Identifier: Apache-2.0
"""lagrange foam: the case it writes and the forces it reads. OpenFOAM itself is not needed here."""

import importlib.machinery
import importlib.util
import json
import subprocess
from pathlib import Path

CLI = Path(__file__).resolve().parents[1]
ROBOT = """<robot name="b"><link name="base link"><inertial><mass value="1"/>
<inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size="1.0 0.4 0.2"/></geometry></collision></link></robot>"""


def foam():
    loader = importlib.machinery.SourceFileLoader("lagrange_foam", str(CLI / "lagrange_foam"))
    spec = importlib.util.spec_from_loader("lagrange_foam", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def write_case(tmp_path: Path, *extra: str) -> Path:
    robot = tmp_path / "box.urdf"
    robot.write_text(ROBOT)
    case = tmp_path / "case"
    command = [str(CLI / "lagrange"), "foam", str(robot), "--out", str(case), "--write-only"]
    command += ["--speed", "30", *extra]
    subprocess.run(command, check=True, capture_output=True, timeout=120)
    return case


def test_a_case_has_what_openfoam_reads(tmp_path: Path) -> None:
    case = write_case(tmp_path, "--ground", "--clearance", "0.1")
    for name in ("system/blockMeshDict", "system/snappyHexMeshDict", "system/controlDict",
                 "system/fvSchemes", "system/fvSolution", "0/U", "0/p", "0/k", "0/omega", "0/nut",
                 "constant/transportProperties", "constant/turbulenceProperties",
                 "constant/triSurface/base_link.stl", "Allrun"):  # fmt: skip
        assert (case / name).exists(), name
    assert "magUInf 30.0" in (case / "system/controlDict").read_text()
    assert "movingWallVelocity" in (case / "0/U").read_text()
    plan = json.loads((case / "plan.json").read_text())
    assert abs(plan["area"] - 0.08) < 0.002  # the box's 0.4 by 0.2 end
    assert abs(plan["low"][2] - plan["box_low"][2] - 0.1) < 1e-9  # the road, as asked


def test_with_no_ground_the_floor_slips_far_below(tmp_path: Path) -> None:
    case = write_case(tmp_path)
    assert "movingWallVelocity" not in (case / "0/U").read_text()
    plan = json.loads((case / "plan.json").read_text())
    assert plan["low"][2] - plan["box_low"][2] > plan["size"]


def test_a_case_is_not_written_over(tmp_path: Path) -> None:
    case = tmp_path / "case"
    case.mkdir()
    (case / "mine").write_text("keep")
    robot = tmp_path / "box.urdf"
    robot.write_text(ROBOT)
    result = subprocess.run([str(CLI / "lagrange"), "foam", str(robot), "--out", str(case)],
                            capture_output=True, text=True, timeout=60)  # fmt: skip
    assert result.returncode != 0
    assert (case / "mine").read_text() == "keep"


def test_forces_are_the_mean_of_the_last_fifth(tmp_path: Path) -> None:
    folder = tmp_path / "postProcessing/forceCoeffs1/0"
    folder.mkdir(parents=True)
    rows = "\n".join(
        f"{i} 9 {1.0 if i <= 80 else 0.5 + (i % 2) * 0.1} 7 {2.0}" for i in range(1, 101)
    )
    (folder / "coefficient.dat").write_text("# Time Cm Cd Cs Cl\n" + rows + "\n")
    result = foam().read_coefficients(tmp_path)
    assert abs(result["drag_coefficient"] - 0.55) < 1e-9
    assert abs(result["drag_moves"] - 0.05) < 1e-9
    assert result["lift_coefficient"] == 2.0
    assert result["iterations"] == 100


def test_the_comparison_reads_both_formats(tmp_path: Path) -> None:
    loader = importlib.machinery.SourceFileLoader(
        "lagrange_foam_compare", str(CLI / "lagrange_foam_compare")
    )
    spec = importlib.util.spec_from_loader("lagrange_foam_compare", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    field = tmp_path / "p"
    field.write_text("internalField nonuniform List<scalar> 2\n(\n1.5\n-2\n)\n;\n")
    assert module.read_foam_list(field, vector=False).tolist() == [1.5, -2.0]
    field.write_text("internalField nonuniform List<vector> 2\n(\n(1 2 3)\n(4 5 6)\n)\n;\n")
    assert module.read_foam_list(field, vector=True).tolist() == [[1, 2, 3], [4, 5, 6]]


def test_link_forces_are_read_per_link(tmp_path: Path) -> None:
    folder = tmp_path / "postProcessing/forces_base_link/0"
    folder.mkdir(parents=True)
    rows = "\n".join(f"{i} (10 0 2) (8 0 2) (2 0 0)" for i in range(1, 11))
    (folder / "force.dat").write_text(
        "# Time total_x total_y total_z pressure_x pressure_y pressure_z"
        " viscous_x viscous_y viscous_z\n" + rows + "\n"
    )
    links = foam().read_link_forces(tmp_path)
    assert links == [{"name": "base_link", "total_n": [10.0, 0.0, 2.0],
                      "pressure_n": [8.0, 0.0, 2.0], "viscous_n": [2.0, 0.0, 0.0]}]  # fmt: skip


def test_a_case_asks_for_each_links_force(tmp_path: Path) -> None:
    case = write_case(tmp_path)
    assert "forces_base_link" in (case / "system/controlDict").read_text()


def test_patch_values_are_read_by_name(tmp_path: Path) -> None:
    field = tmp_path / "C"
    field.write_text(
        "internalField nonuniform List<vector> 1\n(\n(0 0 0)\n)\n;\nboundaryField\n{\n"
        "    inlet\n    {\n        type calculated;\n        value uniform (0 0 0);\n    }\n"
        "    body_wing\n    {\n        type calculated;\n"
        "        value nonuniform List<vector> 2\n(\n(1 2 3)\n(4 5 6)\n)\n;\n    }\n}\n"
    )
    internal, patches = foam().read_field(field, vector=True)
    assert internal == [[0.0, 0.0, 0.0]]
    assert patches == {"body_wing": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]}

# SPDX-License-Identifier: Apache-2.0
"""lagrange foam-hover: the hover case it writes. OpenFOAM itself is not needed here."""

import json
import subprocess
from pathlib import Path

CLI = Path(__file__).resolve().parents[1]
QUAD = CLI.parents[1] / "assets/robots/quad_x500/quad_x500.urdf"


def write_case(tmp_path: Path, *extra: str) -> Path:
    case = tmp_path / "case"
    command = [str(CLI / "lagrange"), "foam-hover", str(QUAD), "--out", str(case),
               "--write-only", *extra]  # fmt: skip
    subprocess.run(command, check=True, capture_output=True, timeout=120)
    return case


def test_the_rotors_are_discs_carrying_the_weight(tmp_path: Path) -> None:
    case = write_case(tmp_path, "--height", "1.5")
    plan = json.loads((case / "plan.json").read_text())
    assert len(plan["discs"]) == 4
    weight = plan["mass"] * 9.81
    assert abs(sum(d["thrust"] for d in plan["discs"]) - weight) < 1e-9
    for d in plan["discs"]:
        assert abs(d["hub"][2] - 1.5) < 1e-9  # the rotor plane where asked
        assert abs(d["radius"] - 0.127) < 1e-9
    options = (case / "system/fvOptions").read_text()
    assert options.count("vectorSemiImplicitSource") == 4
    # Down on the air, by the thrust over the density (the kinematic momentum equation).
    assert f"(0 0 {-plan['discs'][0]['thrust'] / 1.2:.9g})" in options
    assert "cylinderToCell" in (case / "system/topoSetDict").read_text()
    assert "totalPressure" in (case / "0/p").read_text()
    assert not (case / "constant/triSurface/building.stl").exists()


def test_a_building_beside_it(tmp_path: Path) -> None:
    case = write_case(tmp_path, "--building", "0.6,-3,3.6,3,12")
    plan = json.loads((case / "plan.json").read_text())
    assert plan["building"] == [0.6, -3.0, 0.0, 3.6, 3.0, 12.0]
    assert plan["domain"][1][2] >= 13.0  # open air over the roof
    assert "building" in (case / "system/snappyHexMeshDict").read_text()
    assert (case / "constant/triSurface/building.stl").exists()

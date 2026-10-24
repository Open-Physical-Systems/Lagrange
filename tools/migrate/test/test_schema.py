"""migrate_schema: the synthetic ExampleMotorAPI_0 -> _1 migration round-trips a file, and the
sample assets are up to date (formats.md §10)."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
from lagrange_migrate import schema
from pxr import Sdf

REPO = Path(os.environ.get("LAGRANGE_REPO", Path(__file__).resolve().parents[3]))

OLD = """#usda 1.0
def Xform "robot"
{
    def Xform "left_motor" (
        prepend apiSchemas = ["ExampleMotorAPI_0", "PortAPI_1:dc_in"]
    )
    {
        float example:torqueConstantMilli = 85
        float example:torqueConstantMilli.timeSamples = {
            0: 85,
            10: 90,
        }
        bool example:legacyFlag = 1
        float example:resistance = 0.12
    }
    def Xform "bus" ( prepend apiSchemas = ["ElectricalBusAPI_1"] )
    {
        double elec:bus:capacitance = 0.0047
    }
}
"""


def write(tmp_path: Path, text: str, name: str = "old.usda") -> Path:
    path = tmp_path / name
    path.write_text(text)
    return path


def test_the_synthetic_migration_rewrites_attributes_and_the_schema(tmp_path: Path) -> None:
    old = write(tmp_path, OLD)
    new = tmp_path / "new.usda"
    assert schema.main([str(old), "-o", str(new)]) == 0

    layer = Sdf.Layer.FindOrOpen(str(new))
    motor = layer.GetPrimAtPath("/robot/left_motor")
    assert list(motor.GetInfo("apiSchemas").prependedItems) == [
        "ExampleMotorAPI_1",
        "PortAPI_1:dc_in",
    ]
    kt = motor.properties["example:torqueConstant"]
    assert kt.default == pytest.approx(0.085)
    assert layer.QueryTimeSample(kt.path, 10) == pytest.approx(0.090)
    assert "example:torqueConstantMilli" not in motor.properties
    assert "example:legacyFlag" not in motor.properties
    assert motor.properties["example:resistance"].default == pytest.approx(0.12)
    # Prims without an old schema are left alone.
    bus = layer.GetPrimAtPath("/robot/bus")
    assert list(bus.GetInfo("apiSchemas").prependedItems) == ["ElectricalBusAPI_1"]


def test_a_migrated_file_is_up_to_date(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    old = write(tmp_path, OLD)
    assert schema.main([str(old), "--check"]) == 1
    assert "ExampleMotorAPI_0 -> ExampleMotorAPI_1" in capsys.readouterr().out
    assert schema.main([str(old), "--in-place"]) == 0
    assert (tmp_path / "old.usda.bak").read_text() == OLD
    assert schema.main([str(old), "--check"]) == 0


def test_two_versions_of_a_family_on_one_prim_are_refused(tmp_path: Path) -> None:
    both = write(
        tmp_path,
        OLD.replace('"ExampleMotorAPI_0",', '"ExampleMotorAPI_0", "ExampleMotorAPI_1",'),
    )
    assert schema.main([str(both), "--check"]) == 2


def test_every_family_migrates_one_version_at_a_time() -> None:
    # A chain without gaps from each registered start.
    for family, version in schema.MIGRATIONS:
        assert schema.newest(family, version) > version
        assert all(
            (family, v) in schema.MIGRATIONS for v in range(version, schema.newest(family, version))
        )


@pytest.mark.parametrize(
    "asset",
    sorted(str(p.relative_to(REPO)) for p in (REPO / "assets").rglob("*.usda")),
)
def test_the_sample_assets_are_up_to_date(asset: str) -> None:
    assert schema.main([str(REPO / asset), "--check"]) == 0

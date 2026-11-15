# SPDX-License-Identifier: Apache-2.0
"""tools/ci/check_sensor_catalog.py on the installed tree's sources: the catalog against
sensors.md §9, both ways (RFC 0016 §5)."""

import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools/ci/check_sensor_catalog.py"


def test_catalog_agrees_with_sensors_md() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_a_wrong_default_is_caught(tmp_path: Path) -> None:
    catalog = yaml.safe_load(
        (ROOT / "modules/sensors/catalog/sensor_catalog.yaml").read_text(encoding="utf-8")
    )
    rangefinder = {p["path"]: p for p in catalog["kinds"]["rangefinder"]["params"]}
    rangefinder["range/min"]["default"] = 0.7
    path = tmp_path / "catalog.yaml"
    path.write_text(yaml.safe_dump(catalog, allow_unicode=True), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--catalog", str(path)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "rangefinder: `range/min` default 0.7" in result.stderr

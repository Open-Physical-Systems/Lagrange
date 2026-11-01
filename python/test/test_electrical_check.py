"""The electrical hand check passes, and fails when a formula is wrong."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "tools/ci/check_electrical.py"

needs_ngspice = pytest.mark.skipif(shutil.which("ngspice") is None, reason="needs ngspice")


@needs_ngspice
def test_simple_circuits_agree() -> None:
    done = subprocess.run(
        [sys.executable, str(SCRIPT), "--rounds", "2"], capture_output=True, text=True, check=False
    )
    assert done.returncode == 0, done.stdout + done.stderr
    assert "PASSED" in done.stdout


@needs_ngspice
def test_a_wrong_formula_is_caught(tmp_path: Path) -> None:
    wrong = tmp_path / "wrong.py"
    wrong.write_text(
        SCRIPT.read_text().replace(
            "volts = pack.ocv * load / (load + pack.r0)",
            "volts = 1.01 * pack.ocv * load / (load + pack.r0)",
        )
    )
    done = subprocess.run(
        [sys.executable, str(wrong), "--rounds", "1"], capture_output=True, text=True, check=False
    )
    assert done.returncode == 1, done.stdout + done.stderr
    assert "FAIL" in done.stdout

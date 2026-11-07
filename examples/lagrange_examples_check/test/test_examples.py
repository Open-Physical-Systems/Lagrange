"""Every example runs through its launch file and logs its result."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[3] / "tools/ci/check_examples.py"


def test_every_example_gets_its_result() -> None:
    done = subprocess.run(
        [sys.executable, str(SCRIPT)], capture_output=True, text=True, check=False
    )
    assert done.returncode == 0, done.stdout

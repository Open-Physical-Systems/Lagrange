"""migrate_recording: a recording reports its format version; newer ones are refused."""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
from lagrange_migrate import recording


def test_format_version_is_read_from_run_meta() -> None:
    # RunMeta { format_version: 1, profile_yaml: "x" }
    assert recording.format_version(bytes([0x08, 0x01, 0x12, 0x01, ord("x")])) == 1
    assert recording.format_version(bytes([0x12, 0x00])) == 0
    assert recording.format_version(bytes([0x08, 0x02])) == 2


def test_a_sample_recording_is_up_to_date(tmp_path: Path) -> None:
    sample = tmp_path / "sample.mcap"
    written = subprocess.run(
        [
            "ros2",
            "run",
            "lagrange_mock_server",
            "lagrange_mock_server",
            "--write-sample",
            str(sample),
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=120,
    )
    if written.returncode != 0 or not sample.exists():
        pytest.skip(f"the mock server could not write a sample: {written.stderr[-300:]}")
    assert recording.version_of(sample) == recording.CURRENT_FORMAT
    assert recording.main(["--check", str(sample)]) == 0


def test_a_file_that_is_not_a_recording_is_refused(tmp_path: Path) -> None:
    other = tmp_path / "other.mcap"
    other.write_bytes(b"not mcap")
    assert recording.main(["--check", str(other)]) == 2

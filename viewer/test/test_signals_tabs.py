"""The Signals workspace's second tabs (docs/design/ui/05-signals): Measure, FFT and Saved sets,
pressed as a user does (--input) on the sample AMR driving its demo: a wheel velocity (1 kHz)
and a motor current (10 kHz)."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

# Where things are at --size 1600x1000: the tabs of the left and right docks.
MEASURE_TAB = "1381,115"
FFT_TAB = "1377,447"
SAVED_SETS_TAB = "98,115"
SET_NAME = "110,144"
SAVE_PLOTTED = "260,144"
FIRST_SET_PLOT = "216,178"

VELOCITY = "joint/left_wheel_joint/velocity"
CURRENT = "motor/left_drive_motor/current_q"


def tool(package: str, name: str) -> str:
    return f"{get_package_prefix(package)}/lib/{package}/{name}"


@pytest.fixture(scope="module")
def server():
    try:
        get_package_prefix("lagrange_server")
    except PackageNotFoundError:
        pytest.skip("needs lagrange_server")
    socket = f"/tmp/lagrange_signals_{os.getpid()}.sock"
    process = subprocess.Popen(
        [tool("lagrange_server", "lagrange_server"), "--no-ros", "--listen", f"unix:{socket}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(300):
        if Path(socket).exists():
            break
        time.sleep(0.1)
    yield socket
    process.send_signal(signal.SIGTERM)
    process.wait(timeout=20)
    Path(socket).unlink(missing_ok=True)


@pytest.fixture
def run_view(server, tmp_path):
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}

    def run(scope: list[str], script: str, frames: int = 150) -> dict:
        done = subprocess.run(
            [
                tool("lagrange_viewer", "lagrange_viewer"),
                "--headless",
                "--gpu",
                os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
                "--connect",
                f"unix:{server}",
                "--size",
                "1600x1000",
                "--frames",
                str(frames),
                "--workspace",
                "signals",
                "--cursors",
                "--scope",
                ",".join(scope),
                "--input",
                script,
            ],
            capture_output=True,
            text=True,
            timeout=150,
            check=False,
            env=environment,
        )
        assert done.returncode == 0, done.stderr
        report = json.loads(done.stdout.strip().splitlines()[-1])
        return report

    return run


def test_measure_gives_the_numbers_between_the_cursors(run_view) -> None:
    measure = run_view([VELOCITY, CURRENT], f"30:click:{MEASURE_TAB}")["signals"]["measure"]
    assert set(measure) == {VELOCITY, CURRENT}
    for label, m in measure.items():
        assert m["seconds"] == pytest.approx(3.8, abs=1e-3), label  # the cursors' default places
        assert m["count"] > 0, label
        assert m["min"] <= m["mean"] <= m["max"], label
        assert m["rms"] >= abs(m["mean"]) - 1e-9, label
    # The raw samples, not the decimated points: ten times as many at 10 kHz as at 1 kHz.
    # (The streams start when the viewer subscribes: A may be before the first sample.)
    assert measure[CURRENT]["count"] == pytest.approx(10 * measure[VELOCITY]["count"], rel=0.05)
    assert measure[VELOCITY]["count"] <= 3801


def test_fft_is_the_spectrum_of_the_raw_samples(run_view) -> None:
    for first, dt in ((CURRENT, 1e-4), (VELOCITY, 1e-3)):
        fft = run_view([first, VELOCITY if first == CURRENT else CURRENT], f"30:click:{FFT_TAB}")[
            "signals"
        ]["fft"]
        assert fft["trace"] == first  # the first plotted, until another is chosen
        n = fft["samples"]
        assert n >= 1024 and n & (n - 1) == 0, n
        assert fft["bin_hz"] == pytest.approx(1.0 / (n * dt), rel=1e-6), first
        assert 0.0 < fft["peak_hz"] <= 0.5 / dt, first
        assert fft["peak_amplitude"] >= 0.0


def test_a_saved_set_is_kept_and_plotted_again(run_view, tmp_path) -> None:
    saved = tmp_path / "cfg/lagrange/signal_sets.tsv"
    script = (
        f"30:click:{SAVED_SETS_TAB};45:click:{SET_NAME};50:key:ctrl+a;52:text:drive train;"
        f"54:key:enter;65:click:{SAVE_PLOTTED}"
    )
    report = run_view([VELOCITY, CURRENT], script, 100)
    assert report["signals"]["saved_sets"] == 1
    lines = saved.read_text().splitlines()
    assert lines == [f"drive train\t{VELOCITY}\t0", f"drive train\t{CURRENT}\t0"]
    # Another viewer, another signal plotted: the set comes back in its place.
    report = run_view([VELOCITY], f"30:click:{SAVED_SETS_TAB};50:click:{FIRST_SET_PLOT}", 90)
    assert report["signals"]["plotted_from_set"] == 2
    assert set(report["builder"]["scope_values"]) == {VELOCITY, CURRENT}

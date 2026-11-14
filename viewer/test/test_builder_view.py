"""The Builder view, headless, without a server (RFC 0016 WP-D).

The view draws the builder's own document (a new one: base_link, a 0.3 x 0.2 x 0.1 m box) on
the Lit pipeline with the builder's camera, and a click on the box selects base_link, drawn in
accent. Run on the software Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names another.
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import zlib
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

# The Builder view's middle at 1600x1000 with the default layout (12-builder SPEC: left 270 px,
# right 350 px, bottom 220 px), away from its overlay boxes and tool bars.
VIEW = (340, 200, 1060, 680)
BOX_CENTRE = (760, 450)


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")


def read_png(path: Path):
    """Decode an 8-bit RGBA PNG without filtering (what the viewer writes)."""
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    offset, width, height, idat = 8, 0, 0, b""
    while offset < len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        kind = data[offset + 4 : offset + 8]
        body = data[offset + 8 : offset + 8 + length]
        if kind == b"IHDR":
            width, height, depth, colour = struct.unpack(">IIBB", body[:10])
            assert (depth, colour) == (8, 6)
        elif kind == b"IDAT":
            idat += body
        offset += 12 + length
    raw = zlib.decompress(idat)
    stride = width * 4 + 1
    return [raw[y * stride + 1 : (y + 1) * stride] for y in range(height)]


def pixels(rows, test) -> int:
    x0, y0, x1, y1 = VIEW
    found = 0
    for y in range(y0, y1):
        row = rows[y]
        for x in range(x0, x1):
            r, g, b = row[4 * x], row[4 * x + 1], row[4 * x + 2]
            found += 1 if test(r, g, b) else 0
    return found


def lit(r, g, b) -> bool:
    return r + g + b > 300  # the box's lit faces; the background and grid are dark


def accent(r, g, b) -> bool:
    return r > 180 and 80 < g < 160 and b < 90  # the accent #E8772E


def shoot(tmp_path, name: str, *extra: str) -> list:
    shot = tmp_path / f"{name}.png"
    environment = {**os.environ, "HOME": str(tmp_path), "XDG_CONFIG_HOME": str(tmp_path / "cfg")}
    done = subprocess.run(
        [
            viewer(),
            "--headless",
            "--gpu",
            os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
            "--size",
            "1600x1000",
            "--frames",
            "10",
            "--workspace",
            "builder",
            "--screenshot",
            str(shot),
            *extra,
        ],
        capture_output=True,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    assert lines and json.loads(lines[-1]).get("workspace") == "Builder"
    return read_png(shot)


def test_builder_view_draws_the_new_documents_box(tmp_path):
    rows = shoot(tmp_path, "builder_view")
    # The box fills part of the middle of the view: thousands of lit pixels.
    assert pixels(rows, lit) > 8000


def test_a_click_on_the_box_selects_base_link_in_accent(tmp_path):
    before = pixels(shoot(tmp_path, "before"), accent)
    x, y = BOX_CENTRE
    after = pixels(shoot(tmp_path, "after", "--input", f"3:click:{x},{y}"), accent)
    # The outline, the marker and the label "base_link" are drawn in accent.
    assert after > before + 200, (before, after)

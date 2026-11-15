"""The launcher (docs/design/ui/01-launcher), pressed as a user does (--input): its four menus,
the links of the left column, the Recent table and the robot-class cards."""

from __future__ import annotations

import json
import os
import stat
import subprocess
from pathlib import Path
from typing import Any

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_prefix

# Where things are at --size 1600x1000.
FILE_MENU = "102,14"
VIEW_MENU = "137,14"
TOOLS_MENU = "177,14"
HELP_MENU = "216,14"
# Menu items, one row every 17 px from y = 43 (separators add 5 px).
FILE_BUILD = "120,99"
FILE_RECENT = "150,121"
FILE_RECENT_FIRST = "300,121"
VIEW_DRONE = "150,77"
VIEW_BUILDER = "150,201"
TOOLS_CONFIG = "200,99"
HELP_ITEMS = ["250,43", "250,60", "250,77"]
LINKS = ["60,917", "60,935", "60,953"]
CARDS = {"amr": (500, 330), "drone": (887, 330), "huge_world": (693, 520), "bench": (887, 520)}
CONNECT = "1400,353"
RECENT_ROWS = [703, 726, 749, 772, 795]

SITE = "https://open-physical-systems.github.io/Lagrange/"
PAGES = [
    SITE,
    SITE + "guide/cli.html#importing-a-robot",
    "https://github.com/Open-Physical-Systems/Lagrange/issues/new?template=bug.yml",
]


def viewer() -> str:
    try:
        return f"{get_package_prefix('lagrange_viewer')}/lib/lagrange_viewer/lagrange_viewer"
    except PackageNotFoundError:
        pytest.skip("needs lagrange_viewer installed")
        raise


@pytest.fixture
def launcher(tmp_path: Path) -> Any:
    """Runs a headless viewer on the launcher and gives (stats, stderr).

    lagrange_server is a stand-in that only sleeps, found first on AMENT_PREFIX_PATH, so that
    starting a sample is seen in what the viewer prints without running a simulation. The default
    socket is in a folder of the test's own, so a server running on this machine is not seen.
    """
    fake = tmp_path / "prefix" / "lib" / "lagrange_server" / "lagrange_server"
    fake.parent.mkdir(parents=True)
    fake.write_text("#!/bin/sh\nexec sleep 30\n")
    fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    (tmp_path / "run").mkdir(mode=0o700)
    config = tmp_path / "cfg" / "lagrange"
    config.mkdir(parents=True)
    recent = [
        {
            "stage": f"/data/{name}.usda",
            "robot": robot,
            "profile": "applied",
            "endpoint": f"unix:{tmp_path}/{name}.sock",
            "opened": "2026-10-09 10:00",
        }
        for name, robot in [("warehouse_a", "amr_diffdrive"), ("open_field", "quad_x500")]
    ]
    (config / "recent.json").write_text(json.dumps(recent))
    environment = {
        **os.environ,
        "HOME": str(tmp_path),
        "XDG_CONFIG_HOME": str(tmp_path / "cfg"),
        "XDG_RUNTIME_DIR": str(tmp_path / "run"),
        "AMENT_PREFIX_PATH": f"{tmp_path / 'prefix'}:{os.environ.get('AMENT_PREFIX_PATH', '')}",
    }

    def run(
        script: str, frames: int = 24, screenshot: Path | None = None
    ) -> tuple[dict[str, Any], str]:
        extra = ["--screenshot", str(screenshot)] if screenshot else []
        done = subprocess.run(
            [
                viewer(),
                "--headless",
                "--gpu",
                os.environ.get("LAGRANGE_TEST_GPU", "llvmpipe"),
                "--launcher",
                "--size",
                "1600x1000",
                "--frames",
                str(frames),
                "--input",
                script,
                *extra,
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
            env=environment,
        )
        assert done.returncode == 0, done.stderr
        return json.loads(done.stdout.strip().splitlines()[-1]), done.stderr

    run.tmp = tmp_path  # type: ignore[attr-defined]
    return run


def double_click(frame: int, at: str) -> str:
    return f"{frame}:click:{at};{frame + 3}:click:{at}"


def test_help_menu_opens_each_page_outside_the_viewer(launcher: Any) -> None:
    for item, page in zip(HELP_ITEMS, PAGES, strict=True):
        stats, stderr = launcher(f"4:click:{HELP_MENU};8:click:{item}")
        assert f"would open {page}\n" in stderr
        assert stats["launcher"] is True  # a link never leaves the launcher


def test_left_column_links_open_each_page(launcher: Any) -> None:
    for at, page in zip(LINKS, PAGES, strict=True):
        _stats, stderr = launcher(f"4:click:{at}")
        assert f"would open {page}\n" in stderr


def test_tools_menu_opens_the_configuration_folder(launcher: Any) -> None:
    _stats, stderr = launcher(f"4:click:{TOOLS_MENU};8:click:{TOOLS_CONFIG}")
    assert f"would open {launcher.tmp / 'cfg' / 'lagrange'}\n" in stderr


@pytest.mark.parametrize("menu,item", [(FILE_MENU, FILE_BUILD), (VIEW_MENU, VIEW_BUILDER)])
def test_file_and_view_menus_open_the_builder_without_a_server(
    launcher: Any, menu: str, item: str
) -> None:
    stats, stderr = launcher(f"4:click:{menu};8:click:{item}")
    assert stats["launcher"] is False and stats["workspace"] == "Builder"
    assert stats["connected"] is False and "connecting to" not in stderr


def test_view_menu_picks_a_robot_class_that_connect_starts(launcher: Any) -> None:
    _stats, stderr = launcher(f"4:click:{VIEW_MENU};8:click:{VIEW_DRONE};14:click:{CONNECT}")
    assert "quad_x500/quad_x500.usda" in stderr and "--air-density 1.225" in stderr


def test_file_menu_reopens_a_recent_entry(launcher: Any) -> None:
    stats, stderr = launcher(
        f"4:click:{FILE_MENU};8:move:{FILE_RECENT};12:click:{FILE_RECENT_FIRST}", frames=30
    )
    assert f"connecting to unix:{launcher.tmp}/warehouse_a.sock" in stderr
    assert stats["launcher"] is False


def test_recent_row_is_picked_by_a_click_and_reopened_by_a_double_click(launcher: Any) -> None:
    second = f"450,{RECENT_ROWS[1]}"
    stats, stderr = launcher(f"4:click:{second}")
    assert stats["launcher"] is True and "connecting to" not in stderr
    stats, stderr = launcher(double_click(4, second))
    assert f"connecting to unix:{launcher.tmp}/open_field.sock" in stderr
    assert stats["launcher"] is False


def test_recent_filter_keeps_the_matching_rows(launcher: Any) -> None:
    # The filter field, then "quad": the first row left is open_field's.
    first = f"450,{RECENT_ROWS[0]}"
    _stats, stderr = launcher(f"4:click:1060,647;8:text:quad;{double_click(12, first)}")
    assert f"connecting to unix:{launcher.tmp}/open_field.sock" in stderr


@pytest.mark.parametrize(
    "card,expected",
    [
        ("drone", ["quad_x500/quad_x500.usda", "open_field/open_field.usda"]),
        ("huge_world", ["tiled_town/tiled_town.usda", "--spawn 1036,1036"]),
    ],
)
def test_double_clicking_a_card_starts_its_sample(
    launcher: Any, card: str, expected: list[str]
) -> None:
    x, y = CARDS[card]
    stats, stderr = launcher(double_click(4, f"{x},{y}"))
    started = [line for line in stderr.splitlines() if "lagrange_viewer: starting" in line]
    assert len(started) == 1, stderr
    assert "--listen unix:" in started[0]
    for word in expected:
        assert word in started[0]
    assert stats["launcher"] is False


def test_a_card_click_selects_and_connect_starts_it(launcher: Any, tmp_path: Path) -> None:
    x, y = CARDS["huge_world"]
    image = tmp_path / "picked.png"
    _stats, stderr = launcher(f"4:click:{x},{y}", screenshot=image)
    assert "starting" not in stderr  # one click only picks it
    _stats, stderr = launcher(f"4:click:{x},{y};10:click:{CONNECT}")
    assert "tiled_town/tiled_town.usda" in stderr


def test_the_default_card_starts_the_server_as_it_is(launcher: Any) -> None:
    x, y = CARDS["amr"]
    _stats, stderr = launcher(double_click(4, f"{x},{y}"))
    started = next(line for line in stderr.splitlines() if "lagrange_viewer: starting" in line)
    assert started.split("--listen ")[1].count(" ") == 0  # the endpoint and nothing else


def test_a_class_without_a_sample_cannot_be_started(launcher: Any) -> None:
    x, y = CARDS["bench"]
    stats, stderr = launcher(double_click(4, f"{x},{y}"))
    assert stats["launcher"] is True and "starting" not in stderr

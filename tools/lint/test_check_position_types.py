"""Checks check_position_types.py against its fixtures.

Run: python3 -m unittest discover -s tools/lint
"""

from __future__ import annotations

import unittest
from pathlib import Path

import check_position_types as lint

FIXTURES = Path(__file__).parent / "fixtures"


def bad_lines(path: Path) -> set[int]:
    lines = path.read_text(encoding="utf-8").splitlines()
    return {number for number, line in enumerate(lines, start=1) if "BAD" in line.split("//")[-1]}


def reported_lines(path: Path) -> set[int]:
    return {int(problem.split(":")[1]) for problem in lint.check_file(path)}


class PositionTypes(unittest.TestCase):
    def test_reports_every_bad_line_and_nothing_else(self) -> None:
        for name in ("positions_bad.hpp", "positions_bad.proto"):
            path = FIXTURES / name
            with self.subTest(name):
                self.assertEqual(reported_lines(path), bad_lines(path))

    def test_accepts_good_code(self) -> None:
        self.assertEqual(lint.check_file(FIXTURES / "positions_good.hpp"), [])

    def test_names(self) -> None:
        self.assertTrue(lint.is_position_name("originWorld"))
        self.assertTrue(lint.is_position_name("wheel_pos"))
        self.assertTrue(lint.is_position_name("cameraLocal"))
        self.assertFalse(lint.is_position_name("positionRender"))
        self.assertFalse(lint.is_position_name("composite"))
        self.assertFalse(lint.is_position_name("localTime"))

    def test_scope(self) -> None:
        self.assertTrue(lint.in_scope(Path("core/world/x.hpp")))
        self.assertTrue(lint.in_scope(Path("proto/lagrange/viewer/v1/scene.proto")))
        self.assertFalse(lint.in_scope(Path("viewer/render/src/Camera.cpp")))
        self.assertFalse(lint.in_scope(Path("proto/gen/x.hpp")))

    def test_main_exit_code(self) -> None:
        self.assertEqual(lint.main(["--all-paths", str(FIXTURES / "positions_good.hpp")]), 0)
        self.assertEqual(lint.main(["--all-paths", str(FIXTURES / "positions_bad.hpp")]), 1)
        # Outside the scope, a file is not checked unless --all-paths is given.
        self.assertEqual(lint.main([str(FIXTURES / "positions_bad.hpp")]), 0)


if __name__ == "__main__":
    unittest.main()

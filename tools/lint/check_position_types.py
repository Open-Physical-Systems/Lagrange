#!/usr/bin/env python3
"""Refuses world and region positions stored as float (CLAUDE.md invariant 6).

docs/design/huge-worlds.md section 1: no struct in core/, modules/, proto/ or recordings stores
a position in the world or region frame as float. Positions use double; only the render frame,
computed last on the CPU, may be float, and its names end in "Render".

A declaration is reported when its type is a float type and its name says it is a position:
a word of the name is one of POSITION_WORDS, or the name ends in "World" or "Local". For a real
exception, put "lagrange-lint: allow-float-position (<reason>)" on the same line.

Usage: check_position_types.py [--all-paths] FILE...
Without --all-paths, only files under the directories in SCOPE are checked, so the script can be
given every changed file (pre-commit does that).
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

SCOPE = ("core/", "modules/", "server/", "bridges/", "io/", "proto/")
CPP_SUFFIXES = {".h", ".hpp", ".cpp", ".cc"}
PROTO_SUFFIXES = {".proto"}
SKIP_PARTS = ("/gen/", "/generated/")
ALLOW = "lagrange-lint: allow-float-position"

POSITION_WORDS = {
    "position",
    "positions",
    "pos",
    "origin",
    "translation",
    "location",
    "ecef",
    "enu",
}
FRAME_SUFFIXES = {"world", "local"}

# Float types that can hold a position: scalars, small vectors, and fixed arrays of float.
_FLOAT_TYPE = (
    r"(?:float[234]?|Vector3f|Vector2f|Vec3f|Vec2f|glm::vec[234]|"
    r"std::array<\s*float\s*,\s*\d+\s*>)"
)
_CPP_DECL = re.compile(
    rf"(?:^|[(,;:{{<]|\s)(?:const\s+|static\s+|constexpr\s+|mutable\s+)*{_FLOAT_TYPE}"
    r"\s*[&*]?\s*(?P<name>[A-Za-z_]\w*)\s*(?=[;=,){\[]|$)"
)
_PROTO_DECL = re.compile(r"^\s*(?:repeated\s+|optional\s+)?float\s+(?P<name>\w+)\s*=")


def words(name: str) -> list[str]:
    """Splits camelCase and snake_case into lower-case words: "originWorld" -> origin, world."""
    spaced = re.sub(r"([a-z0-9])([A-Z])", r"\1 \2", name).replace("_", " ")
    return [w.lower() for w in spaced.split()]


def is_position_name(name: str) -> bool:
    parts = words(name)
    if not parts or parts[-1] == "render":
        return False
    return bool(POSITION_WORDS.intersection(parts)) or parts[-1] in FRAME_SUFFIXES


def strip_comment(line: str, proto: bool) -> str:
    del proto  # both languages use // comments
    return line.split("//", 1)[0]


def check_file(path: Path) -> list[str]:
    proto = path.suffix in PROTO_SUFFIXES
    pattern = _PROTO_DECL if proto else _CPP_DECL
    problems = []
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        return [f"{path}: cannot read: {error}"]
    for number, line in enumerate(text.splitlines(), start=1):
        if ALLOW in line:
            continue
        code = strip_comment(line, proto)
        for match in pattern.finditer(code):
            name = match.group("name")
            if is_position_name(name):
                problems.append(
                    f"{path}:{number}: '{name}' is a position stored as float; world and region "
                    "positions are double (invariant 6). Only names ending in Render may be float."
                )
    return problems


def in_scope(path: Path) -> bool:
    text = path.as_posix()
    if any(part in text for part in SKIP_PARTS):
        return False
    return text.startswith(SCOPE) or any(f"/{scope}" in text for scope in SCOPE)


def main(argv: list[str]) -> int:
    all_paths = "--all-paths" in argv
    files = [Path(arg) for arg in argv if arg != "--all-paths"]
    problems: list[str] = []
    for path in files:
        if path.suffix not in CPP_SUFFIXES | PROTO_SUFFIXES:
            continue
        if not all_paths and not in_scope(path):
            continue
        problems.extend(check_file(path))
    for problem in problems:
        print(problem)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

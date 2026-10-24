"""Migrates simulation profiles to the newest ``schema_version`` (formats.md §5 and §10).

One step per version, each a function from the parsed profile of version N to version N + 1.
A step may also give a text edit that keeps the file's comments and layout; when every step of
a migration has one, the file is edited as text, otherwise it is written anew from the data
(and its comments are lost, which the tool says).

Version 0 is a profile without ``schema_version``: the form of the reference examples in
CLAUDE.md §11.1, written before the field existed.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

CURRENT_VERSION = 1


@dataclass(frozen=True)
class Step:
    data: Callable[[dict[str, Any]], dict[str, Any]]
    text: Callable[[str], str] | None = None  # the same change, keeping comments


def _add_version_1(profile: dict[str, Any]) -> dict[str, Any]:
    return {"schema_version": 1, **profile}


def _add_version_1_text(text: str) -> str:
    # After the leading comment block, before the first key.
    lines = text.splitlines(keepends=True)
    at = 0
    while at < len(lines) and (lines[at].startswith("#") or not lines[at].strip()):
        at += 1
    return "".join(lines[:at]) + "schema_version: 1\n" + "".join(lines[at:])


# from version -> the step to from version + 1
STEPS: dict[int, Step] = {0: Step(_add_version_1, _add_version_1_text)}


def version_of(profile: dict[str, Any]) -> int:
    version = profile.get("schema_version", 0)
    if not isinstance(version, int) or version < 0:
        raise ValueError(f"schema_version must be a non-negative integer, not {version!r}")
    return version


def migrate(text: str) -> tuple[str, list[str], bool]:
    """The profile in ``text`` at CURRENT_VERSION, what was done, and whether comments were
    kept."""
    profile = yaml.safe_load(text)
    if not isinstance(profile, dict):
        raise ValueError("a profile is a mapping")
    version = version_of(profile)
    if version > CURRENT_VERSION:
        raise ValueError(f"schema_version {version} is newer than this tool's {CURRENT_VERSION}")
    steps: list[Step] = []
    for start in range(version, CURRENT_VERSION):
        if start not in STEPS:
            raise ValueError(f"no migration from schema_version {start}")
        steps.append(STEPS[start])
    done = [f"schema_version {v} -> {v + 1}" for v in range(version, CURRENT_VERSION)]
    if all(step.text is not None for step in steps):
        for step in steps:
            assert step.text is not None
            text = step.text(text)
        return text, done, True
    for step in steps:
        profile = step.data(profile)
    return yaml.safe_dump(profile, sort_keys=False), done, False


def validate(text: str, schema_path: Path) -> list[str]:
    """Schema errors of the profile in ``text`` (profiles/schema.json)."""
    import jsonschema

    schema = json.loads(schema_path.read_text())
    validator = jsonschema.Draft202012Validator(schema)
    return [
        f"/{'/'.join(str(p) for p in error.absolute_path)}: {error.message}"
        for error in validator.iter_errors(yaml.safe_load(text))
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="migrate_profile",
        description=f"Brings simulation profiles up to schema_version {CURRENT_VERSION}.",
    )
    parser.add_argument("files", nargs="+", type=Path, help="profile YAML files")
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("-o", "--output", type=Path, help="write the one file migrated here")
    where.add_argument(
        "--in-place", action="store_true", help="rewrite the files, keeping FILE.bak"
    )
    where.add_argument(
        "--check", action="store_true", help="only report; exit 1 if a file needs migrating"
    )
    parser.add_argument(
        "--schema", type=Path, help="validate the result against this profiles/schema.json"
    )
    args = parser.parse_args(argv)
    if args.output is not None and len(args.files) != 1:
        parser.error("--output takes one file")
    outdated = False
    for path in args.files:
        original = path.read_text()
        try:
            text, done, kept = migrate(original)
        except (ValueError, yaml.YAMLError) as error:
            print(f"migrate_profile: {path}: {error}", file=sys.stderr)
            return 2
        if not done:
            print(f"{path}: up to date")
        for line in done:
            print(f"{path}: {line}")
        if done and not kept and re.search(r"(^|\s)#", original):
            print(f"{path}: written anew; its comments are not kept", file=sys.stderr)
        if args.schema is not None:
            problems = validate(text, args.schema)
            for problem in problems:
                print(f"{path}: {problem}", file=sys.stderr)
            if problems:
                return 2
        if not done:
            continue
        outdated = True
        if args.check:
            continue
        if args.in_place:
            shutil.copyfile(path, path.with_name(path.name + ".bak"))
            path.write_text(text)
        else:
            args.output.write_text(text)
    return 1 if args.check and outdated else 0


if __name__ == "__main__":
    sys.exit(main())

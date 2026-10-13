#!/usr/bin/env python3
"""Create or update GitHub labels from .github/labels.yml using the gh CLI.

The file is a list of one-line flow mappings:
    - { name: "type/bug", color: "d73a4a", description: "Something is wrong" }
It is parsed with a regular expression so that the script needs only the standard library.
Pass --dry-run to print the labels without calling gh.
"""

from __future__ import annotations

import re
import subprocess
import sys

LINE = re.compile(
    r'^- \{ name: "(?P<name>[^"]+)",\s+color: "(?P<color>[0-9a-fA-F]{6})",'
    r'\s+description: "(?P<description>[^"]*)" \}$'
)


def parse(path: str) -> list[dict[str, str]]:
    labels = []
    with open(path, encoding="utf-8") as handle:
        for number, raw in enumerate(handle, start=1):
            line = raw.rstrip("\n")
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            match = LINE.match(line)
            if match is None:
                sys.exit(f"{path}:{number}: cannot parse label line: {line}")
            labels.append(match.groupdict())
    return labels


def main() -> int:
    args = [a for a in sys.argv[1:] if a != "--dry-run"]
    dry_run = "--dry-run" in sys.argv[1:]
    if len(args) != 1:
        sys.exit("usage: sync_labels.py [--dry-run] <labels.yml>")
    labels = parse(args[0])
    names = [label["name"] for label in labels]
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        sys.exit(f"duplicate label names: {', '.join(duplicates)}")
    for label in labels:
        if dry_run:
            print(f'{label["name"]:32} #{label["color"]}  {label["description"]}')
            continue
        subprocess.run(
            ["gh", "label", "create", label["name"], "--color", label["color"],
             "--description", label["description"], "--force"],
            check=True,
        )
    print(f"{len(labels)} labels {'parsed' if dry_run else 'synced'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

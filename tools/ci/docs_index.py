#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The README's list of every document: docs_index.py writes it, --check fails when it is stale."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BEGIN = "<!-- docs-index begin: written by tools/ci/docs_index.py -->"
END = "<!-- docs-index end -->"
GROUPS = (
    ("User guide", "docs/guide"),
    ("Physics reference", "docs/physics"),
    ("Domains and physics", "docs/design/domains"),
    ("Viewer UI design", "docs/design/ui"),
    ("Design", "docs/design"),
    ("Plan and evaluation", "docs/plan"),
    ("Release notes", "docs/release"),
    ("Validation", "docs/validation"),
    ("Videos", "docs/videos"),
    ("Design proposals (RFCs)", "rfcs"),
)
# The documentation site's own menu page and build output (tools/docs/build.sh), not documents.
SITE_ONLY = ("docs/index.md",)
SITE_BUILD = "docs/_build/"


def title_of(path: Path) -> str:
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("# "):
            return line[2:].strip().replace("|", "/")
    return path.stem


def group_of(relative: str) -> str:
    return next(name for name, folder in GROUPS if relative.startswith(folder + "/"))


def section() -> str:
    documents: dict[str, list[str]] = {name: [] for name, _ in GROUPS}
    for folder in ("docs", "rfcs"):
        for path in sorted((ROOT / folder).rglob("*.md")):
            relative = path.relative_to(ROOT).as_posix()
            if relative in SITE_ONLY or relative.startswith(SITE_BUILD):
                continue
            if any(relative.startswith(f + "/") for _, f in GROUPS):
                documents[group_of(relative)].append(f"- [{title_of(path)}]({relative})")
            else:
                raise SystemExit(f"{relative} is in no group of tools/ci/docs_index.py")
    lines = [BEGIN]
    for name, entries in documents.items():
        if entries:
            lines += ["", f"<details><summary>{name} ({len(entries)})</summary>", "", *entries]
            lines += ["", "</details>"]
    return "\n".join([*lines, "", END])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    readme = ROOT / "README.md"
    text = readme.read_text(encoding="utf-8")
    if BEGIN not in text or END not in text:
        raise SystemExit("the README has no docs-index markers")
    head, rest = text.split(BEGIN, 1)
    tail = rest.split(END, 1)[1]
    updated = head + section() + tail
    if updated == text:
        return 0
    if args.check:
        print("README.md's list of documents is stale: run tools/ci/docs_index.py", file=sys.stderr)
        return 1
    readme.write_text(updated, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())

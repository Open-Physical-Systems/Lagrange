"""RFC 0007 and roadmap §9 ("old sample files migrate"): every sample robot, world and profile
that a release since v0.5.0 shipped is up to date for this release's tools, or migrates to it.
The files are taken from the git tags, so the test needs the source tree's history; it is
skipped in a tree without it (a source tarball)."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest
from lagrange_migrate import profile, schema
from pxr import Sdf

REPO = Path(os.environ.get("LAGRANGE_REPO", Path(__file__).resolve().parents[3]))
TAGS = ["v0.5.0", "v0.6.0", "v0.7.0"]


def git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(REPO), *args], capture_output=True, text=True, check=True
    ).stdout


def tags() -> list[str]:
    try:
        present = set(git("tag", "--list").split())
    except (OSError, subprocess.CalledProcessError):
        return []
    return [t for t in TAGS if t in present]


def files_at(tag: str, prefix: str, suffix: str) -> list[str]:
    return [
        f
        for f in git("ls-tree", "-r", "--name-only", tag).splitlines()
        if f.startswith(prefix) and f.endswith(suffix)
    ]


@pytest.mark.parametrize("tag", tags() or [pytest.param("none", marks=pytest.mark.skip)])
def test_the_sample_stages_of_a_release_need_no_schema_migration(tag: str) -> None:
    stages = files_at(tag, "assets/", ".usda")
    assert stages, f"{tag} has sample stages"
    for path in stages:
        text = git("show", f"{tag}:{path}")
        layer = Sdf.Layer.CreateAnonymous(".usda")
        assert layer.ImportFromString(text), f"{tag}:{path} does not parse"
        report = schema.migrate_layer(layer)
        # Up to date, or migrated by this release's steps (an older family version).
        assert not report.changed or report.changes, f"{tag}:{path}"


@pytest.mark.parametrize("tag", tags() or [pytest.param("none", marks=pytest.mark.skip)])
def test_the_example_profiles_of_a_release_migrate_and_validate(tag: str) -> None:
    examples = files_at(tag, "profiles/examples/", ".yaml")
    assert examples, f"{tag} has example profiles"
    schema_path = REPO / "profiles" / "schema.json"
    json.loads(schema_path.read_text())
    for path in examples:
        text, _done, _kept = profile.migrate(git("show", f"{tag}:{path}"))
        errors = profile.validate(text, schema_path)
        assert not errors, f"{tag}:{path}: {errors}"

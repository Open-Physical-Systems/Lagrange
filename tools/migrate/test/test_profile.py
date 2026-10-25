"""migrate_profile: a profile of the reference examples' form (no schema_version, CLAUDE.md
§11.1) becomes a valid version 1 profile with its comments kept; the shipped examples are up
to date."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
from lagrange_migrate import profile

REPO = Path(os.environ.get("LAGRANGE_REPO", Path(__file__).resolve().parents[3]))
SCHEMA = REPO / "profiles" / "schema.json"

# The bench-mode reference of CLAUDE.md §11.1, as it was written: no schema_version.
VERSION_0 = """# profiles/examples/bench_electrical_replay.yaml
stage: robots/amr_diffdrive.usda

domains:
  rigid_body:   { enabled: false, substitute: replay, source: logs/real_amr_run.mcap }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.0001 }
  thermal:      { enabled: true,  dt: 0.1 }
  aerodynamics: { enabled: false, substitute: zero }
  weather:      { enabled: false, substitute: clear }
  emi:          { enabled: false, substitute: zero }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: false, substitute: ground_truth }  # high-speed testing

views: [electrical_schematic, waveforms]
record: { format: mcap, path: runs/bench_001.mcap }
"""


def test_version_0_becomes_a_valid_version_1_with_its_comments(tmp_path: Path) -> None:
    old = tmp_path / "bench.yaml"
    old.write_text(VERSION_0)
    assert profile.validate(VERSION_0, SCHEMA), "version 0 does not validate as version 1"
    new = tmp_path / "bench_v1.yaml"
    assert profile.main([str(old), "-o", str(new), "--schema", str(SCHEMA)]) == 0
    text = new.read_text()
    assert profile.validate(text, SCHEMA) == []
    assert text.startswith("# profiles/examples/bench_electrical_replay.yaml\nschema_version: 1\n")
    assert "# high-speed testing" in text
    # Nothing else changed.
    assert text.replace("schema_version: 1\n", "", 1) == VERSION_0


def test_a_migration_without_a_text_edit_writes_the_data(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(profile.STEPS, 0, profile.Step(profile._add_version_1))
    text, done, kept = profile.migrate(VERSION_0)
    assert done == ["schema_version 0 -> 1"]
    assert not kept
    assert text.startswith("schema_version: 1\n")


def test_a_newer_version_is_refused() -> None:
    with pytest.raises(ValueError, match="newer"):
        profile.migrate("schema_version: 99\nstage: x\ndomains: {}\n")


def test_the_shipped_examples_are_up_to_date() -> None:
    examples = sorted(str(p) for p in (REPO / "profiles" / "examples").glob("*.yaml"))
    assert examples
    assert profile.main([*examples, "--check", "--schema", str(SCHEMA)]) == 0

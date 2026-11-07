# SPDX-License-Identifier: Apache-2.0
"""external_assets.yaml is well formed, and every fetched file a robot names is in it."""

import re
from pathlib import Path

import yaml

ASSETS = Path(__file__).resolve().parents[1]
MANIFEST = yaml.safe_load((ASSETS / "external_assets.yaml").read_text())


def test_every_asset_is_pinned_and_hashed() -> None:
    assert MANIFEST["schema_version"] == 1
    for asset_id, asset in MANIFEST["assets"].items():
        assert re.fullmatch(r"[a-z0-9_]+", asset_id)
        assert re.fullmatch(r"[0-9a-f]{40}", asset["commit"]), asset_id
        assert asset["source"].startswith("https://github.com/"), asset_id
        assert asset["commit"] in asset["license_url"], asset_id
        assert asset["license"] and asset["description"], asset_id
        for name, spec in asset["files"].items():
            assert "/" not in name, name
            assert re.fullmatch(r"[0-9a-f]{64}", spec["sha256"]), name


def test_robot_files_name_only_listed_assets() -> None:
    named = set()
    for robot in (ASSETS / "robots").rglob("*"):
        if robot.suffix in {".urdf", ".sdf", ".xml", ".usda"}:
            named |= set(re.findall(r"lagrange-asset://([^\"'@<>\s]+)", robot.read_text()))
    assert named, "the Lexus and the X500 V2 name fetched meshes"
    for uri in named:
        asset_id, _, name = uri.partition("/")
        assert name in MANIFEST["assets"].get(asset_id, {}).get("files", {}), uri

"""collision_bake on the sample rack: CoACD splits it into convex parts that keep the gaps
between the boards open, the result is what the stage loader reads as hulls, and the same seed
bakes the same hulls."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from lagrange_collision_bake import bake, sample
from pxr import Usd, UsdGeom, UsdPhysics


def baked(tmp_path: Path, name: str = "rack") -> tuple[Usd.Stage, dict]:
    source = tmp_path / f"{name}.usda"
    out = tmp_path / f"{name}_baked.usda"
    report = tmp_path / f"{name}.json"
    sample.write_rack(str(source))
    assert bake.main([str(source), "-o", str(out), "--report", str(report)]) == 0
    stage = Usd.Stage.Open(str(out))
    return stage, json.loads(report.read_text())


def hulls(stage: Usd.Stage) -> list[UsdGeom.Mesh]:
    return [
        UsdGeom.Mesh(p)
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.MeshCollisionAPI)
        and UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()
        == UsdPhysics.Tokens.convexHull
    ]


def test_the_rack_becomes_convex_hulls_and_stays_visible(tmp_path: Path) -> None:
    stage, report = baked(tmp_path)
    parts = hulls(stage)
    # Four uprights and three boards: at least one hull each, and not a lump per rack.
    assert 7 <= len(parts) <= 40
    assert report["meshes"][0]["hulls"] == len(parts)
    frame = stage.GetPrimAtPath("/rack/frame")
    assert not frame.HasAPI(UsdPhysics.CollisionAPI), "the drawn mesh no longer collides"
    for part in parts:
        assert part.GetPurposeAttr().Get() == UsdGeom.Tokens.guide
        assert str(part.GetPath()).startswith("/rack/frame_collision/hull_")


def test_the_hulls_keep_the_gaps_between_the_boards_open(tmp_path: Path) -> None:
    stage, _ = baked(tmp_path)
    # No hull reaches into the middle of the gap between the first two boards, away from the
    # uprights: there the lidar must see through.
    gap = (sample.BOARD_HEIGHTS_M[0] + sample.BOARD_HEIGHTS_M[1]) / 2
    for part in hulls(stage):
        points = np.asarray(part.GetPointsAttr().Get())
        lo, hi = points.min(axis=0), points.max(axis=0)
        spans_gap = lo[2] < gap - 0.2 and hi[2] > gap + 0.2
        in_middle = lo[0] < 0.0 < hi[0] and lo[1] < 0.0 < hi[1]
        assert not (spans_gap and in_middle), f"{part.GetPath()} fills the gap"


def test_the_same_seed_bakes_the_same_hulls(tmp_path: Path) -> None:
    a, _ = baked(tmp_path, "a")
    b, _ = baked(tmp_path, "b")
    pa, pb = hulls(a), hulls(b)
    assert len(pa) == len(pb)
    for x, y in zip(pa, pb, strict=True):
        assert list(x.GetPointsAttr().Get()) == list(y.GetPointsAttr().Get())


def test_a_stage_without_mesh_colliders_is_left_alone(tmp_path: Path) -> None:
    source = tmp_path / "plain.usda"
    stage = Usd.Stage.CreateNew(str(source))
    UsdGeom.Cube.Define(stage, "/box")
    stage.Save()
    out = tmp_path / "plain_out.usda"
    assert bake.main([str(source), "-o", str(out)]) == 0
    assert hulls(Usd.Stage.Open(str(out))) == []

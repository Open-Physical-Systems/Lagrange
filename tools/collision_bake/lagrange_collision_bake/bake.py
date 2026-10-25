"""Bakes mesh colliders into convex hulls (formats.md §2: collision geometry is convex
decompositions, never render meshes).

Every mesh with PhysicsCollisionAPI that is not already a convex hull (its
``physics:approximation`` is not ``convexHull``) is decomposed with CoACD (Wei et al., "Approximate
Convex Decomposition for 3D Meshes with Collision-Aware Concavity and Tree Search", SIGGRAPH 2022).
The mesh stays as the visible geometry without its collision schemas; a sibling Xform
``<name>_collision`` with the mesh's transform holds one ``convexHull`` mesh per part, purpose
``guide``. The stage loader reads those parts as convex hull colliders.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


@dataclass
class Settings:
    threshold: float = 0.05  # CoACD's concavity threshold, relative to the mesh's size
    max_hulls: int = -1  # -1: as many as the threshold needs
    seed: int = 0  # CoACD's random seed: the same seed bakes the same hulls
    # CoACD's remeshing of the input into a manifold: "auto" when it is not one (a scan,
    # overlapping parts), "off" for meshes of closed shells. Remeshing works on a voxel grid of
    # `preprocess_resolution` cells across the mesh, which grows the surface by about a cell.
    preprocess: str = "auto"
    preprocess_resolution: int = 50


@dataclass
class MeshReport:
    path: str
    triangles: int
    hulls: int
    hull_vertices: int
    seconds: float


@dataclass
class Report:
    meshes: list[MeshReport] = field(default_factory=list)

    @property
    def hulls(self) -> int:
        return sum(m.hulls for m in self.meshes)


def _triangles(mesh: UsdGeom.Mesh) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(mesh.GetPointsAttr().Get() or [], dtype=np.float64).reshape(-1, 3)
    counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
    indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
    faces = []
    at = 0
    for count in counts:
        for k in range(1, count - 1):  # a fan per face
            faces.append((indices[at], indices[at + k], indices[at + k + 1]))
        at += count
    return points, np.asarray(faces, dtype=np.int32).reshape(-1, 3)


def _needs_bake(prim: Usd.Prim) -> bool:
    if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
        return False
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        return bool(approximation != UsdPhysics.Tokens.convexHull)
    return True


def _decompose(points: np.ndarray, faces: np.ndarray, settings: Settings) -> list[Any]:
    import coacd  # imported here: the module is optional until a bake is run

    coacd.set_log_level("error")
    parts: list[Any] = coacd.run_coacd(
        coacd.Mesh(points, faces),
        threshold=settings.threshold,
        max_convex_hull=settings.max_hulls,
        preprocess_mode=settings.preprocess,
        preprocess_resolution=settings.preprocess_resolution,
        seed=settings.seed,
    )
    return parts


def bake_stage(stage: Usd.Stage, settings: Settings) -> Report:
    """Bakes every mesh collider of ``stage`` that needs it, in its edit target."""
    report = Report()
    targets = [prim for prim in stage.Traverse() if _needs_bake(prim)]
    for prim in targets:
        started = time.perf_counter()
        mesh = UsdGeom.Mesh(prim)
        points, faces = _triangles(mesh)
        if len(faces) == 0:
            continue
        parts = _decompose(points, faces, settings)

        holder_path = prim.GetPath().GetParentPath().AppendChild(f"{prim.GetName()}_collision")
        holder = UsdGeom.Xform.Define(stage, holder_path)
        local = UsdGeom.Xformable(prim).GetLocalTransformation(Usd.TimeCode.Default())
        holder.MakeMatrixXform().Set(Gf.Matrix4d(local))
        vertices = 0
        for k, (part_points, part_faces) in enumerate(parts):
            hull = UsdGeom.Mesh.Define(stage, holder_path.AppendChild(f"hull_{k:03d}"))
            hull.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in part_points])
            hull.CreateFaceVertexCountsAttr([3] * len(part_faces))
            hull.CreateFaceVertexIndicesAttr([int(i) for f in part_faces for i in f])
            hull.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(hull.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(hull.GetPrim()).CreateApproximationAttr(
                UsdPhysics.Tokens.convexHull
            )
            vertices += len(part_points)
        # The mesh stays as what is drawn; its collision is now the hulls.
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
        report.meshes.append(
            MeshReport(
                path=str(prim.GetPath()),
                triangles=len(faces),
                hulls=len(parts),
                hull_vertices=vertices,
                seconds=time.perf_counter() - started,
            )
        )
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="collision_bake",
        description="Decomposes a stage's mesh colliders into convex hulls (CoACD).",
    )
    parser.add_argument("stage", type=Path, help="USD stage to bake")
    parser.add_argument("-o", "--output", type=Path, required=True, help="the baked stage")
    parser.add_argument(
        "--threshold", type=float, default=Settings.threshold, help="concavity, 0.01 to 1"
    )
    parser.add_argument("--max-hulls", type=int, default=Settings.max_hulls)
    parser.add_argument("--seed", type=int, default=Settings.seed)
    parser.add_argument(
        "--preprocess",
        choices=["auto", "on", "off"],
        default=Settings.preprocess,
        help="remesh into a manifold first (auto: when it is not one; off for closed shells)",
    )
    parser.add_argument(
        "--preprocess-resolution",
        type=int,
        default=Settings.preprocess_resolution,
        help="voxels across the mesh for the remeshing; the surface grows by about one",
    )
    parser.add_argument("--report", type=Path, help="also write the report as JSON")
    args = parser.parse_args(argv)
    if not 0.01 <= args.threshold <= 1.0:
        parser.error("--threshold must be from 0.01 to 1")
    stage = Usd.Stage.Open(str(args.stage))
    if stage is None:
        print(f"collision_bake: cannot open {args.stage}", file=sys.stderr)
        return 2
    settings = Settings(
        args.threshold, args.max_hulls, args.seed, args.preprocess, args.preprocess_resolution
    )
    report = bake_stage(stage, settings)
    layer: Sdf.Layer = stage.GetRootLayer()
    layer.Export(str(args.output))
    for mesh in report.meshes:
        print(
            f"{mesh.path}: {mesh.triangles} triangles -> {mesh.hulls} convex hulls "
            f"({mesh.hull_vertices} vertices, {mesh.seconds:.1f} s)"
        )
    print(f"{len(report.meshes)} meshes baked into {report.hulls} hulls -> {args.output}")
    if args.report is not None:
        args.report.write_text(
            json.dumps(
                {"settings": asdict(settings), "meshes": [asdict(m) for m in report.meshes]},
                indent=2,
            )
            + "\n"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

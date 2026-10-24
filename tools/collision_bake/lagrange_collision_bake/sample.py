"""The sample warehouse rack: four uprights and three boards as one mesh, the shape a scanned
or CAD rack arrives in. Non-convex: a single convex hull would fill the gaps between the
boards, which the lidar must see through."""

from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

WIDTH_M = 1.2  # x
DEPTH_M = 0.4  # y
HEIGHT_M = 2.0  # z
UPRIGHT_M = 0.04  # square section
BOARD_M = 0.03  # thickness
BOARD_HEIGHTS_M = (0.3, 1.0, 1.7)  # board centres


def _box(
    lo: tuple[float, float, float], hi: tuple[float, float, float], base: int
) -> tuple[list[Gf.Vec3f], list[int]]:
    points = [
        Gf.Vec3f(hi[0] if k & 1 else lo[0], hi[1] if k & 2 else lo[1], hi[2] if k & 4 else lo[2])
        for k in range(8)
    ]
    quads = [0, 2, 3, 1, 4, 5, 7, 6, 0, 1, 5, 4, 2, 6, 7, 3, 0, 4, 6, 2, 1, 3, 7, 5]
    return points, [base + i for i in quads]


def rack_boxes() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """The rack's parts as boxes (low and high corners), its base centred at the origin."""
    w, d, u, b = WIDTH_M / 2, DEPTH_M / 2, UPRIGHT_M, BOARD_M / 2
    boxes = [
        (
            (sx * w - (u if sx > 0 else 0), sy * d - (u if sy > 0 else 0), 0.0),
            (sx * w + (0 if sx > 0 else u), sy * d + (0 if sy > 0 else u), HEIGHT_M),
        )
        for sx in (-1, 1)
        for sy in (-1, 1)
    ]
    boxes += [((-w, -d, z - b), (w, d, z + b)) for z in BOARD_HEIGHTS_M]
    return boxes


def write_rack(path: str, x: float = 0.0, y: float = 0.0) -> None:
    """A stage with the rack at (x, y) as one mesh collider marked for decomposition."""
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/rack")
    stage.SetDefaultPrim(root.GetPrim())
    root.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    points: list[Gf.Vec3f] = []
    indices: list[int] = []
    for lo, hi in rack_boxes():
        p, i = _box(lo, hi, len(points))
        points += p
        indices += i
    mesh = UsdGeom.Mesh.Define(stage, "/rack/frame")
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4] * (len(indices) // 4))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.36, 0.62)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        UsdPhysics.Tokens.convexDecomposition
    )
    stage.GetRootLayer().documentation = (
        "Sample warehouse rack (lagrange_collision_bake/sample.py): one non-convex mesh, its "
        "collision marked for convex decomposition. rack_baked.usda is this file after "
        "`collision_bake --preprocess off` (its parts are closed shells, so no remeshing)."
    )
    stage.Save()

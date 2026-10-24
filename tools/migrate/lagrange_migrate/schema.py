"""Migrates USD files from older versions of the Lagrange schemas (RFC 0002, formats.md §10).

A schema family carries its version in its name (``ElectricalMotorAPI_1``). A change that is not
an addition with a fallback makes a new version, and comes with one migration function per
(family, from version) here, which rewrites the attributes of a prim that applies the old
version. The driver chains the functions to the newest version it knows and renames the
applied schema. It works on the layer (Sdf), so the old schemas need not be registered.

Until v1.0 every schema is version 1 and may change without a migration. The machinery is
proven by the synthetic family ``ExampleMotorAPI`` (``_0`` to ``_1``), which no real file uses.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from pxr import Sdf

# "ElectricalMotorAPI_1", or a multiple-apply schema's "PortAPI_1:dc_out".
_APPLIED = re.compile(r"^(?P<family>[A-Za-z][A-Za-z0-9]*)_(?P<version>\d+)(?P<instance>:.+)?$")


class PrimEdit:
    """What a migration function may do to one prim: the edits are recorded for the report."""

    def __init__(self, prim: Sdf.PrimSpec, instance: str) -> None:
        self.prim = prim
        self.instance = instance  # ":dc_out" for a multiple-apply schema, else ""
        self.changes: list[str] = []

    def has(self, name: str) -> bool:
        return name in self.prim.properties

    def rename(self, old: str, new: str) -> None:
        """Renames attribute ``old`` to ``new``, keeping its value and metadata."""
        if not self.has(old):
            return
        if self.has(new):
            raise MigrationError(f"{self.prim.path}: both {old} and {new} are authored")
        spec = self.prim.properties[old]
        spec.name = new
        self.changes.append(f"renamed {old} to {new}")

    def scale(self, name: str, factor: float, reason: str) -> None:
        """Multiplies an authored numeric value (a unit change)."""
        if not self.has(name):
            return
        spec = self.prim.properties[name]
        if spec.default is not None:
            spec.default = type(spec.default)(spec.default * factor)
        for time in spec.layer.ListTimeSamplesForPath(spec.path):
            value = spec.layer.QueryTimeSample(spec.path, time)
            spec.layer.SetTimeSample(spec.path, time, type(value)(value * factor))
        self.changes.append(f"scaled {name} by {factor:g} ({reason})")

    def remove(self, name: str, reason: str) -> None:
        if not self.has(name):
            return
        self.prim.RemoveProperty(self.prim.properties[name])
        self.changes.append(f"removed {name} ({reason})")


class MigrationError(Exception):
    pass


Migration = Callable[[PrimEdit], None]

# (family, from version) -> the function that moves a prim to from version + 1.
MIGRATIONS: dict[tuple[str, int], Migration] = {}


def migration(family: str, from_version: int) -> Callable[[Migration], Migration]:
    def register(function: Migration) -> Migration:
        key = (family, from_version)
        if key in MIGRATIONS:
            raise ValueError(f"two migrations from {family}_{from_version}")
        MIGRATIONS[key] = function
        return function

    return register


def newest(family: str, version: int) -> int:
    """The version the registry can bring ``family_version`` to."""
    while (family, version) in MIGRATIONS:
        version += 1
    return version


# --- the migrations ------------------------------------------------------------------------


@migration("ExampleMotorAPI", 0)
def _example_motor_0_to_1(prim: PrimEdit) -> None:
    """Synthetic (formats.md §10): version 0 gave the torque constant in mN·m/A under another
    name and had a flag that version 1 dropped."""
    prim.rename("example:torqueConstantMilli", "example:torqueConstant")
    prim.scale("example:torqueConstant", 1e-3, "mN·m/A to N·m/A")
    prim.remove("example:legacyFlag", "no longer used")


# --- the driver ----------------------------------------------------------------------------


@dataclass
class Report:
    changes: list[str] = field(default_factory=list)

    @property
    def changed(self) -> bool:
        return bool(self.changes)


def _token_list(op: Sdf.TokenListOp) -> list[tuple[str, list[str]]]:
    """The lists of a list op that hold items, by name."""
    if op.isExplicit:
        return [("explicitItems", list(op.explicitItems))]
    return [
        (name, list(getattr(op, name)))
        for name in ("prependedItems", "appendedItems", "addedItems", "orderedItems")
        if list(getattr(op, name))
    ]


def migrate_layer(layer: Sdf.Layer) -> Report:
    """Migrates every prim of ``layer`` in place."""
    report = Report()
    prims: list[Sdf.PrimSpec] = []
    layer.Traverse(
        Sdf.Path.absoluteRootPath,
        lambda path: prims.append(layer.GetPrimAtPath(path)) if path.IsPrimPath() else None,
    )
    for prim in prims:
        if prim is None or "apiSchemas" not in prim.ListInfoKeys():
            continue
        op: Sdf.TokenListOp = prim.GetInfo("apiSchemas")
        seen: dict[tuple[str, str], int] = {}
        rewritten: dict[str, list[str]] = {}
        for list_name, items in _token_list(op):
            out = []
            for item in items:
                match = _APPLIED.match(item)
                if match is None:
                    out.append(item)
                    continue
                family = match["family"]
                version = int(match["version"])
                instance = match["instance"] or ""
                key = (family, instance)
                if key in seen and seen[key] != version:
                    raise MigrationError(
                        f"{prim.path}: applies {family}_{seen[key]} and {family}_{version}"
                    )
                seen[key] = version
                target = newest(family, version)
                if target == version:
                    out.append(item)
                    continue
                edit = PrimEdit(prim, instance)
                for step in range(version, target):
                    MIGRATIONS[(family, step)](edit)
                new_item = f"{family}_{target}{instance}"
                out.append(new_item)
                report.changes.append(f"{prim.path}: {item} -> {new_item}")
                report.changes.extend(f"{prim.path}:   {change}" for change in edit.changes)
            rewritten[list_name] = out
        if any(rewritten[name] != items for name, items in _token_list(op)):
            for name, items in rewritten.items():
                setattr(op, name, items)
            prim.SetInfo("apiSchemas", op)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="migrate_schema",
        description="Brings USD files up to the newest versions of the Lagrange schemas.",
    )
    parser.add_argument("files", nargs="+", type=Path, help=".usda/.usdc/.usd files")
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("-o", "--output", type=Path, help="write the one file migrated here")
    where.add_argument(
        "--in-place", action="store_true", help="rewrite the files, keeping FILE.bak"
    )
    where.add_argument(
        "--check", action="store_true", help="only report; exit 1 if a file needs migrating"
    )
    args = parser.parse_args(argv)
    if args.output is not None and len(args.files) != 1:
        parser.error("--output takes one file")
    outdated = False
    for path in args.files:
        layer = Sdf.Layer.FindOrOpen(str(path))
        if layer is None:
            print(f"migrate_schema: cannot open {path}", file=sys.stderr)
            return 2
        try:
            report = migrate_layer(layer)
        except MigrationError as error:
            print(f"migrate_schema: {path}: {error}", file=sys.stderr)
            return 2
        for line in report.changes:
            print(f"{path}: {line}")
        if not report.changed:
            print(f"{path}: up to date")
            continue
        outdated = True
        if args.check:
            continue
        if args.in_place:
            shutil.copyfile(path, path.with_name(path.name + ".bak"))
            layer.Save()
        else:
            layer.Export(str(args.output))
    return 1 if args.check and outdated else 0


if __name__ == "__main__":
    sys.exit(main())

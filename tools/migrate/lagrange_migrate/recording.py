"""Checks recordings (MCAP, formats.md §7) against the newest ``RunMeta.format_version``.

Version 1 is the only format so far: every recording written by Lagrange up to v1.0 is version
1, and later 1.x releases only add channels and fields (RFC 0007), which older readers skip.
A version 2 would bring one step here, a function from the messages of version 1 to those of
version 2, as migrate_profile has for profiles.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from mcap.exceptions import McapError
from mcap.reader import make_reader

CURRENT_FORMAT = 1
META_TOPIC = "/lagrange/meta"


def _varint(data: bytes, at: int) -> tuple[int, int]:
    value = 0
    shift = 0
    while True:
        byte = data[at]
        at += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, at
        shift += 7


def format_version(meta: bytes) -> int:
    """RunMeta.format_version (field 1, a varint) from a serialised RunMeta; 0 when absent."""
    at = 0
    while at < len(meta):
        key, at = _varint(meta, at)
        field, wire = key >> 3, key & 7
        if wire == 0:
            value, at = _varint(meta, at)
            if field == 1:
                return value
        elif wire == 2:
            length, at = _varint(meta, at)
            at += length
        elif wire == 1:
            at += 8
        elif wire == 5:
            at += 4
        else:
            raise ValueError(f"RunMeta has an unknown wire type {wire}")
    return 0


def version_of(path: Path) -> int:
    """The format version of a recording: that of its first RunMeta."""
    with path.open("rb") as stream:
        for _schema, _channel, message in make_reader(stream).iter_messages(topics=[META_TOPIC]):
            return format_version(message.data)
    raise ValueError(f"{path}: no {META_TOPIC} message: not a Lagrange recording")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="migrate_recording",
        description="Checks Lagrange recordings (MCAP) against the newest format version.",
    )
    parser.add_argument("files", nargs="+", type=Path, help=".mcap files")
    parser.add_argument(
        "--check", action="store_true", help="only report; exit 1 if a file needs migrating"
    )
    args = parser.parse_args(argv)
    for path in args.files:
        try:
            version = version_of(path)
        except (OSError, ValueError, McapError) as error:
            print(f"migrate_recording: {error}", file=sys.stderr)
            return 2
        if version > CURRENT_FORMAT:
            print(
                f"migrate_recording: {path}: format_version {version} is newer than this "
                f"tool's {CURRENT_FORMAT}",
                file=sys.stderr,
            )
            return 2
        if version < CURRENT_FORMAT:
            print(
                f"migrate_recording: {path}: format_version {version}: no migration from it "
                "(every Lagrange release wrote version 1)",
                file=sys.stderr,
            )
            return 2
        print(f"{path}: format_version {version}, up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())

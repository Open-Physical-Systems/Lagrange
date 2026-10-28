#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fetch the large assets of external_assets.yaml into the user's asset directory.

Each file comes from the upstream repository at a pinned commit and must match its SHA-256;
the licence and a NOTICE land next to it. Robot files name the files
`lagrange-asset://<id>/<file>` (formats.md §9). Usage: fetch_assets [ID...] [--list] [--force]
"""

import argparse
import hashlib
import os
import sys
import urllib.request
from pathlib import Path

import yaml

RAW = "https://raw.githubusercontent.com"


def manifest_path() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here.parent / "external_assets.yaml", here / "external_assets.yaml"):
        if candidate.is_file():
            return candidate
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("lagrange_assets")) / "external_assets.yaml"


def asset_dir() -> Path:
    if env := os.environ.get("LAGRANGE_ASSET_DIR"):
        return Path(env)
    data = os.environ.get("XDG_DATA_HOME") or str(Path.home() / ".local" / "share")
    return Path(data) / "lagrange" / "assets"


def raw_url(source: str, commit: str, path: str) -> str:
    repo = source.removeprefix("https://github.com/").rstrip("/")
    return f"{RAW}/{repo}/{commit}/{path}"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def download(url: str, to: Path) -> None:
    part = to.with_name(to.name + ".part")
    request = urllib.request.Request(url, headers={"User-Agent": "lagrange-fetch-assets"})
    with urllib.request.urlopen(request, timeout=120) as response, part.open("wb") as f:
        while block := response.read(1 << 20):
            f.write(block)
    part.replace(to)


def fetch(asset_id: str, asset: dict, root: Path, force: bool) -> bool:
    out = root / asset_id
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    for name, spec in asset["files"].items():
        target = out / name
        if not force and target.is_file() and sha256(target) == spec["sha256"]:
            continue
        print(f"{asset_id}: {name}")
        download(raw_url(asset["source"], asset["commit"], spec["path"]), target)
        if sha256(target) != spec["sha256"]:
            print(f"FAILED: {target}: SHA-256 differs from the manifest", file=sys.stderr)
            target.unlink()
            ok = False
    download(asset["license_url"], out / "LICENSE")
    (out / "NOTICE").write_text(
        f"{asset['description']}\n"
        f"From {asset['source']} at commit {asset['commit']}, licence {asset['license']}"
        " (LICENSE).\nFetched for Lagrange by fetch_assets; not part of Lagrange's own"
        " sources.\n"
    )
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("ids", nargs="*", help="assets to fetch (default: all)")
    parser.add_argument("--list", action="store_true", help="list the assets and exit")
    parser.add_argument("--force", action="store_true", help="download again")
    args = parser.parse_args()
    assets = yaml.safe_load(manifest_path().read_text())["assets"]
    if args.list:
        for asset_id, asset in assets.items():
            print(f"{asset_id:20} {asset['license']:14} {asset['description']}")
        return 0
    unknown = [i for i in args.ids if i not in assets]
    if unknown:
        print(f"unknown asset(s): {', '.join(unknown)}; see --list", file=sys.stderr)
        return 2
    root = asset_dir()
    ok = all([fetch(i, assets[i], root, args.force) for i in (args.ids or list(assets))])
    print(f"assets in {root}" if ok else "some files failed their checksum")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

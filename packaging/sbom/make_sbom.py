#!/usr/bin/env python3
"""Writes an SPDX 2.3 software bill of materials (JSON) for a Lagrange release (roadmap §9).

Lagrange's own packages come from their package.xml files; third-party components from the
tables of docs/design/dependencies.md, which list each one's version, source and licence as
checked by hand. A licence is given as an SPDX identifier only where the table's text maps to
one without doubt; otherwise it is NOASSERTION and the table's words go in the comment.

    python3 packaging/sbom/make_sbom.py [--repo DIR] OUT.spdx.json
"""

from __future__ import annotations

import argparse
import datetime
import json
import re
import subprocess
import uuid
from pathlib import Path

# Licence words of dependencies.md that name one SPDX identifier.
SPDX = {
    "apache-2.0": "Apache-2.0",
    "mit": "MIT",
    "expat": "MIT",
    "mit (expat)": "MIT",
    "bsd-3-clause": "BSD-3-Clause",
    "bsd-2-clause": "BSD-2-Clause",
    "bsl-1.0": "BSL-1.0",
    "zlib": "Zlib",
    "mpl-2.0": "MPL-2.0",
    "lgpl-2.1": "LGPL-2.1-only",
}


def spdx_id(text: str) -> str:
    """'SPDXRef-' + a name made of the characters SPDX allows."""
    return "SPDXRef-" + re.sub(r"[^A-Za-z0-9.-]", "-", text)


def licence_of(text: str) -> str:
    head = re.split(r"[;(,]", text.strip())[0].strip().lower()
    return SPDX.get(head, "NOASSERTION")


def package_files(repo: Path) -> list[Path]:
    """The package.xml files of the source tree: git's list when there is one, else a walk
    that leaves out colcon's build, install and log trees (including build-asan and so on)."""
    try:
        listed = subprocess.run(
            ["git", "-C", str(repo), "ls-files", "*package.xml"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.split()
        return sorted(repo / f for f in listed)
    except (OSError, subprocess.CalledProcessError):
        return sorted(
            p
            for p in repo.rglob("package.xml")
            if not any(part.startswith(("build", "install", "log", ".git")) for part in p.parts)
        )


def own_packages(repo: Path) -> list[dict]:
    out = []
    for path in package_files(repo):
        text = path.read_text()
        name = re.search(r"<name>([^<]+)</name>", text).group(1)
        version = re.search(r"<version>([^<]+)</version>", text).group(1)
        licence = re.search(r"<license>([^<]+)</license>", text).group(1)
        out.append(
            {
                "SPDXID": spdx_id(name),
                "name": f"ros-lyrical-{name.replace('_', '-')}",
                "versionInfo": version,
                "supplier": "Organization: Open Physical Systems",
                "downloadLocation": "https://github.com/Open-Physical-Systems/Lagrange",
                "filesAnalyzed": False,
                "licenseConcluded": licence if licence == "Apache-2.0" else "NOASSERTION",
                "licenseDeclared": licence if licence == "Apache-2.0" else "NOASSERTION",
                "copyrightText": "NOASSERTION",
                "comment": f"{path.parent.relative_to(repo)}/package.xml",
            }
        )
    return out


def third_party(repo: Path) -> list[dict]:
    """Rows of the dependency tables: | Name | role | source | version | licence | ... |"""
    out = []
    seen = set()
    for line in (repo / "docs/design/dependencies.md").read_text().splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 5 or not line.startswith("|") or set(cells[0]) <= {"-", " "}:
            continue
        name, _role, source, version, licence = cells[:5]
        if name in {"Dependency", "Package", "Tool", "Data"} or name in seen:
            continue
        seen.add(name)
        out.append(
            {
                "SPDXID": spdx_id("dep-" + name),
                "name": re.sub(r"[`*]", "", name),
                "versionInfo": re.sub(r"[`*]", "", version) or "NOASSERTION",
                "supplier": "NOASSERTION",
                "downloadLocation": "NOASSERTION",
                "filesAnalyzed": False,
                "licenseConcluded": "NOASSERTION",
                "licenseDeclared": licence_of(licence),
                "copyrightText": "NOASSERTION",
                "comment": f"source: {source}; licence as recorded: {licence}",
            }
        )
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", type=Path)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    ours = own_packages(args.repo)
    theirs = third_party(args.repo)
    version = next(p["versionInfo"] for p in ours if p["name"] == "ros-lyrical-lagrange-core")
    now = datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ")
    relationships = [
        {
            "spdxElementId": "SPDXRef-DOCUMENT",
            "relationshipType": "DESCRIBES",
            "relatedSpdxElement": p["SPDXID"],
        }
        for p in ours
    ]
    relationships += [
        {
            "spdxElementId": spdx_id("lagrange_core"),
            "relationshipType": "DEPENDS_ON",
            "relatedSpdxElement": p["SPDXID"],
        }
        for p in theirs
    ]
    document = {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"lagrange-{version}",
        "documentNamespace": f"https://github.com/Open-Physical-Systems/Lagrange/sbom/{version}-"
        f"{uuid.uuid4()}",
        "creationInfo": {"created": now, "creators": ["Tool: lagrange make_sbom.py"]},
        "packages": ours + theirs,
        "relationships": relationships,
    }
    args.out.write_text(json.dumps(document, indent=1, ensure_ascii=False) + "\n")
    print(f"wrote {args.out}: {len(ours)} Lagrange packages, {len(theirs)} third-party")


if __name__ == "__main__":
    main()

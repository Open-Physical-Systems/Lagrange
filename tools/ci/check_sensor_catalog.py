#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The sensor catalog against sensors.md §9, both ways (RFC 0016 §5).

    tools/ci/check_sensor_catalog.py [--catalog FILE] [--doc FILE] [--verbose]

For every kind of modules/sensors/catalog/sensor_catalog.yaml, its section in
docs/design/domains/sensors.md (the `### <kind>` heading; `current_sensor and voltage_sensor`
holds two) is found, and:

- every parameter of the section's `| Parameter |` tables is in the catalog, and every catalog
  parameter is in a table, when the section has one (sections without a table describe their
  parameters in prose and are listed by --verbose);
- every catalog default is one of the numbers of its row's Default cell, in SI after the units
  written beside them (°, mV, µs, ...), within the doc's rounding (0.2 %), or for text the word
  itself;
- every signal of the section's `| Signal |` tables is in the catalog, and every catalog signal is
  named in the section.

Grouping rules for the tables (plan R10): a cell `a`, `b` names two parameters; `<q>/x` stands
for any one path segment; a cell `a` ... `b` (or …) names a and b and every catalog parameter
between them in the catalog's order; "(text)", "(also ...)" and the like are notes. A default
cell with a word only ("none", "the stage's only ...") checks nothing for a catalog parameter
without a numeric default. Exit 1 on any mismatch.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "modules/sensors/catalog/sensor_catalog.yaml"
DOC = ROOT / "docs/design/domains/sensors.md"

# Kinds documented outside §9 (the 4D imaging radar has its own section in §2).
SECTION_HEADINGS = {"imaging_radar": "4D imaging radar (as built)"}

# Factors to SI of the units the Default cells write beside their numbers.
UNIT_FACTORS = {
    "°": math.pi / 180.0,
    "deg": math.pi / 180.0,
    "mm": 1e-3,
    "cm": 1e-2,
    "km": 1e3,
    "µm": 1e-6,
    "um": 1e-6,
    "nm": 1e-9,
    "mv": 1e-3,
    "µv": 1e-6,
    "nv": 1e-9,
    "ma": 1e-3,
    "mΩ": 1e-3,
    "mω": 1e-3,
    "ms": 1e-3,
    "µs": 1e-6,
    "us": 1e-6,
    "ns": 1e-9,
    "khz": 1e3,
    "mhz": 1e6,
    "ghz": 1e9,
    "kpa": 1e3,
    "hpa": 1e2,
    "kn": 1e3,
    "%": 1e-2,
    "mah": 3.6,
    "mg": 1e-6,
    "µg": 1e-9,
    "g": 1e-3,
}
# The doc rounds to three or four figures (0.098 K for 200/2047 K).
RELATIVE_TOLERANCE = 5e-3
# Units with a slash, to SI.
SLASH_UNITS = {"mm/h": 1e-3 / 3600.0, "km/h": 1.0 / 3.6, "mg/m³": 1e-6, "µg/m³": 1e-9}

NUMBER = re.compile(
    r"(?<![\w.^/])([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)(?:\s*\u00d7\s*10\^?([+-]?\d+))?\s*"
    r"(°|[a-zA-Zµμ%Ω]+)?",
)
BACKTICK = re.compile(r"`([^`]+)`")


def sections(doc: str) -> dict[str, str]:
    """The text under each ### heading (up to the next ## or ###), by heading."""
    out: dict[str, str] = {}
    current = None
    lines: list[str] = []
    for line in doc.splitlines():
        if line.startswith("## ") or line.startswith("### "):
            if current is not None:
                out[current] = "\n".join(lines)
            current = line.lstrip("#").strip() if line.startswith("### ") else None
            lines = []
        elif current is not None:
            lines.append(line)
    if current is not None:
        out[current] = "\n".join(lines)
    return out


def section_of(kind: str, all_sections: dict[str, str]) -> str | None:
    if kind in SECTION_HEADINGS:
        return all_sections.get(SECTION_HEADINGS[kind])
    for heading, text in all_sections.items():
        names = [n.strip() for n in re.split(r"\band\b|,", heading)]
        if kind in names:
            return text
    return None


def split_row(line: str) -> list[str]:
    cells = re.split(r"(?<!\\)\|", line.strip().strip("|"))
    return [c.strip().replace("\\|", "|") for c in cells]


def tables(text: str, first: str) -> list[tuple[list[str], list[list[str]]]]:
    """Every Markdown table whose header's first cell is `first`: (header, rows)."""
    out = []
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        if lines[i].startswith("|") and split_row(lines[i])[0] == first:
            header = split_row(lines[i])
            rows = []
            i += 2  # the |---| line
            while i < len(lines) and lines[i].startswith("|"):
                rows.append(split_row(lines[i]))
                i += 1
            out.append((header, rows))
        else:
            i += 1
    return out


def names_in_cell(cell: str) -> list[str]:
    """The parameter names a Parameter cell gives, notes removed."""
    cell = re.sub(r"\((?:text|also [^)]*|[^)`]*)\)", "", cell)
    return [n.strip() for n in BACKTICK.findall(cell)]


def paths_of_segment(
    kind: str,
    segment: str,
    names: list[str],
    paths: list[str],
    problems: list[str],
    shared_paths: set[str],
) -> list[str]:
    """The catalog paths one `;`-separated part of a Parameter cell names."""
    out: list[str] = []
    if ("..." in segment.replace("/...", "") or "…" in segment) and len(names) >= 1:
        first, last = names[0], names[-1]
        head, _, suffix = first.partition("_")
        if suffix and (len(names) == 1 or last.partition("_")[2] == suffix):
            # fx_range ... tz_range: every per-axis parameter of that suffix.
            family = re.compile(rf"^[a-z0-9]{{{len(head)}}}_{re.escape(suffix)}$")
            return [p for p in paths if family.match(p)]
        if first in paths and last in paths:
            a, b = paths.index(first), paths.index(last)
            return paths[min(a, b) : max(a, b) + 1]
        problems.append(f"{kind}: row `{first}` ... `{last}`: its ends are not catalog parameters")
        return out
    for name in names:
        if name.endswith("/..."):
            found = [p for p in paths if p.startswith(name[:-3])]
        else:
            found = [p for p in paths if pattern_of(name).match(p)]
        if not found and not any(pattern_of(name).match(p) for p in shared_paths):
            problems.append(f"{kind}: sensors.md parameter `{name}` is not in the catalog")
        out += found
    return out


def pattern_of(name: str) -> re.Pattern[str]:
    return re.compile(
        "^"
        + re.sub(r"<[^>]+>", "[^/]+", re.escape(name).replace(r"\<", "<").replace(r"\>", ">"))
        + "$"
    )


def numbers_in(cell: str, unit_column: str) -> list[float]:
    """The numbers of a Default cell in SI: each with the unit written after it, else the unit
    column's."""
    cell = (
        cell.replace("\u2212", "-").replace("μ", "µ").replace("\u2009", " ").replace("\u202f", " ")
    )
    # 20 000 is twenty thousand.
    cell = re.sub(r"(?<=\d) (?=\d{3}\b)", "", cell)
    # Multiples and fractions of π, and plain fractions a/b: their values.
    cell = re.sub(
        r"(\d*\.?\d*)\s*π(?:\s*/\s*(\d+))?",
        lambda m: repr(
            (float(m.group(1)) if m.group(1) else 1.0)
            * math.pi
            / (float(m.group(2)) if m.group(2) else 1.0)
        ),
        cell,
    )
    cell = re.sub(
        r"(?<![\w.])(\d+(?:\.\d+)?)\s*/\s*(\d+(?:\.\d+)?)(?![\w.])",
        lambda m: repr(float(m.group(1)) / float(m.group(2))),
        cell,
    )
    for slash_unit, slash_factor in SLASH_UNITS.items():

        def to_si(m: re.Match[str], f: float = slash_factor) -> str:
            return repr(float(m.group(1)) * f)

        cell = re.sub(r"(\d+(?:\.\d+)?(?:e-?\d+)?)\s*" + re.escape(slash_unit), to_si, cell)
    # Leave out what a parenthesis says about where a number comes from (document numbers).
    values: list[float] = []
    column_factor = UNIT_FACTORS.get(
        unit_column.strip().lower(), UNIT_FACTORS.get(unit_column.strip(), 1.0)
    )
    for match in NUMBER.finditer(cell):
        text, power, unit = match.groups()
        try:
            value = float(text.replace("\u2212", "-"))
        except ValueError:
            continue
        if power is not None:
            value *= 10.0 ** int(power.replace("\u2212", "-"))
        factor: float | None = None
        if unit:
            factor = UNIT_FACTORS.get(unit) or UNIT_FACTORS.get(unit.lower())
        values.append(value)
        values.append(value * (factor if factor is not None else column_factor))
        if text.startswith("-") or cell[max(0, match.start() - 1)] == "±":
            values.append(-values[-1])
    if "±" in cell:
        values += [-v for v in values]
    if re.search(r"\bnone\b|\boff\b|\bno\b", cell, re.IGNORECASE):
        values.append(0.0)
    if "∞" in cell or re.search(r"\binf\b", cell):
        values += [math.inf, -math.inf]
    return values


def close(a: float, b: float) -> bool:
    if math.isinf(a) or math.isinf(b):
        return a == b
    return a == b or abs(a - b) <= RELATIVE_TOLERANCE * max(abs(a), abs(b))


def check_kind(
    kind: str,
    entry: dict[str, Any],
    text: str,
    verbose: bool,
    shared_signals: set[str],
    shared_paths: set[str],
    all_kinds: dict[str, Any],
) -> list[str]:
    problems: list[str] = []
    params = entry.get("params") or []
    paths = [p["path"] for p in params]
    by_path = {p["path"]: p for p in params}
    param_tables = tables(text, "Parameter") + tables(text, "Setting")
    if not param_tables:
        if verbose:
            print(f"{kind}: no parameter table in sensors.md (described in prose)")
    else:
        matched: set[str] = set()
        for header, rows in param_tables:
            has_unit = len(header) >= 4 and header[1] == "Unit"
            default_col = header.index("Default") if "Default" in header else None
            for row in rows:
                if not row or not row[0]:
                    continue
                row_paths: list[str] = []
                named = False
                for segment in row[0].split(";"):
                    names = names_in_cell(segment)
                    if not names:
                        continue
                    named = True
                    row_paths += paths_of_segment(
                        kind, segment, names, paths, problems, shared_paths
                    )
                if not named:
                    continue
                matched.update(row_paths)
                if default_col is None or default_col >= len(row):
                    continue
                cell = row[default_col]
                if not cell.strip() and re.search(r"\bas the\b", row[-1]):
                    cell = row[-1]  # an empty default "as the light sensor's"
                if not cell.strip():
                    continue
                if (other := re.search(r"\bas the ([a-z ]+?)'s", cell)) is not None:
                    # "as the light sensor's": the same parameter's default in that kind.
                    other_params = {
                        q["path"]: q
                        for q in (
                            all_kinds.get(other.group(1).replace(" ", "_"), {}).get("params") or []
                        )
                    }
                    for path in row_paths:
                        mine, theirs = (
                            by_path[path].get("default"),
                            other_params.get(path, {}).get("default"),
                        )
                        if mine != theirs:
                            problems.append(
                                f"{kind}: `{path}` default {mine} is not "
                                f"{other.group(1)}'s {theirs}"
                            )
                    continue
                unit = row[1] if has_unit else ""
                values = numbers_in(cell, unit)
                for path in row_paths:
                    p = by_path[path]
                    if "default" not in p:
                        continue
                    d = p["default"]
                    if isinstance(d, bool) or p.get("type") in ("text", "enum"):
                        if (
                            BACKTICK.search(cell)
                            and str(d).lower() not in cell.lower()
                            and not (isinstance(d, bool) or p.get("type") == "bool")
                        ):
                            problems.append(
                                f"{kind}: `{path}` default '{d}' is not in sensors.md's '{cell}'"
                            )
                        continue
                    try:
                        dv = float(d)
                    except (TypeError, ValueError):
                        problems.append(f"{kind}: `{path}` default '{d}' is not a number")
                        continue
                    if not any(close(dv, v) for v in values):
                        problems.append(
                            f"{kind}: `{path}` default {d} is not in sensors.md's '{cell}'"
                        )
        prose = BACKTICK.findall(re.sub(r"^\|.*$", "", text, flags=re.MULTILINE))
        for path in paths:
            if path in matched:
                continue
            if path in prose:
                # Named in the section's text, not its table (imaging_radar's RadarAPI_1 settings):
                # its default is not checked.
                if verbose:
                    print(f"{kind}: `{path}` is named in prose only; its default is not checked")
                continue
            problems.append(
                f"{kind}: catalog parameter `{path}` is in no sensors.md parameter table"
            )
    # Signals: a section shared by several kinds checks its tables against their union.
    signal_names = {s["name"] for s in entry.get("signals") or []}
    for _header, rows in tables(text, "Signal"):
        for row in rows:
            for name in BACKTICK.findall(row[0]):
                m = re.match(r"sensor/<n>/(.+)", name)
                if m and m.group(1) not in signal_names | shared_signals:
                    problems.append(f"{kind}: sensors.md signal `{name}` is not in the catalog")
    for name in signal_names:
        last = name.rsplit("/", 1)[-1]
        if not re.search(r"(?<![\w])" + re.escape(last) + r"(?![\w])", text):
            problems.append(
                f"{kind}: catalog signal `{name}` is not named in its sensors.md section"
            )
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--catalog", type=Path, default=CATALOG)
    parser.add_argument("--doc", type=Path, default=DOC)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    catalog = yaml.safe_load(args.catalog.read_text(encoding="utf-8"))
    doc = args.doc.read_text(encoding="utf-8")
    all_sections = sections(doc)
    problems: list[str] = []
    kinds = catalog.get("kinds") or {}
    for kind, entry in kinds.items():
        text = section_of(kind, all_sections)
        if text is None:
            problems.append(f"{kind}: no section `### {kind}` in {args.doc.name}")
            continue
        shared = {
            s["name"]
            for other, e in kinds.items()
            if other != kind and section_of(other, all_sections) is text
            for s in e.get("signals") or []
        }
        shared_paths = {
            p["path"]
            for other, e in kinds.items()
            if other != kind and section_of(other, all_sections) is text
            for p in e.get("params") or []
        }
        problems += check_kind(kind, entry, text, args.verbose, shared, shared_paths, kinds)
    # Every §9 kind section has a catalog entry.
    nine = doc[doc.index("## 9. Built-in custom sensor kinds") :]
    for heading in sections(nine):
        for name in re.split(r"\band\b|,", heading):
            name = name.strip()
            if re.fullmatch(r"[a-z0-9_]+", name) and name not in kinds:
                problems.append(f"sensors.md §9 kind `{name}` is not in the catalog")
    for p in problems:
        print(p, file=sys.stderr)
    if verbose_ok := not problems:
        print(f"sensor catalog: {len(kinds)} kinds agree with {args.doc.name}")
    return 0 if verbose_ok else 1


if __name__ == "__main__":
    sys.exit(main())

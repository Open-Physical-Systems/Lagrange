# SPDX-License-Identifier: Apache-2.0
"""Remove pip-only rosdep keys from the dependency fields of a bloom debian/control.

Usage: drop_pip_depends.py CONTROL ROSDEP_YAML
"""

import re
import sys
from pathlib import Path

import yaml


def pip_names(rosdep_yaml: Path) -> set[str]:
    names: set[str] = set()
    for rule in (yaml.safe_load(rosdep_yaml.read_text()) or {}).values():
        for os_rule in rule.values():
            if isinstance(os_rule, dict) and "pip" in os_rule:
                names.update(os_rule["pip"]["packages"])
    return names


def main() -> None:
    control, rosdep_yaml = Path(sys.argv[1]), Path(sys.argv[2])
    drop = pip_names(rosdep_yaml)
    fields = re.compile(r"^(Depends|Build-Depends|Pre-Depends|Recommends):(.*)$")
    out = []
    for line in control.read_text().splitlines():
        m = fields.match(line)
        if m:
            kept = [
                d.strip()
                for d in m.group(2).split(",")
                if d.strip() and d.strip().split()[0] not in drop
            ]
            line = f"{m.group(1)}: {', '.join(kept)}"
        out.append(line)
    control.write_text("\n".join(out) + "\n")


if __name__ == "__main__":
    main()

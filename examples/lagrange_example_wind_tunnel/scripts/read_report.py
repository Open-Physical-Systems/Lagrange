#!/usr/bin/env python3
"""Reads what the wind tunnel found."""

import json
import sys
from pathlib import Path


def main() -> int:
    report = json.loads(Path(sys.argv[1]).read_text())
    print(
        f"RESULT: Cd {report['drag_coefficient']:.2f}, Cl {report['lift_coefficient']:.2f}, "
        f"drag {report['force_n'][0]:.2f} N at {report['speed_m_s']:.0f} m/s, "
        f"Reynolds {report['reynolds']:.2g}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

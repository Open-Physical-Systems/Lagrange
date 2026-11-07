"""Derives the drive-cycle files test_battery_dataset reads from the LG HG2 data.

Kollmeyer, Naguib and Skells, "LG 18650HG2 Li-ion Battery Data", Mendeley Data v2, 2020,
doi:10.17632/b5mj79w5w9.2, CC BY 4.0. Each output row is one second of the cycler's 0.1 s log:
the mean current over that second (positive discharging), which moves the charge and the RC
pairs, and the current, voltage and case temperature at its end, where the voltage was read
(the series resistance's drop is the current's at that instant, not the second's mean). Usage:
python3 make_drive_cycles.py DIRECTORY (the downloaded CSVs, checked by SHA-256 below); writes the
files next to this script.
"""

import csv
import hashlib
import sys
from pathlib import Path

BASE = "https://data.mendeley.com/public-files/datasets/b5mj79w5w9/files"
SOURCES = {
    "551_US06.csv": (
        "us06_25degC.csv",
        f"{BASE}/ee36611b-918a-4aff-87e2-d1f5de2aea25/file_downloaded",
        "4e43498d29ee21da6d6dcd7ef47bbedcd2cd3ed92100a0e8094976dbd0fa2d96",
    ),
    "551_UDDS.csv": (
        "udds_25degC.csv",
        f"{BASE}/0723af12-3756-4bf9-a825-e672cc4b81f3/file_downloaded",
        "1d6433fa388be9d411f8554bcfd3410e64ea605a2545ccf94ce362809b4b688c",
    ),
    "551_LA92.csv": (
        "la92_25degC.csv",
        f"{BASE}/526591a4-a5fd-4cb6-a33e-6b126fadffff/file_downloaded",
        "baa7c87f649bdc247614b4b22ee590107e407e2bd45aedebb7ec3dd1022420e2",
    ),
    "556_US06.csv": (
        "us06_40degC.csv",
        f"{BASE}/aa1c1ded-1889-4253-8eec-db019bb950bd/file_downloaded",
        "c80068d34aacec1e79888aa0b1196914e4de032322156bcc26d69cfb2ea881ac",
    ),
    "567_US06.csv": (
        "us06_10degC.csv",
        f"{BASE}/37f67dac-ee9e-428c-b995-1e623abb2d88/file_downloaded",
        "5f7000176770e529d1dce909b74b83a8be32c86ae5c79fe95f00c3a2aebec940",
    ),
    "589_US06.csv": (
        "us06_0degC.csv",
        f"{BASE}/55746932-9400-4042-b7be-10eecc1856c3/file_downloaded",
        "d3b52e7c538fdee4451158526e5b83eab92c9698b22f6d88f551d009207b08a9",
    ),
}


def seconds(clock: str) -> float:
    hours, minutes, rest = clock.split(":")
    return int(hours) * 3600 + int(minutes) * 60 + float(rest)


def derive(source: Path, target: Path) -> int:
    lines = source.read_text(encoding="latin-1").splitlines()
    header = next(k for k, line in enumerate(lines) if line.startswith("Time Stamp"))
    rows: list[tuple[float, float, float, float, float]] = []
    for line in lines[header + 2 :]:
        x = line.split(",")
        if len(x) < 12 or not x[3] or not x[8]:
            continue
        # Prog Time, voltage, current (negative discharging in the log), temperature, Ah.
        rows.append((seconds(x[3]), float(x[8]), -float(x[9]), float(x[10]), -float(x[11])))
    start = rows[0][0]
    out: list[list[str]] = []
    second = 1
    total = 0.0
    count = 0
    for t, v, i, temp, _ah in rows:
        total += i
        count += 1
        if t - start >= second:
            out.append([f"{second}", f"{total / count:.4f}", f"{i:.4f}", f"{v:.5f}", f"{temp:.2f}"])
            second += 1
            total = 0.0
            count = 0
    with target.open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["t_s", "mean_current_A", "current_A", "voltage_V", "case_temperature_C"])
        w.writerows(out)
    return len(out)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage: make_drive_cycles.py DIRECTORY")
    directory = Path(sys.argv[1])
    here = Path(__file__).resolve().parent
    for name, (target, url, digest) in SOURCES.items():
        path = directory / name
        if not path.is_file():
            sys.exit(f"make_drive_cycles: {path} missing; download it from {url}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            sys.exit(f"make_drive_cycles: {path} is not the published file (SHA-256 differs)")
        print(f"{target}: {derive(path, here / target)} s")


if __name__ == "__main__":
    main()

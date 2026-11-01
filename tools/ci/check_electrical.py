#!/usr/bin/env python3
"""Checks the electrical network on circuits simple enough to work out by hand.

Each round draws new values for every circuit below. Each circuit is worked out three ways, none
using another's result: a closed-form formula here, ngspice on a netlist written here, and
Lagrange runs of a robot written here, its power system from an overlay, from the netlist, and
from a <lagrange> block in the URDF naming the netlist. A difference over the tolerance fails
the check (exit 1).

    tools/ci/check_electrical.py [--rounds N | --forever] [--seed S] [--verbose]

Needs the workspace sourced and ngspice. The circuits:

    wire          battery + harness + resistor: V = OCV * R / (R + r + w), terminals V + I w
    resistive     battery (OCV, r) + resistor: V = OCV * R / (R + r)
    constant_power battery + load of power P: V = (OCV + sqrt(OCV^2 - 4 P r)) / 2
    rc_pair       battery with an R1 C1 pair + resistor, at one time constant and at the end
    two_batteries two batteries + resistor: V = (V1/r1 + V2/r2) / (1/r1 + 1/r2 + 1/R)
    charge_drain  state of charge after a steady current: SoC = SoC0 - I t / Q
    motor         torque on a free flywheel: w = tau t / J, i = tau / (N eta Kt),
                  P = k R i^2 + Kt i N w (k 1.5 for a PMSM, 1 for brushed), then as constant_power
"""

from __future__ import annotations

import argparse
import itertools
import math
import random
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import lagrange
from ament_index_python.packages import get_package_prefix

MODEL_TOLERANCE = 0.005  # relative: Lagrange against the formula
SPICE_TOLERANCE = 0.001  # relative: ngspice against the formula
BUS_CAPACITANCE = 0.001  # F; the bus settles in r * C, well under a millisecond here
BUS = "bus/dc/voltage"


@dataclass
class Pack:
    """A battery with a flat voltage curve: one cell in series, as a plain voltage source."""

    name: str
    ocv: float  # V
    r0: float  # ohm
    r1: float = 0.0  # ohm
    c1: float = 0.0  # F
    capacity: float = 3600.0  # C
    soc: float = 1.0
    wire: float = 0.0  # ohm, the port's harness to the bus


@dataclass
class Check:
    """One quantity at one time. `spice` is None where ngspice has no such quantity."""

    what: str
    signal: str
    time: float
    formula: float
    spice: float | None = None


def battery_yaml(pack: Pack) -> str:
    pair = f"elec:battery:r1: {pack.r1}\n        elec:battery:c1: {pack.c1}" if pack.r1 else ""
    return f"""  - create: power/{pack.name}
    schemas:
      BatteryAPI_1:
        elec:battery:chemistry: lfp
        elec:battery:seriesCells: 1
        elec:battery:capacity: {pack.capacity}
        elec:battery:initialSoc: {pack.soc}
        elec:battery:ocvSoc: [0.0, 1.0]
        elec:battery:ocvVoltage: [{pack.ocv}, {pack.ocv}]
        elec:battery:r0: {pack.r0}
        {pair}
        elec:battery:cutoffVoltage: 0.1
        elec:battery:maxChargeCurrent: 1000.0
      PortAPI_1:dc_out:
        elec:port:dc_out:connectedTo: power/dc
        elec:port:dc_out:wireResistance: {pack.wire}
"""


def bus_yaml() -> str:
    return f"""  - create: power/dc
    schemas:
      ElectricalBusAPI_1:
        elec:bus:capacitance: {BUS_CAPACITANCE}
        elec:bus:undervoltageTrip: 0.5
        elec:bus:overvoltageTrip: 1000.0
"""


def load_yaml(model: str, power: float) -> str:
    return f"""  - create: power/load
    schemas:
      ElectricalLoadAPI_1:
        elec:load:model: {model}
        elec:load:power: {power}
        elec:powerPort: power/dc
"""


def motor_yaml(kind: str, r: float, kt: float, n: float, eta: float) -> str:
    return f"""  - prim: spin
    create_child: drive
    schemas:
      ElectricalMotorAPI_1:
        elec:motorType: {kind}
        elec:windingResistance: {r}
        elec:inductance: 0.0003
        elec:torqueConstant: {kt}
        elec:polePairs: 1
        elec:gearRatio: {n}
        elec:gearEfficiency: {eta}
        elec:maxCurrent: 1000.0
        elec:joint: spin
        elec:powerPort: power/dc
"""


def idle_motor_yaml() -> str:
    """The server runs the network only for a robot with a motor; this one is never commanded."""
    return motor_yaml("pmsm", 0.1, 0.1, 1.0, 1.0)


def urdf(inertia: float) -> str:
    """A flywheel about the vertical on a fixed base: gravity does no work on it."""
    i = inertia
    block = f'<inertia ixx="{i}" iyy="{i}" ixy="0" ixz="0" iyz="0" izz="{i}"/>'
    return f"""<robot name="bench">
  <link name="world"/>
  <joint name="mount" type="fixed"><parent link="world"/><child link="base"/></joint>
  <link name="base"><inertial><mass value="1"/>{block}</inertial></link>
  <joint name="spin" type="continuous"><parent link="base"/><child link="wheel"/>
    <axis xyz="0 0 1"/><limit effort="1000" velocity="1000"/></joint>
  <link name="wheel"><inertial><mass value="1"/>{block}</inertial></link>
</robot>
"""


def directives(pack: Pack) -> str:
    """What SPICE cannot say about a battery."""
    return (
        f"*lagrange battery V_{pack.name} capacity={pack.capacity} cutoff=0.1 "
        f"max_charge=1000 soc={pack.soc}\n"
    )


BUS_DIRECTIVE = "*lagrange bus dc undervoltage=0.5 overvoltage=1000\n"
IDLE_MOTOR = "*lagrange motor spin node=dc name=drive r=0.1 l=0.0003 kt=0.1 imax=1000\n"


def run_both(
    folder: Path,
    overlay: str,
    netlist: str,
    inertia: float,
    checks: list[Check],
    torque: float | None = None,
) -> list[tuple[Check, float, float | None]]:
    """The checks from a run with the overlay, then from a run with the netlist in its place."""
    file = folder / "bench.overlay.yaml"
    file.write_text("schema_version: 1\napply:\n" + overlay)
    rows = compare(checks, run_lagrange(folder, file, inertia, checks, torque))
    circuit = folder / "power.cir"
    circuit.write_text("* bench\n" + BUS_DIRECTIVE + netlist + ".end\n")
    imported = [Check(c.what + " [netlist]", c.signal, c.time, c.formula) for c in checks]
    rows += compare(imported, run_lagrange(folder, circuit, inertia, imported, torque))
    # The same netlist, named from the URDF: the directives move into its <lagrange> block.
    lines = netlist.splitlines()
    plain = [line for line in lines if not line.startswith("*lagrange")]
    circuit.write_text("* bench\n" + "\n".join(plain) + "\n.end\n")
    block = "".join(xml_of(line) for line in [BUS_DIRECTIVE.strip(), *lines])
    tagged = folder / "tagged.urdf"
    tagged.write_text(
        urdf(inertia).replace(
            "</robot>",
            f'<lagrange><schematic file="power.cir"/>{block}</lagrange></robot>',
        )
    )
    in_urdf = [Check(c.what + " [URDF]", c.signal, c.time, c.formula) for c in checks]
    rows += compare(in_urdf, read_run(tagged, in_urdf, torque))
    return rows


def xml_of(line: str) -> str:
    """A *lagrange line as an element of the <lagrange> block; other lines as nothing."""
    words = line.split()
    if not words or words[0] != "*lagrange":
        return ""
    named_by = {"battery": "source", "bus": "node", "motor": "joint"}[words[1]]
    values = dict(word.split("=") for word in words[3:])
    attributes = " ".join(f'{key}="{value}"' for key, value in values.items())
    return f'<{words[1]} {named_by}="{words[2]}" {attributes}/>'


def run_lagrange(
    folder: Path, extras: Path, inertia: float, checks: list[Check], torque: float | None
) -> list[float]:
    """Builds the robot with the tool users have and reads each check's signal at its time."""
    robot = folder / "bench.urdf"
    robot.write_text(urdf(inertia))
    usd = folder / "bench.usda"
    tool = Path(get_package_prefix("lagrange_server")) / "lib/lagrange_server/lagrange_import"
    done = subprocess.run(
        [str(tool), str(robot), str(usd), "--overlay", str(extras)],
        check=False,
        capture_output=True,
        text=True,
    )
    if done.returncode != 0:
        raise RuntimeError(done.stderr)
    return read_run(usd, checks, torque)


def read_run(robot: Path, checks: list[Check], torque: float | None) -> list[float]:
    """Runs the robot and reads each check's signal at its time."""
    sim = lagrange.Simulation(str(robot), world=None, joint_control=torque is not None)
    if torque is not None and not sim.command_joint("spin", "effort", torque):
        raise RuntimeError("the joint took no effort command")
    values = []
    for check in sorted(checks, key=lambda c: c.time):
        sim.step_until(check.time)
        values.append((check, sim.read(check.signal)[0]))
    return [next(v for c, v in values if c is check) for check in checks]


def run_spice(folder: Path, elements: list[str], times: list[float]) -> list[float]:
    """The bus voltage at each time, from ngspice on the elements given (bus node `dc`)."""
    end = max(times)
    # Finer than the bus's r * C, or a constant-power source's Newton steps can leave for its
    # negative-voltage solution.
    step = min(end / 2000, 2e-5)
    data = folder / "out.txt"
    netlist = folder / "circuit.cir"
    lines = [
        "* hand check",
        *elements,
        ".options method=gear",
        f".tran {step:.6g} {end:.6g} 0 {step:.6g} uic",
        ".control",
        "run",
        f"wrdata {data} v(dc)",
        ".endc",
        ".end",
        "",
    ]
    netlist.write_text("\n".join(lines))
    subprocess.run(["ngspice", "-b", str(netlist)], check=True, capture_output=True, text=True)
    rows = [[float(x) for x in line.split()] for line in data.read_text().splitlines()]
    return [min(rows, key=lambda row: abs(row[0] - t))[1] for t in times]


def spice_pack(pack: Pack, start: float) -> list[str]:
    n = pack.name
    lines = [f"V_{n} {n}_a 0 DC {pack.ocv}", f"R_{n} {n}_a {n}_b {pack.r0}"]
    if pack.r1:
        lines += [f"R1_{n} {n}_b dc {pack.r1}", f"C1_{n} {n}_b dc {pack.c1} IC=0"]
    else:
        lines.append(f"Rw_{n} {n}_b dc 1e-9")
    return [f"C_bus dc 0 {BUS_CAPACITANCE} IC={start}"] * (n == "a") + lines


def netlist_of(elements: list[str], *packs: Pack) -> str:
    return "\n".join(elements) + "\n" + "".join(directives(p) for p in packs)


def resistive(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    pack = Pack("a", rng.uniform(12, 60), rng.uniform(0.02, 0.5))
    power = rng.uniform(50, 800)
    load = pack.ocv**2 / power  # Lagrange's load is its power at the voltage the bus starts at
    volts = pack.ocv * load / (load + pack.r0)
    amps = volts / load
    checks = [
        Check("bus voltage (V)", BUS, 0.2, volts),
        Check("load current (A)", "load/load/current", 0.2, amps),
        Check("battery current (A)", "battery/a/current", 0.2, amps),
    ]
    elements = [*spice_pack(pack, pack.ocv), f"R_load dc 0 {load}"]
    checks[0].spice = run_spice(folder, elements, [0.2])[0]
    overlay = bus_yaml() + battery_yaml(pack) + load_yaml("resistive", power) + idle_motor_yaml()
    return run_both(folder, overlay, netlist_of(elements, pack) + IDLE_MOTOR, 1.0, checks)


def wire(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    pack = Pack("a", rng.uniform(12, 60), rng.uniform(0.02, 0.2))
    pack.wire = rng.uniform(0.02, 0.5)
    power = rng.uniform(50, 800)
    start = pack.ocv  # with nothing drawn, the harness drops nothing
    load = start**2 / power
    amps = pack.ocv / (load + pack.r0 + pack.wire)
    volts = amps * load
    checks = [
        Check("bus voltage (V)", BUS, 0.2, volts),
        Check("battery current (A)", "battery/a/current", 0.2, amps),
    ]
    # Node a_b is the battery's terminal, the harness the resistor on to the bus.
    elements = [
        f"C_bus dc 0 {BUS_CAPACITANCE} IC={start}",
        f"V_a a_a 0 DC {pack.ocv}",
        f"R_a a_a a_b {pack.r0}",
        f"Rw_a a_b dc {pack.wire}",
        f"R_load dc 0 {load}",
    ]
    checks[0].spice = run_spice(folder, elements, [0.2])[0]
    overlay = bus_yaml() + battery_yaml(pack) + load_yaml("resistive", power) + idle_motor_yaml()
    terminals = [
        Check("battery terminals (V)", "battery/a/terminal_voltage", 0.2, volts + amps * pack.wire),
        Check("battery heat (W)", "battery/a/loss_power", 0.2, amps**2 * pack.r0),
    ]
    file = folder / "terminals.overlay.yaml"
    file.write_text("schema_version: 1\napply:\n" + overlay)
    rows = compare(terminals, run_lagrange(folder, file, 1.0, terminals, None))
    netlist = netlist_of(elements, pack) + IDLE_MOTOR
    return rows + run_both(folder, overlay, netlist, 1.0, checks)


def constant_power(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    pack = Pack("a", rng.uniform(12, 60), rng.uniform(0.02, 0.3))
    power = rng.uniform(0.05, 0.6) * pack.ocv**2 / (4 * pack.r0)  # under the most it can give
    volts = (pack.ocv + math.sqrt(pack.ocv**2 - 4 * power * pack.r0)) / 2
    checks = [
        Check("bus voltage (V)", BUS, 0.2, volts),
        Check("load current (A)", "load/load/current", 0.2, power / volts),
    ]
    # As `lagrange export spice` writes it; the gate keeps Newton from the negative solution.
    source = f"B_load dc 0 I = {power} / max(V(dc), 0.001) * ((V(dc) >= 0) ? 1 : 0)"
    elements = [*spice_pack(pack, pack.ocv), source]
    checks[0].spice = run_spice(folder, elements, [0.2])[0]
    overlay = (
        bus_yaml() + battery_yaml(pack) + load_yaml("constant_power", power) + idle_motor_yaml()
    )
    return run_both(folder, overlay, netlist_of(elements, pack) + IDLE_MOTOR, 1.0, checks)


def rc_pair(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    pack = Pack("a", rng.uniform(12, 60), rng.uniform(0.02, 0.2))
    pack.r1 = rng.uniform(0.05, 0.5)
    pack.c1 = rng.uniform(1, 10)
    power = rng.uniform(100, 800)
    load = pack.ocv**2 / power
    through = pack.r0 + load  # what the pair's voltage v1 drives besides R1
    settled = pack.ocv * pack.r1 / (pack.r1 + through)
    tau = pack.c1 * pack.r1 * through / (pack.r1 + through)

    def volts(t: float) -> float:
        v1 = settled * (1 - math.exp(-t / tau))
        return (pack.ocv - v1) * load / through

    times = [tau, 8 * tau]
    checks = [Check(f"bus voltage at {t:.3g} s (V)", BUS, t, volts(t)) for t in times]
    elements = [*spice_pack(pack, volts(0)), f"R_load dc 0 {load}"]
    spice = run_spice(folder, elements, times)
    for check, value in zip(checks, spice, strict=True):
        check.spice = value
    overlay = bus_yaml() + battery_yaml(pack) + load_yaml("resistive", power) + idle_motor_yaml()
    return run_both(folder, overlay, netlist_of(elements, pack) + IDLE_MOTOR, 1.0, checks)


def two_batteries(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    a = Pack("a", rng.uniform(40, 50), rng.uniform(0.05, 0.3))
    b = Pack("b", a.ocv + rng.uniform(-1, 1), rng.uniform(0.05, 0.3))
    power = rng.uniform(100, 500)
    # Lagrange's bus starts at the batteries' parallel voltage; the load is its power there.
    start = (a.ocv / a.r0 + b.ocv / b.r0) / (1 / a.r0 + 1 / b.r0)
    load = start**2 / power
    volts = (a.ocv / a.r0 + b.ocv / b.r0) / (1 / a.r0 + 1 / b.r0 + 1 / load)
    checks = [
        Check("bus voltage (V)", BUS, 0.2, volts),
        Check("battery a current (A)", "battery/a/current", 0.2, (a.ocv - volts) / a.r0),
        Check("battery b current (A)", "battery/b/current", 0.2, (b.ocv - volts) / b.r0),
    ]
    elements = [*spice_pack(a, start), *spice_pack(b, start), f"R_load dc 0 {load}"]
    checks[0].spice = run_spice(folder, elements, [0.2])[0]
    overlay = (
        bus_yaml()
        + battery_yaml(a)
        + battery_yaml(b)
        + load_yaml("resistive", power)
        + idle_motor_yaml()
    )
    return run_both(folder, overlay, netlist_of(elements, a, b) + IDLE_MOTOR, 1.0, checks)


def charge_drain(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    pack = Pack("a", 48.0, rng.uniform(0.02, 0.2), capacity=rng.uniform(500, 2000))
    power = rng.uniform(200, 800)
    load = pack.ocv**2 / power
    amps = pack.ocv / (load + pack.r0)
    seconds = 3.0
    checks = [
        Check("state of charge", "battery/a/soc", seconds, 1 - amps * seconds / pack.capacity)
    ]
    overlay = bus_yaml() + battery_yaml(pack) + load_yaml("resistive", power) + idle_motor_yaml()
    elements = [*spice_pack(pack, pack.ocv), f"R_load dc 0 {load}"]
    return run_both(folder, overlay, netlist_of(elements, pack) + IDLE_MOTOR, 1.0, checks)


def motor(rng: random.Random, folder: Path) -> list[tuple[Check, float, float | None]]:
    kind, k = rng.choice([("pmsm", 1.5), ("dc_brushed", 1.0)])
    pack = Pack("a", rng.uniform(36, 60), rng.uniform(0.02, 0.2))
    r = rng.uniform(0.1, 0.5)
    kt = rng.uniform(0.05, 0.2)
    n = rng.uniform(5, 20)
    eta = rng.uniform(0.8, 1.0)
    torque = rng.uniform(1, 5)
    inertia = rng.uniform(1, 3)
    amps = torque / (n * eta * kt)
    seconds = 0.5

    def power(t: float) -> float:
        speed = torque * t / inertia
        return k * r * amps**2 + kt * amps * n * speed

    def volts(t: float) -> float:
        return (pack.ocv + math.sqrt(pack.ocv**2 - 4 * power(t) * pack.r0)) / 2

    checks = [
        Check("joint speed (rad/s)", "joint/spin/velocity", seconds, torque * seconds / inertia),
        Check("joint torque (N m)", "joint/spin/torque", seconds, torque),
        Check("motor q current (A)", "motor/drive/current_q", seconds, amps),
        Check(
            "drive bus current (A)",
            "motor/drive/bus_current",
            seconds,
            power(seconds) / volts(seconds),
        ),
        Check("bus voltage (V)", BUS, seconds, volts(seconds)),
    ]
    wiring = spice_pack(pack, pack.ocv)
    elements = [
        *wiring,
        f"V_p p 0 PWL(0 {power(0)} {seconds} {power(seconds)})",
        "B_drive dc 0 I = V(p) / max(V(dc), 0.001)",
    ]
    checks[4].spice = run_spice(folder, elements, [seconds])[0]
    overlay = bus_yaml() + battery_yaml(pack) + motor_yaml(kind, r, kt, n, eta)
    drive = (
        f"*lagrange motor spin node=dc name=drive type={kind} r={r} l=0.0003 kt={kt} imax=1000 "
        f"gear={n} efficiency={eta} pole_pairs=1\n"
    )
    return run_both(folder, overlay, netlist_of(wiring, pack) + drive, inertia, checks, torque)


def compare(checks: list[Check], values: list[float]) -> list[tuple[Check, float, float | None]]:
    return [(c, v, c.spice) for c, v in zip(checks, values, strict=True)]


def error(value: float, formula: float) -> float:
    return abs(value - formula) / max(abs(formula), 1e-9)


CIRCUITS = {
    "wire": wire,
    "resistive": resistive,
    "constant_power": constant_power,
    "rc_pair": rc_pair,
    "two_batteries": two_batteries,
    "charge_drain": charge_drain,
    "motor": motor,
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Checks the electrical network against hand formulas and ngspice."
    )
    parser.add_argument("--rounds", type=int, default=5, help="rounds of every circuit (default 5)")
    parser.add_argument("--forever", action="store_true", help="keep drawing until one fails")
    parser.add_argument("--seed", type=int, default=1, help="same seed, same circuits (default 1)")
    parser.add_argument("--verbose", action="store_true", help="print every quantity")
    args = parser.parse_args()
    rng = random.Random(args.seed)
    checked = 0
    failures = 0
    worst = {"Lagrange": 0.0, "ngspice": 0.0}
    rounds = itertools.count(1) if args.forever else range(1, args.rounds + 1)
    for round_number in rounds:
        for name, circuit in CIRCUITS.items():
            with tempfile.TemporaryDirectory() as folder:
                results = circuit(rng, Path(folder))
            for check, value, spice in results:
                checked += 1
                model = error(value, check.formula)
                wire = error(spice, check.formula) if spice is not None else 0.0
                worst["Lagrange"] = max(worst["Lagrange"], model)
                worst["ngspice"] = max(worst["ngspice"], wire)
                bad = model > MODEL_TOLERANCE or wire > SPICE_TOLERANCE
                failures += bad
                if bad or args.verbose:
                    third = f"{spice:.6g}" if spice is not None else "-"
                    print(
                        f"{'FAIL' if bad else 'ok':4} round {round_number} {name:15}"
                        f"{check.what:32} formula {check.formula:<11.6g}"
                        f"ngspice {third:<11} Lagrange {value:.6g}"
                    )
        if args.forever and failures:
            break
        print(
            f"round {round_number}: {checked} quantities, {failures} failed; worst off the "
            f"formula: Lagrange {worst['Lagrange']:.3%}, ngspice {worst['ngspice']:.3%}",
            flush=True,
        )
    print("FAILED" if failures else "PASSED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

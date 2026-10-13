"""Simulation profiles: parsing, validation against profiles/schema.json, canonical text.

A profile is YAML text. The server is the authority on whether it is valid: the viewer sends
text and gets issues back, so both sides always use the same schema. The domain facts below
(what can replace a domain, which backends exist, what a step costs) are what a real server
reads from its module registry; here they are sample data for the mock server.
"""

from __future__ import annotations

import contextlib
import json
import os
from dataclasses import dataclass, field
from functools import cache
from pathlib import Path
from typing import Any

import jsonschema
import yaml
from lagrange.viewer.v1 import command_pb2, profile_pb2

SCHEMA_VERSION = 1

# The eight physics domains, in the order every user interface lists them.
CORE_DOMAINS = (
    "rigid_body",
    "electrical",
    "thermal",
    "aerodynamics",
    "weather",
    "emi",
    "radiation",
    "sensors",
)
# Additional modules. Whether a robot uses them depends on its class.
EXTRA_DOMAINS = ("comms", "traffic", "vehicle_dynamics", "rotor_aero", "hydrodynamics")

_NS_PER_S = 1_000_000_000


@dataclass(frozen=True)
class Substitute:
    token: str  # what the profile says: "zero", "replay", ...
    kind: str  # ideal | constant | replay
    description: str
    needs_source: bool = False


@dataclass(frozen=True)
class DomainFacts:
    substitutes: tuple[Substitute, ...]
    backends: tuple[str, ...] = ()
    fidelities: tuple[str, ...] = ()
    default_dt_ns: int = 0  # 0: the domain has no step of its own
    min_dt_ns: int = 0
    max_dt_ns: int = 0
    # CPU milliseconds per simulated second at the default step. Sample data.
    cost_ms_per_sim_s: float = 0.0
    depends_on: tuple[str, ...] = ()
    description: str = ""


def _replay(what: str) -> Substitute:
    return Substitute("replay", "replay", f"recorded {what} from an MCAP file", needs_source=True)


FACTS: dict[str, DomainFacts] = {
    "rigid_body": DomainFacts(
        substitutes=(
            Substitute("fixed", "ideal", "robot held in place"),
            Substitute("load_profile", "constant", "torque against speed from a table"),
            _replay("poses and joint states"),
        ),
        backends=("mujoco", "jolt", "physx"),
        default_dt_ns=1_000_000,
        min_dt_ns=100_000,
        max_dt_ns=10_000_000,
        cost_ms_per_sim_s=310.0,
        depends_on=("electrical", "aerodynamics", "weather"),
        description="contacts, joints",
    ),
    "electrical": DomainFacts(
        substitutes=(
            Substitute("ideal", "ideal", "commanded torque delivered exactly"),
            _replay("currents and voltages"),
        ),
        fidelities=("averaged", "ideal_bus", "fmu"),
        default_dt_ns=100_000,
        min_dt_ns=10_000,
        max_dt_ns=1_000_000,
        cost_ms_per_sim_s=120.0,
        depends_on=("rigid_body", "thermal"),
        description="battery, motors, bus",
    ),
    "thermal": DomainFacts(
        substitutes=(
            Substitute("ambient", "constant", "everything stays at ambient temperature"),
            _replay("temperatures"),
        ),
        default_dt_ns=100_000_000,
        min_dt_ns=1_000_000,
        max_dt_ns=1_000_000_000,
        cost_ms_per_sim_s=4.0,
        depends_on=("electrical", "weather"),
        description="lumped network",
    ),
    "aerodynamics": DomainFacts(
        substitutes=(
            Substitute("zero", "ideal", "no drag on any link"),
            _replay("wrenches"),
        ),
        fidelities=("drag", "drag_plus_field", "lbm"),
        default_dt_ns=1_000_000,
        min_dt_ns=100_000,
        max_dt_ns=100_000_000,
        cost_ms_per_sim_s=40.0,
        depends_on=("rigid_body", "weather"),
        description="drag, wind field",
    ),
    "weather": DomainFacts(
        substitutes=(Substitute("clear", "constant", "clear, dry and still air"),),
        default_dt_ns=10_000_000,
        min_dt_ns=1_000_000,
        max_dt_ns=1_000_000_000,
        cost_ms_per_sim_s=6.0,
        description="rain, fog, dust",
    ),
    "emi": DomainFacts(
        substitutes=(
            Substitute("zero", "ideal", "no field anywhere"),
            _replay("field samples"),
        ),
        default_dt_ns=10_000_000,
        min_dt_ns=100_000,
        max_dt_ns=100_000_000,
        cost_ms_per_sim_s=50.0,
        depends_on=("electrical", "rigid_body"),
        description="sources, couplings",
    ),
    "radiation": DomainFacts(
        substitutes=(Substitute("zero", "ideal", "no dose"),),
        default_dt_ns=1_000_000_000,
        min_dt_ns=100_000_000,
        max_dt_ns=10_000_000_000,
        cost_ms_per_sim_s=20.0,
        depends_on=("rigid_body",),
        description="dose fields",
    ),
    "sensors": DomainFacts(
        substitutes=(
            Substitute("ground_truth", "ideal", "no rendering; true state is published"),
            _replay("sensor messages"),
        ),
        default_dt_ns=5_000_000,
        min_dt_ns=1_000_000,
        max_dt_ns=1_000_000_000,
        cost_ms_per_sim_s=90.0,
        depends_on=("rigid_body", "weather", "emi"),
        description="lidar, camera, IMU",
    ),
    "comms": DomainFacts(
        substitutes=(Substitute("ideal", "ideal", "every packet arrives at once"),),
        default_dt_ns=10_000_000,
        min_dt_ns=1_000_000,
        max_dt_ns=1_000_000_000,
        cost_ms_per_sim_s=20.0,
        depends_on=("emi", "rigid_body"),
        description="link quality",
    ),
    "traffic": DomainFacts(
        substitutes=(Substitute("zero", "ideal", "no other road users"),),
        backends=("builtin", "sumo"),
        default_dt_ns=100_000_000,
        min_dt_ns=10_000_000,
        max_dt_ns=1_000_000_000,
        cost_ms_per_sim_s=15.0,
        description="pedestrians, vehicles",
    ),
    "vehicle_dynamics": DomainFacts(
        substitutes=(Substitute("ideal", "ideal", "rigid wheels, no tyre model"),),
        default_dt_ns=1_000_000,
        description="tyres, suspension",
    ),
    "rotor_aero": DomainFacts(
        substitutes=(Substitute("zero", "ideal", "no rotor thrust"),),
        fidelities=("coefficients", "bem"),
        default_dt_ns=1_000_000,
        description="rotor thrust and torque",
    ),
    "hydrodynamics": DomainFacts(
        substitutes=(Substitute("zero", "ideal", "no water forces"),),
        default_dt_ns=1_000_000,
        description="marine vehicles",
    ),
}

# Modules the sample robots (wheeled, arms) have no use for.
_UNUSED_BY_GROUND_ROBOTS = ("vehicle_dynamics", "rotor_aero", "hydrodynamics")


class ProfileError(ValueError):
    """The schema file cannot be found or read."""


@cache
def schema_text() -> str:
    """profiles/schema.json: from LAGRANGE_PROFILE_SCHEMA, the installed package, or the tree."""
    candidates: list[Path] = []
    override = os.environ.get("LAGRANGE_PROFILE_SCHEMA")
    if override:
        candidates.append(Path(override))
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )

        with contextlib.suppress(PackageNotFoundError):
            candidates.append(
                Path(get_package_share_directory("lagrange_profiles")) / "schema.json"
            )
    except ImportError:
        pass
    candidates.append(Path(__file__).resolve().parents[2] / "profiles" / "schema.json")
    for candidate in candidates:
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    raise ProfileError(
        "profiles/schema.json not found; set LAGRANGE_PROFILE_SCHEMA or build lagrange_profiles"
    )


@cache
def _validator() -> jsonschema.Draft202012Validator:
    return jsonschema.Draft202012Validator(json.loads(schema_text()))


@dataclass
class Issue:
    message: str
    pointer: str = ""
    error: bool = True


@dataclass
class Checked:
    """A profile after validation. `data` is None when the text is not usable at all."""

    data: dict[str, Any] | None
    issues: list[Issue] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return self.data is not None and not any(issue.error for issue in self.issues)

    def to_proto(self) -> command_pb2.ProfileValidation:
        result = command_pb2.ProfileValidation(valid=self.valid)
        if self.data is not None and isinstance(self.data.get("domains"), dict):
            result.wiring.CopyFrom(wiring(self.data))
        for issue in self.issues:
            result.issues.add(
                severity=(
                    command_pb2.ProfileIssue.SEVERITY_ERROR
                    if issue.error
                    else command_pb2.ProfileIssue.SEVERITY_WARNING
                ),
                json_pointer=issue.pointer,
                message=issue.message,
            )
        return result


def _pointer(parts: Any) -> str:
    return "".join(f"/{part}" for part in parts)


def check(text: str) -> Checked:
    """Parse and validate profile text. Never raises for bad input."""
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as error:
        mark = getattr(error, "problem_mark", None)
        problem = getattr(error, "problem", None) or "not valid YAML"
        where = f"line {mark.line + 1}: " if mark is not None else ""
        return Checked(None, [Issue(f"{where}{problem}")])
    if not isinstance(data, dict):
        return Checked(None, [Issue("a profile must be a mapping with 'stage' and 'domains'")])

    issues: list[Issue] = []
    errors = sorted(_validator().iter_errors(data), key=lambda e: list(map(str, e.absolute_path)))
    for error in errors:
        message = error.message
        if len(message) > 200:
            message = message[:197] + "..."
        issues.append(Issue(message, _pointer(error.absolute_path)))

    domains = data.get("domains")
    if isinstance(domains, dict):
        for name, settings in domains.items():
            if not isinstance(settings, dict):
                continue
            step = settings.get("dt")
            if isinstance(step, (int, float)) and not isinstance(step, bool) and step > 0:
                ns = step * _NS_PER_S
                if abs(ns - round(ns)) > 1e-6:
                    issues.append(
                        Issue("dt must be a whole number of nanoseconds", f"/domains/{name}/dt")
                    )
            facts = FACTS.get(str(name))
            if facts is None or settings.get("enabled") is not False:
                continue
            token = settings.get("substitute")
            if isinstance(token, dict):
                token = token.get("kind")
            known = {substitute.token for substitute in facts.substitutes}
            known |= {substitute.kind for substitute in facts.substitutes}
            if isinstance(token, str) and token not in known:
                issues.append(
                    Issue(
                        f"'{token}' cannot replace {name}; choose one of "
                        + ", ".join(substitute.token for substitute in facts.substitutes),
                        f"/domains/{name}/substitute",
                    )
                )
    return Checked(data, issues)


# --- wiring ------------------------------------------------------------------------------------

# What each module reads from another: (writer, reader, signal, symbol, unit, stiff).
# A real server takes this from the SignalSpec lists of its modules (invariant 2); here it is
# the same information written out for the modules the profile can name.
_WIRES: tuple[tuple[str, str, str, str, str, bool], ...] = (
    ("electrical", "rigid_body", "joint/*/torque", "τ", "N·m", True),
    ("rigid_body", "electrical", "joint/*/velocity", "ω", "rad/s", True),
    ("electrical", "thermal", "*/loss_power", "P", "W", False),
    ("thermal", "electrical", "*/temperature", "T", "K", False),
    ("aerodynamics", "rigid_body", "link/*/ext_wrench", "F, τ", "N, N·m", False),
    ("rigid_body", "aerodynamics", "link/*/velocity", "v", "m/s", False),
    ("weather", "aerodynamics", "weather/wind", "v_wind", "m/s", False),
    ("weather", "rigid_body", "contact/*/friction_scale", "μ_wet", "1", False),
    ("weather", "thermal", "weather/ambient", "T_amb", "K", False),
    ("weather", "sensors", "weather/extinction", "sigma_ext", "1/m", False),
    ("rigid_body", "sensors", "body/*/pose", "pose", "m", False),
    ("emi", "sensors", "emi/flux_density", "B", "T", False),
    ("electrical", "emi", "motor/*/current", "I", "A", False),
    ("rigid_body", "emi", "body/*/pose", "pose", "m", False),
    ("rigid_body", "radiation", "body/*/pose", "pose", "m", False),
    ("emi", "comms", "emi/noise_floor", "N0", "dBm", False),
    ("rigid_body", "comms", "body/*/pose", "pose", "m", False),
)
_GPU_DOMAINS = ("sensors", "weather")


def wiring(data: dict[str, Any]) -> command_pb2.Wiring:
    """The modules and substitutes the profile names, and the signals between them."""
    result = command_pb2.Wiring()
    domains = data.get("domains", {})
    present: dict[str, dict[str, Any]] = {
        name: settings for name, settings in domains.items() if isinstance(settings, dict)
    }
    steps: list[int] = []
    for name in (*CORE_DOMAINS, *EXTRA_DOMAINS):
        settings = present.get(name)
        facts = FACTS.get(name)
        if settings is None or facts is None:
            continue
        enabled = bool(settings.get("enabled", False))
        token = settings.get("substitute")
        if isinstance(token, dict):
            token = token.get("kind")
        node = result.nodes.add(module=name, domain=name, substitute=not enabled)
        if enabled:
            node.detail = str(
                settings.get("backend")
                or settings.get("fidelity")
                or (facts.backends or facts.fidelities or ("built-in",))[0]
            )
            node.dt_ns = _step_ns(settings, facts)
            node.on_gpu = name in _GPU_DOMAINS
            if node.dt_ns > 0:
                steps.append(node.dt_ns)
        else:
            node.detail = str(token or (facts.substitutes[0].token if facts.substitutes else ""))
    by_name = {node.module: node for node in result.nodes}
    for writer, reader, signal, symbol, unit, stiff in _WIRES:
        if writer not in by_name or reader not in by_name:
            continue
        result.wires.add(
            signal=signal,
            symbol=symbol,
            source_unit=unit,
            sink_unit=unit,
            unit_ok=True,
            stiff=stiff and not by_name[writer].substitute and not by_name[reader].substitute,
            **{"from": writer, "to": reader},
        )
        by_name[writer].outputs.append(symbol)
        by_name[reader].inputs.append(symbol)
    result.base_tick_ns = min(steps, default=0)
    # Modules exchange signals at the mechanical step, or at the smallest step there is.
    rigid = by_name.get("rigid_body")
    result.sync_ns = rigid.dt_ns if rigid is not None and rigid.dt_ns > 0 else result.base_tick_ns
    return result


# --- canonical text ---------------------------------------------------------------------------


def _scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        text = repr(value)
        if "e" in text or "E" in text:  # YAML 1.1 readers are picky about exponents
            text = f"{value:.12f}".rstrip("0")
            if text.endswith("."):
                text += "0"
        return text
    if value is None:
        return "null"
    dumped: str = yaml.safe_dump(str(value), default_flow_style=True, width=1 << 20)
    return dumped.removesuffix("\n").removesuffix("\n...")


def _flow(value: Any) -> str:
    if isinstance(value, dict):
        return "{" + ", ".join(f"{key}: {_flow(item)}" for key, item in value.items()) + "}"
    if isinstance(value, list):
        return "[" + ", ".join(_flow(item) for item in value) + "]"
    return _scalar(value)


_HEAD_KEYS = ("schema_version", "stage", "environment", "seed", "deterministic", "tier", "batch")


def canonical(data: dict[str, Any]) -> str:
    """Profile text in the one layout every tool writes: a domain per line, fixed key order.

    The viewer edits a profile by replacing single lines, so this layout is part of the
    contract of ProfileState.yaml.
    """
    lines = ["# Lagrange simulation profile"]
    lines += [f"{key}: {_flow(data[key])}" for key in _HEAD_KEYS if key in data]
    domains = data.get("domains")
    if isinstance(domains, dict):
        ordered = [name for name in (*CORE_DOMAINS, *EXTRA_DOMAINS) if name in domains]
        ordered += [name for name in domains if name not in ordered]
        lines += ["", "domains:"]
        lines += [f"  {name}: {_flow(domains[name])}" for name in ordered]
    rest = [key for key in data if key not in _HEAD_KEYS and key != "domains"]
    if rest:
        lines.append("")
        lines += [f"{key}: {_flow(data[key])}" for key in rest]
    return "\n".join(lines) + "\n"


def default_profile(stage_path: str) -> str:
    """The profile the mock server presents for a scene that came without one."""
    data: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "stage": stage_path or "untitled",
        "domains": {
            "rigid_body": {"enabled": True, "backend": "mujoco", "dt": 0.001},
            "electrical": {"enabled": True, "fidelity": "averaged", "dt": 0.0001},
            "thermal": {"enabled": True, "dt": 0.1},
            "aerodynamics": {"enabled": True, "fidelity": "drag", "dt": 0.01},
            "weather": {"enabled": False, "substitute": "clear"},
            "emi": {"enabled": False, "substitute": "zero"},
            "radiation": {"enabled": False, "substitute": "zero"},
            "sensors": {"enabled": False, "substitute": "ground_truth"},
        },
        "views": ["physics_3d", "waveforms"],
    }
    return canonical(data)


# --- what the domain panel draws ----------------------------------------------------------------


def _step_ns(settings: dict[str, Any], facts: DomainFacts) -> int:
    step = settings.get("dt")
    if isinstance(step, (int, float)) and not isinstance(step, bool) and step > 0:
        return round(step * _NS_PER_S)
    return facts.default_dt_ns


def state(text: str, data: dict[str, Any], *, restart_required: bool = False) -> Any:
    """ProfileState for a valid profile."""
    result = profile_pb2.ProfileState(
        yaml=text,
        schema_version=SCHEMA_VERSION,
        json_schema=schema_text(),
        restart_required=restart_required,
    )
    domains = data.get("domains", {})
    for name in (*CORE_DOMAINS, *EXTRA_DOMAINS):
        facts = FACTS[name]
        settings = domains.get(name)
        present = isinstance(settings, dict)
        settings = settings if present else {}
        enabled = bool(settings.get("enabled", False))
        entry = result.domains.add(domain=name, enabled=enabled, available=True)
        if name in _UNUSED_BY_GROUND_ROBOTS and not present:
            entry.available = False
            entry.unavailable_reason = "not used by this robot"
        token = settings.get("substitute")
        if isinstance(token, dict):
            token = token.get("kind")
        entry.substituted = not enabled
        if enabled:
            entry.active_module = str(
                settings.get("backend")
                or settings.get("fidelity")
                or (facts.backends or facts.fidelities or ("builtin",))[0]
            )
        else:
            entry.active_module = str(token or facts.substitutes[0].token)
        for substitute in facts.substitutes:
            entry.substitutes.add(
                token=substitute.token,
                kind=substitute.kind,
                description=substitute.description,
                needs_source=substitute.needs_source,
            )
        entry.backends.extend(facts.backends)
        entry.fidelities.extend(facts.fidelities)
        entry.dt_ns = _step_ns(settings, facts)  # when disabled: the step it would start with
        entry.min_dt_ns = facts.min_dt_ns
        entry.max_dt_ns = facts.max_dt_ns
        if enabled and facts.default_dt_ns > 0 and entry.dt_ns > 0:
            # Cost grows with the number of steps taken per simulated second.
            entry.cpu_ms_per_sim_s = facts.cost_ms_per_sim_s * facts.default_dt_ns / entry.dt_ns
        entry.depends_on.extend(facts.depends_on)
    return result

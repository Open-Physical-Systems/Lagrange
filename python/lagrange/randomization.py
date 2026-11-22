"""Domain randomisation for learning: what LagrangeEnv(randomize=...) draws at each reset.

    randomize = {
        "mass": {"bodies": "*", "scale": (0.8, 1.2)},  # each body's mass (and inertia) times k
        "friction": (0.5, 1.2),                         # every collider's coefficient
        "motor_resistance": (0.9, 1.1),
        "start": {"x": (-1, 1), "y": (-1, 1), "yaw": (-3.14, 3.14)},
        "push": {"force": (5, 20), "duration": 0.1, "count": 2},
        "wind_speed": (0, 8),                           # a site value, as set_site
    }

Each key is drawn from the environment's seeded generator in the order given, so the same seed
gives the same draws. A key's value is a range (low, high), drawn uniformly, or a dict that names
what it applies to ("bodies", "joints", "motors", "batteries", "rotors", "sensors": a glob or a
list of them) and how: "scale" (a factor on the file's value) or "value" (the value itself), and
"per": "each" (default: one draw per target) or "all" (one draw for them all). What a bare range
means for each key is in KEYS. Every value drawn is applied on the files' values, so an episode
never inherits the last one's.

The values go to Simulation.reset_to as parameters (Simulation.parameters lists them all); the
push schedule to Simulation.set_pushes. docs/guide/rl.md has the table and what each costs.
"""

from __future__ import annotations

import fnmatch
import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# Site values (Simulation.set_site's names): a range is the value.
SITE = (
    "gravity",
    "wind_speed",
    "wind_from",
    "temperature",
    "air_density",
    "pressure",
    "relative_humidity",
    "rain_rate",
    "visibility",
    "snow_rate",
    "snow_cover",
)

# key: (what a bare range means, the target kind, the attribute or field names it sets).
_SCALED = "scale"
_VALUE = "value"
KEYS: dict[str, tuple[str, str, tuple[str, ...]]] = {
    "mass": (_SCALED, "bodies", ("mass",)),
    "com": (_VALUE, "bodies", ("com_x", "com_y", "com_z")),  # an offset, m, on each axis
    "inertia": (
        _SCALED,
        "bodies",
        ("inertia_xx", "inertia_yy", "inertia_zz", "inertia_xy", "inertia_xz", "inertia_yz"),
    ),
    "friction": (_VALUE, "colliders", ("friction",)),
    "joint_damping": (_VALUE, "joints", ("damping",)),
    "joint_friction": (_VALUE, "joints", ("friction",)),
    "motor_resistance": (_SCALED, "motors", ("elec:windingResistance",)),
    "motor_torque_constant": (_SCALED, "motors", ("elec:torqueConstant",)),
    "motor_current_limit": (_SCALED, "motors", ("elec:maxCurrent",)),
    "battery_capacity": (_SCALED, "batteries", ("elec:battery:capacity",)),
    "battery_resistance": (_SCALED, "batteries", ("elec:battery:r0",)),  # series R0
    "rotor_thrust": (_SCALED, "rotors", ("aero:rotor:thrustCoeff",)),
    "rotor_torque": (_SCALED, "rotors", ("aero:rotor:powerCoeff",)),
    "sensor_noise": (_SCALED, "sensors", ()),  # every setting with "noise" or "random_walk"
    "sensor_bias": (_VALUE, "sensors", ()),  # IMU: {"accel": (lo, hi), "gyro": (lo, hi)}
    "sensor_latency": (_VALUE, "sensors", ("latency",)),
    "start": (_VALUE, "start", ()),  # {"x", "y", "z": m, "yaw": rad}: ranges
    "push": (_VALUE, "push", ()),
}
# The dict entries each key takes besides its targets, "scale", "value" and "per".
_TARGETS = {
    "bodies": "bodies",
    "colliders": "bodies",
    "joints": "joints",
    "motors": "motors",
    "batteries": "batteries",
    "rotors": "rotors",
    "sensors": "sensors",
}
_EXTRA = {
    "mass": {"inertia"},
    "sensor_bias": {"accel", "gyro"},
    "start": {"x", "y", "z", "yaw"},
    "push": {"bodies", "force", "torque", "duration", "count", "window", "vertical"},
}
_SCHEMA_TARGETS = {
    "motors": "elec:torqueConstant",
    "batteries": "elec:battery:capacity",
    "rotors": "aero:rotor:thrustCoeff",
}


def _span(name: str, value: Any) -> tuple[float, float]:
    """A range (low, high) of finite numbers, low <= high; a number is a range of one value."""
    if isinstance(value, int | float) and not isinstance(value, bool):
        value = (value, value)
    try:
        low, high = (float(v) for v in value)
    except (TypeError, ValueError):
        raise ValueError(f"randomize[{name!r}]: a range is (low, high), not {value!r}") from None
    if not (math.isfinite(low) and math.isfinite(high)) or low > high:
        raise ValueError(f"randomize[{name!r}]: ({low}, {high}) is not a range low <= high")
    return low, high


def _patterns(value: Any) -> list[str]:
    return [value] if isinstance(value, str) else list(value)


def check(randomize: Mapping[str, Any] | None) -> dict[str, Any]:
    """The randomize= dict checked for its keys and the shape of each value (no simulation
    needed); ValueError naming what is wrong."""
    spec = dict(randomize or {})
    for name, value in spec.items():
        if name in SITE:
            _span(name, value)
            continue
        if name not in KEYS:
            raise ValueError(
                f"randomize: no key {name!r}; the keys are {', '.join(sorted(KEYS))} and the "
                f"site values {', '.join(SITE)}"
            )
        _meaning, kind, _fields = KEYS[name]
        if name in ("sensor_bias", "start", "push") and not isinstance(value, Mapping):
            raise ValueError(f"randomize[{name!r}] is a dict: {', '.join(sorted(_EXTRA[name]))}")
        if not isinstance(value, Mapping):
            _span(name, value)
            continue
        allowed = {"scale", "value", "per"} | _EXTRA.get(name, set())
        if kind in _TARGETS:
            allowed.add(kind if kind != "colliders" else "bodies")
        unknown = set(value) - allowed
        if unknown:
            raise ValueError(
                f"randomize[{name!r}]: no entry {', '.join(sorted(unknown))}; it takes "
                f"{', '.join(sorted(allowed))}"
            )
        if "scale" in value and "value" in value:
            raise ValueError(f"randomize[{name!r}]: scale or value, not both")
        special = name in ("sensor_bias", "start", "push")
        if not special and "scale" not in value and "value" not in value:
            raise ValueError(f"randomize[{name!r}]: a dict needs a scale or a value range")
        if name == "com" and "scale" in value:
            raise ValueError("randomize['com'] is an offset (m): a value range, not a scale")
        if name in ("sensor_bias", "start", "push") and ("scale" in value or "value" in value):
            raise ValueError(f"randomize[{name!r}] takes {', '.join(sorted(_EXTRA[name]))}")
        for key in ("scale", "value", "accel", "gyro", "x", "y", "z", "yaw", "force", "torque"):
            if key in value:
                _span(f"{name}.{key}", value[key])
        if value.get("per", "each") not in ("each", "all"):
            raise ValueError(f"randomize[{name!r}]: per is 'each' or 'all'")
        if name == "push":
            _span("push.duration", value.get("duration", 0.1))
            _span("push.count", value.get("count", 1))
            if "window" in value:
                _span("push.window", value["window"])
            if "force" not in value and "torque" not in value:
                raise ValueError("randomize['push'] needs a force (N) or a torque (N·m) range")
    return spec


def push_bodies(spec: Mapping[str, Any]) -> list[str] | None:
    """The bodies the run must take pushes on (Simulation(pushes=...)), or None for no pushes.
    "" stands for the robot's first body."""
    push = spec.get("push")
    if push is None:
        return None
    return _patterns(push.get("bodies", [""]))


@dataclass
class _Entry:
    name: str
    mode: str  # scale or value
    span: tuple[float, float]
    per: str
    groups: list[list[str]]  # parameters drawn together: one draw per group
    extra: dict[str, Any] = field(default_factory=dict)


class Randomizer:
    """randomize= resolved against one simulation's parameters: draw() gives the parameters for
    Simulation.reset_to and the pushes for Simulation.set_pushes."""

    def __init__(self, spec: Mapping[str, Any], parameters: Mapping[str, float]) -> None:
        self.spec = check(spec)
        self.base = dict(parameters)
        self.entries: list[_Entry] = []
        names = list(self.base)
        for name, value in self.spec.items():
            if name in SITE:
                self.entries.append(
                    _Entry(name, _VALUE, _span(name, value), "all", [[f"site/{name}"]])
                )
                continue
            options: Mapping[str, Any] = value if isinstance(value, Mapping) else {}
            meaning, kind, fields = KEYS[name]
            mode = _SCALED if "scale" in options else _VALUE if "value" in options else meaning
            span = (
                (0.0, 0.0)
                if name in ("sensor_bias", "start", "push")
                else _span(name, options[mode] if options else value)
            )
            per = options.get("per", "each")
            if name == "push":
                self.entries.append(_Entry(name, _VALUE, (0.0, 0.0), "all", [], dict(options)))
                continue
            if name == "start":
                groups = [[f"start/{axis}"] for axis in ("x", "y", "z", "yaw") if axis in options]
                spans = {
                    f"start/{axis}": _span(f"start.{axis}", options[axis])
                    for axis in ("x", "y", "z", "yaw")
                    if axis in options
                }
                if not groups:
                    raise ValueError("randomize['start'] takes x, y, z or yaw ranges")
                if "start/x" not in self.base:
                    raise ValueError("randomize['start']: the run has no robot to start elsewhere")
                self.entries.append(
                    _Entry(name, _VALUE, (0.0, 0.0), "each", groups, {"spans": spans})
                )
                continue
            groups = self._targets(name, kind, fields, options, names)
            if not groups:
                raise ValueError(f"randomize[{name!r}]: nothing in this run to apply it to")
            extra: dict[str, Any] = {}
            if name == "mass" and options.get("inertia", True):
                extra["inertia"] = True
            if name == "sensor_bias":
                extra = {
                    axis: _span(f"sensor_bias.{axis}", options[axis])
                    for axis in ("accel", "gyro")
                    if axis in options
                }
                if not extra:
                    raise ValueError("randomize['sensor_bias'] takes accel or gyro ranges")
            self.entries.append(_Entry(name, mode, span, per, groups, extra))

    def _targets(
        self,
        name: str,
        kind: str,
        fields: tuple[str, ...],
        options: Mapping[str, Any],
        names: list[str],
    ) -> list[list[str]]:
        """The parameters each target of `name` sets: one list per target."""
        target_key = "bodies" if kind == "colliders" else kind
        patterns = _patterns(options.get(target_key, "*"))

        def chosen(target: str) -> bool:
            return any(fnmatch.fnmatchcase(target, p) for p in patterns)

        groups: list[list[str]] = []
        if kind in ("bodies", "colliders", "joints", "sensors"):
            prefix = {
                "bodies": "body/",
                "colliders": "body/",
                "joints": "joint/",
                "sensors": "sensor/",
            }[kind]
            targets: dict[str, list[str]] = {}
            for parameter in names:
                if not parameter.startswith(prefix):
                    continue
                target, _, setting = parameter[len(prefix) :].rpartition("/")
                if not chosen(target):
                    continue
                if kind == "sensors":
                    if name == "sensor_noise":
                        wanted = "noise" in setting or "random_walk" in setting
                    elif name == "sensor_bias":
                        wanted = setting in ("accel_bias_mean", "gyro_bias_mean")
                    else:
                        wanted = setting in fields
                else:
                    wanted = setting in fields
                if wanted:
                    targets.setdefault(target, []).append(parameter)
            if kind == "colliders" and chosen("world") and "world/friction" in self.base:
                targets["world"] = ["world/friction"]
            groups = list(targets.values())
        else:  # schema attributes: "<prim path>.<attribute>", targets by path or last name
            marker = _SCHEMA_TARGETS[kind]
            prims = [p.rpartition(".")[0] for p in names if p.endswith("." + marker)]
            for prim in prims:
                if not (chosen(prim) or chosen(prim.rsplit("/", 1)[-1])):
                    continue
                group = [f"{prim}.{f}" for f in fields if f"{prim}.{f}" in self.base]
                if group:
                    groups.append(group)
        return groups

    def draw(
        self, rng: np.random.Generator, episode_start: float, episode_seconds: float
    ) -> tuple[dict[str, float], list[tuple[Any, ...]], dict[str, Any]]:
        """Parameters for reset_to, pushes for set_pushes, and what was drawn, for info."""
        values: dict[str, float] = {}
        pushes: list[tuple[Any, ...]] = []
        drawn: dict[str, Any] = {}

        def put(parameter: str, mode: str, k: float) -> None:
            if mode == _SCALED:
                values[parameter] = values.get(parameter, self.base[parameter]) * k
            else:
                values[parameter] = k

        for entry in self.entries:
            if entry.name == "push":
                pushes = self._pushes(entry.extra, rng, episode_start, episode_seconds)
                drawn["push"] = [
                    {
                        "body": p[0],
                        "start": p[1],
                        "duration": p[2],
                        "force": list(p[3:6]),
                        "torque": list(p[6:9]),
                    }
                    for p in pushes
                ]
                continue
            if entry.name == "start":
                for group in entry.groups:
                    k = float(rng.uniform(*entry.extra["spans"][group[0]]))
                    values[group[0]] = k
                continue
            if entry.name == "sensor_bias":
                for group in entry.groups:
                    for parameter in group:
                        axis = parameter.rsplit("/", 1)[-1].split("_", 1)[0]
                        if axis in entry.extra:
                            values[parameter] = float(rng.uniform(*entry.extra[axis]))
                continue
            shared = float(rng.uniform(*entry.span)) if entry.per == "all" else None
            for group in entry.groups:
                k = shared if shared is not None else float(rng.uniform(*entry.span))
                before = values.get(group[0], self.base.get(group[0], 1.0))
                for parameter in group:
                    if entry.name == "com":  # an offset on the file's position
                        values[parameter] = values.get(parameter, self.base[parameter]) + k
                    else:
                        put(parameter, entry.mode, k)
                if entry.extra.get("inertia"):  # the same shape, denser by the mass's factor
                    body = group[0].rsplit("/", 1)[0]
                    for axis in KEYS["inertia"][2]:
                        put(f"{body}/{axis}", _SCALED, values[group[0]] / before)
        drawn.update(values)
        return values, pushes, drawn

    @staticmethod
    def _pushes(
        options: Mapping[str, Any], rng: np.random.Generator, start: float, seconds: float
    ) -> list[tuple[Any, ...]]:
        bodies = _patterns(options.get("bodies", [""]))
        low, high = _span("push.count", options.get("count", 1))
        count = int(rng.integers(int(low), int(high) + 1))
        window = _span("push.window", options.get("window", (0.0, seconds)))
        out = []
        for _ in range(count):
            body = bodies[int(rng.integers(len(bodies)))]
            duration = float(rng.uniform(*_span("push.duration", options.get("duration", 0.1))))
            at = float(rng.uniform(window[0], max(window[0], window[1] - duration)))
            force = [0.0, 0.0, 0.0]
            torque = [0.0, 0.0, 0.0]
            if "force" in options:
                magnitude = float(rng.uniform(*_span("push.force", options["force"])))
                force = list(_direction(rng, bool(options.get("vertical", False))) * magnitude)
            if "torque" in options:
                magnitude = float(rng.uniform(*_span("push.torque", options["torque"])))
                torque = list(_direction(rng, True) * magnitude)
            out.append((body, start + at, duration, *force, *torque))
        return out


def _direction(rng: np.random.Generator, three_d: bool) -> np.ndarray:
    """A unit vector, uniform on the sphere, or on the horizontal circle."""
    if three_d:
        v = rng.normal(size=3)
        return np.asarray(v / np.linalg.norm(v))
    angle = float(rng.uniform(0.0, 2.0 * math.pi))
    return np.array([math.cos(angle), math.sin(angle), 0.0])

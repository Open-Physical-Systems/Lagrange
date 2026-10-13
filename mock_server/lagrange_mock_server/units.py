"""SI units for signals, as lagrange.viewer.v1.Unit messages.

The `kind` numbers follow lagrange::QuantityKind in docs/design/architecture.md section 3.
They must be kept equal to the C++ enum once core/signals/Unit.hpp exists.
"""

from __future__ import annotations

from lagrange.viewer.v1 import common_pb2

KIND_TORQUE = 1
KIND_VOLTAGE = 5
KIND_CURRENT = 6
KIND_TEMPERATURE = 9
KIND_ANGULAR_VELOCITY = 11
KIND_ANGULAR_POSITION = 12
KIND_POSITION = 14
KIND_RATIO = 24


def _unit(display: str, kind: int, **exponents: int) -> common_pb2.Unit:
    return common_pb2.Unit(scale=1.0, kind=kind, display=display, **exponents)


METRE = _unit("m", KIND_POSITION, length=1)
RADIAN = _unit("rad", KIND_ANGULAR_POSITION, angle=1)
RADIAN_PER_SECOND = _unit("rad/s", KIND_ANGULAR_VELOCITY, time=-1, angle=1)
NEWTON_METRE = _unit("N·m", KIND_TORQUE, length=2, mass=1, time=-2)
AMPERE = _unit("A", KIND_CURRENT, current=1)
VOLT = _unit("V", KIND_VOLTAGE, length=2, mass=1, time=-3, current=-1)
KELVIN = _unit("K", KIND_TEMPERATURE, temperature=1)
ONE = _unit("1", KIND_RATIO)
# Units of prim attributes. kind 0: no quantity kind assigned yet (lagrange::QuantityKind).
# "ohm", not the Greek letter: the viewer's bundled IBM Plex files have no Greek glyphs.
OHM = _unit("ohm", 0, length=2, mass=1, time=-3, current=-2)
HENRY = _unit("H", 0, length=2, mass=1, time=-2, current=-2)
NEWTON_METRE_PER_AMPERE = _unit("N·m/A", 0, length=2, mass=1, time=-2, current=-1)
JOULE_PER_KELVIN = _unit("J/K", 0, length=2, mass=1, time=-2, temperature=-1)
WATT_PER_KELVIN = _unit("W/K", 0, length=2, mass=1, time=-3, temperature=-1)
AMPERE_HOUR = _unit("A·h", 0, time=1, current=1)
AMPERE_HOUR.scale = 3600.0  # one ampere hour is 3600 coulombs
KILOGRAM = _unit("kg", 0, mass=1)

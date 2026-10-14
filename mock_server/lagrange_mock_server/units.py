"""SI units for signals, as lagrange.viewer.v1.Unit messages.

The `kind` numbers are those of lagrange::QuantityKind in core/signals/include/lagrange/signals/
Unit.hpp and must stay equal to it.
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
KIND_CHARGE = 7
KIND_RESISTANCE = 8
KIND_LINEAR_ACCELERATION = 26
KIND_INDUCTANCE = 28
KIND_MASS = 31


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
# Units of prim attributes. kind 0: no single quantity kind (QuantityKind::Unspecified).
# U+2126 OHM SIGN, the unit symbol (IBM Plex Mono has it; it has no Greek capital omega).
OHM = _unit("\u2126", KIND_RESISTANCE, length=2, mass=1, time=-3, current=-2)
HENRY = _unit("H", KIND_INDUCTANCE, length=2, mass=1, time=-2, current=-2)
NEWTON_METRE_PER_AMPERE = _unit("N·m/A", 0, length=2, mass=1, time=-2, current=-1)
JOULE_PER_KELVIN = _unit("J/K", 0, length=2, mass=1, time=-2, temperature=-1)
WATT_PER_KELVIN = _unit("W/K", 0, length=2, mass=1, time=-3, temperature=-1)
AMPERE_HOUR = _unit("A·h", KIND_CHARGE, time=1, current=1)
AMPERE_HOUR.scale = 3600.0  # one ampere hour is 3600 coulombs
KILOGRAM = _unit("kg", KIND_MASS, mass=1)
METRE_PER_SECOND_SQUARED = _unit("m/s²", KIND_LINEAR_ACCELERATION, length=1, time=-2)

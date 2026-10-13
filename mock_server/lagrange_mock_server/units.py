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

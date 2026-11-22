"""Repository checks of the schema libraries (RFC 0002): every schema registers, carries a
version USD understands, and every numeric attribute states its unit in lagrangeUnit."""

import json
import os
from pathlib import Path

import pytest
from pxr import Sdf, Usd

# Attribute types that are not physical quantities and need no unit.
UNITLESS_TYPES = {"string", "token", "bool", "asset"}

EXPECTED = {
    "sensor": [
        "SensorAPI_1",
        "LidarAPI_1",
        "ImuAPI_1",
        "CameraSensorAPI_1",
        "ThermalCameraAPI_1",
        "GnssAPI_1",
        "MagnetometerAPI_1",
        "BarometerAPI_1",
    ],
    "electrical": [
        "PortAPI_1",
        "BatteryAPI_1",
        "ElectricalMotorAPI_1",
        "ElectricalBusAPI_1",
        "ElectricalLoadAPI_1",
        "ElectricalConverterAPI_1",
        "ElectricalBranchAPI_1",
    ],
    "thermal": ["ThermalAPI_1", "ThermalLinkAPI_1", "ThermalMaterialAPI_1", "HeatSourceAPI_1"],
    "aero": ["AeroAPI_1", "RotorAPI_1"],
    "weather": ["WeatherAPI_1"],
    "emi": ["EmiSourceAPI_1", "EmiSusceptibilityAPI_1"],
    "sensor_material": ["SensorMaterialAPI_1"],
    "comms": ["RadioLinkAPI_1", "CanBusAPI_1"],
    "vehicle": ["VehicleAPI_1", "WheelAPI_1"],
    "road": ["RoadLaneAPI_1", "RoadSignalAPI_1"],
    "human": ["HumanAPI_1"],
    "water": ["WaterAPI_1"],
    "joint": ["JointDynamicsAPI_1"],
    "world": ["WorldTileAPI_1", "TreeAPI_1"],
    "tether": ["TetherAPI_1"],
    "fire": ["FireAPI_1"],
    "light": ["LampAPI_1"],
    "actuator": [
        "SuctionGripperAPI_1",
        "MagnetGripperAPI_1",
        "NozzleAPI_1",
        "TankAPI_1",
        "ServoAPI_1",
        "SolenoidAPI_1",
        "BrakeAPI_1",
    ],
}


def libraries():
    return os.environ["LAGRANGE_SCHEMA_LIBRARIES"].split(";")


@pytest.mark.parametrize("library", libraries())
def test_schemas_register_with_a_version(library):
    registry = Usd.SchemaRegistry()
    for name in EXPECTED[library]:
        definition = registry.FindAppliedAPIPrimDefinition(name)
        assert definition is not None, f"{name} is not registered"
        family, version = Usd.SchemaRegistry.ParseSchemaFamilyAndVersionFromIdentifier(name)
        assert version >= 1 and name == f"{family}_{version}"


@pytest.mark.parametrize("library", libraries())
def test_every_numeric_attribute_has_a_unit(library):
    registry = Usd.SchemaRegistry()
    missing = []
    for name in EXPECTED[library]:
        definition = registry.FindAppliedAPIPrimDefinition(name)
        for prop in definition.GetPropertyNames():
            attribute = definition.GetAttributeDefinition(prop)
            if not attribute:
                continue
            if str(attribute.GetTypeName().scalarType) in UNITLESS_TYPES:
                continue
            unit = attribute.GetMetadata("lagrangeUnit")
            if not unit:
                missing.append(f"{name}.{prop}")
    assert not missing, f"attributes without lagrangeUnit: {missing}"


def test_an_applied_schema_resolves_its_unit_on_an_authored_prim():
    stage = Usd.Stage.CreateInMemory()
    prim = stage.DefinePrim("/robot/front_lidar", "Xform")
    prim.AddAppliedSchema("SensorAPI_1")
    prim.AddAppliedSchema("LidarAPI_1")
    assert prim.HasAPI("LidarAPI_1")
    attribute = prim.GetAttribute("sensor:lidar:rangeMax")
    # A required attribute has no fallback: unauthored, it has no value.
    assert attribute.Get() is None
    # The unit is in the prim definition (where the loader reads it) and, once the attribute is
    # authored, on the attribute itself (OpenUSD spike, check 5).
    definition = prim.GetPrimDefinition().GetAttributeDefinition("sensor:lidar:rangeMax")
    assert definition.GetMetadata("lagrangeUnit") == "m"
    attribute.Set(40.0)
    assert attribute.GetMetadata("lagrangeUnit") == "m"
    assert prim.GetAttribute("sensor:lidar:rangeNoiseStd").Get() == pytest.approx(0.02)
    assert Sdf.ValueTypeNames.Double2 == prim.GetAttribute("sensor:lidar:verticalFov").GetTypeName()


def test_a_multiple_apply_port_takes_its_instance_name():
    stage = Usd.Stage.CreateInMemory()
    bus = stage.DefinePrim("/robot/power/dc_48v", "Xform")
    bus.AddAppliedSchema("ElectricalBusAPI_1")
    battery = stage.DefinePrim("/robot/power/battery", "Xform")
    battery.AddAppliedSchema("BatteryAPI_1")
    battery.AddAppliedSchema("PortAPI_1:dc_out")
    assert battery.HasAPI("PortAPI_1", "dc_out")
    kind = battery.GetAttribute("elec:port:dc_out:kind")
    assert kind.Get() == "dc"
    definition = battery.GetPrimDefinition().GetAttributeDefinition("elec:port:dc_out:maxCurrent")
    assert definition.GetMetadata("lagrangeUnit") == "A"
    connected = battery.GetRelationship("elec:port:dc_out:connectedTo")
    assert connected
    connected.SetTargets([bus.GetPath()])
    assert connected.GetTargets() == [bus.GetPath()]


def test_motor_constants_carry_their_units():
    definition = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("ElectricalMotorAPI_1")
    units = {
        name: definition.GetAttributeDefinition(name).GetMetadata("lagrangeUnit")
        for name in ("elec:torqueConstant", "elec:backEmfConstant", "elec:rotorInertia")
    }
    assert units == {
        "elec:torqueConstant": "N.m/A",
        "elec:backEmfConstant": "V.s/rad",
        "elec:rotorInertia": "kg.m2",
    }


def test_lidar_patterns_and_fisheye_lenses_are_additions():
    # RFC 0015: the tokens are appended, the new attributes carry their units, and the rosette's
    # numbers and the cutoff angle have no fallback (the loader's documented values instead).
    registry = Usd.SchemaRegistry()
    lidar = registry.FindAppliedAPIPrimDefinition("LidarAPI_1")
    pattern = lidar.GetAttributeDefinition("sensor:lidar:scanPattern")
    assert list(pattern.GetMetadata("allowedTokens")) == [
        "rotating",
        "solid_state",
        "custom",
        "flash",
        "rosette",
    ]
    assert pattern.GetFallbackValue() == "rotating"
    units = {
        "sensor:lidar:channelElevations": "rad",
        "sensor:lidar:rosette:fov": "rad",
        "sensor:lidar:rosette:pointRate": "1/s",
        "sensor:lidar:rosette:speed1": "rad/s",
        "sensor:lidar:rosette:speed2": "rad/s",
        "sensor:lidar:rosette:prismRatio": "1",
    }
    for name, unit in units.items():
        attribute = lidar.GetAttributeDefinition(name)
        assert attribute, name
        assert attribute.GetMetadata("lagrangeUnit") == unit, name
        if name.startswith("sensor:lidar:rosette:"):
            assert attribute.GetFallbackValue() is None, name
    table = lidar.GetAttributeDefinition("sensor:lidar:channelElevations")
    assert len(table.GetFallbackValue()) == 0
    camera = registry.FindAppliedAPIPrimDefinition("CameraSensorAPI_1")
    lens = camera.GetAttributeDefinition("sensor:camera:lensType")
    assert list(lens.GetMetadata("allowedTokens")) == ["gnomonical", "equidistant"]
    assert lens.GetFallbackValue() == "gnomonical"
    cutoff = camera.GetAttributeDefinition("sensor:camera:cutoffAngle")
    assert cutoff.GetMetadata("lagrangeUnit") == "rad"
    assert cutoff.GetFallbackValue() is None


def test_gear_back_efficiency_is_an_addition():
    # RFC 0020 §4.4: appended to ElectricalMotorAPI_1 with a fallback of -1, "the same as
    # gearEfficiency", so every file without it behaves as before; not in frozen_1_0.json.
    motor = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("ElectricalMotorAPI_1")
    back = motor.GetAttributeDefinition("elec:gearBackEfficiency")
    assert back
    assert back.GetFallbackValue() == -1.0
    assert back.GetMetadata("lagrangeUnit") == "1"
    with FROZEN.open() as f:
        assert "elec:gearBackEfficiency" not in json.load(f)["ElectricalMotorAPI_1"]


def test_servo_and_position_mode_are_additions():
    # RFC 0020 §4.3: ServoAPI_1 is a new class, and elec:controlMode's `position` token is
    # appended after the frozen ones, so every 1.x file reads as before.
    motor = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("ElectricalMotorAPI_1")
    tokens = list(motor.GetAttributeDefinition("elec:controlMode").GetMetadata("allowedTokens"))
    assert tokens == ["torque", "velocity", "voltage", "position"]
    servo = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("ServoAPI_1")
    assert servo
    assert servo.GetAttributeDefinition("act:servo:kind").GetFallbackValue() == "pwm"
    assert servo.GetAttributeDefinition("act:servo:kp").GetFallbackValue() is None
    assert servo.GetAttributeDefinition("act:servo:deadband").GetMetadata("lagrangeUnit") == "rad"
    with FROZEN.open() as f:
        assert "ServoAPI_1" not in json.load(f)


def test_open_water_chart_is_an_addition():
    # Marine thrusters: RotorAPI_1's chart attributes are new, with no fallback, so a rotor
    # without them keeps its static coefficients.
    rotor = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("RotorAPI_1")
    for name in ("advanceRatios", "thrustCoeffs", "torqueCoeffs"):
        attribute = rotor.GetAttributeDefinition("aero:rotor:" + name)
        assert attribute
        assert attribute.GetFallbackValue() is None
    with FROZEN.open() as f:
        frozen = json.load(f)
    assert "aero:rotor:advanceRatios" not in json.dumps(frozen)


def test_tree_is_an_addition():
    # Trees (ground.md "Trees"): TreeAPI_1 is a new class of the world library; the owner approved
    # the addition on 2026-11-22.
    tree = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("TreeAPI_1")
    assert tree
    assert tree.GetAttributeDefinition("tree:height").GetFallbackValue() is None
    assert tree.GetAttributeDefinition("tree:naturalFrequency").GetFallbackValue() == 0.5
    assert (
        tree.GetAttributeDefinition("tree:leafAreaDensity").GetMetadata("lagrangeUnit") == "m2/m3"
    )
    with FROZEN.open() as f:
        assert "TreeAPI_1" not in json.load(f)


def test_brake_is_an_addition():
    # RFC 0020 "Also worth adding": BrakeAPI_1 is a new class of the actuator library.
    brake = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("BrakeAPI_1")
    assert brake
    assert brake.GetAttributeDefinition("act:brake:holdingTorque").GetFallbackValue() is None
    assert brake.GetAttributeDefinition("act:brake:coilInductance").GetFallbackValue() == 0
    assert brake.GetAttributeDefinition("act:brake:holdingForce").GetMetadata("lagrangeUnit") == "N"
    with FROZEN.open() as f:
        assert "BrakeAPI_1" not in json.load(f)


def test_road_signal_is_an_addition():
    # OpenDRIVE dynamic signals drive the traffic lights: RoadSignalAPI_1 is a new class of the
    # road library (schema addition approved by the owner on 2026-11-22).
    signal = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("RoadSignalAPI_1")
    assert signal
    assert signal.GetAttributeDefinition("signal:s").GetMetadata("lagrangeUnit") == "m"
    assert signal.GetAttributeDefinition("signal:fromLane").GetFallbackValue() == 0
    assert signal.GetAttributeDefinition("signal:sequence").GetFallbackValue() == -1
    tokens = signal.GetAttributeDefinition("signal:orientation").GetMetadata("allowedTokens")
    assert list(tokens) == ["+", "-", "none"]
    with FROZEN.open() as f:
        frozen = json.load(f)
    assert "RoadSignalAPI_1" not in frozen
    assert not [name for name in frozen["RoadLaneAPI_1"] if name.startswith("signal:")]


def test_solenoid_is_an_addition():
    # RFC 0020 §4.6: SolenoidAPI_1 is a new class of the actuator library.
    solenoid = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("SolenoidAPI_1")
    assert solenoid
    assert solenoid.GetAttributeDefinition("act:solenoid:resistance").GetFallbackValue() is None
    assert solenoid.GetAttributeDefinition("act:solenoid:springRate").GetFallbackValue() == 0
    assert solenoid.GetAttributeDefinition("act:solenoid:stroke").GetMetadata("lagrangeUnit") == "m"
    with FROZEN.open() as f:
        assert "SolenoidAPI_1" not in json.load(f)


def test_stepper_is_an_addition():
    # RFC 0020 §4.5: `stepper` is appended to elec:motorType's tokens and the elec:stepper
    # attributes are new, with fallbacks that change nothing for other motor types.
    motor = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("ElectricalMotorAPI_1")
    tokens = list(motor.GetAttributeDefinition("elec:motorType").GetMetadata("allowedTokens"))
    assert tokens == ["pmsm", "bldc", "dc_brushed", "stepper"]
    assert motor.GetAttributeDefinition("elec:stepper:rotorTeeth").GetFallbackValue() == 50
    assert motor.GetAttributeDefinition("elec:stepper:microsteps").GetFallbackValue() == 1
    assert motor.GetAttributeDefinition("elec:stepper:detentTorque").GetFallbackValue() == 0.0
    assert motor.GetAttributeDefinition("elec:stepper:holdCurrent").GetFallbackValue() is None
    with FROZEN.open() as f:
        frozen = json.load(f)["ElectricalMotorAPI_1"]
    assert not [name for name in frozen if name.startswith("elec:stepper:")]


# RFC 0007: the schemas are frozen at 1.0. Every attribute of every schema at v1.0.0 is listed
# in frozen_1_0.json with its type, fallback, unit and allowed tokens; within 1.x an attribute
# may be added, and allowed tokens appended, but nothing listed may change or go.
FROZEN = Path(__file__).parent / "frozen_1_0.json"


def describe_schemas():
    registry = Usd.SchemaRegistry()
    out = {}
    for library in libraries():
        for name in EXPECTED[library]:
            definition = registry.FindAppliedAPIPrimDefinition(name)
            attributes = {}
            for prop in sorted(definition.GetPropertyNames()):
                attribute = definition.GetAttributeDefinition(prop)
                if attribute:
                    fallback = attribute.GetFallbackValue()
                    attributes[prop] = {
                        "type": str(attribute.GetTypeName()),
                        "fallback": None if fallback is None else str(fallback),
                        "unit": attribute.GetMetadata("lagrangeUnit") or "",
                        "tokens": [str(t) for t in (attribute.GetMetadata("allowedTokens") or [])],
                    }
                else:
                    attributes[prop] = {"type": "relationship"}
            out[name] = attributes
    return out


def test_nothing_frozen_at_1_0_has_changed():
    current = describe_schemas()
    if os.environ.get("LAGRANGE_WRITE_FROZEN") == "1":
        with FROZEN.open("w") as f:
            json.dump(current, f, indent=1, sort_keys=True)
            f.write("\n")
    with FROZEN.open() as f:
        frozen = json.load(f)
    problems = []
    for schema, attributes in frozen.items():
        if schema not in current:
            problems.append(f"{schema} is gone")
            continue
        for prop, was in attributes.items():
            now = current[schema].get(prop)
            if now is None:
                problems.append(f"{schema}.{prop} is gone")
                continue
            for key in ("type", "fallback", "unit"):
                if was.get(key) != now.get(key):
                    problems.append(f"{schema}.{prop} {key}: {was.get(key)} -> {now.get(key)}")
            if now.get("tokens", [])[: len(was.get("tokens", []))] != was.get("tokens", []):
                problems.append(f"{schema}.{prop}: allowed tokens changed, not appended")
    assert not problems, (
        "schemas changed after the 1.0 freeze (RFC 0007: add a *_2 schema and a migration "
        f"instead): {problems}"
    )

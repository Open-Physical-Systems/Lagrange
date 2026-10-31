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
    ],
    "thermal": ["ThermalAPI_1", "ThermalLinkAPI_1"],
    "aero": ["AeroAPI_1", "RotorAPI_1"],
    "weather": ["WeatherAPI_1"],
    "emi": ["EmiSourceAPI_1", "EmiSusceptibilityAPI_1"],
    "sensor_material": ["SensorMaterialAPI_1"],
    "comms": ["RadioLinkAPI_1", "CanBusAPI_1"],
    "vehicle": ["VehicleAPI_1", "WheelAPI_1"],
    "road": ["RoadLaneAPI_1"],
    "human": ["HumanAPI_1"],
    "water": ["WaterAPI_1"],
    "joint": ["JointDynamicsAPI_1"],
    "world": ["WorldTileAPI_1"],
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

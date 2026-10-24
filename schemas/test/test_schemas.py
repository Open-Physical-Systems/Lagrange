"""Repository checks of the schema libraries (RFC 0002): every schema registers, carries a
version USD understands, and every numeric attribute states its unit in lagrangeUnit."""

import os

import pytest
from pxr import Sdf, Usd

# Attribute types that are not physical quantities and need no unit.
UNITLESS_TYPES = {"string", "token", "bool", "asset"}

EXPECTED = {
    "sensor": ["SensorAPI_1", "LidarAPI_1", "ImuAPI_1"],
    "electrical": [
        "PortAPI_1",
        "BatteryAPI_1",
        "ElectricalMotorAPI_1",
        "ElectricalBusAPI_1",
        "ElectricalLoadAPI_1",
        "ElectricalConverterAPI_1",
    ],
    "thermal": ["ThermalAPI_1", "ThermalLinkAPI_1"],
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

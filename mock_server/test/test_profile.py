"""Profiles: the schema, the examples, canonical text, and the server's profile commands."""

import asyncio
from pathlib import Path

import pytest
import yaml
from lagrange_mock_server import profile
from lagrange_mock_server.client import ProtocolClient
from lagrange_mock_server.endpoint import Endpoint
from lagrange_mock_server.replay import ReplayEngine
from lagrange_mock_server.sample import generate_sample
from lagrange_mock_server.server import MockServer

EXAMPLES = sorted((Path(__file__).resolve().parents[2] / "profiles" / "examples").glob("*.yaml"))


def test_there_are_example_profiles():
    assert len(EXAMPLES) >= 8


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.stem)
def test_every_example_profile_is_valid(path):
    checked = profile.check(path.read_text(encoding="utf-8"))
    assert [issue.message for issue in checked.issues] == []
    assert checked.valid


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.stem)
def test_canonical_text_keeps_the_meaning_and_is_stable(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    text = profile.canonical(data)
    assert yaml.safe_load(text) == data
    assert profile.canonical(yaml.safe_load(text)) == text
    # One domain per line: the viewer edits profiles by replacing these lines.
    for name in data["domains"]:
        assert sum(line.startswith(f"  {name}: {{") for line in text.splitlines()) == 1


def test_default_profile_is_valid_and_names_a_substitute_for_every_disabled_domain():
    checked = profile.check(profile.default_profile("synthetic:amr"))
    assert checked.valid
    for name, settings in checked.data["domains"].items():
        assert settings["enabled"] or "substitute" in settings, name


def edited(**changes):
    data = yaml.safe_load(profile.default_profile("stage.usda"))
    for name, settings in changes.items():
        data["domains"][name] = settings
    return profile.canonical(data)


def test_switching_aerodynamics_off_in_the_viewer_gives_a_valid_profile():
    # The exact line the viewer writes (viewer/client/test/test_profile_session.cpp).
    text = profile.default_profile("stage.usda")
    line = next(row for row in text.splitlines() if row.startswith("  aerodynamics:"))
    edited_text = text.replace(line, "  aerodynamics: {enabled: false, substitute: zero}")
    assert edited_text != text
    assert profile.check(edited_text).valid


def test_a_disabled_domain_without_a_substitute_is_an_error():
    checked = profile.check(edited(thermal={"enabled": False}))
    assert not checked.valid
    assert any(issue.pointer == "/domains/thermal" for issue in checked.issues)


def test_a_replay_substitute_needs_a_source():
    assert not profile.check(edited(rigid_body={"enabled": False, "substitute": "replay"})).valid
    assert profile.check(
        edited(rigid_body={"enabled": False, "substitute": "replay", "source": "run.mcap"})
    ).valid


def test_a_substitute_that_cannot_replace_the_domain_is_an_error():
    checked = profile.check(edited(weather={"enabled": False, "substitute": "ground_truth"}))
    assert [issue.pointer for issue in checked.issues] == ["/domains/weather/substitute"]


def test_a_step_that_is_not_whole_nanoseconds_is_an_error():
    checked = profile.check(edited(thermal={"enabled": True, "dt": 0.00000000015}))
    assert [issue.pointer for issue in checked.issues] == ["/domains/thermal/dt"]


def test_text_that_is_not_yaml_reports_the_line():
    checked = profile.check("stage: a\ndomains: {rigid_body: [\n")
    assert not checked.valid and checked.data is None
    assert checked.issues[0].message.startswith("line ")


def test_state_lists_every_domain_with_its_choices():
    text = profile.default_profile("stage.usda")
    state = profile.state(text, yaml.safe_load(text))
    by_name = {entry.domain: entry for entry in state.domains}
    assert list(by_name)[:8] == list(profile.CORE_DOMAINS)
    assert state.json_schema.startswith("{")
    rigid = by_name["rigid_body"]
    assert rigid.enabled and not rigid.substituted and rigid.active_module == "mujoco"
    assert rigid.dt_ns == 1_000_000 and rigid.cpu_ms_per_sim_s > 0
    emi = by_name["emi"]
    assert not emi.enabled and emi.substituted and emi.active_module == "zero"
    assert emi.cpu_ms_per_sim_s == 0 and emi.substitutes[0].token == "zero"
    assert emi.dt_ns > 0  # the step it would start with when switched on
    assert not by_name["rotor_aero"].available and by_name["rotor_aero"].unavailable_reason


def test_halving_a_step_doubles_the_estimated_cost():
    base = profile.default_profile("stage.usda")
    fast = edited(rigid_body={"enabled": True, "backend": "mujoco", "dt": 0.0005})
    cost = [
        profile.state(text, yaml.safe_load(text)).domains[0].cpu_ms_per_sim_s
        for text in (base, fast)
    ]
    assert cost[1] == pytest.approx(2 * cost[0])


async def _receive(client, body, limit=400):
    for _ in range(limit):
        envelope = await client.receive()
        assert envelope is not None
        if envelope.WhichOneof("body") == body:
            return envelope
    raise AssertionError(f"no {body}")


def test_server_validates_and_applies_profiles(tmp_path):
    async def scenario():
        engine = ReplayEngine(generate_sample(duration_s=1.0, seed=1))
        engine.play()
        server = MockServer(engine, Endpoint("unix", str(tmp_path / "s.sock")))
        bound = await server.start()
        client = await ProtocolClient.connect(bound)
        try:
            await client.hello()
            first = (await _receive(client, "profile_state")).profile_state
            assert profile.check(first.yaml).valid

            bad = edited(thermal={"enabled": False})
            await client.command(validate_profile={"yaml": bad})
            result = (await _receive(client, "command_result")).command_result
            assert result.ok and not result.validation.valid
            assert result.validation.issues[0].json_pointer == "/domains/thermal"

            await client.command(apply_profile={"yaml": bad, "restart": True})
            result = (await _receive(client, "command_result")).command_result
            assert not result.ok and "thermal" in result.error

            good = edited(thermal={"enabled": False, "substitute": "ambient"})
            await client.command(apply_profile={"yaml": good, "restart": True})
            result = (await _receive(client, "command_result")).command_result
            assert result.ok
            state = (await _receive(client, "profile_state")).profile_state
            thermal = next(entry for entry in state.domains if entry.domain == "thermal")
            assert not thermal.enabled and thermal.active_module == "ambient"
            assert yaml.safe_load(state.yaml) == yaml.safe_load(good)
            # Applying restarts the run: a snapshot follows, near t = 0.
            snapshot = (await _receive(client, "snapshot")).snapshot
            assert snapshot.sim_time.ns < 500_000_000
        finally:
            await client.close()
            await server.stop()

    asyncio.run(scenario())


def test_validation_carries_the_wiring_the_profile_would_build():
    checked = profile.check(edited(aerodynamics={"enabled": False, "substitute": "zero"}))
    wiring = checked.to_proto().wiring
    nodes = {node.module: node for node in wiring.nodes}
    assert set(nodes) == set(profile.CORE_DOMAINS)
    assert nodes["rigid_body"].detail == "mujoco" and not nodes["rigid_body"].substitute
    assert nodes["rigid_body"].dt_ns == 1_000_000
    assert nodes["aerodynamics"].substitute and nodes["aerodynamics"].detail == "zero"
    assert nodes["aerodynamics"].dt_ns == 0

    wires = {(wire.to, getattr(wire, "from"), wire.symbol): wire for wire in wiring.wires}
    torque = wires[("rigid_body", "electrical", "τ")]
    assert torque.source_unit == torque.sink_unit == "N·m" and torque.unit_ok and torque.stiff
    # A substitute still feeds its consumer, so the consumer never notices (invariant 3).
    wrench = wires[("rigid_body", "aerodynamics", "F, τ")]
    assert wrench.unit_ok and not wrench.stiff
    assert "τ" in nodes["rigid_body"].inputs and "τ" in nodes["electrical"].outputs
    assert all(wire.to in nodes and getattr(wire, "from") in nodes for wire in wiring.wires)
    assert wiring.base_tick_ns == 100_000 and wiring.sync_ns == 1_000_000


def test_text_that_is_not_a_profile_has_no_wiring():
    assert not profile.check("stage: a\ndomains: {rigid_body: [\n").to_proto().HasField("wiring")

"""The town: signals, crossings and self-driving cars, checked on the running simulation.

Every invariant is sampled from a real run of assets/worlds/town/town.usda (docs/design/domains/
traffic.md §8.2). The bounds on distances are from the shapes: a car's box is 4.5 m x 1.8 m.
"""

import math
from pathlib import Path

import lagrange
import pytest

TOWN = str(lagrange.sample_town())
RED, YELLOW, GREEN = 0, 1, 2
DONT_WALK, WALK, FLASHING = 0, 1, 2


def names(sim, prefix):
    found = [b["name"] if isinstance(b, dict) else b for b in sim.bodies()]
    return [n for n in found if n.startswith(prefix)]


def cars(sim):
    a = sim.read("traffic/agents")
    return [
        (a[i + 1], a[i + 2], a[i + 4], a[i + 5], a[i + 6], a[i + 7]) for i in range(0, len(a), 8)
    ]


@pytest.fixture(scope="module")
def run():
    """Four simulated minutes sampled at 4 Hz, shared by the checks below."""
    sim = lagrange.Simulation(None, world=TOWN, traffic=(16, 20, 1))
    walkers = names(sim, "human_walker_")
    pelvis = [n for n in walkers if n.endswith("_pelvis")]
    lamps = names(sim, "traffic_lamp_")
    dark = {n: sim.read(f"body/{n}/pose")[:3] for n in lamps}  # as loaded: all inside the housings
    samples = []
    while sim.time < 240.0:
        sim.step_until(sim.time + 0.25)
        samples.append(
            {
                "t": sim.time,
                "cars": cars(sim),
                "lights": list(sim.read("traffic/lights")),
                "crossings": list(sim.read("traffic/crossings")),
                "brakes": [sim.read(f"traffic/vehicle/{i}/brake")[0] for i in range(16)],
                "left": [sim.read(f"traffic/vehicle/{i}/left")[0] for i in range(16)],
                "right": [sim.read(f"traffic/vehicle/{i}/right")[0] for i in range(16)],
                "wheels": [sim.read(f"traffic/vehicle/{i}/wheel")[0] for i in range(16)],
                "reach": max(sim.read(f"human/walker_{i}/gesture")[0] for i in range(20)),
                "people": [sim.read(f"body/{n}/pose")[:2] for n in pelvis],
                "lit": {
                    n for n in lamps if math.dist(sim.read(f"body/{n}/pose")[:3], dark[n]) > 0.01
                },
            }
        )
    return sim, samples, dark


def test_the_town_is_populated(run):
    _, samples, dark = run
    assert len(samples[0]["cars"]) == 16
    assert len(samples[0]["people"]) == 20
    assert dark


def test_lights_go_through_every_colour(run):
    _, samples, _ = run
    seen = {int(v) for s in samples for v in s["lights"][1::2]}
    assert seen == {RED, YELLOW, GREEN}


def test_a_green_is_never_followed_by_red_without_yellow(run):
    _, samples, _ = run
    previous = samples[0]["lights"][1::2]
    for s in samples[1:]:
        now = s["lights"][1::2]
        for a, b in zip(previous, now, strict=True):
            assert not (int(a) == GREEN and int(b) == RED), f"at {s['t']}"
        previous = now


def test_buttons_bring_walk_phases(run):
    _, samples, _ = run
    pressed = any(v for s in samples for v in s["crossings"][2::3])
    walked = any(int(v) == WALK for s in samples for v in s["crossings"][1::3])
    flashed = any(int(v) == FLASHING for s in samples for v in s["crossings"][1::3])
    assert pressed and walked and flashed


def test_no_vehicle_light_is_open_beside_a_walk_light(run):
    _, samples, dark = run
    walks = [n for n in dark if n.endswith("_walk") and not n.endswith("_dont_walk")]
    opens = [n for n in dark if n.endswith(("_green", "_yellow"))]
    near = {w: [o for o in opens if math.dist(dark[w][:2], dark[o][:2]) < 22.0] for w in walks}
    assert all(near.values())
    walked = 0
    for s in samples:
        for w in walks:
            if w in s["lit"]:
                walked += 1
                assert not s["lit"].intersection(near[w]), f"{w} at {s['t']}"
    assert walked


def test_people_are_on_the_road_only_after_a_walk_light(run):
    _, samples, dark = run
    walks = [n for n in dark if n.endswith("_walk") and not n.endswith("_dont_walk")]
    last_lit = dict.fromkeys(walks, -1e9)
    for s in samples:
        for w in walks:
            if w in s["lit"]:
                last_lit[w] = s["t"]
        for px, py in s["people"]:
            near_x = min(abs(px - g) for g in (-100, 0, 100))
            near_y = min(abs(py - g) for g in (-100, 0, 100))
            on_road = (near_x < 3.0 and abs(py) <= 100.0) or (near_y < 3.0 and abs(px) <= 100.0)
            if on_road:  # kerb to kerb, well inside
                nearest = min(walks, key=lambda w: math.dist(dark[w][:2], (px, py)))
                assert s["t"] - last_lit[nearest] < 30.0, f"({px}, {py}) at {s['t']}"


def test_cars_never_touch_people(run):
    _, samples, _ = run
    for s in samples:
        for px, py in s["people"]:
            for x, y, yaw, _speed, length, width in s["cars"]:
                dx, dy = px - x, py - y
                forward = dx * math.cos(yaw) + dy * math.sin(yaw)
                left = -dx * math.sin(yaw) + dy * math.cos(yaw)
                gap = max(abs(forward) - length / 2.0, abs(left) - width / 2.0)
                assert gap > 0.3, f"at {s['t']}: person ({px}, {py}), car ({x}, {y})"


def test_traffic_keeps_moving(run):
    _, samples, _ = run
    stopped = [0.0] * len(samples[0]["cars"])
    longest = 0.0
    for s in samples:
        for i, car in enumerate(s["cars"]):
            stopped[i] = 0.0 if car[3] > 0.05 else stopped[i] + 0.25
            longest = max(longest, stopped[i])
    assert longest < 90.0, f"a car stood {longest} s: a gridlock"


def test_cars_keep_to_the_road_and_go_places(run):
    _, samples, _ = run
    first, last = samples[0]["cars"], samples[-1]["cars"]
    moved = sum(
        math.hypot(a[0] - b[0], a[1] - b[1]) > 5.0 for a, b in zip(first, last, strict=True)
    )
    assert moved >= len(first) // 2
    for s in samples:
        for x, y, *_rest in s["cars"]:
            on_a_street = min(abs(x - g) for g in (-100, 0, 100)) < 11.0 or (
                min(abs(y - g) for g in (-100, 0, 100)) < 11.0
            )
            assert on_a_street, f"a car at ({x}, {y}) at {s['t']}"


def test_people_stay_out_of_buildings(run):
    _, samples, _ = run
    for s in samples:
        for px, py in s["people"]:
            near_x = min(abs(px - g) for g in (-100, 0, 100))
            near_y = min(abs(py - g) for g in (-100, 0, 100))
            assert min(near_x, near_y) < 8.0, f"someone at ({px}, {py}) at {s['t']}"


def test_each_signal_shows_exactly_one_lamp(run):
    _, samples, dark = run
    heads = {}
    for name in dark:
        if name.endswith(("_red", "_yellow", "_green")):
            heads.setdefault(name.rsplit("_", 1)[0], []).append(name)
        elif name.endswith("_dont_walk"):
            heads.setdefault(name[: -len("_dont_walk")], []).append(name)
        elif name.endswith("_walk"):
            heads.setdefault(name[: -len("_walk")], []).append(name)
    assert heads
    flashing_heads = 0
    for s in samples:
        for head, members in heads.items():
            lit = s["lit"].intersection(members)
            if "crossing" in head and not lit:
                flashing_heads += 1  # the walk light's dark half of a flash
                continue
            assert len(lit) == 1, f"{head} at {s['t']}: {lit}"
    assert flashing_heads


def test_vehicles_differ_and_show_what_they_do(run):
    _, samples, _ = run
    assert len({car[4] for car in samples[0]["cars"]}) >= 4
    lit = 0.0
    for key in ("brakes", "left", "right"):
        values = {v for s in samples for v in s[key]}
        assert lit in values and math.pi in values, key
    assert any(a != b for a, b in zip(samples[0]["wheels"], samples[-1]["wheels"], strict=True))


def test_someone_reaches_for_a_button(run):
    _, samples, _ = run
    assert max(s["reach"] for s in samples) == 1.0


def test_a_restarted_run_is_the_same_run():
    def track():
        sim = lagrange.Simulation(None, world=TOWN, traffic=(8, 8, 3))
        sim.step_until(30.0)
        return sim.state_hash()

    assert track() == track()


def test_spawning_and_removing_a_robot_keeps_the_town_running():
    sim = lagrange.Simulation(None, world=TOWN, traffic=(6, 6, 4))
    sim.step_until(5.0)
    robot = str(Path(TOWN).parents[2] / "robots/sedan/sedan.usda")
    name = sim.spawn(robot, "ego", at=[50.0, 1.75, 0.5, math.pi])
    sim.step_until(sim.time + 3.0)
    sim.delete(name)
    sim.step_until(sim.time + 3.0)
    assert len(cars(sim)) == 6

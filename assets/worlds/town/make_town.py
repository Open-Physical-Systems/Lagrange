#!/usr/bin/env python3
"""Writes the town: its roads as OpenDRIVE and its buildings and street furniture as USD.

Every model here is made by this script, so the town needs no outside assets. Run it from this
directory, then convert the roads as its last line says:

    python3 make_town.py
    ros2 run lagrange_opendrive lagrange_opendrive_import town.xodr town_roads.usda \\
        --root roads --step 4

A 200 m road grid: nine nodes (a crossroads, T junctions, bends at the corners) and two lone
crossings between them. One lane each way, 3 m sidewalks. Sidewalk lanes in junctions are the
walkers' paths: those that go over a driving lane are the crossings the traffic module finds, with
their lights and buttons (domains/traffic.md).
"""

import math
import random
from itertools import pairwise
from pathlib import Path

LANE = 3.5  # m
WALK = 3.0  # m
KERB = LANE
WALK_T = KERB + WALK / 2.0  # m, a sidewalk's middle from the road's
BOX = 10.0  # m, a junction's half size: the stop line
ZEBRA = 7.5  # m from the junction's middle
MID_BOX = 6.0  # m, the half size of a lone crossing
GRID = (-100.0, 0.0, 100.0)
BLOCK_IN = KERB + WALK  # m from a road's middle to where buildings may stand

random.seed(20261101)

# ---------------------------------------------------------------------------------------------
# Roads
# ---------------------------------------------------------------------------------------------

nodes = {(x, y): {"box": BOX, "mid": False} for x in GRID for y in GRID}
for at in ((-50.0, 0.0), (0.0, 50.0)):
    nodes[at] = {"box": MID_BOX, "mid": True}
edges = []
for x in GRID:
    for a, b in pairwise(GRID):
        edges.append(((x, a), (x, b)))
        edges.append(((a, x), (b, x)))
for mid, (a, b) in (
    ((-50.0, 0.0), ((-100.0, 0.0), (0.0, 0.0))),
    ((0.0, 50.0), ((0.0, 0.0), (0.0, 100.0))),
):
    edges.remove((a, b))
    edges += [(a, mid), (mid, b)]

xml_roads = []
xml_junctions = []
next_id = 100
roads = {}  # (a, b) -> (id, length, hdg)


def lane_xml(lane_id, width, kind, marks=(), pred=None, succ=None):
    link = ""
    if pred is not None or succ is not None:
        link = "<link>"
        if pred is not None:
            link += f'<predecessor id="{pred}"/>'
        if succ is not None:
            link += f'<successor id="{succ}"/>'
        link += "</link>"
    mark = "".join(
        f'<roadMark sOffset="0" type="{m}" color="standard" width="0.12"/>' for m in marks
    )
    return (
        f'<lane id="{lane_id}" type="{kind}" level="false">{link}'
        f'<width sOffset="0" a="{width}" b="0" c="0" d="0"/>{mark}</lane>'
    )


node_id = {at: 1000 + i for i, at in enumerate(sorted(nodes))}
for i, (a, b) in enumerate(edges):
    rid = i + 1
    dx, dy = b[0] - a[0], b[1] - a[1]
    dist = math.hypot(dx, dy)
    hdg = math.atan2(dy, dx)
    ux, uy = dx / dist, dy / dist
    box_a, box_b = nodes[a]["box"], nodes[b]["box"]
    x0, y0 = a[0] + ux * box_a, a[1] + uy * box_a
    length = dist - box_a - box_b
    roads[(a, b)] = (rid, length, hdg)
    left = lane_xml(1, LANE, "driving", ["none"]) + lane_xml(2, WALK, "sidewalk")
    right = lane_xml(-1, LANE, "driving", ["none"]) + lane_xml(-2, WALK, "sidewalk")
    xml_roads.append(
        f'<road name="street_{rid}" length="{length}" id="{rid}" junction="-1">'
        f'<link><predecessor elementType="junction" elementId="{node_id[a]}"/>'
        f'<successor elementType="junction" elementId="{node_id[b]}"/></link>'
        f'<type s="0" type="town"><speed max="40" unit="km/h"/></type>'
        f'<planView><geometry s="0" x="{x0}" y="{y0}" hdg="{hdg:.12f}" length="{length}">'
        f"<line/></geometry></planView>"
        f'<lanes><laneSection s="0"><left>{left}</left>'
        f'<center><lane id="0" type="none" level="false">'
        f'<roadMark sOffset="0" type="broken" color="standard" width="0.15"/></lane></center>'
        f"<right>{right}</right></laneSection></lanes></road>"
    )


def arms_of(at):
    """Arms of a node: (edge, this node is the edge's start, outward heading), anticlockwise."""
    arms = []
    for edge in roads:
        if at == edge[0] or at == edge[1]:
            start = at == edge[0]
            other = edge[1] if start else edge[0]
            arms.append((edge, start, math.atan2(other[1] - at[1], other[0] - at[0])))
    return sorted(arms, key=lambda arm: arm[2] % (2 * math.pi))


def walk_road(junction, a, b, name):
    """A straight sidewalk lane in a junction: its lane -1 runs from a to b."""
    global next_id
    rid = next_id
    next_id += 1
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    hdg = math.atan2(b[1] - a[1], b[0] - a[0])
    # Lane -1 lies right of the reference line: the line is half a width to the left.
    x = a[0] - math.sin(hdg) * WALK / 2.0
    y = a[1] + math.cos(hdg) * WALK / 2.0
    xml_roads.append(
        f'<road name="{name}" length="{length:.9f}" id="{rid}" junction="{junction}">'
        f'<planView><geometry s="0" x="{x:.9f}" y="{y:.9f}" hdg="{hdg:.12f}" '
        f'length="{length:.9f}"><line/></geometry></planView>'
        f'<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center>'
        f"<right>{lane_xml(-1, WALK, 'sidewalk')}</right></laneSection></lanes></road>"
    )


crossings = []  # (x0, y0, x1, y1): zebra centre line, kerb to kerb
stop_lines = []  # (x, y, heading across the lane's width)
for at, node in nodes.items():
    jid = node_id[at]
    arms = arms_of(at)
    box = node["box"]
    connections = []

    def world(theta, forward, left, at=at):
        return (
            at[0] + forward * math.cos(theta) - left * math.sin(theta),
            at[1] + forward * math.sin(theta) + left * math.cos(theta),
        )

    # Connecting lanes: in from every arm, out to every other.
    for edge_in, start_in, theta_in in arms:
        rid_in = roads[edge_in][0]
        lane_in = 1 if start_in else -1
        x0, y0 = world(theta_in, box, LANE / 2.0)
        h0 = theta_in + math.pi
        stop_lines.append((x0, y0, theta_in))
        for edge_out, start_out, theta_out in arms:
            if edge_out == edge_in:
                continue
            rid_out = roads[edge_out][0]
            lane_out = -1 if start_out else 1
            # The connecting road's lane -1 lies right of its line: the line starts and ends
            # half a lane to the left of the lanes' middles.
            xs, ys = world(theta_in, box, 0.0)
            xe, ye = world(theta_out, box, 0.0)
            turn = (theta_out - h0 + math.pi) % (2 * math.pi) - math.pi
            chord = math.hypot(xe - xs, ye - ys)
            if abs(turn) < 1e-9:
                geometry, length = "<line/>", chord
            else:
                k = 2.0 * math.sin(turn / 2.0) / chord
                length = turn / k
                geometry = f'<arc curvature="{k:.12f}"/>'
            rid = next_id
            next_id += 1
            contact_in = "start" if start_in else "end"
            contact_out = "start" if start_out else "end"
            xml_roads.append(
                f'<road name="connect_{rid_in}_{rid_out}" length="{length:.9f}" id="{rid}" '
                f'junction="{jid}"><link>'
                f'<predecessor elementType="road" elementId="{rid_in}" '
                f'contactPoint="{contact_in}"/>'
                f'<successor elementType="road" elementId="{rid_out}" '
                f'contactPoint="{contact_out}"/></link>'
                f'<type s="0" type="town"><speed max="25" unit="km/h"/></type>'
                f'<planView><geometry s="0" x="{xs:.9f}" y="{ys:.9f}" hdg="{h0:.12f}" '
                f'length="{length:.9f}">{geometry}</geometry></planView>'
                f'<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/>'
                f"</center><right>{lane_xml(-1, LANE, 'driving', (), lane_in, lane_out)}"
                f"</right></laneSection></lanes></road>"
            )
            connections.append(
                f'<connection id="{rid}" incomingRoad="{rid_in}" connectingRoad="{rid}" '
                f'contactPoint="start"><laneLink from="{lane_in}" to="-1"/></connection>'
            )
    xml_junctions.append(
        f'<junction id="{jid}" name="node_{jid}">{"".join(connections)}</junction>'
    )

    # Walkers' paths. Every arm's sidewalks reach a point on each side.
    if node["mid"]:
        theta = arms[0][2]
        for side in (1.0, -1.0):
            middle = world(theta, 0.0, side * WALK_T)
            walk_road(jid, world(theta, box, side * WALK_T), middle, f"walk_{jid}")
            walk_road(jid, middle, world(theta, -box, side * WALK_T), f"walk_{jid}")
        walk_road(jid, world(theta, 0.0, WALK_T), world(theta, 0.0, -WALK_T), f"crossing_{jid}")
        crossings.append((*world(theta, 0.0, KERB), *world(theta, 0.0, -KERB)))
        continue
    with_zebras = len(arms) >= 3
    reach = ZEBRA if with_zebras else box
    for _edge, _start, theta in arms:
        if with_zebras:
            for side in (1.0, -1.0):
                walk_road(
                    jid,
                    world(theta, box, side * WALK_T),
                    world(theta, ZEBRA, side * WALK_T),
                    f"walk_{jid}",
                )
            walk_road(
                jid,
                world(theta, ZEBRA, WALK_T),
                world(theta, ZEBRA, -WALK_T),
                f"crossing_{jid}",
            )
            # Kerb to kerb, where the corner's curve (radius BLOCK_IN, about (BOX, BOX)) is.
            half = BOX - math.sqrt(BLOCK_IN**2 - (BOX - ZEBRA) ** 2)
            crossings.append((*world(theta, ZEBRA, half), *world(theta, ZEBRA, -half)))
    # Round the corners, anticlockwise from each arm's left side to the next arm's right side.
    for i, (_e, _s, theta) in enumerate(arms):
        next_theta = arms[(i + 1) % len(arms)][2]
        gap = (next_theta - theta) % (2 * math.pi)
        here = world(theta, reach, WALK_T)
        there = world(next_theta, reach, -WALK_T)
        if abs(gap - math.pi / 2) < 1e-6:
            walk_road(jid, here, there, f"walk_{jid}")  # the inside of a corner
        else:
            # Past the arms that are not there, by the outer corners.
            points = [here]
            steps = round(gap / (math.pi / 2))
            for s in range(steps):
                points.append(world(theta + s * math.pi / 2, WALK_T, WALK_T))
            points.append(there)
            for p, q in pairwise(points):
                walk_road(jid, p, q, f"walk_{jid}")

Path("town.xodr").write_text(
    '<?xml version="1.0" standalone="yes"?>\n'
    "<!-- The town's roads: written by make_town.py; do not edit by hand. -->\n"
    "<OpenDRIVE>\n"
    '<header revMajor="1" revMinor="6" name="town" version="1" north="0" south="0" '
    'east="0" west="0"/>\n' + "\n".join(xml_roads) + "\n" + "\n".join(xml_junctions) + "\n"
    "</OpenDRIVE>\n"
)

# ---------------------------------------------------------------------------------------------
# Meshes
# ---------------------------------------------------------------------------------------------

LOOKS = {
    # name: (metallic, roughness); illustrative, as surfaceLookPreset's.
    "brick": (0.0, 0.85),
    "plaster": (0.0, 0.8),
    "concrete": (0.0, 0.9),
    "pavement": (0.0, 0.92),
    "kerb_stone": (0.0, 0.85),
    "road_paint": (0.0, 0.7),
    "asphalt": (0.0, 0.95),
    "window_glass": (0.0, 0.05),
    "frame_metal": (0.9, 0.4),
    "steel": (0.9, 0.35),
    "bark": (0.0, 0.95),
    "leaves": (0.0, 0.75),
    "lawn": (0.0, 0.95),
    "awning_cloth": (0.0, 0.9),
    "roof_felt": (0.0, 0.9),
    "wood": (0.0, 0.65),
}


class Mesh:
    def __init__(self):
        self.points, self.normals, self.indices, self.counts = [], [], [], []
        self.smooth = False  # without normals the reader shades every face flat

    def quad(self, a, b, c, d):
        """A quad with its corners anticlockwise seen from outside."""
        base = len(self.points)
        self.points += [a, b, c, d]
        self.normals += [(0.0, 0.0, 1.0)] * 4
        self.indices += [base, base + 1, base + 2, base + 3]
        self.counts.append(4)

    def tri(self, a, b, c):
        self.indices += [a, b, c]
        self.counts.append(3)

    def box(self, x0, x1, y0, y1, z0, z1, bottom=False):
        self.quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))
        self.quad((x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1))
        self.quad((x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1))
        self.quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))
        self.quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))
        if bottom:
            self.quad((x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0))

    def face_box(self, f, u0, u1, v0, v1, d0, d1):
        """A box on a facade: f maps (along the wall, up, out of the wall) to the world."""
        c = [f(u, v, d) for d in (d0, d1) for v in (v0, v1) for u in (u0, u1)]
        # 0 u0v0d0, 1 u1v0d0, 2 u0v1d0, 3 u1v1d0, 4 u0v0d1, 5 u1v0d1, 6 u0v1d1, 7 u1v1d1
        self.quad(c[4], c[5], c[7], c[6])  # front
        self.quad(c[6], c[7], c[3], c[2])  # top
        self.quad(c[0], c[1], c[5], c[4])  # under
        self.quad(c[1], c[3], c[7], c[5])  # one side
        self.quad(c[2], c[0], c[4], c[6])  # the other

    def front(self, f, u0, u1, v0, v1, d):
        """Only the face of a box on a facade: what shows of something a few cm proud."""
        self.quad(f(u0, v0, d), f(u1, v0, d), f(u1, v1, d), f(u0, v1, d))

    def ledge(self, f, u0, u1, v0, v1, d):
        """The front and the top of a ledge."""
        self.front(f, u0, u1, v0, v1, d)
        self.quad(f(u0, v1, d), f(u1, v1, d), f(u1, v1, 0.0), f(u0, v1, 0.0))

    def blob(self, cx, cy, cz, rx, ry, rz):
        """A low-poly ball, smooth: an octahedron cut once."""
        self.smooth = True
        base = len(self.points)
        rings = 3
        sides = 7
        pts = [(0.0, 0.0, 1.0)]
        for r in range(1, rings):
            polar = math.pi * r / rings
            for s in range(sides):
                az = 2 * math.pi * (s + (r % 2) * 0.5) / sides
                pts.append(
                    (
                        math.sin(polar) * math.cos(az),
                        math.sin(polar) * math.sin(az),
                        math.cos(polar),
                    )
                )
        pts.append((0.0, 0.0, -1.0))
        for p in pts:
            bump = 0.85 + 0.3 * random.random()
            self.points.append(
                (cx + rx * p[0] * bump, cy + ry * p[1] * bump, cz + rz * p[2] * bump)
            )
            self.normals.append(tuple(round(x, 3) + 0.0 for x in p))
        for s in range(sides):
            self.tri(base, base + 1 + s, base + 1 + (s + 1) % sides)
            last = base + 1 + (rings - 2) * sides
            self.tri(last + sides, last + (s + 1) % sides, last + s)
        for r in range(rings - 2):
            top = base + 1 + r * sides
            low = top + sides
            for s in range(sides):
                n = (s + 1) % sides
                self.tri(top + s, low + s, low + n)
                self.tri(top + s, low + n, top + n)

    def cylinder(self, cx, cy, r, z0, z1, sides=8):
        self.smooth = True
        base = len(self.points)
        for s in range(sides):
            a = 2 * math.pi * s / sides
            n = (round(math.cos(a), 3), round(math.sin(a), 3), 0.0)
            for z in (z0, z1):
                self.points.append((cx + r * n[0], cy + r * n[1], z))
                self.normals.append(n)
        for s in range(sides):
            p = base + 2 * s
            q = base + 2 * ((s + 1) % sides)
            self.tri(p, q, q + 1)
            self.tri(p, q + 1, p + 1)


def fmt(v):
    text = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def usd_mesh(name, mesh, colour, look, indent="    "):
    if not mesh.points:
        return ""
    counts = ", ".join(map(str, mesh.counts))
    indices = ", ".join(map(str, mesh.indices))
    pts = ", ".join(f"({fmt(p[0])},{fmt(p[1])},{fmt(p[2])})" for p in mesh.points)
    normals = ""
    if mesh.smooth:
        nor = ", ".join(f"({fmt(n[0])},{fmt(n[1])},{fmt(n[2])})" for n in mesh.normals)
        normals = f"""{indent}    normal3f[] normals = [{nor}] (
{indent}        interpolation = "vertex"
{indent}    )
"""
    c = f"({colour[0]:.3f},{colour[1]:.3f},{colour[2]:.3f})"
    return f"""{indent}def Mesh "{name}" (
{indent}    prepend apiSchemas = ["MaterialBindingAPI"]
{indent})
{indent}{{
{indent}    int[] faceVertexCounts = [{counts}]
{indent}    int[] faceVertexIndices = [{indices}]
{normals}{indent}    point3f[] points = [{pts}]
{indent}    color3f[] primvars:displayColor = [{c}]
{indent}    uniform token subdivisionScheme = "none"
{indent}    rel material:binding = </town/looks/{look}>
{indent}}}
"""


def usd_collider(name, x0, x1, y0, y1, z0, z1, indent="    "):
    return f"""{indent}def Cube "{name}" (
{indent}    prepend apiSchemas = ["PhysicsCollisionAPI"]
{indent})
{indent}{{
{indent}    uniform token purpose = "guide"
{indent}    double size = 1
{indent}    float3 xformOp:scale = ({x1 - x0}, {y1 - y0}, {z1 - z0})
{indent}    double3 xformOp:translate = ({(x0 + x1) / 2}, {(y0 + y1) / 2}, {(z0 + z1) / 2})
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
{indent}}}
"""


def usd_layer(body, doc):
    return f'''#usda 1.0
(
    defaultPrim = "town"
    doc = "{doc} Written by make_town.py."
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

over "town"
{{
{body}}}
'''


def looks_layer():
    out = ['    def Scope "looks"\n    {\n']
    for name, (metallic, roughness) in LOOKS.items():
        out.append(
            f"""        def Material "{name}"
        {{
            token outputs:surface.connect = </town/looks/{name}/surface.outputs:surface>
            def Shader "surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                float inputs:metallic = {metallic}
                float inputs:roughness = {roughness}
                token outputs:surface
            }}
        }}
"""
        )
    out.append("    }\n")
    return "".join(out)


# ---------------------------------------------------------------------------------------------
# Buildings
# ---------------------------------------------------------------------------------------------

BRICKS = [(0.42, 0.18, 0.13), (0.5, 0.27, 0.2), (0.33, 0.16, 0.13), (0.55, 0.36, 0.28)]
PLASTERS = [(0.82, 0.76, 0.63), (0.74, 0.7, 0.66), (0.78, 0.63, 0.48), (0.66, 0.72, 0.72),
            (0.86, 0.84, 0.8)]  # fmt: skip
CONCRETES = [(0.55, 0.55, 0.53), (0.46, 0.46, 0.46), (0.62, 0.6, 0.56)]
DOORS = [(0.13, 0.2, 0.15), (0.3, 0.12, 0.08), (0.1, 0.12, 0.2)]
AWNINGS = [(0.55, 0.1, 0.1), (0.12, 0.3, 0.2), (0.15, 0.2, 0.4), (0.8, 0.6, 0.2), (0.4, 0.4, 0.42)]
SHOPS = [(0.1, 0.1, 0.11), (0.25, 0.12, 0.08), (0.08, 0.16, 0.25), (0.7, 0.68, 0.62)]


def building(name, x0, x1, y0, y1, floors, style, fronts):
    """A building's prims, in the block's frame. `fronts`: which sides face a street."""
    storey = 3.3
    ground = 4.0
    height = ground + (floors - 1) * storey
    walls, frames, glass, trim, door, roof, awning, shop = (Mesh() for _ in range(8))
    glass_tower = style == "tower"
    if style == "brick":
        wall, look = random.choice(BRICKS), "brick"
    elif style == "plaster":
        wall, look = random.choice(PLASTERS), "plaster"
    else:
        wall, look = random.choice(CONCRETES), "concrete"
    awning_colour = random.choice(AWNINGS)
    shop_colour = random.choice(SHOPS)
    door_colour = random.choice(DOORS)
    has_shops = style != "tower" and random.random() < 0.75

    walls.box(x0, x1, y0, y1, 0.0, height)
    roof.box(x0 + 0.3, x1 - 0.3, y0 + 0.3, y1 - 0.3, height, height + 0.05)
    trim.box(x0 - 0.15, x1 + 0.15, y0 - 0.15, y1 + 0.15, height, height + 0.5)
    roof.box(x0 + 0.25, x1 - 0.25, y0 + 0.25, y1 - 0.25, height + 0.5, height + 0.55)
    if floors > 3:
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        trim.box(cx - 1.6, cx + 1.6, cy - 1.2, cy + 1.2, height + 0.5, height + 3.0)
        frames.box(cx + 3, cx + 4.2, cy + 1, cy + 2, height + 0.5, height + 1.8)

    # Facades: u along the wall, v up, d out.
    faces = {
        "south": (x1 - x0, lambda u, v, d: (x0 + u, y0 - d, v)),
        "north": (x1 - x0, lambda u, v, d: (x1 - u, y1 + d, v)),
        "east": (y1 - y0, lambda u, v, d: (x1 + d, y0 + u, v)),
        "west": (y1 - y0, lambda u, v, d: (x0 - d, y1 - u, v)),
    }
    bay_width = 2.6 if glass_tower else 3.2
    for side in fronts:
        width, f = faces[side]
        bays = max(1, int(width // bay_width))
        bay = width / bays
        if glass_tower:
            glass.front(f, 0.1, width - 0.1, 0.3, height - 0.2, 0.03)
            for b in range(bays + 1):
                u = b * bay
                frames.front(f, max(u - 0.05, 0), min(u + 0.05, width), 0, height, 0.06)
            for n in range(floors + 1):
                v = min(n * storey + 0.7, height)
                frames.front(f, 0, width, v - 0.08, v + 0.08, 0.06)
            continue
        # Upper floors.
        for n in range(1, floors):
            base = ground + (n - 1) * storey
            if n > 1 or not has_shops:
                trim.ledge(f, 0.0, width, base - 0.08, base + 0.04, 0.05)
            for b in range(bays):
                u = (b + 0.5) * bay
                hw = 0.62
                z0, z1 = base + 0.95, base + 2.5
                frames.front(f, u - hw - 0.08, u + hw + 0.08, z0 - 0.08, z1 + 0.08, 0.03)
                glass.front(f, u - hw, u + hw, z0, z1, 0.04)
                frames.front(f, u - 0.025, u + 0.025, z0, z1, 0.05)
                trim.ledge(f, u - hw - 0.14, u + hw + 0.14, z0 - 0.2, z0 - 0.08, 0.1)
        # The ground floor: shop fronts or plain windows and a door.
        if has_shops:
            shop.face_box(f, 0.0, width, 0.0, 3.8, 0.0, 0.08)
            trim.face_box(f, 0.0, width, 3.8, 4.0, 0.0, 0.2)
            pier = 0.5
            n_shops = max(1, int(width // 7.0))
            piece = width / n_shops
            for s in range(n_shops):
                a, b = s * piece + pier, (s + 1) * piece - pier
                door_at = a + 0.6
                frames.face_box(f, a, b, 0.45, 3.2, 0.08, 0.12)
                glass.face_box(f, door_at + 1.1, b - 0.06, 0.5, 3.15, 0.1, 0.14)
                glass.face_box(f, door_at, door_at + 1.0, 0.04, 2.6, 0.1, 0.14)
                # A sign board and an awning over the window.
                door.face_box(f, a + 0.2, b - 0.2, 3.3, 3.72, 0.08, 0.16)
                awning.quad(
                    f(b, 3.25, 0.12),
                    f(a, 3.25, 0.12),
                    f(a, 2.75, 1.3),
                    f(b, 2.75, 1.3),
                )
                awning.quad(f(b, 2.75, 1.3), f(a, 2.75, 1.3), f(a, 2.45, 1.3), f(b, 2.45, 1.3))
        else:
            door_bay = random.randrange(bays)
            for b in range(bays):
                u = (b + 0.5) * bay
                if b == door_bay:
                    trim.face_box(f, u - 0.75, u + 0.75, 0.0, 2.75, 0.0, 0.06)
                    door.face_box(f, u - 0.6, u + 0.6, 0.0, 2.6, 0.0, 0.09)
                    trim.face_box(f, u - 1.0, u + 1.0, 2.75, 2.95, 0.0, 0.35)
                else:
                    frames.front(f, u - 0.75, u + 0.75, 0.85, 3.05, 0.03)
                    glass.front(f, u - 0.68, u + 0.68, 0.92, 2.98, 0.04)
                    trim.ledge(f, u - 0.85, u + 0.85, 0.72, 0.85, 0.1)
    parts = [
        usd_mesh("walls", walls, wall, look, "        "),
        usd_mesh("frames", frames, (0.17, 0.17, 0.18), "frame_metal", "        "),
        usd_mesh("glass", glass, (0.07, 0.1, 0.13), "window_glass", "        "),
        usd_mesh(
            "trim", trim, tuple(min(c * 1.25 + 0.1, 0.9) for c in wall), "concrete", "        "
        ),
        usd_mesh("door", door, door_colour, "wood", "        "),
        usd_mesh("roof", roof, (0.16, 0.16, 0.17), "roof_felt", "        "),
        usd_mesh("awning", awning, awning_colour, "awning_cloth", "        "),
        usd_mesh("shop", shop, shop_colour, "concrete", "        "),
        usd_collider("collision", x0, x1, y0, y1, 0.0, height, "        "),
    ]
    return f'    def Xform "{name}"\n    {{\n' + "".join(parts) + "    }\n"


def lots(a0, a1, depth_room):
    """Lots along a street from a0 to a1: their (start, end)."""
    out = []
    at = a0
    while a1 - at > 8.0:
        width = random.uniform(13.0, 24.0)
        if a1 - at - width < 11.0:
            width = a1 - at
        out.append((at, at + width))
        at += width
    if out:
        out[-1] = (out[-1][0], a1)
    return out


def style_of(downtown):
    r = random.random()
    if downtown and r < 0.45:
        return "tower", random.randint(8, 13)
    if r < 0.5:
        return "brick", random.randint(3, 6)
    if r < 0.85:
        return "plaster", random.randint(2, 5)
    return "concrete", random.randint(3, 7)


def block_buildings(tag, x0, x1, y0, y1, downtown):
    """Building rows on all four sides of a block; the corners go to the x rows."""
    out = []
    depth = 15.0
    i = 0
    for a, b in lots(x0, x1, depth):
        style, floors = style_of(downtown)
        out.append(
            building(f"{tag}_s{i}", a, b, y0, y0 + depth, floors, style, ["south", "east", "west"])
        )
        i += 1
    for a, b in lots(x0, x1, depth):
        style, floors = style_of(downtown)
        out.append(
            building(f"{tag}_n{i}", a, b, y1 - depth, y1, floors, style, ["north", "east", "west"])
        )
        i += 1
    for a, b in lots(y0 + depth, y1 - depth, depth):
        style, floors = style_of(downtown)
        out.append(building(f"{tag}_w{i}", x0, x0 + depth, a, b, floors, style, ["west"]))
        i += 1
    for a, b in lots(y0 + depth, y1 - depth, depth):
        style, floors = style_of(downtown)
        out.append(building(f"{tag}_e{i}", x1 - depth, x1, a, b, floors, style, ["east"]))
        i += 1
    return out


def outer_row(tag, x0, x1, y0, y1, facing):
    """A row of buildings outside the ring road, facing it. Row along the longer side."""
    out = []
    along_x = (x1 - x0) > (y1 - y0)
    span = lots(x0, x1, 0) if along_x else lots(y0, y1, 0)
    for i, (a, b) in enumerate(span):
        style, floors = style_of(False)
        if along_x:
            out.append(building(f"{tag}{i}", a, b, y0, y1, floors, style, [facing]))
        else:
            out.append(building(f"{tag}{i}", x0, x1, a, b, floors, style, [facing]))
    return out


IN = BLOCK_IN
blocks = {
    "sw": (-100 + IN, -IN, -100 + IN, -IN),
    "se": (IN, 100 - IN, -100 + IN, -IN),
    "nw": (-100 + IN, -IN, IN, 100 - IN),
    "ne": (IN, 100 - IN, IN, 100 - IN),
}
EDGE = 100 + IN

layers = {}
layers["town_buildings_sw"] = block_buildings("sw", *blocks["sw"], False)
layers["town_buildings_se"] = block_buildings("se", *blocks["se"], True)
layers["town_buildings_ne"] = block_buildings("ne", *blocks["ne"], True)
# The north-west block is the park: a row only on its far sides.
x0, x1, y0, y1 = blocks["nw"]
layers["town_buildings_outer_1"] = [
    *outer_row("outer_s", -EDGE, EDGE, -EDGE - 16, -EDGE, "north"),
    *outer_row("outer_n", -EDGE, EDGE, EDGE, EDGE + 16, "south"),
]
layers["town_buildings_outer_2"] = [
    *outer_row("outer_w", -EDGE - 16, -EDGE, -EDGE, EDGE, "east"),
    *outer_row("outer_e", EDGE, EDGE + 16, -EDGE, EDGE, "west"),
]

for layer, prims in layers.items():
    text = usd_layer("".join(prims), f"The town's buildings: {layer}.")
    Path(f"{layer}.usda").write_text(text)

# ---------------------------------------------------------------------------------------------
# The street: pavements, kerbs, paint, trees, lamps, benches
# ---------------------------------------------------------------------------------------------

asphalt, pavement, kerb, paint, lawn = Mesh(), Mesh(), Mesh(), Mesh(), Mesh()
bark, leaves, poles, benches, bins = Mesh(), Mesh(), Mesh(), Mesh(), Mesh()
street_colliders = []
SLAB = 0.02  # m above the road: the kerb is drawn, walkers stay on the level
KERB_H = 0.12  # m
KERB_W = 0.22  # m


def fill(mesh, outline, z):
    centre = (sum(p[0] for p in outline) / len(outline), sum(p[1] for p in outline) / len(outline))
    base = len(mesh.points)
    mesh.points.append((centre[0], centre[1], z))
    mesh.normals.append((0.0, 0.0, 1.0))
    for p in outline:
        mesh.points.append((p[0], p[1], z))
        mesh.normals.append((0.0, 0.0, 1.0))
    n = len(outline)
    for i in range(n):
        mesh.tri(base, base + 1 + i, base + 1 + (i + 1) % n)


def kerb_run(p, q, out):
    """A kerb stone run from p to q, `out` the side of the road (a unit vector)."""
    ax, ay = p
    bx, by = q
    ox, oy = out
    t = KERB_W
    pts = lambda z, d: ((ax + ox * d, ay + oy * d, z), (bx + ox * d, by + oy * d, z))  # noqa: E731
    a0, b0 = pts(KERB_H, 0.0)
    a1, b1 = pts(KERB_H, t)
    c0, d0 = pts(0.0, 0.0)
    e = (bx - ax) * oy - (by - ay) * ox
    if e > 0:
        kerb.quad(a0, b0, b1, a1)
        kerb.quad(c0, d0, b0, a0)
    else:
        kerb.quad(b0, a0, a1, b1)
        kerb.quad(d0, c0, a0, b0)


# Junctions' boxes and the pavement slabs.
for at, node in nodes.items():
    b = node["box"]
    asphalt.box(at[0] - b, at[0] + b, at[1] - b, at[1] + b, 0.0, 0.012)

R_CORNER = BLOCK_IN
for _tag, (x0, x1, y0, y1) in blocks.items():
    outline = []
    # Going round the block anticlockwise; each corner's curve is about the junction's box corner.
    corners = [
        ((x1 + BOX - IN, y0 - BOX + IN), math.pi / 2, math.pi),  # south-east: from +y... to -x
        ((x1 + BOX - IN, y1 + BOX - IN), math.pi, 1.5 * math.pi),
        ((x0 - BOX + IN, y1 + BOX - IN), 1.5 * math.pi, 2.0 * math.pi),
        ((x0 - BOX + IN, y0 - BOX + IN), 0.0, 0.5 * math.pi),
    ]
    arcs = []
    for (cx, cy), a0, a1 in corners:
        pts = []
        for s in range(9):
            a = a0 + (a1 - a0) * s / 8
            pts.append((cx + R_CORNER * math.cos(a), cy + R_CORNER * math.sin(a)))
        # Anticlockwise round the block, the arc about the outer centre runs clockwise.
        arcs.append(list(reversed(pts)))
    for arc in arcs:
        outline += arc
    # Lawn in the park, pavement everywhere else, as one slab with the walkway round it.
    fill(pavement, outline, SLAB)

    # Kerbs: along the curves, and along the straights with gaps at the lone crossings.
    mid = {(-50.0, -IN): "x", (-50.0, IN): "x", (-IN, 50.0): "y", (IN, 50.0): "y"}
    centre = ((x0 + x1) / 2, (y0 + y1) / 2)
    for arc in arcs:
        for p, q in pairwise(arc):
            mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
            ox, oy = centre[0] - mx, centre[1] - my
            size = math.hypot(ox, oy)
            kerb_run(p, q, (ox / size, oy / size))
    for i in range(4):
        p = arcs[i][-1]
        q = arcs[(i + 1) % 4][0]
        mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
        ox, oy = (0.0, 0.0)
        if abs(p[0] - q[0]) > abs(p[1] - q[1]):
            oy = 1.0 if centre[1] > my else -1.0
        else:
            ox = 1.0 if centre[0] > mx else -1.0
        # A lowered kerb at a lone crossing.
        cuts = []
        for (cut_x, cut_y), axis in mid.items():
            if axis == "x" and abs(my - cut_y) < 0.01 and min(p[0], q[0]) < cut_x < max(p[0], q[0]):
                cuts.append(cut_x)
            if axis == "y" and abs(mx - cut_x) < 0.01 and min(p[1], q[1]) < cut_y < max(p[1], q[1]):
                cuts.append(cut_y)
        if cuts:
            cut = cuts[0]
            if ox == 0.0:
                sign = 1.0 if q[0] > p[0] else -1.0
                kerb_run(p, (cut - sign * 2.0, p[1]), (ox, oy))
                kerb_run((cut + sign * 2.0, p[1]), q, (ox, oy))
            else:
                sign = 1.0 if q[1] > p[1] else -1.0
                kerb_run(p, (p[0], cut - sign * 2.0), (ox, oy))
                kerb_run((p[0], cut + sign * 2.0), q, (ox, oy))
        else:
            kerb_run(p, q, (ox, oy))

# Outside the ring road: a pavement all round, then what lies beyond.
for x0, x1, y0, y1 in (
    (-EDGE, EDGE, -EDGE, -100 - KERB),
    (-EDGE, EDGE, 100 + KERB, EDGE),
    (-EDGE, -100 - KERB, -100 - KERB, 100 + KERB),
    (100 + KERB, EDGE, -100 - KERB, 100 + KERB),
):
    pavement.quad((x0, y0, SLAB), (x1, y0, SLAB), (x1, y1, SLAB), (x0, y1, SLAB))
ring = 100 + KERB
for p, q, out in (
    ((-ring, -ring), (ring, -ring), (0.0, -1.0)),
    ((ring, ring), (-ring, ring), (0.0, 1.0)),
    ((-ring, ring), (-ring, -ring), (-1.0, 0.0)),
    ((ring, -ring), (ring, ring), (1.0, 0.0)),
):
    kerb_run(p, q, out)
# Beyond the buildings, ground for the eye.
far = EDGE + 16.0
lawn.quad((-400, -400, 0.015), (400, -400, 0.015), (400, -far, 0.015), (-400, -far, 0.015))
lawn.quad((-400, far, 0.015), (400, far, 0.015), (400, 400, 0.015), (-400, 400, 0.015))
lawn.quad((-400, -far, 0.015), (-far, -far, 0.015), (-far, far, 0.015), (-400, far, 0.015))
lawn.quad((far, -far, 0.015), (400, -far, 0.015), (400, far, 0.015), (far, far, 0.015))

# Paint: zebra crossings, and stop lines.
PAINT_Z = 0.016
STRIPE = 0.5
for x0, y0, x1, y1 in crossings:
    length = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    vx, vy = -uy, ux
    n = int(length / (2 * STRIPE))
    start = (length - (2 * n - 1) * STRIPE) / 2.0
    for i in range(n):
        a = start + i * 2 * STRIPE
        b = a + STRIPE
        corners = [
            (x0 + ux * a + vx * 1.5, y0 + uy * a + vy * 1.5),
            (x0 + ux * a - vx * 1.5, y0 + uy * a - vy * 1.5),
            (x0 + ux * b - vx * 1.5, y0 + uy * b - vy * 1.5),
            (x0 + ux * b + vx * 1.5, y0 + uy * b + vy * 1.5),
        ]
        area = (corners[1][0] - corners[0][0]) * (corners[2][1] - corners[0][1]) - (
            corners[1][1] - corners[0][1]
        ) * (corners[2][0] - corners[0][0])
        if area < 0:
            corners.reverse()
        paint.quad(*[(p[0], p[1], PAINT_Z) for p in corners])
for x, y, theta in stop_lines:
    # Across the lane (x, y its middle), 0.4 m deep, on the road's side of the junction.
    c, s = math.cos(theta), math.sin(theta)
    corners = []
    for f, left in ((0.0, -1.7), (0.4, -1.7), (0.4, 1.7), (0.0, 1.7)):
        corners.append((x + f * c - left * s, y + f * s + left * c, PAINT_Z))
    paint.quad(*corners)

# ---------------------------------------------------------------------------------------------
# Trees, lamps, benches on the sidewalks, and the park
# ---------------------------------------------------------------------------------------------


def tree(x, y, size=1.0, collide=True):
    h = 3.0 * size
    bark.cylinder(x, y, 0.16 * size, 0.0, h)
    for _ in range(9):
        # Many small clumps: a crown's outline is not one ball.
        a = random.uniform(0.0, 2.0 * math.pi)
        r = random.uniform(0.0, 1.5) * size
        leaves.blob(
            x + r * math.cos(a),
            y + r * math.sin(a),
            h + random.uniform(-0.2, 2.2) * size,
            random.uniform(0.8, 1.25) * size,
            random.uniform(0.8, 1.25) * size,
            random.uniform(0.7, 1.05) * size,
        )
    if collide:
        street_colliders.append((x - 0.16, x + 0.16, y - 0.16, y + 0.16, 0.0, h))


def street_lamp(x, y, out_x, out_y):
    poles.cylinder(x, y, 0.07, 0.0, 7.0)
    tip = (x + out_x * 1.6, y + out_y * 1.6)
    lo, hi = min(x, tip[0]) - 0.04, max(x, tip[0]) + 0.04
    lo2, hi2 = min(y, tip[1]) - 0.04, max(y, tip[1]) + 0.04
    poles.box(lo, hi, lo2, hi2, 6.95, 7.03)
    poles.box(tip[0] - 0.25, tip[0] + 0.25, tip[1] - 0.12, tip[1] + 0.12, 6.88, 6.98)
    street_colliders.append((x - 0.07, x + 0.07, y - 0.07, y + 0.07, 0.0, 7.0))


FURNITURE_T = KERB + 0.45  # m, between the kerb and the walkers
for (a, b), (_rid, _length, hdg) in roads.items():
    ux, uy = math.cos(hdg), math.sin(hdg)
    for side in (1.0, -1.0):
        nx, ny = -uy * side, ux * side
        s = 8.0
        i = 0
        while s < math.hypot(b[0] - a[0], b[1] - a[1]) - 8.0:
            # From the road's start node: keep clear of the junctions and their corners.
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            if 20.0 < s < dist - 20.0:
                x = a[0] + ux * s + nx * FURNITURE_T
                y = a[1] + uy * s + ny * FURNITURE_T
                if i % 2 == 0:
                    street_lamp(x, y, -nx, -ny)
                else:
                    tree(x, y, random.uniform(0.75, 1.05))
                if i % 3 == 1:
                    # A bench or a bin against the building side.
                    bx = a[0] + ux * (s + 3.0) + nx * (BLOCK_IN - 0.45)
                    by = a[1] + uy * (s + 3.0) + ny * (BLOCK_IN - 0.45)
                    if random.random() < 0.5:
                        hx = 0.9 * abs(ux) + 0.25 * abs(uy)
                        hy = 0.25 * abs(ux) + 0.9 * abs(uy)
                        benches.box(bx - hx, bx + hx, by - hy, by + hy, 0.42, 0.48)
                        benches.box(
                            bx - hx * abs(ux) - 0.05 * abs(uy) + 0.2 * nx,
                            bx + hx * abs(ux) + 0.05 * abs(uy) + 0.2 * nx,
                            by - hy * abs(uy) - 0.05 * abs(ux) + 0.2 * ny,
                            by + hy * abs(uy) + 0.05 * abs(ux) + 0.2 * ny,
                            0.5,
                            0.9,
                        )
                        poles.box(bx - hx, bx - hx + 0.1, by - hy, by - hy + 0.1, 0.0, 0.42)
                        poles.box(bx + hx - 0.1, bx + hx, by + hy - 0.1, by + hy, 0.0, 0.42)
                    else:
                        bins.cylinder(bx, by, 0.25, 0.0, 0.85, 10)
                i += 1
            s += 14.0

# The park: lawn, paths, trees.
x0, x1, y0, y1 = blocks["nw"]
inset = 0.8
lawn.quad(
    (x0 + inset, y0 + inset, SLAB + 0.008),
    (x1 - inset, y0 + inset, SLAB + 0.008),
    (x1 - inset, y1 - inset, SLAB + 0.008),
    (x0 + inset, y1 - inset, SLAB + 0.008),
)
cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
pz = SLAB + 0.012
pavement.quad((x0, cy - 1.5, pz), (x1, cy - 1.5, pz), (x1, cy + 1.5, pz), (x0, cy + 1.5, pz))
pavement.quad((cx - 1.5, y0, pz), (cx + 1.5, y0, pz), (cx + 1.5, y1, pz), (cx - 1.5, y1, pz))
placed = []
for _ in range(32):
    for _try in range(30):
        x = random.uniform(x0 + 4, x1 - 4)
        y = random.uniform(y0 + 4, y1 - 4)
        if (
            abs(x - cx) > 4
            and abs(y - cy) > 4
            and all(math.hypot(x - a, y - b) > 8 for a, b in placed)
        ):
            placed.append((x, y))
            tree(x, y, random.uniform(1.1, 1.7))
            break

body = looks_layer()
body += usd_mesh("junction_asphalt", asphalt, (0.2, 0.2, 0.21), "asphalt")
body += usd_mesh("pavements", pavement, (0.34, 0.33, 0.31), "pavement")
body += usd_mesh("kerbs", kerb, (0.44, 0.43, 0.41), "kerb_stone")
body += usd_mesh("road_paint", paint, (0.72, 0.72, 0.7), "road_paint")
body += usd_mesh("lawn", lawn, (0.2, 0.36, 0.13), "lawn")
trees = usd_mesh("trunks", bark, (0.24, 0.17, 0.11), "bark")
trees += usd_mesh("leaves", leaves, (0.13, 0.3, 0.1), "leaves")
Path("town_trees.usda").write_text(usd_layer(trees, "The town's trees."))
body += usd_mesh("street_poles", poles, (0.14, 0.15, 0.16), "steel")
body += usd_mesh("benches", benches, (0.35, 0.22, 0.12), "wood")
body += usd_mesh("bins", bins, (0.12, 0.16, 0.13), "steel")
for i, c in enumerate(street_colliders):
    body += usd_collider(f"street_collider_{i}", *c)
Path("town_street.usda").write_text(usd_layer(body, "The town's street: paint, kerbs, furniture."))

subs = ",\n".join(
    f"        @./{name}.usda@" for name in [*layers, "town_street", "town_trees", "town_roads"]
)
Path("town.usda").write_text(
    f'''#usda 1.0
(
    defaultPrim = "town"
    doc = """A small town for the traffic domain: a road grid with signalled junctions, zebra
    crossings with buttons, two lone crossings, buildings, a park. Written by make_town.py; the
    roads are town_roads.usda, converted from town.xodr. The roads lie on the ground plane that
    the loader adds; the traffic module adds the signals, the cars and the people."""
    kilogramsPerUnit = 1
    metersPerUnit = 1
    subLayers = [
{subs}
    ]
    upAxis = "Z"
)

def Xform "town"
{{
}}
'''
)

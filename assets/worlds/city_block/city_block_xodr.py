#!/usr/bin/env python3
"""Writes city_block.xodr: the city block's road network in OpenDRIVE 1.6.

A four-way junction at the origin, 24 m across, with four 58 m arms (two 3.5 m lanes each way,
3 m sidewalks, 50 km/h) and twelve connecting roads (straight on, right and left from every
arm; the turns are single arcs, 8.5 m radius right from the outer lane and 12 m left from the
inner one). Every reference line leaves the junction, so incoming traffic
uses the left lanes (1, 2) and outgoing traffic the right ones (-1, -2): right-hand traffic.
Run it from this directory to regenerate the file.
"""

import math

HALF = 12.0  # m, the junction's half width
ARM = 58.0  # m
LANE = 3.5  # m
WALK = 3.0  # m

arms = [  # id, outward heading
    (1, 0.0),
    (2, math.pi / 2),
    (3, math.pi),
    (4, -math.pi / 2),
]


def base(theta, t=0.0):
    """The point at the junction's edge on arm heading theta, t to the left."""
    return (
        HALF * math.cos(theta) - t * math.sin(theta),
        HALF * math.sin(theta) + t * math.cos(theta),
    )


def lane_xml(lane_id, width, kind, marks, pred=None, succ=None):
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


roads = []
connections = []
next_id = 100

for arm_id, theta in arms:
    x, y = base(theta)
    left = (
        lane_xml(1, LANE, "driving", ["broken"])
        + lane_xml(2, LANE, "driving", ["solid"])
        + lane_xml(3, WALK, "sidewalk", [])
    )
    right = (
        lane_xml(-1, LANE, "driving", ["broken"])
        + lane_xml(-2, LANE, "driving", ["solid"])
        + lane_xml(-3, WALK, "sidewalk", [])
    )
    roads.append(
        f'<road name="arm_{arm_id}" length="{ARM}" id="{arm_id}" junction="-1">'
        f'<link><predecessor elementType="junction" elementId="1000"/></link>'
        f'<type s="0" type="town"><speed max="50" unit="km/h"/></type>'
        f'<planView><geometry s="0" x="{x:.6f}" y="{y:.6f}" hdg="{theta:.12f}" length="{ARM}">'
        f"<line/></geometry></planView>"
        f'<lanes><laneSection s="0"><left>{left}</left>'
        f'<center><lane id="0" type="none" level="false">'
        f'<roadMark sOffset="0" type="solid" color="standard" width="0.15"/></lane></center>'
        f"<right>{right}</right></laneSection></lanes></road>"
    )


def connecting(incoming, outgoing, start_t, end_t, links):
    """A connecting road from incoming arm (inward, at its lateral start_t) to outgoing arm
    (outward, at end_t), one arc or line; links: (incoming lane, connecting lane, outgoing lane)."""
    global next_id
    (in_id, in_theta), (out_id, out_theta) = incoming, outgoing
    x0, y0 = base(in_theta, start_t)
    x1, y1 = base(out_theta, end_t)
    h0 = in_theta + math.pi
    turn = (out_theta - h0 + math.pi) % (2 * math.pi) - math.pi
    chord = math.hypot(x1 - x0, y1 - y0)
    if abs(turn) < 1e-9:
        geometry, length = "<line/>", chord
    else:
        k = 2.0 * math.sin(turn / 2.0) / chord
        length = turn / k
        geometry = f'<arc curvature="{k:.12f}"/>'
        # The arc must end where the outgoing lane starts.
        ex = x0 + (math.sin(h0 + k * length) - math.sin(h0)) / k
        ey = y0 - (math.cos(h0 + k * length) - math.cos(h0)) / k
        assert math.hypot(ex - x1, ey - y1) < 1e-9, (ex, ey, x1, y1)
    road_id = next_id
    next_id += 1
    lanes = "".join(lane_xml(c, LANE, "driving", [], pred=i, succ=o) for i, c, o in links)
    roads.append(
        f'<road name="connect_{in_id}_{out_id}" length="{length:.9f}" id="{road_id}" junction="1000">'
        f'<link><predecessor elementType="road" elementId="{in_id}" contactPoint="start"/>'
        f'<successor elementType="road" elementId="{out_id}" contactPoint="start"/></link>'
        f'<type s="0" type="town"><speed max="30" unit="km/h"/></type>'
        f'<planView><geometry s="0" x="{x0:.9f}" y="{y0:.9f}" hdg="{h0:.12f}" length="{length:.9f}">'
        f"{geometry}</geometry></planView>"
        f'<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center>'
        f"<right>{lanes}</right></laneSection></lanes></road>"
    )
    links_xml = "".join(f'<laneLink from="{i}" to="{c}"/>' for i, c, o in links)
    connections.append(
        f'<connection id="{road_id}" incomingRoad="{in_id}" connectingRoad="{road_id}" '
        f'contactPoint="start">{links_xml}</connection>'
    )


for i, incoming in enumerate(arms):
    straight = arms[(i + 2) % 4]
    # The arms go anticlockwise; driving in from one, the next is on the right.
    right = arms[(i + 1) % 4]
    left = arms[(i + 3) % 4]
    connecting(incoming, straight, 0.0, 0.0, [(1, -1, -1), (2, -2, -2)])
    connecting(incoming, right, LANE, -LANE, [(2, -1, -2)])
    connecting(incoming, left, 0.0, 0.0, [(1, -1, -1)])

xml = (
    '<?xml version="1.0" standalone="yes"?>\n'
    "<!-- The city block's roads: written by city_block_xodr.py; do not edit by hand. -->\n"
    "<OpenDRIVE>\n"
    '<header revMajor="1" revMinor="6" name="city_block" version="1" north="0" south="0" '
    'east="0" west="0"/>\n'
    + "\n".join(roads)
    + f'\n<junction id="1000" name="crossing">{"".join(connections)}</junction>\n'
    "</OpenDRIVE>\n"
)
with open("city_block.xodr", "w") as out:
    out.write(xml)

#!/usr/bin/env python3

"""Subscribes to Gazebo sensor topics and counts what arrives (Gazebo renders a sensor only while
something subscribes to it). Prints one JSON line when sent SIGTERM or SIGINT.

    gz_count.py TOPIC=TYPE ...      TYPE: laserscan or image
"""

from __future__ import annotations

import json
import signal
import sys
import threading

from gz.msgs.image_pb2 import Image
from gz.msgs.laserscan_pb2 import LaserScan
from gz.transport import Node

TYPES = {"laserscan": LaserScan, "image": Image}


def main() -> int:
    counts: dict[str, int] = {}
    rays: dict[str, int] = {}
    stop = threading.Event()
    node = Node()
    for arg in sys.argv[1:]:
        topic, kind = arg.rsplit("=", 1)
        counts[topic] = 0
        rays[topic] = 0

        def callback(msg: object, topic: str = topic, kind: str = kind) -> None:
            counts[topic] += 1
            if kind == "laserscan":
                rays[topic] = len(msg.ranges)  # type: ignore[attr-defined]
            else:
                rays[topic] = msg.width * msg.height  # type: ignore[attr-defined]

        if not node.subscribe(TYPES[kind], topic, callback):
            print(f"cannot subscribe to {topic}", file=sys.stderr)
            return 1
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    print("ready", file=sys.stderr, flush=True)
    stop.wait()
    print(json.dumps({"messages": counts, "size_per_message": rays}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

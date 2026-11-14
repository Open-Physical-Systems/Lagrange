#!/usr/bin/env python3

"""Gazebo's X3 quadcopter hovering, timed in-process (gz.sim's Python TestFixture), for
docs/comparison.md. Its MulticopterVelocityControl flies only after a message on /X3/enable, so
this script loads the world, takes one step, enables the controller with a zero velocity
command, then times SECONDS of hovering in one server.run call.

    GZ_IP=127.0.0.1 GZ_PARTITION=... bench_gazebo_quad.py gz_quad.sdf SECONDS
"""

import time

T0 = time.perf_counter()

import json  # noqa: E402
import resource  # noqa: E402
import sys  # noqa: E402

import gz.math  # noqa: E402,F401  (registers Pose3d)
from gz.msgs.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs.odometry_pb2 import Odometry  # noqa: E402
from gz.msgs.twist_pb2 import Twist  # noqa: E402
from gz.sim import TestFixture  # noqa: E402
from gz.transport import Node  # noqa: E402


def cpu() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return r.ru_utime + r.ru_stime


def main() -> int:
    world, seconds = sys.argv[1], float(sys.argv[2])
    heights: list[float] = []  # from its odometry topic: no Python in the simulation loop

    fixture = TestFixture(world)
    fixture.finalize()
    server = fixture.server()
    server.run(True, 1, False)
    t_first = time.perf_counter()
    node = Node()
    node.subscribe(Odometry, "/model/x3/odometry", lambda msg: heights.append(msg.pose.position.z))
    enable = node.advertise("/X3/enable", Boolean)
    twist = node.advertise("/X3/gazebo/command/twist", Twist)
    on = Boolean()
    on.data = True
    time.sleep(1.0)  # discovery
    enable.publish(on)
    twist.publish(Twist())
    time.sleep(0.5)
    t_enabled = time.perf_counter()
    c0 = cpu()
    server.run(True, round(seconds / 0.001) - 1, False)
    t_end = time.perf_counter()
    time.sleep(0.5)  # the last odometry messages
    stepping = t_end - t_enabled
    print(
        json.dumps(
            {
                "sim": "gazebo",
                "startup_s": t_first - T0,
                "stepping_s": stepping,
                "rtf": (seconds - 0.001) / stepping,
                "cpu_cores": (cpu() - c0) / stepping,
                "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
                "height_at_end_m": heights[-1] if heights else None,
                "height_min_m": min(heights) if heights else None,
            }
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

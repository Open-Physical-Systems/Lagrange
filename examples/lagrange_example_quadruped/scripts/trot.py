#!/usr/bin/env python3
"""Stands the robot up and trots forward: feet on an ellipse, leg angles by inverse kinematics, sent
as positions; Lagrange's joint loops hold them at 1 kHz. An illustration, not a locomotion
controller: only its heading is fed back; no balance, and only the gait keeps it up."""

import contextlib
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEGS = ["front_left", "front_right", "rear_left", "rear_right"]
PHASE = {"front_left": 0.0, "rear_right": 0.0, "front_right": 0.5, "rear_left": 0.5}
THIGH = 0.2  # m, as in robot/quadruped.urdf
SHIN = 0.2
HEIGHT = 0.28  # m from hip to foot
STRIDE = 0.05  # m, half of it each way
LIFT = 0.05  # m
PERIOD = 0.4  # s
HEADING_GAIN = 3.0  # share of the stride per radian off
RISE_SECONDS = 1.5
STAND_SECONDS = 3.0
WALK_DISTANCE = 1.0  # m


def leg_angles(x: float, z: float) -> tuple[float, float]:
    """Thigh and knee for a foot at x forward, z below the hip. The knee bends backwards."""
    reach = min(math.hypot(x, z), THIGH + SHIN - 1e-3)
    knee = -math.acos((reach * reach - THIGH * THIGH - SHIN * SHIN) / (2.0 * THIGH * SHIN))
    # A positive angle about +y swings a hanging leg backwards.
    thigh = math.atan2(-x, z) - math.atan2(SHIN * math.sin(knee), THIGH + SHIN * math.cos(knee))
    return thigh, knee


class Trot(Node):
    def __init__(self) -> None:
        super().__init__("trot")
        self.declare_parameter("once", False)
        self.done = False
        self.command = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.position = None
        self.height = 0.0
        self.start = None
        self.first_stamp = None
        self.next_log = 0.0
        self.yaw = 0.0

    def on_odom(self, odom: Odometry) -> None:
        self.position = odom.pose.pose.position
        q = odom.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

    def targets(self, t: float) -> dict[str, float]:
        out = {}
        walking = t > STAND_SECONDS
        # From straight legs, as the file starts, down to the stance, slowly.
        down = min(t / RISE_SECONDS, 1.0)
        height = THIGH + SHIN - 0.002 + (HEIGHT - THIGH - SHIN + 0.002) * down
        for leg in LEGS:
            x, z = 0.0, height
            if walking:
                phase = ((t - STAND_SECONDS) / PERIOD + PHASE[leg]) % 1.0
                # Heading: longer strides on the side it has turned towards bring it back.
                towards = 1.0 if leg.endswith("left") else -1.0
                stride = STRIDE * (1.0 + towards * max(-0.8, min(0.8, HEADING_GAIN * self.yaw)))
                if phase < 0.5:  # on the ground: the foot goes back under the body
                    x = stride * (1.0 - 4.0 * phase)
                else:  # in the air: forward and up
                    swing = (phase - 0.5) * 2.0
                    x = stride * (2.0 * swing - 1.0)
                    z = HEIGHT - LIFT * math.sin(math.pi * swing)
            thigh, knee = leg_angles(x, z)
            out[f"{leg}_hip_joint"] = 0.0
            out[f"{leg}_thigh_joint"] = thigh
            out[f"{leg}_knee_joint"] = knee
        return out

    def on_states(self, states: JointState) -> None:
        now = states.header.stamp.sec + states.header.stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = now
        t = now - self.first_stamp
        targets = self.targets(t)
        command = JointState()
        command.header.stamp = states.header.stamp
        known = set(states.name)
        for joint, target in targets.items():
            if joint in known:
                command.name.append(joint)
                command.position.append(target)
        self.command.publish(command)
        if self.position is not None and t >= self.next_log:
            self.next_log = t + 0.25
            self.get_logger().info(
                f"t {t:.2f} s: x {self.position.x:.2f} y {self.position.y:.2f} "
                f"z {self.position.z:.2f}"
            )
        if self.position is None or self.done or t < STAND_SECONDS:
            return
        if self.start is None:
            self.start = self.position.x
            self.get_logger().info(f"standing at {self.position.z:.2f} m, walking")
        gone = self.position.x - self.start
        if gone >= WALK_DISTANCE:
            self.finish(
                f"trotted {gone:.2f} m forward in {t - STAND_SECONDS:.1f} s, "
                f"{self.position.y:+.2f} m sideways, body at {self.position.z:.2f} m"
            )
        elif self.position.z < 0.15:
            self.get_logger().error(f"fell over: body at {self.position.z:.2f} m after {t:.1f} s")
            raise SystemExit(1)

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = Trot()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

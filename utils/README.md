# Utilities

Helper packages to use beside Lagrange: tools you run next to the simulator, not parts of it.
Each folder is a ROS 2 package, built with the rest of the workspace.

| Package | What it is |
|---|---|
| [`lagrange_tx16s`](lagrange_tx16s/README.md) | A TX16S-style radio on screen: fly a robot over `/cmd_vel` from a real RadioMaster TX16S (USB) or the keyboard, with its telemetry on the radio's screen |

They talk to the simulator the way your own code does, over ROS 2 topics, and draw (when they
draw) with the viewer's design: SDL3, Dear ImGui and its theme, no web technology.

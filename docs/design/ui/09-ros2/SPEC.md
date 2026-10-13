# 09 · ROS 2 workspace: SPEC

## Purpose
Show how the sim connects to the user's robot stack: topics and rates, the node graph, the
ros2_control hardware interface, simulated time, and ROS logs.

## Dock layout
```
left 440 px  : "Topics" | "Services" | "Actions" | "Params"
center flex  : top "Node graph" | "TF tree" ; bottom 300 px "Console" | "rosout"
right 340 px : "ros2_control" | "Time"
domain strip: label "ROS 2" with chips: distro + domain id, RMW, /clock, ros2_control, Nav2, Open-RMF
```

## Data source
- The viewer gets ROS graph info **through the sim server** (the bridge reports its graph and
  stats in `Stats` and a `RosGraph` message). The viewer does not need to join the ROS domain
  itself. This keeps remote viewing working.
- TODO(decide in RFC 0003): optionally let the viewer join the ROS domain directly for echo.

## Panels
### Topics
- Search and a "sim | all" segmented control (topics from the bridge versus all topics in the domain).
- A table of topic (mono), short type, Hz, bandwidth. Status color for a problem rate (for
  example `/gnss/fix` no fix).
- Selected topic → "latest message" section showing a pretty-printed message in mono with
  comments for sim-specific notes (for example NaN count caused by fog dropouts), then Echo, Plot
  field, Record topic.

### Node graph
- Nodes: the `lagrange_ros2_bridge` (accent, selected), controller_manager (its controllers
  listed), robot_state_publisher, a Nav2 group (dashed box: controller_server, bt_navigator,
  planner_server), external nodes (rviz2, dimmed).
- Edges are labeled by topic. The ros2_control link from bridge to controller_manager is drawn
  dashed accent as "SystemInterface · shm".
- Modes: nodes + topics, nodes only. The TF tree tab shows frames with rates and a staleness warning.

### Console / rosout
The same log component as Scene, with an extra Debug level; the source shows the node name in brackets.

### ros2_control
- Hardware: `LagrangeSystem`, its plugin name and read/write rate, an active pill, and the note
  "Effort commands go through the electrical domain".
- Joint table: joint · interface, command, state, effort (units noted under the table).
- Controllers: name, rate, state pill.
- Simulated time: use_sim_time, /clock, wall clock, RTF and target, overload policy combo ("slow
  down, never skip").
- Bridge: namespace, frame prefix, sensor QoS.
- Footer: "Copy launch snippet" (a ROS 2 launch fragment for the user's stack), "Restart controllers".

## Mock server must provide
A static RosGraph for the sample (nodes, topics, rates), a ros2_control state, and log lines.

## Acceptance
Topic rates match the sensor list in the Sensors workspace (same source of truth).

# The viewer's workspaces

`lagrange_viewer` (or `lagrange viewer`, or the viewer that `lagrange` starts beside the
simulator) groups its panels into workspaces, one tab each along the top. This page lists what
each workspace offers beyond the 3D view, with the keys and flags that reach it. The viewer
talks to the simulator only through its protocol: everything below comes from the server it is
connected to, or from the viewer itself where a section says so. The screens' design is
`docs/design/ui/` (one folder per workspace).

| Workspace | Key | `--workspace` |
|---|---|---|
| Scene | Ctrl+1 | `scene` |
| Domains | Ctrl+2 | `domains` |
| Electrical | Ctrl+3 | `electrical` |
| Signals | Ctrl+4 | `signals` |
| Sensors | Ctrl+5 | `sensors` |
| Environment | Ctrl+6 | `environment` |
| Replay | Ctrl+7 | `replay` |
| ROS 2 | Ctrl+8 | `ros2` |
| World | Ctrl+9 | `world` |
| Performance | Ctrl+0 | `performance` |
| Builder | Ctrl+B | `builder` ([The robot builder](builder.md)) |

## The command palette

**Ctrl K** (or the menu's *Command palette*, or the `Ctrl K` hint at the right of the menu bar)
opens one search over the viewer's commands, the scene's prims, the signals and the ROS 2
topics, best match first. Type a few letters of any of them in any order (the match is fuzzy);
**Enter** runs the chosen line: a command runs, a prim is selected and shown where it is
inspected (the Scene workspace), a signal is added to the scope in Signals, a topic opens the
ROS 2 workspace. **Esc** or a click outside closes it.

## Domains

Beside the domain table, the schedule, the profile and the wiring, each group has secondary
tabs:

- **Modules** (beside Domains): every module the server runs, as it measures them: its step,
  rate, CPU per step, and what one millisecond of simulation costs. Substitutes cost nothing and
  say so.
- **Coupling** (beside Schedule): every writer → reader pair of the wiring, what passes between
  them and at which rates, whether the pair is solved together or read through a hold (the
  100 µs electrical reads the 100 ms thermal 1000 times per value).
- **Diff** (beside Profile): the line diff of the applied profile and the one being edited.
- **Schema** (beside Profile): the keys of `profiles/schema.json`, with their types and
  descriptions, by domain.

The **Profile** panel's YAML is the profile being edited: typing in it changes the domain rows,
and switching a row changes the text; the server validates the draft as it does any edit, and
Apply rebuilds the run with it. Every domain the run has is in the table with its module and
switch, its cost is the server's measured CPU per simulated second, and a domain switched off
shows its substitute in the preview. The substitute picker offers the selected domain's
substitutes (the first domain's before any click). Headless, `--set-domain <d>=on|off`
(repeatable) and `--apply-profile` do the same as the switches and Apply.

## Sensors

Besides lidar, IMU and camera views, the **Radar** view draws a radar's detections as a fan and
over range and azimuth. In the 3D view, Overlays → **Radar detections** (`--overlays radar`)
draws each detection as a box sized by its radar cross-section (−20 to +20 dBsm), coloured by
radial velocity (blue closing, orange moving away) with a Doppler tick along the line of sight,
and the radar's field of view as a faint wedge. It works for a `RadarAPI_1` radar and an
`imaging_radar`; the sedan's `front_radar` is the sample ([Tutorial: a car in a city
block](tutorial-cars.md)).

## Environment and World

A weather preset sends its whole weather at once and the run is rebuilt once for it. Wind with
the aerodynamics domain off is said to push nothing. The **World** workspace lists everything
that moves on its own: other robots (spawned or `--add-robot`), traffic vehicles and pedestrians,
scenario entities and people, with their speeds.

## Replay

With a recording open (`lagrange_server --replay run.mcap`, or File → Open recording), the
Replay workspace compares it with a second Lagrange recording: a real robot's log converted into
one, or another simulated run (another engine, profile or parameter).

```bash
lagrange_viewer --workspace replay --compare other.mcap [--suite suite.yaml]
```

- **Compare**: pick the second recording and press Compare.
- **Sim vs real**: each signal of both runs plotted together, aligned by the lag (up to ±0.5 s)
  that matches them best, resampled to 100 Hz by default, with its error.
- **Validation**: a pass or fail per signal, the normalised RMS error over the second run's
  range against a limit, by driving phase (stopped, accelerate, cruise, brake). **Export report**
  writes JSON to `<config>/validation/`.
- **History**, and the **Source strip** (what wrote the recording).

Without a suite every scalar signal both recordings share is compared with a 5 % limit. A suite
file names signals, components, limits and the method:

```yaml
suite: amr_drive
method: {alignment: cross-correlation, resample_hz: 100, max_lag_s: 0.5, window: [0, 10]}
signals:
  - {sim: motor/left_drive_motor/current_q, real: motor/left_drive_motor/current_q, limit: 0.05}
  - {sim: body/base_link/twist, component: 0, limit: 0.05}
```

The limit is a fraction of the real signal's range (NRMSE). When the real signal is constant in
the window the NRMSE says nothing, so the row reads *const ref* and is judged by its RMSE in the
signal's unit: against `abs_limit` when the entry gives one, else against the limit times the
larger of the constant's size and the range of the sim signal.

Both files are read on the viewer's machine, so a replay served from another machine cannot be
compared yet, and **Re-simulate from here…** is shown but disabled (both need protocol additions;
`docs/design/viewer.md`, "Replay comparison").

## ROS 2

The viewer does not join the ROS domain; the workspace shows the graph the server reports.

- **Topics** and **Node graph**, as before; hidden topics (an action's) stay out, as
  `ros2 topic list` leaves them out.
- **Actions**: the graph's actions, found from their hidden `<name>/_action/...` topics, with the
  nodes that serve and use them; one nobody serves any more is marked.
- **Services** and **Params**: the server's graph report does not carry them yet, so each tab
  says so and gives the command to run from a terminal in the same ROS domain.
- **TF tree**: odom over the robot's base, the bodies by the scene's hierarchy, sensors on their
  bodies and cameras' optical frames, each with who publishes it (`/tf` or `/tf_static`) and how
  it moves; frames that nobody publishes (no `robot_state_publisher`) or that went stale are
  warned about. A paused run has no stale frames.

## Performance

Six cards at the top: **real-time factor** (against the target), **CPU per 1 ms of sim**,
**viewer frame** (ms and fps), **protocol** (deltas per second, kB/s, dropped), **determinism**
and **GPU per sensor frame** (the modules the server runs on the GPU). The server sends no
determinism check, so that card says *not reported* rather than guess. New tabs (the
simulation's own threads and GPU passes are not in what the server reports, so these measure the
viewer):

- **Threads**: the viewer's threads from `/proc`, once a second, and its UI thread's frame by
  phase (panels, recording, waiting for the GPU, presenting).
- **GPU passes**: the viewer's own passes, timed with Vulkan timestamp queries (when the device
  supports them).
- **Memory**: the server's resident and GPU memory as it reports them, the viewer's resident
  memory with its peak, and the video memory the viewer holds against its budget
  (`VK_EXT_memory_budget`).

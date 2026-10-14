# 01 · Launcher: SPEC

## Purpose
This is the first window when the viewer is not connected. It gets a user from "just installed"
to "seeing their robot" in under a minute: check the system, pick a server, then open a stage,
import a robot, start from a template, or open a recording.

## Layout (no workspace bar, no domain strip)
Menu bar (File, View, Tools, Help), then three fixed columns (not dockable), then the status bar.
- **Left 380 px, About:** mark and "lagrange" wordmark, version and license, a one-line
  description, the **System check** list, and links at the bottom.
- **Center, flexible, Start:**
  1. three action tiles: Open stage (Ctrl O), Import robot (Ctrl I), Open recording (Ctrl R → replay)
  2. "Start from a robot class": an 8-card grid, 4 columns
  3. "Recent" table
- **Right 400 px, Simulation server:** a radio list of servers, the connection detail box, the
  primary "Connect and open" button, and a drop zone.

## Panels
### System check
Four rows, each with a ✓ / ! / ✗ icon and two text lines:
1. GPU name and VRAM, Vulkan ray-query support, then the **resulting tier** (Tier 0/1/2)
2. OS and session type (Wayland/X11), window backend, HiDPI scale
3. ROS 2 sourced (`ROS_DISTRO`), RMW, `ROS_DOMAIN_ID`
4. Optional CUDA fast path

Data comes from local probing (Vulkan instance or device query, environment variables), not the server.

### Robot-class cards
AMR, Robot arm, Drone, Car, Autonomous car, Huge world, Electrical bench, Empty stage. Each
card has line art, a name, a one-line description, the domain dots it enables, and its key
integrations. Clicking selects it (accent border). Double-clicking or Connect opens the matching
sample stage and profile.

### Recent
Columns: stage, robot, profile (mono), server, opened (relative time). Double-click reopens it.
Stored in `~/.config/lagrange/recent.json`.

### Server list
Rows: radio, name, mono address and transport, and a status pill (ready / older protocol / v0.2+).
- "Start local simulator" spawns `lagrange_server` (v0.2+; disabled in v0.1 with that pill).
- Discovery: localhost mock server, entries from saved servers, and optionally Zenoh scouting.
  TODO(verify): Zenoh scouting API.
- The detail box shows the protocol version (and whether it is compatible), the transport
  (shm fast path on/off), the server tier, and which stage and profile it will open.

## Interactions
- Dropping a URDF, xacro, SDF, MJCF, USD or MCAP file anywhere sets the open target. MCAP starts
  replay mode.
- An incompatible protocol shows a warning pill and disables Connect with an explanation.

## States
- No GPU with Vulkan: the system check shows ✗, the tier falls back to Tier 0 (CPU), and the
  viewer still runs with a software or llvmpipe renderer warning.
- ROS not sourced: ! with the hint "source /opt/ros/lyrical/setup.bash".

## Mock server must provide (v0.1)
`Hello/ServerInfo` with version, tier, transports, and sample stages for each robot class (even
if they are static).

## Acceptance
- From a cold start to the Scene workspace showing the sample AMR takes under 5 s on a warm disk
  (viewer budget §13.4).
- The keyboard alone can pick a card and connect (Tab, arrow keys, Enter).

# 06 · Sensors workspace: SPEC

## Purpose
Show what each sensor actually outputs, and **why** it is degraded: weather, EMI, materials, and
the timing and noise models. This is where the realism is visible.

## Dock layout
```
left 260 px  : "Sensors" | "Rigs"
center flex  : 2×2 grid of sensor view panels (each is a dockable panel)
right 320 px : "Degradation" | "Model" | "Timing"
domain strip right side: segmented "Measured | Ground truth | Diff"
```

## Panels
### Sensor list
- Rows: status dot, name, a status pill (ok / degraded / fog / EMI 6.2° / no fix), and a mono line
  with the configuration and actual rate and latency.
- "Publishing" section: the ROS 2 topics these sensors publish.

### Sensor views (one panel type per sensor kind)
- **Lidar bird's-eye view**: a point cloud colored by intensity (two tones: strong and
  attenuated), with points flagged as rain backscatter drawn in warn color. Range rings, a robot
  marker, an annotated obstacle, and an overlay "fog limit 41.2 m · beyond view ↑". The legend
  sits bottom left and the point count and deskew status bottom right. Modes: intensity, range, ring.
- **Camera**: the image (rgb, depth or labels). Footer mono line: resolution, rolling shutter,
  exposure and gain, contrast loss caused by fog, lens drops.
- **IMU**: accel x and gyro z scopes plus a bias and noise table. A tab switches to wheel encoders.
- **Magnetometer · EMI**: a compass with true heading (white) versus measured (violet), a
  deviation arc and pill, and a table of |B| measured versus site reference, the main source with
  distance, self-interference and steel distortion. A gnss tab shows fix status and a sky plot.
- Images and point clouds arrive as `SensorFrame` over the shm fast path and are uploaded
  straight to GPU textures and buffers.

### Degradation (selected sensor)
- Effective max range: a bar from the current limit to the clear-weather limit.
- Contributions table: each source (fog, rain, wet floor, dust, EMI) with its parameters and the
  effect it causes.
- Noise and timing: range σ, angular σ, dropouts, latency ± jitter, motion distortion, clock offset.
- Toggles: show removed returns as ghosts, overlay ground truth. Actions: "Compare with real
  scan…" (Replay workspace), "Export params".

## States
- Sensors domain off (substitute ground truth): views show ground truth with a blue
  "ground truth" pill, and Degradation reads "not simulated".
- Tier 1 (no ray tracing): lidar is computed with compute-shader BVH; show a "Tier 1 · compute" pill.

## Mock server must provide
A synthetic point cloud (warehouse aisle) and a camera image at their rates, IMU and
magnetometer signals, degradation breakdown values, and GNSS no-fix.

## Acceptance
- The lidar view renders 30k points per frame at under 2 ms GPU.
- Switching Measured/Ground truth/Diff happens without reconnecting.

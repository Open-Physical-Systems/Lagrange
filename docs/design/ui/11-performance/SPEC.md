# 11 · Performance: SPEC

## Purpose
Explain why the simulation is or isn't real time: per-module cost, the multirate schedule, GPU
passes, the viewer frame, the protocol, determinism, and actionable suggestions.

## Layout
- KPI row (6 cards, not dockable): real-time factor, CPU per 1 ms of sim (mean / budget, p99,
  meter), GPU per sensor frame (breakdown), viewer frame (fps against the 16.6 ms budget),
  protocol (deltas per second, shm MB/s, dropped), determinism (bit-exact with the last check time).
- Dock below:
```
center flex  : top 320 px "Scheduler · last 10 ms" | "Threads" ; bottom "Modules" | "GPU passes" | "Memory"
right 380 px : top 200 px "RTF history" ; bottom "Resources"
```
- Domain strip chips show **cost** instead of config (for example "Rigid body 0.31 ms"). Right
  side: Live / Capture 10 s and Export trace.

## Panels
### Scheduler (flame or timeline chart, custom draw)
- 10 ms window, sync barriers every 1 ms (dotted).
- Rows: electrical (100 µs ticks), rigid_body (blocks), emi·comms, ros2 bridge, recorder, GPU ·
  sensors (lidar RT and camera blocks with labels), GPU · weather, viewer client (one long bar,
  "separate process · not in the sim budget").
- Slow steps are highlighted in warn with an annotation (for example "step 7: 0.42 ms (contact burst)").

### Modules
A table of module (domain dot), backend, dt, steps/s, mean, p99 (warn when over the threshold),
budget share. The shares add up to the mean CPU per 1 ms shown in the KPI.

### RTF history
The last 120 s with a dashed target line at 1.00 and dips annotated with the cause (from events).

### Resources and suggestions
- Meters: sim CPU threads, GPU busy, VRAM, sim memory, recorder buffer.
- Suggestions: a pill and one sentence each, generated from rules (for example when p99 is over
  the budget during contacts, suggest solver iterations or another backend).
- Actions: "Open in Tracy" (if the server is built with Tracy; TODO(verify) Tracy license and
  integration), Save capture.

## Mock server must provide
`Stats` at 10 Hz with per-module mean and p99, schedule trace samples, GPU pass timings, and RTF history.

## Acceptance
- The numbers on this screen are consistent with each other: the module shares sum to the CPU
  KPI, and the GPU breakdown sums to the GPU KPI.
- Profiling overhead is shown and stays under 1 %.

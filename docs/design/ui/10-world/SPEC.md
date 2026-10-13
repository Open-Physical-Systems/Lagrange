# 10 · World (huge worlds): SPEC

## Purpose
Operate large, georeferenced worlds with fleets: see which tiles are loaded at which level of
detail, where the active physics regions are, the floating-origin state, streaming budgets, and
georeference sources.

## Dock layout
```
left 300 px  : "Agents" | "Regions"
center flex  : "Map" | "3D"
right 320 px : "Streaming" | "Geodesy"
domain strip right: pill "traffic → SUMO co-sim · N distant agents"
```

## Panels
### Agents
- Rows: color dot (accent = focus robot, blue = vehicle, cyan = drone), name, status pill
  (focus, active, Wi-Fi weak…), and a mono line with region, speed, fidelity or integration
  (Autoware, PX4 SITL).
- Distant agents section: SUMO vehicles (kinematic), pedestrians (social force), and the
  promotion distance to full physics.

### Map (top-down, custom draw, pans and zooms)
- Base: terrain contours and roads (from OSM/OpenDRIVE).
- Tile grid (256 m tiles) colored by state: LOD0 physics + sensors (accent), LOD1 render +
  collision, LOD2 render only, LOD3 proxy, not loaded (empty).
- Active-region circles around each agent (150 m; vehicles larger). Agents are drawn as dots
  with labels. The AV's planned path is dashed blue.
- The floating-origin crosshair sits on the anchored robot.
- Region partition lines (R1…R4) are dashed. They are ready for distributed handoff later.
- Overlay: a legend with counts per tile state, a scale bar, and the cursor position in ENU with its tile index.
- Modes: tiles · LOD, terrain, roads, coverage (Wi-Fi).
- Double-clicking an agent focuses it and moves the floating-origin anchor (with confirmation).

### Streaming / Geodesy
- Meters: GPU memory (warn above 75 %), host cache, disk read, load queue.
- Floating origin: anchored to (combo), origin ENU, re-center threshold, last re-center (sim
  time), and the note "float64 world · f32 local render".
- Georeference: datum (WGS84 → local ENU), origin lat/lon (from stage metadata), terrain
  GeoTIFF, OSM, 3D Tiles tileset, OpenDRIVE file. All are links.
- Regions: "N regions · boundaries ready for multi-server handoff" and "distributed simulation is post-1.0".

## Mock server must provide
A synthetic tile grid with LOD states, agents with regions, and streaming stats.

## Acceptance
- The map draws 10k tiles at 60 fps (instanced quads).
- No float32 jitter when zoomed in 5 km from the origin (positions stay double until the render offset).

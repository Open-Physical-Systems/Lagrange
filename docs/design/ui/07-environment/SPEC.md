# 07 · Environment & fields: SPEC

## Purpose
Edit the environment layer (time and sun, weather, wind, EMI sources, radiation) and see the
invisible physics as overlays: EMI |B| and |E|, wind vectors, fog and dust density, surface
temperature, dose rate and Wi-Fi coverage. Probe any point for values.

## Dock layout
```
left 320 px  : "Environment" | "Presets"
center flex  : "Viewport · fields" | "Slice 2D"
right 300 px : "Overlays" | "Field info"
```

## Panels
### Environment (edits the USD environment layer live)
- Header: the environment layer path and "Swap…" to pick another preset layer.
- Sections, each with a domain dot:
  - **Time and sun**: local time, sun elevation (derived), ambient.
  - **Weather**: rain rate, drop-size model, fog visibility, dust, surface wetness meter.
  - **Wind**: mean speed and direction, gust model with intensity, CFD field (or "none — indoor stage").
  - **EMI sources**: a list of source name, frequency and current or power, and a mono detail
    line (baked VDB, robot self-emission, path loss).
  - **Radiation**: collapsed while the domain is off.
- Live edits send `Command{set_attribute}` on the environment layer; the header pill says
  "environment layer edited · live".

### Viewport · fields
- The same renderer as Scene, with field overlays:
  - **Slice** (XY/XZ/YZ/volume) at a chosen height, colored with viridis, opacity from the Overlays panel.
  - Wind arrows (length proportional to speed).
  - The source drawn in the scene (a power line as a violet line with a label).
  - Weather particles optional.
- A probe crosshair with a tooltip of every field at that point (|B| with frequency, wind, σ_fog, air temperature).
- A legend overlay at bottom right: field name, unit, colormap bar and range, wind legend.
- Field data arrives as `FieldSlice`; volumes come as NanoVDB bricks uploaded to the GPU.

### Overlays
- A checklist of overlay layers, each with a colormap swatch and unit. Only one scalar slice at a
  time; vectors can be combined.
- Selected layer: source VDB, grid size, bounds, GPU memory, range min/max, opacity, colormap.
- "Coupled into": the downstream effects (magnetometer heading, CAN error frames, Wi-Fi SNR and loss).

## States
- A domain is off: its overlays are disabled with "domain off".
- A large VDB is loading: show a progress meter in the Field info tab.

## Mock server must provide
A 2D |B| slice and wind samples near the sample power line, a probe query response, and coupling numbers.

## Acceptance
Changing rain rate updates the Sensors workspace degradation within one sensor frame.

# Lagrange UI · 12 · Builder workspace (draft)

Design for the robot builder of `rfcs/0016-robot-builder.md` (Draft, for the owner). Unlike
00–11 this screen was not drawn by the owner: it was made from the design system's own
stylesheet (the `<style>` block of `02-scene/reference.html`, copied unchanged, plus a short
block of builder additions marked at its end) and waits for the owner's review. Native C++20 ·
Vulkan · Dear ImGui docking; the HTML is a picture of the target, never code to ship.

## Contents
| File | What it is |
|---|---|
| `SPEC.md` | Purpose, modes, dock layout, panels, states, cost in Build mode, new widgets proposed, acceptance |
| `reference.png` | Target rendering at 2× (3200×2000 for a 1600×1000 window) |
| `reference.html` + `fonts/` | The same screen as an offline page |
| `../shared/` | tokens, theme header, conventions (unchanged) |

## How `reference.png` was made
```
google-chrome --headless=new --disable-gpu --hide-scrollbars --window-size=1600,1000 \
  --force-device-scale-factor=2 --screenshot=$PWD/reference.png file://$PWD/reference.html
```
then reduced to 256 colours to stay under the repository's 1 MB limit (Pillow,
`quantize(colors=256, method=MEDIANCUT, dither=NONE)`). Chrome on the developer's machine is a
documentation tool only; nothing of it is in the product.
The viewport drawing is an isometric projection of the sample document, computed by a small
throwaway script; its numbers are the document's.

## The sample document shown (illustrative)
| Part | Shape | Mass | Joint to parent |
|---|---|---|---|
| `base_link` | box 0.50 × 0.36 × 0.12 m | 8.00 kg (override) | root |
| `left_wheel`, `right_wheel` | cylinder r 0.08 m, 0.04 m wide, 1100 kg/m³ | 0.885 kg each | continuous, y |
| `caster` | sphere r 0.035 m, 1100 kg/m³ | 0.198 kg | fixed |
| `mast` | cylinder r 0.02 m, 0.20 m, 2700 kg/m³ | 0.679 kg | fixed |
| `pan_link` | box 0.06 × 0.06 × 0.04 m, 1240 kg/m³ | 0.179 kg | revolute z, −170…170° |
| `tilt_link` | box 0.04 × 0.08 × 0.04 m, 1240 kg/m³ | 0.159 kg | revolute y, −45…60° |
| sensors | `lidar` (lidar) on `mast`, `camera` (camera) on `tilt_link`, `front_range` (kind `rangefinder`) and `imu` on `base_link` | | |

Total 10.99 kg, COM (0.007, 0.000, 0.003) m in `base_link`'s frame (its origin at the box's
centre, 0.13 m above the ground). The joint limits and densities are picked for the picture, not
recommended values.

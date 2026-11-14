# RFC 0015: USD attributes for lidar scan patterns and fisheye lenses

| | |
|---|---|
| Status | Accepted (2026-11-14) by the owner, with the proposed answer to each open question; gated merges of its steps pre-approved |
| Created | 2026-11-12 |
| Affects | `schemas/sensor/` (additions to `LidarAPI_1` and `CameraSensorAPI_1`), `io/stage/` (reader and writer), `io/importers/` (SDFormat), `server/src/Robot.cpp` (URDF, no change in meaning), `schemas/CHANGELOG.md`, `docs/design/formats.md` §3.8 and §9, `docs/design/domains/sensors.md` |
| Builds on | RFC 0002 (schemas), RFC 0007 (the 1.x freeze) |

## Summary

Three lidar scan patterns (a channel table, a flash array, a Livox-style rosette) and fisheye
cameras up to 360° (Kannala–Brandt, with a cutoff angle) were added in PR #330, read from URDF
only. USD cannot say them, so `lagrange_import` silently writes a different sensor, and for a
fisheye over 180° a camera the loader then refuses; SDFormat has a place for some of it and
Lagrange reads none of it. This RFC proposes, all as additions under RFC 0007:

- `LidarAPI_1`: the tokens `flash` and `rosette` appended to `sensor:lidar:scanPattern`;
  `sensor:lidar:channelElevations` (a channel table); five `sensor:lidar:rosette:*` attributes.
- `CameraSensorAPI_1`: `sensor:camera:lensType` (`gnomonical`, `equidistant`: SDFormat's names)
  and `sensor:camera:cutoffAngle`; the Kannala–Brandt k1..k4 are the existing
  `distortionModel = "equidistant"` and `distortionCoeffs`; the lens's field of view comes from
  `UsdGeomCamera`'s focal length and aperture through the lens's own mapping.
- SDFormat: `<lens><type>` and `<cutoff_angle>` read as SDFormat defines them; what SDFormat
  has no element for is read from `lagrange:`-prefixed custom elements.
- The writer writes all of it, so URDF → USD → run gives the same beams and pixels, bit for bit.

Every existing file keeps its meaning; nothing to migrate.

## Motivation

`sensors.md` "Lidar scan patterns (as built)" and "Fisheye cameras (as built)" both end with the
same debt: SDFormat and USD "cannot carry them yet: owed, with a schema addition". USD is the
authoring format (CLAUDE.md §4) and the importers' target, so today:

Checked on `origin/dev` 71880eb with the rig of `server/test/test_optics_import.cpp`
(`lagrange_import rig.urdf rig.usda`, then `lagrange_server --robot rig.usda`):

| URDF sensor | What `lagrange_import` writes | Effect |
|---|---|---|
| rosette, 500 points, 0.8 rad cone | a 2D scanner: 500 columns over ±π, one ring | a different sensor, no note |
| flash 16 × 4 over ±0.5 × ±0.2 rad | a 16 × 4 grid at even angles (not even tangents) | beams move up to the difference between angle and tangent spacing |
| channel table of 3 elevations | `channels = 2` (the URDF's `<samples>`), `verticalFov = (0, 0)` | the table and its ring count are lost |
| fisheye, 200° across, cutoff 1.8, k1, k4 | `focalLength = -1.85` (from tan(fov/2)), `distortionModel = "equidistant"` | the loader refuses: "the camera's focalLength and horizontalAperture must be positive" |

Below 180° the fisheye is written with a positive focal length from tan(fov/2), which the reader
turns back into a pinhole of that angle with equidistant distortion: a different image again.

## How it is read today

| Path | Where | What |
|---|---|---|
| URDF lidar | `server/src/Robot.cpp:630` `lidarPattern` | `<ray><pattern>` grid/flash/rosette, `<scan><vertical><angles>`, `<rosette>` (fov, point_rate, speed_1, speed_2, prism_ratio) → settings `pattern`, `vertical_angle_<i>`, `rosette_*` |
| URDF camera | `server/src/Robot.cpp:682` `cameraLens` | `<lens><type>` equidistant/kannala_brandt/fisheye, `<cutoff_angle>`, `<k1>`..`<k4>` → `projection` 1, `distortion_model` 2, `fisheye_cutoff`, `distortion_k1..4` |
| Settings → geometry | `core/world/include/lagrange/world/LidarLayout.hpp` (`LidarLayout::from`), `FisheyeLens.hpp` (`FisheyeLens::from`) | the only consumers; every path ends in these settings |
| USD lidar | `io/stage/src/StageReader.cpp:902` | `channels`, `horizontalSamples`, `horizontalFov`, `verticalFov`, ranges, noise, `motionDistortion`; `scanPattern` is not read |
| USD camera | `io/stage/src/StageReader.cpp:806` | `horizontal_fov = 2 atan(aperture / 2 / focal)` (a pinhole's); `distortionModel` + `distortionCoeffs` → `distortion_model`, `distortion_k*`; no `projection` |
| USD writer | `io/stage/src/StageWriter.cpp:247` (lidar), `:266` (camera) | writes the grid only; the camera's focal length from tan(fov/2) |
| SDFormat | `io/importers/src/SdfImporter.cpp:401` `readSensor` | lidar `<scan>`, `<range>`, noise; camera fov, image, clip. `<lens>` is ignored, so an SDFormat fisheye is a pinhole |

## Design

### 1. `LidarAPI_1` additions

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `sensor:lidar:scanPattern` | uniform token | — | `rotating` (unchanged) | appended: `flash` (a focal-plane array, every beam at once, beams evenly spaced on the image plane), `rosette` (two Risley prisms). `rotating`, `solid_state` and `custom` keep today's meaning: the even grid |
| `sensor:lidar:channelElevations` | double[] | rad | `[]` | one elevation per channel, any order; non-empty: the rings are these sorted from the lowest up, and `verticalFov` is not used. `channels`, when authored, must equal its length (else a load error) |
| `sensor:lidar:rosette:fov` | double | rad | none | full cone angle; δ₁ + δ₂ = fov / 2 |
| `sensor:lidar:rosette:pointRate` | double | 1/s | none | points fired per second |
| `sensor:lidar:rosette:speed1`, `sensor:lidar:rosette:speed2` | double | rad/s | none | the prisms' speeds; opposite signs counter-rotate |
| `sensor:lidar:rosette:prismRatio` | double | 1 | none | δ₂ / δ₁ |

- With `scanPattern = "rosette"`, `horizontalSamples` (already required) is the points per scan
  and `channels` must be 1 or unauthored; the `rosette:*` attributes are read only then.
- With `scanPattern = "flash"`, `motionDistortion` is not used (every beam fires at once), as in
  URDF; outer angles beyond ±π/2 are refused at load, as in URDF.
- The `rosette:*` attributes have **no fallback** on purpose. Their values when unauthored are
  the loader's, as documented in `sensors.md` (today the Livox Mid-40's 38.4° and 100 000 points/s
  from its data sheet, `TODO(verify)` its revision, and illustrative prism speeds, `TODO(verify)`
  against a recorded pattern), and the loader notes that they were assumed. A schema fallback
  would freeze those numbers for the 1.x line (RFC 0007: a fallback never changes), including
  the speeds nobody has measured. `exposureTime` already follows this convention ("No
  fallback: …").
- Why not make `solid_state` mean flash: files written since v1.0 with `solid_state` run as a
  grid today; giving it a new meaning changes them, which RFC 0007 allows only in `LidarAPI_2`.
  `solid_state` stays a label (MEMS scanners are solid state and not flash arrays either).

### 2. `CameraSensorAPI_1` additions

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `sensor:camera:lensType` | uniform token | — | `gnomonical` | the lens's mapping, SDFormat's `<lens><type>` names: `gnomonical` (a pinhole, today's behaviour), `equidistant` (the fisheye: r = f θ_d). More of SDFormat's (`stereographic`, `equisolid_angle`, `orthographic`) can be appended when they are drawn |
| `sensor:camera:cutoffAngle` | double | rad | none | half-angle beyond which an `equidistant` lens sees nothing; unauthored: the larger of half the field of view and π/2, as URDF and Gazebo; at most π |

With `lensType = "equidistant"`:

- **Distortion**: `distortionModel` `"equidistant"` with its four `distortionCoeffs` gives
  k1..k4 (Kannala–Brandt as OpenCV's fisheye module and ROS's `equidistant` camera_info write
  it; already the attribute's documented meaning), or `"none"` for all k = 0. `"plumb_bob"` is a
  load error.
- **Field of view from the lens, not a pinhole**: `UsdGeomCamera`'s `focalLength` f and
  `horizontalAperture` w are physical quantities, and a fisheye maps a ray θ off the axis to the
  image height f·θ_d(θ). So the angle across the width is
  `hfov = 2 θ_d⁻¹(w / (2 f))`, the inverse of the same mapping `FisheyeLens` uses
  (`FisheyeLens::undistorted`), instead of `2 atan(w / 2f)`. A real fisheye's data sheet focal
  length and sensor width give its real field of view, and 200° is a positive focal length.
  `w / (2 f)` beyond the largest θ_d before the cutoff is a load error.
- `UsdGeomCamera.projection` stays `perspective`: OpenUSD v26.08 (vendored) allows only
  `perspective` and `orthographic` (checked in its `usdGeom` generatedSchema.usda), so Hydra
  and usdview keep drawing a pinhole preview; Lagrange's sensor reads `lensType`.
- With `lensType = "gnomonical"` nothing changes: a pinhole render with the sensor model's
  `equidistant` distortion when asked, as now (`sensors.md`: "stays on the pinhole render").

### 3. Two applied schemas instead? (`LidarPatternAPI_1`, `FisheyeLensAPI_1`)

| | Attributes on the existing schemas (proposed) | New applied schemas |
|---|---|---|
| Opt-in | a token on the sensor (`scanPattern`, `lensType`) | `HasAPI` |
| Overlap with what exists | none: `scanPattern` and `distortionModel`/`Coeffs` already exist and are extended | `LidarPatternAPI_1` would say the pattern beside `scanPattern`; `FisheyeLensAPI_1` would need k1..k4 beside `distortionCoeffs`, or depend on them: two places that can disagree, and a precedence rule |
| Freeze test | new rows in the schema test; `frozen_1_0.json` untouched; appended tokens are allowed by `test_schemas.py` | the same |
| A later breaking change | `LidarAPI_2` / `CameraSensorAPI_2` for the whole sensor | only the small schema |
| The inspector and `lagrange check` | one group per sensor | two groups, one empty unless the other says so |
| Rosette's five numbers | namespaced `rosette:*` on a lidar that may not be a rosette | only on rosettes |

Proposed: attributes on the existing schemas, because both already hold half of the idea
(`scanPattern`, the equidistant coefficients) and a second schema would duplicate it. The one
part that would read better as its own schema is the rosette's five numbers; question 1 asks
whether the owner prefers `LidarRosetteAPI_1` for them alone.

### 4. SDFormat

SDFormat 1.12 (`/opt/ros/lyrical/opt/sdformat_vendor/share/sdformat/1.12/`, libsdformat 16.1.0,
`sdformat_vendor` 0.3.4 in Lyrical):

| Lagrange | SDFormat element | Status in the spec |
|---|---|---|
| fisheye lens | `<camera><lens><type>equidistant</type>` | defined (`camera.sdf`: gnomonical, stereographic, equidistant, equisolid_angle, orthographic, custom) |
| cutoff | `<lens><cutoff_angle>` | defined (default 1.5707, 0…π) |
| field of view across the width | `<horizontal_fov>` with `<lens><scale_to_hfov>true` | defined; `scale_to_hfov` false (the image from `custom_function`'s `f`) is not modelled: a note, read as true |
| k1..k4 | none: `<camera><distortion>` is Brown–Conrady (k1, k2, k3, p1, p2, center), no k4 | `<lens><lagrange:k1>`..`<lagrange:k4>` |
| scan pattern | none (`lidar.sdf`: scan, range, noise, visibility_mask) | `<lidar><lagrange:pattern>grid\|flash\|rosette</lagrange:pattern>` |
| channel table | none | `<lidar><scan><vertical><lagrange:angles>` (rad, space separated) |
| rosette | none | `<lidar><lagrange:rosette>` with `fov`, `point_rate`, `speed_1`, `speed_2`, `prism_ratio` |

Custom elements: libsdformat keeps an element whose name has a namespace prefix in the
element tree without a warning (checked with libsdformat 16.1.0: `<lagrange:pattern>` inside
`<lidar>` and `<lagrange:k1>` inside `<lens>` load silently and are in `Sensor::Element()`, with
or without an `xmlns:lagrange` declaration), while an unknown plain element is copied with a
warning ("XML Element[k2], child of element[lens], not defined in SDF. Copying[k2] as
children of [lens]"). SDFormat documents custom elements as namespaced ones (sdformat.org,
"Custom elements and attributes"; `TODO(verify)` the spec version that introduced them). The
importer reads the prefixed form, and also the plain one that URDF uses, with the libsdformat
warning left to show. A file written this way still loads in Gazebo, which ignores the custom
elements and draws its own equidistant lens and grid lidar (`TODO(verify)` with gz-sim 10).

The importer reads the lens through libsdformat's API (`sdf::Camera::LensType()`,
`LensCutoffAngle()`, `LensScaleToHfov()`, present in 16.1.0) and the custom elements through
`Sensor::Element()` as it does for custom sensors (`customParameters`,
`SdfImporter.cpp:370`). The result is the same settings map the URDF reader fills, so the
patterns and lenses behave the same from all three formats.

### 5. Writer (`io/stage/src/StageWriter.cpp`)

- Lidar: `scanPattern` from `pattern` (`flash`, `rosette`; the grid writes nothing new),
  `channelElevations` from `vertical_angle_*` (and `channels` = its length), the five
  `rosette:*` always written for a rosette (so the file does not depend on the loader's
  assumptions).
- Camera with `projection` 1: `lensType = "equidistant"`, `cutoffAngle` when given,
  `focalLength = w / (2 θ_d(hfov / 2))` instead of `w / (2 tan(hfov / 2))`, and the coefficients
  as now. This also fixes the negative focal length above.
- URDF reading (`Robot.cpp`) does not change.

### 6. Migration

None. Every addition is optional, with a fallback (`scanPattern`, `channelElevations`,
`lensType`) that is today's behaviour or no fallback and no effect unless its pattern or lens is
chosen (`rosette:*`, `cutoffAngle`). `frozen_1_0.json` does not change; `schemas/CHANGELOG.md`
gets an entry; `migrate_schema --check` passes every existing file unchanged. A file using the
new attributes, opened by Lagrange 1.2 or older, runs as a grid lidar and a pinhole camera with
equidistant distortion (the old reader ignores unknown attributes): worth a line in the release
notes.

## Test plan

1. `schemas/test/test_schemas.py`: the new attributes have units (`rad`, `1/s`, `rad/s`, `1`);
   the tokens are appended, not inserted; nothing in `frozen_1_0.json` changed.
2. Reader (`io/stage/test/test_stage.cpp`): each new attribute reaches the settings
   `LidarLayout::from` and `FisheyeLens::from` read; `channels` disagreeing with
   `channelElevations`, `plumb_bob` with an equidistant lens, a focal length the lens cannot
   reach, a flash lidar beyond ±π/2 are each refused with the prim path; an old file (each
   sample `.usda`, and one with `scanPattern = "solid_state"` and one with a pinhole
   `equidistant` distortion) gives exactly the settings it gives today.
3. Round trip (`server/test/test_optics_import.cpp`): the rig URDF → `lagrange_import` →
   USD → load gives, for every beam of every lidar, the same `LidarLayout::direction` bit for bit
   (scans 0 and 7 for the rosette), and the same `FisheyeLens` (k, cutoff) and focal length to
   1e-12 relative (it passes through θ_d⁻¹); then a CPU run of each gives the same
   `sensor/<n>/ranges` and camera depth (CPU lidar, software camera device) as the URDF.
4. SDFormat (`io/importers` tests): an SDFormat rig with `<lens><type>equidistant`,
   `<cutoff_angle>`, `<lagrange:k1..k4>`, and lidars with `<lagrange:pattern>`,
   `<lagrange:angles>`, `<lagrange:rosette>` gives the URDF rig's settings exactly; a plain
   `<lens><type>stereographic` is a note and a pinhole, as in URDF.
5. Determinism: `lagrange determinism` on a profile with the imported rig (CPU reference,
   invariant 7).
6. The existing `test_lidar_patterns`, `test_fisheye` and the sample assets' `test_usd_assets`
   unchanged.

## Alternatives considered

| Alternative | Why not |
|---|---|
| New applied schemas | §3 |
| An explicit `sensor:camera:fieldOfView` for the fisheye | Duplicates `UsdGeomCamera`'s focal length and aperture, which a fisheye has like any lens; two ways to say the angle can disagree |
| `UsdGeomCamera.projection = "fisheye"` | Not an allowed token in OpenUSD v26.08; a value outside the token list fails USD validation and other tools |
| Reinterpret `distortionModel = "equidistant"` as the fisheye render | Changes what existing files draw (they get the pinhole render today): a `CameraSensorAPI_2` change under RFC 0007 |
| Rosette numbers with fallbacks | Would freeze unverified prism speeds for 1.x (§1) |
| SDFormat: put k1..k4 in `<distortion>` | `<distortion>`'s k1..k3 are Brown–Conrady coefficients in SDFormat; reusing them for Kannala–Brandt would change their meaning for Gazebo |

## Open questions (with a proposed answer)

1. *The rosette's five numbers: `LidarAPI_1` attributes or a `LidarRosetteAPI_1`?* Proposed:
   attributes, namespaced `rosette:`, for one place per sensor (§3).
2. *Rosette attributes without fallbacks*, the loader's documented values with a note when
   unauthored? Proposed: yes, until a measured Livox pattern pins the speeds; then they can be
   given as recommended values in the docs, still not as frozen fallbacks.
3. *Field of view from focal length and aperture through the lens* (§2), or an explicit angle?
   Proposed: through the lens; the writer computes the focal length, so URDF users never see it.
4. *Accept the plain (unprefixed) SDFormat elements* that the URDF form uses, with libsdformat's
   warning? Proposed: yes, so one `<sensor>` block can be pasted between URDF and SDFormat.
5. *A `lagrange:` namespace URI* for `xmlns:lagrange` in SDFormat files? Proposed: the
   documentation site's URL, `https://open-physical-systems.github.io/Lagrange`, documented in
   `formats.md` §9; libsdformat does not require the declaration.

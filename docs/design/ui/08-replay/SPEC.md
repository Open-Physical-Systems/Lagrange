# 08 · Replay & validation: SPEC

## Purpose
Open MCAP recordings, scrub through runs with domain event lanes, compare a simulated run with a
real robot log, and run validation suites. This is how the project's realism claims are proven.

## Dock layout
```
left 280 px  : top "Recording" | "Compare" (auto) ; bottom "Channels" + "Bookmarks"
center flex  : top 400 px "Timeline" | "Events · N" ; bottom "Sim vs real" | "Overlay in 3D"
right 340 px : "Validation" | "History"
```
The workspace bar shows replay controls (Previous event, Play, Next event, t / duration, speed,
loop). The domain strip label becomes **Source** and every chip reads "replay" (Radiation: "not
recorded"). On the right: the recording's original profile and **Re-simulate from here…**,
which starts a live sim seeded from this time.

## Panels
### Recording / Channels / Bookmarks
- File facts: name, duration, size and compression, channels, messages, recorded at, "opens in Foxglove, ros2 bag, Python".
- Channels: checkbox, mono topic, rate. Unchecked channels are not decoded, which saves CPU.
- Bookmarks: name and time; clicking seeks.

### Timeline (custom draw)
- A ruler 0:00…end and lanes per domain with colored bars and markers:
  - slip (rigid body)
  - regen and **brownout** (electrical, red with a label)
  - derate (thermal)
  - downpour (weather)
  - magnetometer deviation and packet loss (EMI · comms)
  - lidar range drop (sensors)
  - goals (nav2, diamonds)
  - bookmarks (accent diamonds)
- An accent loop-range band, an accent playhead, and an overview strip (whole run) with the
  window highlighted.
- Events · N tab: a sortable list of `Event`s with jump-to.

### Sim vs real
- An overlay plot: sim solid (series color) and real dashed (t2), labeled with the real topic
  name. Aligned by cross-correlation; the lag is shown.
- An error plot (real − sim) with a green tolerance band.
- Metric cards: RMSE, NRMSE (range), max |error|, lag, pass/fail against tolerance.
- Error by driving phase: meters per phase (accelerate, cruise, regen brake, stopped).

### Validation
- Suite header (`validation/<suite>.yaml`, real log date) and an "N / M pass" pill.
- A mapping table: sim signal ↔ real topic, NRMSE, limit, ✓/✗.
- Failure explanation card: a title, a sentence with the probable cause, and actions (open the
  parameter in the Inspector, plot both).
- Method: alignment, resample rate, metric, window.
- Footer: "also runs nightly in CI", Export report, **Run suite**.

## States
- No compare log: the Sim vs real panel shows "Load a real log to compare" with a file picker.
- Topics without a mapping are listed under "unmapped" with a suggestion.

## Mock server must provide
Replay mode reading an MCAP (the mock server is the replay engine; CLAUDE.md §2), events, and a
second MCAP as the "real" log. Validation metrics can be computed in the viewer (Python tool later).

## Acceptance
- Scrubbing a 1.8 GB file keeps 60 fps (indexed reads).
- The validation result of the canvas example is reproducible from the two logs.

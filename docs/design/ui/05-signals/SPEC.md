# 05 · Signals & scopes: SPEC

## Purpose
This is a general oscilloscope for any of the ~200+ signals across all domains: browse, plot,
trigger, measure with cursors, compute statistics, and export.

## Dock layout
```
left 300 px  : "Signals" | "Saved sets · N"
center flex  : one tab per scope ("Scope · drive train", "Scope · thermal soak", "+")
right 300 px : top "Cursors" | "Measure" (auto height) ; bottom "Statistics" | "FFT"
```

## Panels
### Signals browser
- Search (filters name, path and unit) with a match count "31 of 214".
- Filter pills: domain (domain colored), "unit: any ▾", "rate ≥ 100 Hz".
- Grouped by domain (collapsible). Each row has a checkbox, a series color swatch (when
  plotted), the mono signal path, the unit and the rate.
- Drag a row onto a plot to add it; double-click adds it to the selected plot.
- Data: `SignalCatalog`.

### Scope
- Toolbar: Live/Paused, window (1 s / 10 s / 60 s / run), **trigger** (signal, edge, level,
  once/normal/auto) with a "fired at …" pill, Link X, Cursors A·B toggle, Export CSV, Save to MCAP.
- Plots: N stacked ImPlot subplots with a linked X axis. Each has a right-aligned title with the
  unit and a legend, the series palette, and a dotted threshold line for the trigger level.
- Cursors: A (t1 line, "A" tag) and B (accent line, "B" tag), draggable on any plot.
- **Decimation**: per pixel column keep min and max (two points), computed on a worker thread
  from the ring buffer. ImGui draws only the decimated points.
- The time axis sits under the last plot (−10 s … 0).

### Cursors
A table of signal, A, B, B−A (mono, right-aligned), with t in s as the first row.

### Statistics / stream health
- Statistics over the visible window: min, max, mean, RMS.
- Stream health: samples in the window, drawn after decimation, ring-buffer fill (meter),
  dropped samples (must be 0), and plot CPU time per frame.

## Performance budget (CLAUDE.md §13.4)
At least 16 signals at 10 kHz with no drops; plot time under 2 ms per frame.

## Mock server must provide
`SignalCatalog` with ~200 signals across domains, `SignalChunk` streams at their native rates,
and a trigger evaluated server-side or client-side (decided in RFC 0003).

## Acceptance
- A trigger "iq ↓ −8 A once" pauses the scope exactly at the crossing.
- The cursor table updates while dragging.
- CSV export contains the raw (not decimated) samples in the visible window.

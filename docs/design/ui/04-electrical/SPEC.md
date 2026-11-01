# 04 · Electrical workspace: SPEC

## Purpose
Show the robot's power system live: a schematic with current flow, high-rate scopes for voltage,
current and power, limits and events (brownout, regen overvoltage, fuse I²t), and battery and
motor model parameters.

## Dock layout
```
left 300 px  : "Network" | "Probes · N"
center flex  : top "Schematic" | "Netlist" | "3D overlay" ; bottom 372 px "Scopes" | "+ scope"
right 340 px : "Inspector" | "History"
```

## Panels
### Network
- A table of component, V, I, P (right-aligned mono, indented by topology). The selected
  component is synced with the schematic, the tree and the viewport.
- **Limits and events** section: a pill and text for recent events; meters for OV limit, UV trip,
  fuse I²t and driver current limit (warn color past 85 %); a final line "no brownout · MCU rail
  stayed above 3.0 V".

### Schematic (node graph, read-only layout plus auto-layout)
- Built from the USD electrical topology (`PortAPI` relationships). Never hand-placed per robot.
  The layout is stored per stage.
- Nodes: battery (state of charge meter, V, I, temperature, "ECM · FMU"), fuse, the DC bus as a
  vertical bar with a voltage label and UV/OV limits, drivers (duty, efficiency), motors (A, rpm,
  winding temperature), DC-DC, loads.
- Wires: yellow; **width ∝ |I|**; dashes animate along the current direction at a speed
  proportional to |I|. Regen current is drawn blue with reversed arrows. Current labels sit on
  the wires. Motors show their mechanical output (τ → wheel) in rigid-body gray.
- Header: "Auto-layout", "Export SPICE" (ngspice netlist), "Export Modelica".
- The 3D overlay tab tints wires and motors in the viewport by current.

### Circuit (proposed; the owner asked for it, 2026-11-01)
- A tab next to Schematic, not in the reference image. The same power system as a circuit
  diagram: each bus is a yellow rail with a branch per component hanging down to a ground symbol,
  batteries first, then the bus's capacitor, then what it feeds.
- Symbols in `t1`: a battery (long plate positive) under a resistor for its r0, RC pairs side by
  side, a capacitor, a resistor, a circled M for a motor, a circled P for a constant-power load.
  Wires in the electrical colour, blue when the current is reversed. No animation.
- Beside each branch in sans 600 its name, in mono its current, then its values one per line. Above
  the rail in mono its name and voltage, then its limits.
- Clicking a branch or bus selects that prim, as the Schematic does.

### Scopes (ImPlot; see Signals for the full engine)
- Stacked plots sharing the X axis: `bus/dc_48v/voltage`, `motor/*/iq` (left and right),
  `battery/power`. Titles are right-aligned inside each plot.
- Off-scale limits are annotated with text (for example "UV trip 36.0 V ↓ off-scale").
- A hover cursor shows a tooltip with every series value at t.
- Header: LIVE pill, window segmented control (2 s / 10 s / 60 s), "min/max decimation · 10 kHz
  in", Freeze.

### Inspector (battery shown)
- State cards (2×2): state of charge with a meter, terminal V and OCV with sag, current and
  power, cell temperature and state of health.
- An OCV(SoC) mini plot with the current operating point in accent.
- `BatteryAPI_v1` fields: capacity, R0, R1·C1, R2·C2, behaviorModel FMU, parameter source.
- Footer: "Plot all battery signals", "Open FMU".

## Interactions
Clicking a wire or node selects that component. Double-clicking a node opens its FMU parameters.
Alt-clicking any value adds it to the scopes.

## States
Electrical domain off: the schematic stays visible but greyed out, with a banner "Electrical is off
→ ideal torque source. Currents are not simulated."

## Mock server must provide
A topology for the sample AMR, signals at 10 kHz for the bus and motors, events for regen and
limits, and battery state.

## Acceptance
- 4 scope series at 10 kHz each with zero dropped samples and a frame time under 16.6 ms.
- The sum of branch currents matches the battery current in the mock data, since the UI shows both.

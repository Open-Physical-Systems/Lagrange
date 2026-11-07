# Battery

Each `BatteryAPI_1` prim is a pack of cells in series and in parallel. The electrical domain
runs it with the motors, every 50–100 µs, and gives its terminal voltage to the bus and its
state of charge to the signals (`battery/<b>/terminal_voltage`, `soc`, `current`, `power`).

## The equivalent circuit

```mermaid
flowchart LR
    OCV["OCV(SoC)<br/>a curve from the file"] --- R0["R0(T)"] --- RC1["R1 ∥ C1<br/>seconds"] --- RC2["R2 ∥ C2<br/>minutes"] --- V["terminal<br/>voltage"]
```

Per cell, with $I$ the current (positive discharging):

$$
V = \mathrm{OCV}(\mathrm{SoC}) - I\,R_0(T) - V_1 - V_2, \qquad
\frac{dV_k}{dt} = \frac{I}{C_k} - \frac{V_k}{R_k C_k}
$$

- **Open-circuit voltage:** interpolated from the pack's `elec:battery:ocvSoc` and `ocvVoltage`
  table.
- **Series resistance:** $R_0$ gives the instant drop when the current changes. It rises
  linearly with temperature,
  $R_0(T) = R_0\,(1 + \alpha\,(T - T_{ref}))$.
- **RC pairs:** the two pairs give the slower sag and recovery, one over seconds and one over
  minutes. Each is solved exactly over the step.
- **State of charge:** counted from the charge drawn,
  $\mathrm{SoC} \mathrel{-}= I\,\Delta t / Q$.
- **The pack:** cells in series add voltage; cells in parallel share the current.
- **Empty:** the pack is empty when its open-circuit voltage reaches `cutoffVoltage`. The bus
  browns out when its voltage falls under `undervoltageTrip`.

## Checked against a real cell

The LG 18650HG2 data set (Kollmeyer, Naguib and Skells, 2020, CC BY 4.0) has pulse tests and car
drive cycles measured on one cell. The circuit's resistances and capacitances were fitted to one
cycle (UDDS) and then compared with cycles it had never seen:

| Data | RMS voltage error | Target | Result |
|---|---|---|---|
| Pulses, 46–89 % SoC | 5.4–6.2 mV | 20 mV | met |
| Pulses, 24 % SoC | 21.5 mV | 20 mV | **not met**: $R_0$ and the OCV slope rise at low charge |
| LA92 drive cycle, 25 °C (unseen) | 11.7 mV | 20 mV | met |
| US06 drive cycle, 25 °C (unseen, aggressive) | 20.1 mV | 20 mV | **not met by 0.1 mV** |
| US06 at 0 °C | 145 mV | — | **far off**: only $R_0$ follows temperature, and nothing follows the charge yet |

All in `BatteryDataset.*` (`test_battery_dataset`); `tools/ci/realism.sh` runs them.

**What this means for a drone:** at room temperature and above about 30 % charge, the voltage
under a changing load is within about 20 mV per cell (80 mV for a 4S pack). In the cold or near
empty, it reads too high, so a simulated drone flies longer than a real one would. Resistances
that change with charge and temperature need a schema addition, which is owed.

The Holybro sample's pack values (4S 5000 mAh, $R_0$ 4 mΩ per cell) are illustrative, not
measured on a real pack. The model and its signals are in
[electrical.md](../design/domains/electrical.md) §2.

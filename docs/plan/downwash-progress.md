# Rotor downwash and live airflow: progress

Work branch: `feat/rotor-downwash` (from `dev` after #259). Design: `docs/design/domains/rotor_aero.md` §9 (to write).

| Step | What | State |
|---|---|---|
| 0 | Merge `feat/cfd-openfoam` into `dev` | done, #259 |
| 1 | Near wake per rotor (actuator disc) and merged far jet (Bauersfeld et al. 2024) | done: `aerodynamics::Downwash` |
| 2 | Ground outwash (Tanner et al. 2015 fit, wall-jet decay) | done: `outwashSpeed` |
| 3 | `windAt`, downwash on bodies, per-rotor air for the rotors, dust to weather | done: `AeroDrag`, `rotor/<j>/air_velocity`, `env/dust/downwash` |
| 4 | Tests against references | done: `test_downwash` (8), `test_downwash_pair` (Kiran et al. 2025: 53% vs 65% at 4 l, 87% vs 95–98% at 19 l) |
| 5 | Live slice `aero/wind_speed` | done |
| 6 | Protocol: slice orientation, FieldVolume | done (additions; golden frames regenerated) |
| 7 | Viewer particles, "airflow · particles", legend, plane and height slider | done; particles move on the CPU, drawn by the GPU as overlay lines |
| 8 | Actuator discs in tunnel/foam (optional) | todo |
| 9 | Docs, tutorial step, video, README, CLAUDE.md | done: video `drone_downwash.mp4`, picture `drone-downwash.png` |

## Notes

- The far-jet constants (B̃ 10.11, s̃0 −5.817, S 0.07668) were fitted by Bauersfeld et al. to
  multirotors; a lone rotor uses its diameter as l (TODO(verify)).
- Outwash is calibrated at one point (a rotor 1 R up); between 1 R and 2 R the model's peak
  rises slightly with height, where Tanner et al. saw the high-speed layer fade. Known gap.

# Rotor downwash and live airflow: progress

Work branch: `feat/rotor-downwash` (from `dev` after #259). Design: `docs/design/domains/rotor_aero.md` §9 (to write).

| Step | What | State |
|---|---|---|
| 0 | Merge `feat/cfd-openfoam` into `dev` | done, #259 |
| 1 | Near wake per rotor (actuator disc) and merged far jet (Bauersfeld et al. 2024) | done: `aerodynamics::Downwash` |
| 2 | Ground outwash (Tanner et al. 2015 fit, wall-jet decay) | done: `outwashSpeed` |
| 3 | `windAt`, downwash on bodies, per-rotor air for the rotors, dust to weather | `windAt` done; module wiring next |
| 4 | Tests against references | `test_downwash` (8): momentum theory, jet decay and spread, momentum flux = thrust, outwash peak; drone-under-drone thrust loss next |
| 5 | Live horizontal slice `aero/wind_speed` | todo |
| 6 | Protocol: vertical slice orientation | todo |
| 7 | Viewer particles, "Show airflow" | todo |
| 8 | Actuator discs in tunnel/foam (optional) | todo |
| 9 | Docs, tutorial step, video, README, CLAUDE.md | todo |

## Notes

- The far-jet constants (B̃ 10.11, s̃0 −5.817, S 0.07668) were fitted by Bauersfeld et al. to
  multirotors; a lone rotor uses its diameter as l (TODO(verify)).
- Outwash is calibrated at one point (a rotor 1 R up); between 1 R and 2 R the model's peak
  rises slightly with height, where Tanner et al. saw the high-speed layer fade. Known gap.

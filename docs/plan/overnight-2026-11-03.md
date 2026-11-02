# Overnight work, 2026-11-03 (until 07:00 JST)

The owner's list, in order. Each item is a small PR into `dev`, merged when the hooks and the
colcon tests pass. No new item starts after 06:30 JST. Schema, protocol and plugin ABI changes
stop for the owner except the profile key approved in item 2.

| # | Item | Accepted when | State |
|---|---|---|---|
| 1 | Live lattice Boltzmann airflow, the spike: a GPU box of about 4 m round a hovering quadcopter with its rotors as actuator discs | steps/s measured at 8, 5 and 3 cm cells on the RTX 4060 Laptop; jet and outwash compared with Bauersfeld 2024, Tanner 2015 and the OpenFOAM hover cases (`~/cfd/hover_open`, `hover_low`); a go/no-go with the numbers in `rotor_aero.md` | done: go at 4 cm (PR below) |
| 2 | Live LBM downwash, `aerodynamics.downwash_model: lbm` (profile schema addition, approved) | coupled by signals only (rotor `air_velocity`, body forces); feeds `aero/wind_speed` and the particles; fitted model stays the default and the CPU fallback; tested against the same references; a video from a real run | |
| 3 | Fitted model: vortex ring state, the low-hover fountain (fit to `~/cfd/hover_low`), the ceiling effect | each tested against its published source; TODO(verify) where it cannot be checked | |
| 4 | `lagrange PROFILE.yaml --robot ... --world ...` opens the viewer like `lagrange quad` | the CLI test runs it; docs and tutorial updated | |
| 5 | Contact force and joint reaction in Jolt | the physics conformance test checks both engines | |
| 6 | Scopes signal picker with search | follows `docs/design/ui/` and `.claude/rules/viewer-ui.md`; headless viewer test presses it | |
| 7 | `docs/validation` page: every physical model, its reference, its error, its test | says which have none | |
| 8 | Hydrodynamics: added mass, waves and currents (Fossen) | tested against closed-form results | |
| 9 | Thermal camera (`ThermalCameraAPI_1`) | schema change: asks the owner first (the rules approve only item 2's profile key) | |

## Notes

- Item 1 (01:35–): `LbmBox` with a half-precision GPU step; a 3 m box at 4 cm runs at 1.06× real
  time on the RTX 4060 Laptop, 3 cm at 0.35×. At 4 cm and finer it makes the low-hover fountain
  (5.8–7.4 m/s up against OpenFOAM's 7.1); 5 cm and coarser merge the jets too early. Disc
  through-flow 15–20% under OpenFOAM's, low-hover outwash 22–38% under. Go at 4 cm as an option;
  rotor_aero.md §8.

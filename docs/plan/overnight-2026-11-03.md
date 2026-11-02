# Overnight work, 2026-11-03 (until 07:00 JST)

The owner's list, in order. Each item is a small PR into `dev`, merged when the hooks and the
colcon tests pass. No new item starts after 06:30 JST. Schema, protocol and plugin ABI changes
stop for the owner except the profile key approved in item 2.

| # | Item | Accepted when | State |
|---|---|---|---|
| 1 | Live lattice Boltzmann airflow, the spike: a GPU box of about 4 m round a hovering quadcopter with its rotors as actuator discs | steps/s measured at 8, 5 and 3 cm cells on the RTX 4060 Laptop; jet and outwash compared with Bauersfeld 2024, Tanner 2015 and the OpenFOAM hover cases (`~/cfd/hover_open`, `hover_low`); a go/no-go with the numbers in `rotor_aero.md` | done: go at 4 cm (PR below) |
| 2 | Live LBM downwash, `aerodynamics.downwash_model: lbm` (profile schema addition, approved) | coupled by signals only (rotor `air_velocity`, body forces); feeds `aero/wind_speed` and the particles; fitted model stays the default and the CPU fallback; tested against the same references; a video from a real run | |
| 3 | Fitted model: vortex ring state, the low-hover fountain (fit to `~/cfd/hover_low`), the ceiling effect | each tested against its published source; TODO(verify) where it cannot be checked | vortex ring and ceiling done; fountain waiting on OpenFOAM heights |
| 4 | `lagrange PROFILE.yaml --robot ... --world ...` opens the viewer like `lagrange quad` | the CLI test runs it; docs and tutorial updated | done (PR below) |
| 5 | Contact force and joint reaction in Jolt | the physics conformance test checks both engines | done (PR below) |
| 6 | Scopes signal picker with search | follows `docs/design/ui/` and `.claude/rules/viewer-ui.md`; headless viewer test presses it | done (PR below) |
| 7 | `docs/validation` page: every physical model, its reference, its error, its test | says which have none | done (PR below) |
| 8 | Hydrodynamics: added mass, waves and currents (Fossen) | tested against closed-form results | |
| 9 | Thermal camera (`ThermalCameraAPI_1`) | schema change: asks the owner first (the rules approve only item 2's profile key) | |

## Notes

- Item 1 (01:35–): `LbmBox` with a half-precision GPU step; a 3 m box at 4 cm runs at 1.06× real
  time on the RTX 4060 Laptop, 3 cm at 0.35×. At 4 cm and finer it makes the low-hover fountain
  (5.8–7.4 m/s up against OpenFOAM's 7.1); 5 cm and coarser merge the jets too early. Disc
  through-flow 15–20% under OpenFOAM's, low-hover outwash 22–38% under. Go at 4 cm as an option;
  rotor_aero.md §8.
- Item 2 (01:52–): `downwash_model: lbm`. `LiveAir` follows the first multirotor; rotors get the
  box's air less their own (calibrated on a lone disc); fitted model outside it, for other
  multirotors and without a hardware GPU. The sample hovers at about 0.75x real time (steps
  batched per 5 ms; per-ms submits made it 0.57x). Rotors see 1.1 m/s inwards, as OpenFOAM's
  1.05. Video `drone_live_air.mp4`.
- Item 3a (02:10–): vortex-ring state in `bem` from NeuroBEM eqs. 18–19 (checked in the paper's
  text); ceiling effect from Hsiao and Chirarattananon's momentum balance (γ derived from their
  eq. 5, since the extracted eq. 10 was garbled), α0 = 1.6 TODO(verify) for large propellers.
  OpenFOAM hovers at 0.75, 1.0 and 1.5 m running for the fountain fit.
- Item 4: a YAML with `domains:` is a profile; `lagrange PROFILE.yaml` runs `lagrange run
  PROFILE` as the server with the viewer. `test_cli` runs one headless.
- Item 5: Jolt's `joint/<j>/reaction` from its constraint impulses (point, rotation in Jolt's
  own basis, limits, motor) plus the applied effort; `body/<b>/contact_force` from Newton per
  group. Conformance on MuJoCo, Jolt and the Jolt plugin: a resting box's weight, a still and a
  swinging pendulum's pivot (11.348 N on Jolt, 11.349 N on MuJoCo and by energy).
- Item 6: the Signals list already had search, a match count and domain chips (my plan said
  otherwise: wrong). Added what the 05-signals design still lacked: the "unit: any" dropdown chip
  (the designed ChevronDown icon; the ▾ glyph is not in IBM Plex) and dragging a row onto the
  scope. `test_viewer_input` drags one.
- Item 7: `docs/validation/report.md` brought up to date: the models since v1.0 with their
  references and results (including the misses: Kiran 12 points short, the tunnel's Ahmed body
  over four times too high), and a plain list of the models with no reference yet.

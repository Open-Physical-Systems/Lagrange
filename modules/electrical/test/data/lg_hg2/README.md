# LG HG2 cell data for the battery model check

`test_battery_dataset` checks the network's battery equivalent circuit (electrical.md §8,
"Battery ECM pulse discharge") against measurements of a real cell.

## Source and licence

The measurements are from P. Kollmeyer, M. Naguib and M. Skells, "LG 18650HG2 Li-ion Battery
Data", Mendeley Data, version 2, 2020, [doi:10.17632/b5mj79w5w9.2](https://doi.org/10.17632/b5mj79w5w9.2).
The tests were done at McMaster University, Hamilton, Canada, on a new 3 Ah LG HG2 cell. The data
is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) (`LICENSES/CC-BY-4.0.txt`),
and so are the files derived from it here. Neither the authors nor McMaster University endorse
this project.

Changes made to the data (`fit_ecm.py` does all of them):

- `hppc_25degC_validation.csv`: four of the 4-pulse blocks of the 25 °C HPPC test
  (`549_HPPC.csv`), the ones starting at 89, 68, 46 and 24 % state of charge. Time restarts at 0
  in each block, the current is positive when discharging, and the state of charge is the
  cycler's Ah counter over the C/20 discharge capacity.
- `ecm_fit.txt`: an open-circuit voltage curve (mean of the C/20 discharge and charge voltages,
  `549_C20DisCh.csv`) and r0 and two RC pairs fitted by least squares to the other 4-pulse
  blocks between 20 and 97 % state of charge.

`fit_ecm.py` downloads nothing. Download the two files from the dataset (URLs and SHA-256 in
the script) and run `python3 fit_ecm.py DIRECTORY`; it needs NumPy and SciPy, which are used
only here, not by the simulator.

## Result

| Block starts at | Terminal-voltage RMS error | §8 tolerance (20 mV) |
|---|---|---|
| 89 % | 5.4 mV | met |
| 68 % | 6.2 mV | met |
| 46 % | 5.6 mV | met |
| 24 % | 21.5 mV | not met |

One parameter set covers the cell from about 35 % to full charge. Lower down its resistance and
OCV slope grow, and one set is not enough: a state-of-charge-dependent r0 (a schema addition to
`BatteryAPI`) would be the next step. Only 25 °C was checked; the dataset also has 40, 10, 0, −10
and −20 °C for `r0TempCoeff`.

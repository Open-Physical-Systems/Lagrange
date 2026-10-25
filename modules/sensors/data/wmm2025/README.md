# World Magnetic Model 2025

`WMM2025.COF` (the coefficients) and `WMM2025_TestValues.txt` (NOAA's test points) are from
the World Magnetic Model 2025 release of NOAA's National Centers for Environmental Information
and the British Geological Survey, downloaded from
<https://www.ncei.noaa.gov/products/world-magnetic-model> (`WMM2025COF.zip`, 17 December 2024),
unchanged. The model is valid from 2025.0 to 2030.0.

This material is a work of the U.S. Government and is in the public domain: "The WMM source code
is in the public domain and not licensed or under copyright. The information and software may be
used freely by the public." (NCEI). As NCEI asks, this notice identifies the U.S. Government
material incorporated here.

`src/generated/Wmm2025Coefficients.inc` is generated from `WMM2025.COF`. To regenerate it after
replacing the coefficient file (the next model, WMM2030):

```bash
python3 - <<'PY'
lines = open("modules/sensors/data/wmm2025/WMM2025.COF").read().splitlines()
epoch, model = float(lines[0].split()[0]), lines[0].split()[1]
rows = [l.split()[:6] for l in lines[1:] if len(l.split()) >= 6 and not l.startswith("9999")]
# write the header comment, kWmmEpoch, kWmmMaxDegree and kWmmCoefficients as in the .inc
PY
```

`test_wmm` checks the model against every test point.

# Sensor defaults against their sources (2026-11-12)

The sensor work of 2026-11-11 (PR #330) left `TODO(verify)` markers and defaults labelled
*illustrative* in `modules/sensors/`, `bridges/ros2/src/KindTopics.cpp`,
`docs/design/domains/sensors.md` and `docs/guide/sensors.md` (114 lines by
`grep -rn "TODO(verify)\|illustrative"`), and data-sheet values that nobody had checked against the
document. Each was looked up in a primary source (the maker's data sheet or manual, a standard,
the referenced project's source code, a peer-reviewed paper); a value seen only in a search
summary was not taken as verified.

Status: **confirmed** (the source gives the value; the citation is now precise), **corrected** (the
value or its reading changed to the source's; tests that pinned the old one updated),
**unverifiable** (no primary source found or none exists; kept illustrative or `TODO(verify)`, with
the reason in the docs). `file:line` is at 71880eb, before these changes; a row may stand for the
same item in the code and in sensors.md.

Counts (rows below; a row may group several parameters of one kind): 14 corrected, 35 confirmed,
30 unverifiable. They cover all 114 grep lines plus the cited data-sheet values of the parts that
carried no marker (INA226, ACS712, HX711, SBS, Mini45, nanoScan3, SHT40, ...).

## Corrected

| Item | file:line | Old value | New value | Source |
|---|---|---|---|---|
| Livox rosette prism speeds | core/world/include/lagrange/world/LidarLayout.hpp:55; sensors.md:473 | 2π·120, −2π·73.737 rad/s, made up | 2π·7294/60, −2π·4664/60 rad/s | Liu, Zhang and Hong, "Low-cost retina-like robotic lidars based on incommensurable scanning", arXiv:2006.11034, appendix B ("two identical prisms … 7294 rpm and -4664 rpm"). The paper's prototype, not stated to be the Mid-40's; Livox publishes no Mid-40 figure |
| Fog lidar ratio | WeatherEffects.hpp:32; weather.md, physics/sensors.md | 18.5 sr | 18.8 sr | O'Connor, Illingworth and Hogan, J. Atmos. Oceanic Technol. 21, 777–786 (2004), 18.8 ± 0.8 sr at 905 nm for liquid-water cloud, read through Hopkin et al., Atmos. Meas. Tech. 12, 4131–4147 (2019), §3 (O'Connor itself not fetched: publisher 403) |
| IMU `<dynamic_bias_stddev>` reading | sensors.md:167; ImuNoise.cpp | the Gauss-Markov bias's stationary deviation | the driving noise σ_b; stationary deviation σ_b·√(τ/2) | SDFormat 1.12 `noise.sdf` ("the standard deviation of the noise used to drive a process"); gz-sensors `src/GaussianNoiseModel.cc` (main 8ddf4c17, gz-sensors10 6b203821): `sigma_b_d = sqrt(-sigma_b*sigma_b*tau/2*expm1(-2*dt/tau))` |
| SPS30 PM1/PM2.5 precision | ParticleSensor.cpp:22–33; sensors.md:2000 area | ±10 µg/m³ below 100 µg/m³ | ±(5 µg/m³ + 5 % of the reading) | Sensirion SPS30 datasheet Version 2.0 – D1 – June 2023, table 1 (v1.0, March 2020, had ±10). Precision is device-to-device variation on ≥ 150 s averages; still used as 1σ noise, now said so |
| SHT40 noise | HumiditySensor.cpp:18–19 | 0.08 %RH, 0.04 K as 1σ | 0.0267 %RH, 0.0133 K (1σ) | SHT4x datasheet Version 7.3, June 2026, tables 1–2 and footnote: repeatability "is three times the standard deviation (3σ)" (same in Version 1, Oct 2020) |
| WindSonic speed noise | Anemometer.cpp:16 | 1 % of speed (±2 % as 2σ) | 2 % (1σ) | Gill WindSonic User Manual 1405-PS-0019 Issue 30 (Nov 2022), appendix A: "± 2% RMSE (at 12m/s)"; an RMS error is 1σ |
| WindSonic direction noise | Anemometer.cpp:18; sensors.md:1128 | 1.5° (±3° as 2σ) | 1.0° (±2° as 2σ) | Same manual, appendix A: direction "± 2° (at 12m/s)"; ±3° is the 2009 datasheet's |
| BH1750 count and saturation | LightSensor.cpp:17–18; sensors.md light table | 1 lx per count, saturates at 65535 lx | 1/1.2 lx per count, 54 612 lx | ROHM BH1750FVI datasheet 2011.11 Rev.D, p. 7: illuminance = counts / 1.2 |
| PWD22 accuracy beyond 10 km | VisibilitySensor.cpp | ±10 % everywhere | ±15 % from 10 to 20 km (new `noise/fraction_far`) | Vaisala PWD series datasheet B210385EN-F: "±15 % at 10 … 20 km"; PWD22 User's Guide M210543EN-B, table 24 |
| DAVIS346 field of view | EventCamera.cpp:21; sensors.md:769 area | 1.22 rad ("6.9 mm wide array") | 1.2357 rad (70.8°, 6.4 mm array) | iniVation DAVIS346 specification sheet, latest change 15 Aug 2019: "346 pixels x 18.5 um/pixel = 6.4 mm", 4.5 mm lens 70.8° |
| Event camera refractory period | EventCamera.cpp:24; sensors.md:769 | 1e-4 s, illustrative | 5e-4 s | v2e `v2ecore/v2e_args.py` (master cac6384): `--refractory_period` default 0.0005. iniVation publishes none |
| VL53L0X proximity noise | Proximity.cpp:18–20; sensors.md:2048 | 1 mm, illustrative | 12 mm (4 % of the 0.3 m reach) | ST VL53L0X DocID029104 Rev 2 (Apr 2018), table 12: standard deviation 4 % at 33 ms, white target, indoors |
| Ping2 in the sonar example | sensors.md:2191 and the altimeter example | min 0.5 m, frequency left at 750 kHz; TODO for min range and frequency | min 0.3 m, `frequency` 115000; the kind's own defaults stay the Oculus's | Blue Robotics, "Ping Sonar Altimeter and Echosounder" store page, technical details: 115 kHz, 25° at −3 dB, typical minimum range 0.3 m, usable range 100 m, resolution 0.5 % of range |
| ADS-B section numbers | AdsbReceiver.cpp:36, 41–43; sensors.md:2295–2300 | DO-260B §2.2.3.3.1, §2.2.3.2.3.4, §2.2.3.2.6.1 (not read) | ICAO Annex 10 Vol. IV §3.1.2.8.6.4.2, .4.5 (0.4–0.6 s), §3.1.2.6.5.4 (Q bit, 25 ft) | Read in CAA Fiji SD-ATELCOM Vol. IV ed. 1.2 (a transposition of Annex 10 Vol. IV to amendment 91). Velocity 1 kt and vertical rate 64 ft/min: J. Sun, *The 1090 Megahertz Riddle*, 2nd ed., "Airborne velocity" (secondary). DO-260B itself is sold by RTCA and was not read |

Also corrected (wording, no value): the nanoScan3 scan cycle (below), the V2X citation (Bai,
Stancil and Krishnan 2010 is ACM MobiCom, not IEEE VNC; replaced by Meireles et al., IEEE VNC
2010, read), the ATI table name ("Metric Calibrations (SI)", not "Standard Calibrations"), the
RG-15 freezing note ("accuracy may vary near/below freezing", not "does not see rain"), the TFmini-S
document ID (SJ-PM-TFmini-S A01), the IES model's origin (Gillette, Pierpoint and Treado, NBS
1982).

## Confirmed

| Item | file:line | Value | Source |
|---|---|---|---|
| INA226 steps, ranges, conversion times, averages, POR default | PowerMonitor.cpp:6–36 | 2.5 µV, ±81.92 mV, 1.25 mV, 40.96 V (36 V in), 140 µs–8.244 ms, 1–1024, 1.1 ms | TI SBOS547C (June 2011, revised Aug 2026), §5.5, tables 7-2 to 7-5 |
| ACS712ELCTR-05B-T | PowerMonitor.cpp:37–39 | 185 mV/A, 21 mV p-p | Allegro ACS712-DS Rev. 22 (13 Feb 2024), x05B table: VNOISE(PP) "TA = 25°C, 185 mV/A, CF = 47 nF, COUT = open, 2 kHz bandwidth" |
| HX711 | TensionSensor.cpp:5–22 | 24-bit two's complement, ±0.5·AVDD/gain, 50 nV rms at gain 128, 10 SPS | Avia HX711 datasheet (undated; SparkFun copy), table 2 and p. 1 |
| Smart Battery registers | BatteryMonitor.cpp:3–24 | mV, mA (signed), 0.1 K, %, mAh | SBS Rev 1.1 (11 Dec 1998), §5.1.9, 5.1.10, 5.1.11, 5.1.14, 5.1.16 |
| ATI Mini45 SI-580-20 | ForceTorque.cpp:7–33 | 580/580/1160 N, 20 N·m; 1/4 N, 1/188, 1/376 N·m | ati-ia.com Mini45 page, "Metric Calibrations (SI)", 16-bit DAQ/ECAT/Net/TWE/WNet systems; "after dropping four counts of noise" |
| nanoScan3 Core I/O | SafetyScanner.cpp:7–27 | 275°, 0.17°, 70 ms, 40 m, fields 3 m / 10 m | SICK data sheet NANS3-AAAZ30AN1 (1100333), stamped 2026-11-03 |
| nanoScan3 scan cycle | sensors.md:1453 | TODO → 30 ms; t_R = n × 30 ms + 10 ms, n = 2 | nanoScan3 I/O operating instructions 8024602/1W27/2026-05-26 (Japanese edition), §14.2 table 33, §14.3. The example's `update_rate` 25 → 33.333 |
| Young 05103 | Anemometer.cpp:22–27; sensors.md:1136 | 1.0 m/s, 1.1 m/s, 2.7 m (63 %) | R. M. Young manual 05103-90 rev. N062819 |
| WindSonic resolution, threshold | Anemometer.cpp:19–21 | 0.01 m/s, 1°, 0.01 m/s | Manual Issue 30; threshold in the 2009 and WindSonic M (1405-0029 Iss 3) datasheets |
| MS4525DO counts and temperature | Airspeed.cpp:13–19 | 10–90 % of 16383, 11 bits over −50…150 °C | TE MS4525DO datasheet 04/2019 (and ENG-DS-MS4525DO-B1, 09/2015), p. 5 |
| MS4525DO total error band | Airspeed.cpp:15; sensors.md:1170 | TODO → ±1 % of span, not modelled by default | Same, p. 4 (notes 3, 7) |
| SHT40 accuracy, τ63, resolution | HumiditySensor.cpp:13–23 | ±1.8 %RH, ±0.2 °C, 4 s / 2 s (6 s in 2020), 0.01 | SHT4x datasheet V7.3 tables 1–2; Version 1 (Oct 2020) for the 6 s |
| BH1750 accuracy, timing, spectrum | LightSensor.cpp:12–15 | S/A 0.96–1.44, 120 ms; spectral TODO → "approximately human eye response", peak 560 nm | BH1750FVI datasheet 2011.11 Rev.D, pp. 1–2 |
| PWD22 range, accuracy, averaging | VisibilitySensor.cpp:13–20; sensors.md:1308 | 10–20 000 m, ±10 % to 10 km; 60 s lag (TODO) | B210385EN-F; M210543EN-B p. 25 (15 s values averaged into 1 and 10 min) and table 24 ("Time constant 60 seconds") |
| RG-15 accuracy, resolution | RainSensor.cpp:13–17 | ±10 %, 0.2 / 0.02 mm | RG-15 Instructions, SW rev 1.000, 2022.07.18 (the cited 2022-05-18 revision not fetched) |
| Daylight diffuse coefficients | EnvironmentSensors.hpp:87; sensors.md:1235 | clear 0.8 + 15.5·√sin α, overcast 0.3 + 21·sin α klx; 127.5 klx, 0.033 | Robbins and Hunter, SERI/TR-254-1703 (1983), eqs. 2-16, 2-17, 3-6, quoting Gillette, Pierpoint and Treado (NBS 1982); J. IES 13(4) 1984 not read |
| Clear-sky extinction 0.21 | EnvironmentSensors.hpp:89 | 0.21 per air mass | LBL-17727, "Analysis of atmospheric turbidity for daylight calculations" (Lawrence Berkeley Laboratory, 1984): design illuminance extinction coefficients 0.21, 0.32, 0.53 for rural, urban, industrial |
| nanoSSOC-A60 | SunSensor.cpp:11; sensors.md:1248 | ±60°, < 0.5° (3σ); precision < 0.1° | Solar MEMS nanoSSOC-A60 product sheet (solar-mems.com, 2024); the technical specification NSA60-03-MAN not fetched |
| TFmini-S | Rangefinder.cpp:14–21 | 0.1–12 m, ±6 cm / ±1 %, 1 cm, 100 Hz | Benewake Datasheet SJ-PM-TFmini-S A01 (the ± is not defined as σ; 2σ is ours) |
| LightWare SF11/C | sensors.md:1183 | 0.2 to 100 m | LightWare SF11 product sheet (Acroname copy) |
| PMW3901 | OpticalFlow.cpp:16–20; sensors.md:730–733 | 42°, from 80 mm, 7.4 rad/s | PMW3901MB-TXQT Product Datasheet Version 1.00, 20 Mar 2017, table 3 |
| D435 geometry, depth unit | StereoCamera.cpp:15–26 | 848 × 480, 87° × 58°, 50 mm, 1 mm | RealSense D400 Series Datasheet (document 337029, March 2026), table 3-49 |
| D400 disparity 126, subpixel 0.08 | StereoCamera.cpp:24–25 | 126 px, 0.08 px | "Tuning depth cameras for best performance" (Intel RealSense white paper, web version): min-Z = f·b/126; RMS error plotted for subpixel 0.08 |
| Event camera threshold, mismatch | EventCamera.cpp:22–23 | 0.2, 0.03 ln | v2e `v2e_args.py` defaults (the v2e paper calls 0.3 typical) |
| Event camera background rate | EventCamera.cpp:25; sensors.md:770 | 0.1 Hz | Hu, Liu and Delbruck, v2e (CVPRW 2021, arXiv:2006.07722): leak events "typical rates ≈ 0.1 Hz" (v2e's CLI default is 0.01) |
| DW1000/DW3000 ranging | Uwb.cpp:22–24; sensors.md:1948 | 0.1 m (1σ) | DW1000 Datasheet Version 2.10 (2015), p. 1, "a precision of 10 cm"; DW3000 Datasheet Version 1.1 (2020), "an accuracy of 10 cm". Taken as 1σ, on the safe side (DW1000 figure 11 spreads about ±8 cm) |
| SPS30 size, range, interval | ParticleSensor.cpp:22–34 | 0.3–10 µm, 0–1000 µg/m³, 1 s, PM4/PM10 ±25 | SPS30 datasheet v2.0 table 1 |
| Oculus M750d | Sonar.cpp:48–56 | 0.1–120 m, 130° × 20°, 512 beams, 4 mm, 40 Hz | Blueprint Subsea Oculus M-series specifications page (no PDF or revision) |
| Ping2 beam | Sonar.cpp:57 | 25° | Blue Robotics store page and Ping Sonar Technical Guide |
| ADS-B rates, Q bit | AdsbReceiver.cpp:36–46 | 0.4–0.6 s, 25 ft | ICAO Annex 10 Vol. IV (see corrected) |
| Livox Mid-40 FOV, point rate | LidarLayout.hpp:48–51; sensors.md:471–472 | 38.4°, 100 000 points/s | Livox Mid Series User Manual v1.2 (2019.08), table 1.2.1 |
| VL53L0X field of view | Proximity.cpp:21; sensors.md:2045 | 25° | DocID029104 Rev 2, §5.1 |
| Magnetic guide sensor | sensors.md:1778 | TODO → about 1 mm over 16 sensors | Roboteq MGS1600GY datasheet v1.3 (2018), table 11: 160 mm, resolution 1 (max 2) mm, 100 Hz |
| Joint torque sensors | sensors.md:1536 | TODO → Franka: no figure; KUKA LBR iiwa: ±2 % of max torque | FR3 datasheet R02212 2.6 (July 2026); KUKA LBR iiwa spec sheet (third-party copy) |
| Wi-Fi scan dwell | WifiScanner.cpp:34–36; sensors.md:1889 | 2 s scan, "some 20–100 ms" per channel | Linux `net/mac80211/scan.c`: IEEE80211_PROBE_DELAY + IEEE80211_CHANNEL_TIME (HZ/33 each), PASSIVE HZ/9. The 802.11-2020 §11.1.4.3.2 clause number was not read (paywalled) and is dropped; the 2 s stays illustrative |

## Unverifiable (kept illustrative)

These are properties of the installation or the scene, or the maker publishes no figure. sensors.md
§9 now says so once, in its opening ("Defaults"), and per row where a source was looked for.

| Item | file:line | Value | Why |
|---|---|---|---|
| Livox prism speeds for the Mid-40 itself | LidarLayout.hpp:55 | (now the paper's prototype) | Livox does not publish them |
| PMW3901 flow noise, furthest ground | OpticalFlow.cpp:21–24; sensors.md:724, 726 | 0.05 rad/s, 30 m | the datasheet gives none ("80 mm to infinity"); no paper with a figure found |
| D435 max range | StereoCamera.cpp:18; sensors.md:813 | 10 m | datasheet "0.2 m to over 3 m (varies with lighting)"; white paper "~3m to 10m" on a white wall |
| UWB NLOS spread, max range | Uwb.cpp:28–31; sensors.md:1950–1951 | 0.3 m, 50 m | scene-dependent; DW1000 fig. 31 shows some 50–110 m at 6.8 Mb/s (read off a bar chart) |
| SPS30 lag | ParticleSensor.cpp:36; sensors.md:2000 | 8 s | the start-up time; Sensirion gives no step response |
| Lidar clutter calibration | WeatherEffects.hpp:28 | gain 0.11 etc. | kept `TODO(verify)`: needs chamber statistics (DENSE), a dataset task |
| Snow clutter factor | Sensors.cpp:896 | 3× rain | no published snow-to-rain clutter ratio (Rasshofer et al. 2011 give extinction only; Heinzler et al. 2019 has no snow data) |
| Rain "raining" threshold | RainSensor.cpp:18; sensors.md:1275 | 0.1 mm/h | Hydreon defines none (a rain event: first drop to 60 min after the last) |
| Light fog depth, sun-sensor threshold | sensors.md:1222, 1255 | 100 m, 1000 lx | scene choice; Solar MEMS gives no threshold |
| Retroreflector gain, specular lobe | ShapeCaster.hpp:89–90; sensors.md:357, 373 | 100, 2° | material-dependent orders of magnitude |
| Odometry covariances | KindTopics.cpp:51; sensors.md:649 | 1e-3, 1e6, 1e-2 | robot-dependent; diff_drive_controller's defaults are zeros and its description suggests 0.001/0.01 (now noted) |
| Power monitor shunt, divider | PowerMonitor.cpp:45–46; sensors.md:962, 972 | 2 mΩ, 1/11 | the board's |
| Load cell capacity | sensors.md:1057 | 500 N | the cell's |
| F/T noise of one step | ForceTorque.cpp:11 | 1 LSB 1σ | ATI quotes resolution after dropping four counts of noise, no σ |
| Bumper threshold | sensors.md:1362 | 5 N | the switch's |
| Cliff ranges, threshold | Cliff.cpp:16; sensors.md:1382–1383 | 5–150 mm, 60 mm | no part named (Sharp GP2Y0A51SK0F measures 2–15 cm) |
| Landing target decode limits | LandingTarget.cpp:20–22; sensors.md:1702–1704 | 12 px, 70°, 0.5 px | detector- and marker-dependent |
| Line sensor spacing, noise | LineSensor.cpp:17, 20; sensors.md:1759, 1762 | 10 mm, 0.02 | the array's (MGS1600GY: 1 mm resolution, cited) |
| Tag reader range, FOV, incidence, noise | TagReader.cpp:19–25; sensors.md:1822–1827 | 0.1 m, 0.8 rad, 45°, 0.5 mm, 0.005 rad | the reader's and tag's |
| Wi-Fi wall loss, scan time | WifiScanner.cpp:36, 41; sensors.md:1889, 1896 | 10 dB, 2 s | wall- and adapter-dependent |
| Proximity reach | Proximity.cpp:18 | 0.3 m | the skin's |
| Gripper thresholds | GripperSensor.cpp:25–26; sensors.md:2111, 2113 | 0.5 N, 5 mm/s | the gripper's |
| Sonar water, echo noise, detection floor | Sonar.cpp:64–69; sensors.md:2218, 2221–2222 | 10 °C, 35 PSU, pH 8; 1 dB; −180 dB | the site's; the receiver's |
| ADS-B reach, antenna height, GNSS noise | AdsbReceiver.cpp:39, 47, 49; sensors.md:2270–2273 | 100 km, 2 m, 3 m | the installation's and the sender's |
| V2X loss curve | V2x.cpp:37–38; sensors.md:2310–2311, 2326 | 300 m, 50 m | not fitted; Meireles et al. (VNC 2010) measured > 80 % at 400 m with line of sight, zero at 500 m obstructed |
| Vehicle CAN teeth, steering ratio, resolution, noise | VehicleCan.cpp:41–47; sensors.md:2354–2358 | 48, 15.5, 0.1°, 0.1°/s, 0.03 m/s² | the vehicle's |
| ADS-B DO-260B itself | AdsbReceiver.cpp | — | sold by RTCA; Annex 10 used instead |
| IEEE 802.11-2020 §11.1.4.3.2 | WifiScanner.cpp:35 | — | paywalled; replaced by the Linux source |
| J. IES 13(4) 1984, O'Connor 2004 | — | — | not fetched (paywall, 403); read through the reports and paper cited above |
| guide note | docs/guide/sensors.md:254 | "many … TODO(verify)" | reworded to point here |

## How it was checked

Every row was read in the document itself by a research pass that fetched each PDF or page and
quoted it; each value that changed was then read again in the downloaded text or page (all 14
corrected rows).
Tests: `colcon test --packages-select lagrange_sensors lagrange_ros2` in an overlay on 71880eb
(see the branch's report for counts).

# Tutorial: heat in the parts, and a robot's own interference

Two things a robot does to itself: its electronics heat the parts they sit on, and the current
its motors draw disturbs its compass. This tutorial runs both on the samples, reads them on
ROS 2 and in the viewer, and checks the heat against OpenFOAM. `tools/ci/tutorial_heat_and_emi.sh`
runs every step headless.

## 1. Where the heat goes

The sample `assets/robots/led_heatsink/led_heatsink.urdf` is two LEDs and a driver on a circuit
board, on a finned aluminium heat sink. Its URDF says what each part is made of and what heats
it:

```xml
<gazebo reference="board">
  <thermal_material preset="fr4" conductivity="20" conductivity_along="0.3" axis="0 0 1"
                    contact_conductance="2000"/>
  <heat_source name="led_left" power="2.0" efficiency="0.35" xyz="-0.012 0 0.0008"
               size="0.003 0.003 0.001"/>
</gazebo>
<gazebo reference="sink">
  <thermal_material preset="aluminium_6061" emissivity="0.85"/>
</gazebo>
```

Run it with the site's air temperature (there is no default) and look at it in the Thermal
shading, each part coloured cell by cell:

```bash
lagrange assets/robots/led_heatsink/led_heatsink.urdf --ambient-temperature 298.15 --rtf 20 \
    -- --shading thermal
```

From Python, headless:

```python
import lagrange

sim = lagrange.Simulation(
    "assets/robots/led_heatsink/led_heatsink.urdf", world=None, profile="profile.yaml"
)  # domains.thermal.ambient_temperature: 298.15
sim.step_until(600.0)
print(sim.read("thermal/led_left/temperature"), sim.read("thermal/sink/temperature"))
```

On ROS 2, `ros2 launch lagrange_example_heat heat.launch.py` publishes the LED's, the board's
hottest and the sink's temperatures as `sensor_msgs/Temperature`.

## 2. A thermometer on the LED, a pyrometer over the board

The sample plugin `lagrange_thermal_probe` gives `thermistor` and `ir_thermometer` sensors
(`<sensor type="thermistor">`, docs/physics/heat-conduction.md). Load it with
`--plugin lagrange_thermal_probe`; each writes `sensor/<name>/temperature`.

## 3. The same in OpenFOAM

```bash
lagrange foam-cht assets/robots/led_heatsink/led_heatsink.urdf --ambient 298.15 --radiation \
    --out led_case
```

It writes and runs a `chtMultiRegionSimpleFoam` case (in the OpenFOAM container when none is
installed) and prints each part's temperature. Its answer depends on the mesh. The air's boundary
layer needs fine cells: on this sink OpenFOAM matches Lagrange's correlations only at
`--cells-through 4` (2.5 million cells), and coarser meshes read it up to twice as hot. The
numbers are in docs/physics/heat-conduction.md.

## 4. The battery leads and the compass

```bash
lagrange_server --robot assets/robots/quad_x500/quad_x500.usda --world none --air-density 1.225 \
    --geo-origin 47.397742,8.545594,488 --emi-motor-leads 0.005,-0.06,0,-0.06
```

lays a lead from a battery under the frame to each motor, carrying its bus current, its return
5 mm above. The hovering quadcopter's magnetometer writes `sensor/magnetometer/heading_error`
and `interference`, the share ArduPilot's compassmot reports; the viewer's Environment workspace
slices |B| through it (XZ) and lists the effect under Overlays → Coupled into.
`ros2 launch lagrange_example_compass compass.launch.py` reads it on ROS 2
(docs/physics/emi-and-radio.md).

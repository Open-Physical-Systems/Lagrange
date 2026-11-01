# A wind tunnel

Air past a robot, as a mechanical engineer would look at it in CFD: drag and lift per link, then the flow in the viewer with streamlines and a slice coloured by speed, and the body coloured by its pressure coefficient (blue is suction, red is pressure). The viewport shows a colour scale for each.

```bash
ros2 launch lagrange_example_wind_tunnel tunnel.launch.py speed:=15 yaw:=20
ros2 launch lagrange_example_wind_tunnel tunnel.launch.py viewer:=false once:=true
```

Or for your own robot, no launch file: `lagrange tunnel robot.urdf --speed 20`. On a GPU it uses 4 million cells unless told, on the CPU 800,000. The sample sedan over a road, 2.5 million cells, takes 1.6 minutes on an RTX 4060 laptop GPU.

It writes `report.json`, `flow.vti` for ParaView, `flow.nvdb` to use as wind in a run (`--weather ...,wind_field=flow.nvdb`) and `flow_view.usda`, the world you see.

How far to trust it: the solver is lattice Boltzmann with an eddy viscosity, on a uniform grid, with no wall model and periodic sides, in free air, or over a rolling road with `--ground`. A shear wave decays at the exact rate, a cube reads Cd 1.03 against the measured 1.05, and a sphere at Re 20 to 50 reads about 25 % over the Schiller-Naumann curve on a 10-cell sphere. At a robot's Reynolds numbers it gives the size of the force and which shape or angle is better, not an engineering number.

On the example's default 150,000 cells the arms are a cell thick, which reads high (Cd 2.1 here); give more `cells` to see it fall.

What it shows: `lagrange tunnel`, the flow as a world, ParaView output.

# Tutorial: the sample arm with MoveIt 2

The v0.3 arm scenario (`docs/design/robot-classes.md` §4) on your machine: a 6-DoF arm in a
workcell, its motors on a 48 V bus, MoveIt 2 planning a pick-and-place cycle.

## 1. Start the workcell

```bash
ros2 launch lagrange_demos arm6_workcell.launch.py viewer:=true
```

The simulator loads `arm6.urdf` with `arm6.overlay.yaml` (six motors, the supply and a brake
resistor, thermal nodes), runs the controller manager at 1 kHz in lockstep, and spawns
`joint_state_broadcaster`, `arm_controller` and `gripper_controller`. The site's air temperature
is the launch argument `ambient` (default 298.15 K); thermal runs only with one.

## 2. Run the cycle

```bash
ros2 run lagrange_demos pick_place --cycles 3
```

MoveIt plans each move to the named poses of `demos/config/arm6_moveit/arm6.srdf` and executes it
through `arm_controller`. In the viewer's scopes, `motor/elbow_motor/current_q` and
`bus/dc_48v/voltage` follow the moves; decelerating lifts the bus above 48 V (the supply takes
nothing back; the brake resistor switches in above 56 V).

## 3. Heat it up

With motors that are too small for the duty (`motors:=undersized`) and the cycle repeated at
full speed, the shoulder's winding heats until its derating limits torque and the trajectory
controller aborts a move on its tolerance:

```bash
ros2 launch lagrange_demos arm6_workcell.launch.py motors:=undersized rtf:=8
ros2 run lagrange_demos pick_place --cycles 300   # exits 3 when a move fails
```

Watch `thermal/shoulder_lift_motor/temperature` and `/derating`. With
`electrical:=off` (the ideal torque source) the same cycles all complete.
`tools/ci/arm_scenario.sh` runs both and checks them.

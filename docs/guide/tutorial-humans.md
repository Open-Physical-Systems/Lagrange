# Tutorial: people around your robot

People who walk (RFC 0009): an articulated body of 14 rigid segments moved by a walk cycle, which
lidars, radars and cameras see limb by limb. They follow waypoints or wander, and steer round
robots, walls and each other. They are moved, not simulated: nothing balances.

`tools/ci/tutorial_humans.sh` runs these steps headless and checks them.

## 1. A crowd round the sample AMR

```bash
source install/setup.bash
lagrange people                                   # the AMR in the warehouse with 6 people
lagrange robot.urdf world.sdf --humans 10,radius=8,seed=3
```

`--humans N[,seed=S][,radius=M]` places N wandering people within `radius` of the spawn point,
clear of the robots, the scenery and each other.

## 2. One person at a time

```bash
lagrange empty                                    # or any running simulator
lagrange spawn assets/humans/adult.usda --at 3,0 --name ana     # stands
lagrange spawn assets/humans/walker.usda --name bo              # walks a 4 m square
lagrange delete /ana
```

A person is never the robot: in an empty run, the next robot you spawn is still the one ROS drives.
`/get_entities` lists people with the category `CATEGORY_HUMAN`.

## 3. Your own person

A person is a USD prim with `HumanAPI_1` (the prim's origin is on the ground, +x forward):

```usda
def Xform "guard" (
    prepend apiSchemas = ["HumanAPI_1"]
)
{
    double human:height = 1.82                       # m: every segment scales with it
    double human:walkSpeed = 1.1                     # m/s
    uniform token human:behavior = "waypoints"       # stand, waypoints or wander
    double[] human:waypoints = [8, 0, 8, 6, 0, 6]    # x, y per waypoint, world ENU, m
    int human:loop = 1
    double human:dwell = 2                           # s at each waypoint
}
```

Put it in a world file, or spawn it. SDFormat worlds work too: a Gazebo `<actor>`'s `<script>`
trajectory becomes waypoints walked at its mean speed; its skin and animation files are ignored.

## 4. Did the robot get too close?

Each person has `human/<name>/nearest_robot` (m, centre to centre), with `pose`, `speed` and
`gait_phase`, in the viewer's Signals workspace and in recordings. They reach ROS through
`--ros-topics` ([The `lagrange` command](cli.md#plugin-outputs-on-ros-2)), and a goal comes back
the same way, as x, y and active (above 0.5 the person goes there, then carries on):

```yaml
publish:
  - {signal: human/ana/nearest_robot, topic: /ana/nearest_robot, type: std_msgs/msg/Float64}
subscribe:
  - {topic: /ana/goal, type: std_msgs/msg/Float64MultiArray, signal: human/ana/goal, size: 3}
```

## 5. Turning them off

`--domain humans=off`, or `humans: {enabled: false, substitute: zero}` in a profile, hides everyone
for a baseline run.

## What they do not do

- Contact is one-way: a person pushes a robot and a robot cannot push a person, nor do people fall
  or react to being hit.
- No route planning: a person heads straight for the next waypoint and steers locally, so a route
  round a wall needs waypoints round it. A person blocked for 8 s skips to the next target.
- The body follows the ground under it; there are no stairs and feet do not adapt to slopes.
- People and traffic agents do not see each other.
- The look is a capsule mannequin.
- Lidar reflectivity and radar cross-section are the defaults, and a radar's view of a person is
  not validated.
- Body proportions are from memory of published tables and marked `TODO(verify)` in
  `modules/humans/include/lagrange/humans/Body.hpp`.
- A spawn or delete rebuilds the run: people keep their place, and start again on their route.
- Cost, measured on the development machine: 50 wandering people and the sample AMR at 5.3 times
  real time, headless. The viewer's cost is not included.

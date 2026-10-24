# Writing a module

This chapter writes a small module, a wheel odometer for a differential-drive robot, and runs it
next to the MuJoCo physics on the sample AMR. The complete code is in
[`tests/tutorial/`](../../tests/tutorial/) (package `lagrange_tutorial`). Its tests keep this
page true.

Read [Concepts](concepts.md) first for signals, units and the scheduler.

> **v0.2 limitation.** Modules are compiled into the program that runs them. Loading your module
> into `lagrange_server` as a plugin needs the versioned C ABI (`core/plugin_abi/`), which comes
> with a later release. Until then, you build a run in your own program, as below.

## 1. The interface

A module implements `lagrange::IModule` (`core/module/include/lagrange/module/IModule.hpp`):

| Method | When | Do |
|---|---|---|
| `info()` | any time | domain, name, version, whether it is a substitute, deterministic, minimum tier |
| `preferredStep()` | before wiring | its step size; a profile's `dt` can override it |
| `declare(world, config, declarer)` | once, before wiring | look at the world; declare every input and output |
| `init(world, config, bus)` | once, after wiring succeeded | allocate state; write initial outputs |
| `step(context, bus)` | every step | read inputs, compute, write outputs. No allocation, no I/O, no lookups by name |
| `saveState` / `restoreState` | on save, reset, replay | the complete dynamic state, for deterministic runs |

Modules never throw. They return a `Status`, and the scheduler reports failures with the
module's name.

## 2. The odometer

The odometer reads the two wheel speeds and integrates them:

- distance travelled: `d += |v_left + v_right| / 2 · dt`
- heading: `θ += (v_right − v_left) / track · dt`

Here `v = ω · r` is each wheel's rim speed. It has two inputs and two outputs:

| Signal | Direction | Unit |
|---|---|---|
| `joint/left_wheel_joint/velocity`, `joint/right_wheel_joint/velocity` | in, written by MuJoCo every 1 ms | rad/s |
| `base/odometer/distance` | out | m |
| `base/odometer/heading` | out | rad |

The header ([`Odometer.hpp`](../../tests/tutorial/include/lagrange/tutorial/Odometer.hpp)) holds
the settings (joint names, wheel radius, track width), the four `SignalId`s and the state:

```cpp
class Odometer final : public IModule {
public:
    struct Settings {
        std::string leftJoint{"left_wheel_joint"};
        std::string rightJoint{"right_wheel_joint"};
        double wheelRadiusM{0.10};
        double trackWidthM{0.46};
    };
    // ... the IModule methods ...
private:
    Settings settings_;
    SignalId left_, right_, distance_, heading_;
    double distanceM_{0.0};
    double headingRad_{0.0};
};
```

### Declare

Declaring is where units and the timing of inputs are set
([`Odometer.cpp`](../../tests/tutorial/src/Odometer.cpp)):

```cpp
Status Odometer::declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& declarer) {
    // Fail early, with a reason, when the robot has no such joints.
    const auto& joints = world.joints().names;
    for (const std::string& name : {settings_.leftJoint, settings_.rightJoint}) {
        if (std::find(joints.begin(), joints.end(), name) == joints.end()) {
            return Status::error(StatusCode::InvalidConfig, "odometer: no joint " + name);
        }
    }
    const auto input = [&](const std::string& joint) {
        SignalSpec spec;
        spec.name = "joint/" + joint + "/velocity";
        spec.unit = units::rad_per_s;  // checked against the writer's unit at wiring
        spec.hold = Hold::WindowAverage;
        return declarer.input(spec);
    };
    left_ = input(settings_.leftJoint);
    right_ = input(settings_.rightJoint);
    SignalSpec distance;
    distance.name = "base/odometer/distance";
    distance.unit = units::m;
    distance_ = declarer.output(distance);
    // ... heading, in units::rad ...
    return {};
}
```

The odometer steps every 10 ms (`preferredStep()` returns `10_ms`), and physics writes the wheel
speeds every 1 ms. `Hold::WindowAverage` hands the odometer the mean of the ten values committed
since its previous step, so the integral misses nothing in between. The default,
`Hold::ZeroOrder`, would give only the latest value.

### Step

```cpp
Status Odometer::step(const StepContext& context, SignalBus& bus) {
    const double left = bus.read(left_) * settings_.wheelRadiusM;
    const double right = bus.read(right_) * settings_.wheelRadiusM;
    const double dt = context.dt.seconds();
    distanceM_ += std::abs(left + right) / 2.0 * dt;
    headingRad_ += (right - left) / settings_.trackWidthM * dt;
    bus.write(distance_, distanceM_);
    bus.write(heading_, headingRad_);
    return {};
}
```

`saveState` writes `distanceM_` and `headingRad_`; `restoreState` reads them back. With that,
the module is deterministic, and `info()` says so.

## 3. Run it

[`odometer_demo.cpp`](../../tests/tutorial/src/odometer_demo.cpp) builds a run the way
`lagrange_server` does:

```cpp
auto robot = server::loadUrdf(asset("robots/amr_diffdrive/amr_diffdrive.urdf"));
server::addUrdf(*robot, asset("worlds/warehouse_small/warehouse_small.urdf"));

Scheduler scheduler(SchedulerOptions{.deterministic = true});
scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
scheduler.add(std::make_unique<server::DriveCommand>(server::DriveCommand::Settings{}, nullptr));
scheduler.add(std::make_unique<server::DiffDriveController>(drive));
scheduler.add(std::make_unique<tutorial::Odometer>(tutorial::Odometer::Settings{}));

auto wired = scheduler.wire(WorldView(robot->world));   // names, units, order
scheduler.stepUntil(SimTime{10'000'000'000});           // 10 s
```

`DriveCommand` without an external source plays the demo figure-eight. Build and run:

```console
$ colcon build --packages-up-to lagrange_tutorial && source install/setup.bash
$ ros2 run lagrange_tutorial odometer_demo 10
 t (s)     odom (m)   odom (rad)   true (rad)
     1        0.142        0.061        0.061
     2        0.590        0.397        0.395
     3        1.195        0.900        0.887
     4        1.795        1.400        1.378
     5        2.395        1.900        1.868
     6        2.995        2.399        2.357
     7        3.595        2.899        2.846
     8        4.194       -2.884       -2.946
     9        4.795       -2.384       -2.457
    10        5.395       -1.883       -1.966
```

The last column is the true heading change from MuJoCo's pose of `base_link`. The odometer's
heading runs about 2 % ahead of it: the simulated robot turns a little less than ideal
kinematics on the nominal 0.46 m track predict. A real robot shows the same kind of
systematic error, and real odometers are calibrated for it. The distance agrees with the true
path length within 2 % (`test_odometer` checks both).

## 4. Get a unit wrong

Suppose another module reads the wheel speed as a linear velocity:

```cpp
spec.name = "joint/left_wheel_joint/velocity";
spec.unit = units::m_per_s;  // the writer says rad/s
```

The run does not start:

```console
$ ros2 run lagrange_tutorial odometer_demo --wrong-unit
the simulation could not start:
  UnitMismatch joint/left_wheel_joint/velocity: writer rigid_body/mujoco: rad/s (AngularVelocity); reader sensors/wrong_unit: m/s (LinearVelocity)
```

The scheduler never converts. Fix the declaration, or convert in your module with a factor you
name and document.

## 5. Test it

`tests/tutorial/test/test_odometer.cpp` shows the pattern for a module test: wire the module
with the real physics on the sample robot, run, and compare with ground truth from the bus. It
also checks the wiring errors (a wrong unit, a missing joint).

```bash
colcon test --packages-select lagrange_tutorial && colcon test-result --verbose
```

## Checklist

- Every input and output has a unit. Use `units::` constants; never a bare number.
- Choose each input's hold: `ZeroOrder` for states, `WindowAverage` for quantities you integrate.
- A loop between two modules needs one input marked `feedback = true`.
- `step()` allocates nothing and looks nothing up by name; keep the `SignalId`s from `declare()`.
- Name physical constants and give their units and source.
- If `info()` says deterministic, save and restore the complete state, and use
  `lagrange::random` (Philox, seeded by `StepContext::seed`) for noise.
- A new physical model ships with a test against a reference, as here against MuJoCo's ground
  truth.

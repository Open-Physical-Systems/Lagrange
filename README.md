<p align="center">
  <img src="docs/imgs/repo-logo/lagrange-lockup@2x.png" alt="Lagrange" width="480">
</p>

# Lagrange

Lagrange is an open-source (Apache-2.0) robot simulator for ROS 2 Lyrical Luth on Ubuntu 26.04.
Its aim is multi-physics that can be switched on and off: rigid-body motion, electrical, thermal,
aerodynamics, weather, EMI and sensors are separate domains on a unit-checked signal bus, and any
of them can be disabled or replaced by ideal, constant or replayed data.

**Status: early development, nothing usable yet.** The current phase is v0.1: a native Vulkan
viewer, the viewer protocol, and a mock/replay server. There is no physics yet.

## Where to read

| Topic | File |
|---|---|
| Vision, scope and release metrics | [docs/plan/00-overview.md](docs/plan/00-overview.md) |
| Roadmap and v0.1/v0.2 issues | [docs/plan/roadmap.md](docs/plan/roadmap.md) |
| Architecture | [docs/design/architecture.md](docs/design/architecture.md) |
| Formats, USD schemas, profiles | [docs/design/formats.md](docs/design/formats.md) |
| Viewer and protocol | [docs/design/viewer.md](docs/design/viewer.md), [docs/design/protocol.md](docs/design/protocol.md) |
| Design proposals | [rfcs/](rfcs/) |
| Decisions and open questions | [docs/plan/open-questions.md](docs/plan/open-questions.md) |

## Build

Requirements: Ubuntu 26.04 with ROS 2 Lyrical installed.

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p ~/lagrange_ws/src && cd ~/lagrange_ws
git clone -b dev https://github.com/Open-Physical-Systems/Lagrange.git src/lagrange

# One dependency (the Python MCAP library) is not packaged for Ubuntu. Register the project's
# extra rosdep keys once, then let rosdep install it with pip.
echo "yaml file://$PWD/src/lagrange/packaging/rosdep/lagrange.yaml" | \
  sudo tee /etc/ros/rosdep/sources.list.d/10-lagrange.list
rosdep update
PIP_BREAK_SYSTEM_PACKAGES=1 PIP_CONSTRAINT=$PWD/src/lagrange/packaging/pip-constraints.txt \
  rosdep install --from-paths src --ignore-src -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test && colcon test-result --verbose
```

If you prefer not to install pip packages into the system Python, create a virtual environment
with `--system-site-packages`, install `mcap` there, and pass
`-DPython3_EXECUTABLE=<venv>/bin/python` to `colcon build`.

Packages so far:

| Package | Directory | Content |
|---|---|---|
| `lagrange_cmake` | `cmake/` | Shared build options |
| `lagrange_core` | `core/` | Simulation time types |
| `lagrange_proto` | `proto/` | Viewer protocol messages, framing, socket transport, shared memory |
| `lagrange_mock_server` | `mock_server/` | Replay server that speaks the protocol from a recording |

## Try the mock server

There is no viewer yet, but the server and its protocol check already run:

```bash
source install/setup.bash
ros2 run lagrange_mock_server lagrange_mock_server --sample --listen tcp://127.0.0.1:47610 &
ros2 run lagrange_mock_server lagrange_mock_server --check tcp://127.0.0.1:47610
```

`--sample` serves a built-in synthetic recording (an AMR on a figure-eight path; computed, not
simulated). `--write-sample file.mcap` writes it to disk and `--recording file.mcap` serves a
file.

## Contributing

Development happens on the `dev` branch; `main` holds releases only. See
[CONTRIBUTING.md](CONTRIBUTING.md).

## License

Apache-2.0. See [LICENSE](LICENSE).

Lagrange is a project of Open Physical Systems.

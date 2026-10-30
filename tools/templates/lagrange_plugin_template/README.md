# Lagrange plugin template

A plugin package to copy (RFC 0008; the guide is `docs/guide/writing-a-plugin.md`):

```bash
cp -r tools/templates/lagrange_plugin_template ~/ws/src/my_lagrange_plugin
cd ~/ws && colcon build && colcon test --packages-select my_lagrange_plugin
source install/setup.bash
lagrange --plugin my_lagrange_plugin          # found through the lagrange_plugins index
```

Rename the package (`package.xml`, `project()` in `CMakeLists.txt`), the module's domain and
name in `src/my_sensor.c`, and the signals. `tools/ci/check_plugin_template.sh` builds and tests
this template in a scratch workspace.

# A simulated camera

The sample sedan in the city block. A node reads its front camera on ROS 2: colour, depth in metres, and for each pixel the scene prim it shows, as a vision model is trained on.

```bash
ros2 launch lagrange_example_camera camera.launch.py
ros2 launch lagrange_example_camera camera.launch.py viewer:=false once:=true
```

Where to look: `launch/camera.launch.py`, `scripts/read_camera.py`.

What it shows: `/<name>/image_raw` (rgb8), `/<name>/depth/image_raw` (32FC1), `/<name>/instance/image_raw` (32SC1, -1 for none). Cameras need a GPU or a software Vulkan device.

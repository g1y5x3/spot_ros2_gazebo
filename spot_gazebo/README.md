# spot_gazebo

Gazebo Fortress worlds, plugins, and simulation-only sensor adapters for Spot.

## Components

### Initial joint-position system

`spot_initial_joint_position_system` initializes configured joints before the
first physics step. It is used by worlds that start Spot in a seated posture.

### Thermal colormap

`thermal_colormap` subscribes to the Gazebo thermal image, normalizes each
16-bit frame, applies OpenCV's Inferno colormap, and publishes an RViz-friendly
BGR image.

```text
/spot/camera/thermal_camera -> /spot/camera/thermal/colormap
```

Colors are for visualization and do not represent visible-light RGB values.
Because normalization is per frame, colors are relative to each frame rather
than calibrated absolute temperatures.

### Gazebo Velodyne point-cloud adapter

`gazebo_velodyne_pointcloud_adapter` adds missing `intensity`, `ring`, and
per-point `time` fields to Gazebo lidar clouds, then optionally transforms the
cloud into a target frame. Synthetic ring and timing values approximate a
rotating Velodyne sensor; they are intended for simulation integration tests,
not sensor-fidelity validation.

The adapter is disabled by default in `spot_bringup`. Enable it when testing a
Velodyne-compatible consumer:

```bash
ros2 launch spot_bringup spot.gazebo.launch.py velodyne_adapter:=true
```

## Worlds

Production worlds are installed from `worlds/`: `empty_room.sdf`,
`simple_tunnel.sdf`, `edgar_mine.sdf`, and `electrical_substation.sdf`.
Controller launches use `empty_room.sdf` by default.

When `BUILD_TESTING` is enabled, the isolated zero-gravity effort fixture is
installed at `worlds/test/effort_smoke_test.sdf`. Run it with:

```bash
ros2 launch spot_bringup spot.gazebo.launch.py \
  world_file:=test/effort_smoke_test.sdf headless:=true
ros2 run spot_gazebo gazebo_effort_smoke_test
```

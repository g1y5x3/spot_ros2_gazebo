# Spot OCS2 Torque Control in Gazebo Fortress

This workspace provides a conservative torque-control stack for the Spot
simulation on Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress. Upstream OCS2
SQP supplies a centroidal legged-robot policy; a whole-body QP converts that
policy into 12 bounded joint torques. The original CHAMP simulation remains
available as a separate legacy path, but it must never run at the same time as
the effort controller.

The implementation is simulation-only. It uses Gazebo ground-truth odometry
and is intended as a clear foundation for future estimator and hardware
interfaces, not as a real-robot controller.

## Packages

- `spot_effort_controller`: transport-independent standing control and the
  sole ROS-to-Gazebo effort backend.
- `spot_state_estimator`: replaceable 24-state OCS2 adapter.
- `spot_ocs2_mpc`: Spot model configuration, live-state/policy bridge,
  `/cmd_vel` reference generator, and gait manager.
- `spot_wbc`: asynchronous whole-body QP, safe fallback, integrated launches,
  and locomotion smoke test.
- `spot_bringup`, `spot_description`, and `spot_gazebo`: Fortress launch,
  model, sensors, contact bridges, and worlds.
- `champ`, `champ_base`, `champ_config`, and `champ_msgs`: retained legacy
  position-control path.

## Reproducible environment and build

Requirements are Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, `ros_gz`,
Pinocchio, HPIPM, and the build tools declared by the repository `Dockerfile`.
The default Compose service is headless and does not require privileged mode,
host devices, GUI sockets, or host credentials.

```bash
docker compose build ros-humble-dev
docker compose run --rm ros-humble-dev
```

The exact upstream OCS2 revision is pinned in
`dependencies/ocs2_humble.repos`. The Spot stack supplies its own URDF and
configuration, so it does not import `ocs2_robotic_assets`. The import script
applies the documented Humble/Spot compatibility patches, including a narrow
patch that removes that assets dependency and omits only the upstream
ANYmal-specific tests and launch files. The unrelated perceptive-ANYmal
examples are also excluded, so their `elevation_mapping_cupy`
plane-segmentation dependency is not required:

```bash
src/spot_gazebo_ros2/dependencies/import_ocs2.sh
```

This creates the ignored, generated checkout at
`src/spot_gazebo_ros2/third_party/ocs2`. Then build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
mapfile -t selected_paths < <(
  colcon list --packages-up-to spot_wbc | awk '{print $2}'
)
rosdep check --from-paths "${selected_paths[@]}" \
  --ignore-src --rosdistro humble
colcon build --packages-up-to spot_wbc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The selected upstream executable is
`ocs2_legged_robot_ros/legged_robot_sqp_mpc`; no solver is reimplemented in
the Spot packages. The remaining OCS2 libraries are still required for model
dynamics, optimization, ROS policy transport, and the legged-robot MPC node.

## Launch

Start the full stack without a GUI:

```bash
source install/setup.bash
ros2 launch spot_wbc spot.wbc.launch.py headless:=true
```

The launch starts the bridge and controllers before Gazebo, initializes OCS2
from live state, stands in four-foot contact at zero command, and selects the
slow crawl only while a valid nonzero command is present.

The indoor room is also normalized to the controller's world-frame height
convention and can be selected directly:

```bash
ros2 launch spot_wbc spot.wbc.launch.py \
  world_file:=empty_room.sdf headless:=false
```

In `empty_room.sdf`, Spot starts in a symmetric seated crouch at 0.39 m body
height. The fixed-base gravity/PD controller raises it smoothly over five
simulated seconds. WBC policy corrections are deliberately held out of the
torque command during that transition and activate only after the nominal
standing posture has been reached. Wait for the rise to finish before sending
a keyboard command. Other worlds retain their own configured initial posture.

The validated command limits deliberately produce very small, slow crawl
steps. Use the bounded defaults first; tune the limits only after repeating
the standing and per-axis smoke tests.

The OCS2-independent standing controller is available as a separate safety
and actuator check:

```bash
ros2 launch spot_bringup spot.standing.launch.py headless:=true
```

For keyboard operation, launch the visible simulation:

```bash
ros2 launch spot_wbc spot.wbc.launch.py headless:=false
```

In a second sourced terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

`i`/`,` request forward/backward motion, `j`/`l` request positive/negative
yaw, and `J`/`L` request lateral motion in the standard
`teleop_twist_keyboard` layout. The reference generator clamps requests to
0.05 m/s forward, 0.03 m/s lateral, and 0.10 rad/s yaw. Release the keys and
the 0.5 simulated-second timeout smoothly returns velocity to zero and selects
four-foot stance.

The launch argument `stationary_gait:=conservative_trot` selects the diagonal
trot-in-place schedule at zero command. The defaults are
`stationary_gait:=stance` and `moving_gait:=conservative_crawl`; the
one-leg-at-a-time crawl was used for the verified directional tests.

## Automated verification

Run unit and package integration tests:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select \
  spot_effort_controller spot_state_estimator spot_ocs2_mpc spot_wbc
colcon test-result --verbose
```

With the headless full-stack launch running, verify 30 seconds of standing:

```bash
ros2 run spot_effort_controller standing_smoke_test \
  --ros-args -p use_sim_time:=true -p require_wbc_policy:=true
```

Test each axis independently. For the most repeatable evidence, restart the
headless launch before each command:

```bash
ros2 run spot_wbc locomotion_smoke_test \
  --ros-args -p use_sim_time:=true -p axis:=forward
ros2 run spot_wbc locomotion_smoke_test \
  --ros-args -p use_sim_time:=true -p axis:=lateral
ros2 run spot_wbc locomotion_smoke_test \
  --ros-args -p use_sim_time:=true -p axis:=yaw
ros2 run spot_wbc locomotion_smoke_test \
  --ros-args -p use_sim_time:=true -p axis:=combined
```

The locomotion test waits for the controller to settle, applies one bounded
command for six simulated seconds, publishes zero for three seconds, and
fails on insufficient signed motion, low body height, excessive tilt,
non-finite state, missing diagnostics, or an unsafe controller state.

To verify the direct effort path and joint signs, launch
`effort_smoke_test.sdf` with `champ:=false`, then run
`spot_bringup/effort_smoke_test` once per joint. It accepts at most 5 Nm for at
most one second and refuses to run if another ROS publisher owns the command
topic. The full 12-joint command loop is recorded in
`IMPLEMENTATION_STATUS.md`.

## Architecture and command ownership

The control path is:

```text
Gazebo sensors
  -> spot_state_estimator (/spot/ocs2_state)
  -> OCS2 SQP policy + /cmd_vel target
  -> spot_wbc (/spot/effort_command)
  -> effort backend (/spot/joint_trajectory)
  -> Fortress JointTrajectoryController effort array
```

The backend is the only publisher to `/spot/joint_trajectory`. It emits one
effort for each joint in FL/FR/RL/RR order, with position and velocity arrays
empty. Fortress position/velocity PID gains are zero, CHAMP is disabled in
the OCS2 launches, torques are clamped to ±60 Nm and rate-limited, and a
0.1-second command watchdog ramps output to zero.

The WBC torque callback runs at 200 Hz. QP solve work runs in one background
worker at 10 Hz in four-foot stance and 50 Hz in swing modes. The cached value
is only the bounded policy correction, so live posture feedback continues at
200 Hz between QP updates. A stale policy, infeasible QP, invalid input, or
startup state selects the known-safe fixed-base gravity-plus-PD fallback. The
same fallback exclusively owns the configured five-second startup posture
transition before WBC policy corrections are enabled.

## Frames and ordering

- `world` is the Gazebo inertial frame. Odometry pose is in `world`; its twist
  is expressed in the base child frame.
- `base_link` is the Pinocchio free-flyer root.
- `/cmd_vel` linear x/y is body-frame forward/left and angular z is positive
  counter-clockwise yaw. The reference node rotates planar velocity into
  `world` with current estimated yaw.
- OCS2 base orientation order is yaw, pitch, roll (Z/Y/X).
- Contacts are `front_left_ee`, `front_right_ee`, `rear_left_ee`,
  `rear_right_ee`.
- State order is normalized centroidal momentum (6), base XYZ (3), base
  yaw/pitch/roll (3), then 12 joint positions in FL/FR/RL/RR order.

## Tuning

- OCS2 dynamics, costs, constraints, swing trajectory, horizon, and initial
  state: `spot_ocs2_mpc/config/task.info`.
- Gait timings and mode sequences: `spot_ocs2_mpc/config/gait.info` and
  `spot_ocs2_mpc/spot_ocs2_mpc/gait.py`.
- Estimator timing: `spot_state_estimator/config/state_adapter.yaml`.
- Speed, acceleration, timeout, nominal height, reference horizon, default
  posture, and gait selection:
  `spot_ocs2_mpc/config/controller.yaml`.
- WBC rates, weights, friction, force/torque bounds, feedback gains, solver
  tolerance, and fallback gains: `spot_wbc/config/wbc.yaml`.
- Backend torque/rate bounds, watchdog, and standalone standing gains:
  `spot_effort_controller/config/effort_controller.yaml`.

Keep the backend and WBC torque limits consistent. Re-run standing, then
forward, lateral, and yaw smoke tests after changing any controller, gait, or
model parameter.

## Known limitations

- State estimation depends on simulation ground-truth odometry; it is not
  suitable for real hardware.
- The validated locomotion envelope is intentionally small. High-speed,
  aggressive, uneven-terrain, perception-based, and learned control are out
  of scope.
- The diagonal conservative trot-in-place is implemented, selectable, and
  verified at low speed, but the one-leg-at-a-time crawl remains the default
  directional gait because it is more robust in this model.
- Directional axis evidence uses a fresh simulator launch per axis. A combined
  pulse also passed after a 30-second policy-driven standing dwell, but long
  sequences of repeated commands are outside the validated envelope.
- The Python/SciPy WBC is not a hard-real-time implementation. It relies on an
  asynchronous worker and explicit fallback behavior.
- Measured contacts are exposed for future estimator integration, while the
  current WBC contact constraints follow the planned OCS2 schedule.
- On Ctrl-C, the Fortress server may be reported as exit code `-2`; that is
  normal SIGINT termination rather than a controller failure.

Detailed milestone commands, measurements, design decisions, and residual
limitations are recorded in `IMPLEMENTATION_STATUS.md`.

## Legacy CHAMP launch

The pre-existing CHAMP publisher can still be launched for graph and
diagnostic compatibility:

```bash
ros2 launch spot_bringup spot.gazebo.launch.py rviz:=false champ:=true
```

The current model has zero position/velocity PID gains for verified
effort-only ownership, so this legacy path is not a working locomotion
controller without restoring a mutually exclusive position-control model.
Do not start it while `spot.wbc.launch.py`, the standalone standing
controller, or any other effort publisher is active.

## Acknowledgements

- [CHAMP](https://github.com/chvmp/champ/tree/ros2)
- [spot_config](https://github.com/chvmp/robots/tree/master/configs/spot_config)
- [spot_description](https://github.com/clearpathrobotics/spot_ros)

# OCS2 Spot Implementation Status

Last updated: 2026-07-27 (UTC)

This file is the milestone log required by `OCS2_SPOT_GOAL.md`. A milestone is
marked complete only when its stated completion evidence has been collected.
Later milestones are not treated as started until the preceding gate passes.

## Milestone summary

| Milestone | Status | Gate |
|---|---|---|
| 0. Baseline and architecture inventory | Complete with documented pre-existing baseline failure | Passed |
| 1. Reproducible Humble/Fortress environment | Complete | Passed |
| 2. Fortress simulation and torque path | Complete | Passed |
| 3. Effort interface and safe standing | Complete | Passed |
| 4. State adapter | Complete | Passed |
| 5. Minimal Spot OCS2 model | Complete | Passed |
| 6. `cmd_vel` reference generation | Complete | Passed |
| 7. Whole-body controller | Complete | Passed |
| 8. Locomotion and integration | Complete | Passed |

## Definition-of-done evidence map

| Requirement | Current evidence |
|---|---|
| Documented container build | Ubuntu 22.04/Humble/Fortress Dockerfile and safe Compose commands are in the README; pinned OCS2 clean-source build passed. |
| Automated tests | 33 test results pass with zero failures, errors, or skips. |
| Headless Gazebo | `ocs2_test.sdf`, all required controllers, bridges, sensors, and live-state MPC launch successfully. |
| Torque standing | Standalone and policy-driven WBC standing each pass 30 simulated seconds. |
| Asynchronous valid OCS2 policy | Upstream SQP publishes finite 24-state/24-input policies at 10 Hz without running in the torque callback. |
| WBC conversion | The constrained 42-variable QP produces 12 bounded torque commands; live mode-15 QP activity is required by the standing smoke gate. |
| `/cmd_vel` motion | Fresh forward, lateral, and yaw tests pass in the commanded directions; a combined pulse also passes. |
| Safe failures | Backend command watchdog reaches exact zero; stale MPC and rejected/infeasible QPs select bounded standing/degraded fallback. |
| Exclusive command ownership | Legacy position controllers are absent, SDF PID gains are zero, one effort plugin remains, and runtime publisher counts are one at each boundary. |
| Documentation | README and this file cover build, launch, architecture, frames, operator commands, tuning, assumptions, evidence, and limitations. |

## Milestone 0: Baseline and architecture inventory

### Platform inventory

- Container OS: Ubuntu 22.04.5 LTS.
- ROS: ROS 2 Humble (`ROS_DISTRO=humble`).
- Simulator CLI: `ign gazebo 6.18.0`, the Fortress generation. The `gz`
  command is not installed, which is expected for the Fortress naming era.
- ROS/Gazebo packages observed in the container:
  `ros-humble-ros-gz-sim=0.244.24-1jammy.20260422.085839` and
  `ros-humble-ros-gz-bridge=0.244.24-1jammy.20260422.064121`.
- Pinocchio observed in the container:
  `ros-humble-pinocchio=4.0.0-2jammy.20260606.100000`.

### Baseline source and build result

The imported `spot_gazebo_ros2` tree contains a pre-existing zero-length
`COLCON_IGNORE`, so normal workspace discovery returns no Spot packages.
Its `.git` file also refers to missing parent submodule metadata. To preserve
the unchanged tree while obtaining a real baseline result, the source was
copied to `/tmp/spot-baseline.UhkESl/src` and only the copied
`COLCON_IGNORE` was removed.

Commands:

```bash
baseline_tmp=$(mktemp -d /tmp/spot-baseline.XXXXXX)
cp -a src/spot_gazebo_ros2 "$baseline_tmp/src"
rm "$baseline_tmp/src/COLCON_IGNORE"
colcon list --base-paths "$baseline_tmp/src"
colcon --log-base "$baseline_tmp/log-all" build \
  --base-paths "$baseline_tmp/src" \
  --build-base "$baseline_tmp/build-all" \
  --install-base "$baseline_tmp/install-all" \
  --event-handlers console_cohesion+ \
  --continue-on-error
```

Observed result on 2026-07-26:

- Seven packages built: four legacy controller packages plus
  `spot_description`, `spot_gazebo`, and `spot_navigation`.
- `spot_bringup` failed during install because its `CMakeLists.txt` installs a
  nonexistent `spot_bringup/resource` directory:

  ```text
  file INSTALL cannot find
  "/tmp/spot-baseline.UhkESl/src/spot_bringup/resource": No such file or directory.
  ```

- Because the required bringup package did not install, the unchanged baseline
  launch could not be run. This is the exact pre-existing baseline failure for
  the Milestone 0 launch evidence.

### Existing simulation and legacy command path

- The baseline `spot_bringup/launch/spot.gazebo.launch.py` included
  `ros_gz_sim/launch/gz_sim.launch.py`, launched `ros_gz_bridge` and
  `robot_state_publisher`, and started the legacy
  `quadruped_controller_node`.
- `/cmd_vel` was remapped to the legacy controller's smoothed subscription.
- That controller published a 12-position, one-point
  `trajectory_msgs/msg/JointTrajectory` at its controller topic,
  `/spot/joint_trajectory`.
- `spot_bridge.yaml` bridges that topic to
  `/model/spot/joint_trajectory` as
  `ignition.msgs.JointTrajectory`.
- The SDF uses the single
  `ignition::gazebo::systems::JointTrajectoryController` plugin for all 12
  actuated joints. Its existing position gains are P=500, I=1, D=20.
- Physics uses a 1 ms maximum step. Joint states publish at 200 Hz, the IMU at
  100 Hz, and ground-truth odometry at 10 Hz.
- ROS bridges exist for `/clock`, `/spot/joint_states`,
  `/spot/odometry`, `/spot/imu`, lidar, and cameras. Contact sensors exist in
  the lower-leg links but their default Gazebo topics are not bridged.

### Authoritative ordering and frames

The authoritative actuator order is the order configured in the SDF
`JointTrajectoryController`, grouped front-left, front-right, rear-left,
rear-right:

1. `front_left_hip_x`
2. `front_left_hip_y`
3. `front_left_knee`
4. `front_right_hip_x`
5. `front_right_hip_y`
6. `front_right_knee`
7. `rear_left_hip_x`
8. `rear_left_hip_y`
9. `rear_left_knee`
10. `rear_right_hip_x`
11. `rear_right_hip_y`
12. `rear_right_knee`

The authoritative foot/contact frame order follows the same leg order:

1. `front_left_ee`
2. `front_right_ee`
3. `rear_left_ee`
4. `rear_right_ee`

The SDF defines each hip-x axis as +X and each hip-y and knee axis as +Y.
Every actuated SDF joint has an 80 Nm effort limit and 15 rad/s velocity
limit. Dynamic sign behavior still requires the bounded single-joint tests in
Milestones 2 and 3.

### Package-level implementation plan

Keep the Spot implementation separate from upstream solver sources:

- `spot_description`: preserve model geometry, inertia, collisions, and
  sensors; configure only the verified effort backend and required contact
  publishing.
- `spot_bringup`: provide mutually exclusive baseline, effort smoke-test,
  standing, and OCS2 controller launch paths plus bridge configuration.
- `spot_effort_controller`: new Spot-owned package containing the authoritative
  joint map, transport-independent torque command/safety types, Gazebo
  `JointTrajectory` effort backend, smoke-test node, and safe standalone
  standing controller.
- `spot_state_estimator`: new replaceable simulation state adapter using joint
  states, IMU, ground-truth odometry, and contact/gait state.
- `spot_ocs2_mpc`: new Spot-owned package containing model/configuration,
  asynchronous MPC integration, gait schedule, diagnostics, and reference
  manager. It links to the pinned upstream OCS2 packages; solver internals stay
  upstream.
- `spot_wbc`: new transport-independent WBC/QP package consuming the state and
  OCS2 policy and producing bounded 12-joint torques.
- `spot_controller_tests`: tests may live with the owning package; launch tests
  cover headless standing and `/cmd_vel` behavior.

### Milestone 0 assumptions and unresolved issues

- `front_*_ee` and `rear_*_ee` are selected as foot frames because they are the
  fixed end-effector links used by the legacy link map. This convention
  is now authoritative and must not be changed without the escalation required
  by the goal document.
- The SDF contains suspicious pre-existing text in one inertia scalar
  (`0.00743438Source Control`). It was not changed during baseline inventory;
  SDF parsing must be tested at the start of Milestone 2 and corrected only if
  confirmed as a simulator blocker.
- The current top-level container configuration uses `privileged: true`, mounts
  `/dev/dri`, and mounts the host Codex directory. These conflict with the goal
  safety boundary and must be removed or placed behind an explicitly manual,
  out-of-scope GUI profile during Milestone 1.
- OCS2-related source directories are present but their Git metadata is broken
  in the same way as the Spot subtree, so their exact commits are not currently
  provable from Git. No OCS2 dependency is considered pinned yet.

## Milestone 1: Reproducible Humble/Fortress environment

Status: complete on 2026-07-26.

### Environment and safety changes

- The repository-root Dockerfile explicitly installs `ignition-fortress`,
  `ros-humble-ros-gz-sim`, `ros-humble-ros-gz-bridge`,
  `ros-humble-pinocchio`, `python3-vcstool`, colcon, rosdep, and
  `teleop_twist_keyboard`.
- Rosdep initialization is idempotent and the Humble index is populated as the
  unprivileged container user.
- Default Compose no longer uses `privileged: true` or mounts `/dev/dri`, the
  host X11 socket, or the host Codex directory.
- Documented image build:
  `docker compose build ros-humble-dev`.
- Documented shell entry:
  `docker compose run --rm ros-humble-dev`.

### Source strategy and divergence

- OCS2 upstream ROS 2 commit:
  `243b80b8de5a427d7237136786821723aa4e792a`.
- That revision is pinned in `dependencies/ocs2_humble.repos`; the executable
  `dependencies/import_ocs2.sh` imports it, applies the compatibility patches,
  verifies its HEAD value, and stores the ignored generated checkout under
  `third_party/ocs2`.
- `ocs2_robotic_assets` is intentionally not imported. Spot supplies its own
  URDF and configuration; patch `0003` removes the assets dependency from the
  required OCS2 packages and omits upstream ANYmal-only tests and launch files.
- `elevation_mapping_cupy` is intentionally not imported. Patch `0004` marks
  the unused OCS2 perceptive-ANYmal subtree with `COLCON_IGNORE`, removing its
  plane-segmentation packages from workspace discovery.
- Upstream and the local pre-existing OCS2 source were compared recursively.
  The selected upstream commit differs locally only in two excluded packages
  (`ocs2_self_collision` and `ocs2_mobile_manipulator`). Those local changes
  are not used by the selected build and are not part of the dependency
  strategy.
- Four narrow compatibility patches are isolated under
  `dependencies/patches/ocs2`: one qualifies `size_t` as `std::size_t` for
  Humble's compiler/include ordering, one loads Spot joint/contact names from
  the task configuration, one removes the unused robotic-assets boundary, and
  one excludes perceptive-ANYmal examples outside the Spot build.
- Selected solver: OCS2 SQP backed by the upstream HPIPM/BLASFEO wrapper.

### Clean-source verification

A fresh source directory `/tmp/ocs2-humble-clean.6FoNy7` was populated using
only the checked-in `.repos` pin. The exact OCS2 HEAD value was printed and
matched. Rosdep then reported:

```text
All system dependencies have been satisfied
```

The first build exposed the unqualified `size_t` issue after successfully
building `ocs2_core`; after applying the isolated patch, this command passed:

```bash
colcon --log-base /tmp/ocs2-humble-clean.6FoNy7/log build \
  --base-paths /tmp/ocs2-humble-clean.6FoNy7/src \
  --build-base /tmp/ocs2-humble-clean.6FoNy7/build \
  --install-base /tmp/ocs2-humble-clean.6FoNy7/install \
  --packages-up-to ocs2_sqp ocs2_centroidal_model ocs2_ros_interfaces \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result: 14 required transitive packages finished in 1 minute 28 seconds.
Warnings were limited to upstream CMake deprecation notices and Pinocchio's
Boost-Python header probe; there were no build failures.

The Spot package ignore marker was removed and the pre-existing bringup install
error was fixed by no longer installing a nonexistent directory. A workspace
build of `spot_description`, `spot_gazebo`, and `spot_bringup` then passed.

The image definition was not rebuilt from inside this task container because
the safety boundary forbids access to a host Docker socket. The dependency
resolution and clean-source build were instead verified inside the active
Ubuntu 22.04/Humble/Fortress container; the exact external rebuild command is
documented above as the manual container checkpoint.

## Milestone 2: Verify the Fortress simulation and torque path

Status: complete on 2026-07-26.

- Added the local `ocs2_test.sdf` headless world and a `headless` launch
  argument. Fortress 6.18 launched the Spot model without system-plugin load
  errors.
- Corrected one malformed inertia scalar only after `ign sdf -k` confirmed the
  source was invalid; also made pre-existing duplicate visual names unique.
  Model inertia values, collision shapes, and geometry were otherwise
  preserved.
- Added the required Contact system, explicit per-foot Gazebo topics, and ROS
  bridges. Clock, 200 Hz joint state, IMU, odometry, TF, and all four contact
  topics were observed with valid simulation timestamps. Contact messages
  named the intended lower-leg collision and ground-plane collision.
- A direct bridge probe showed that a ROS one-point trajectory containing only
  `joint_names` and `effort` arrived as
  `ignition.msgs.JointTrajectoryPoint.effort`.
- Inspection of the exact Fortress 6.18 source revealed that its trajectory
  plugin always sums position and velocity PID output with effort, even when
  those arrays are empty. All internal PID gains are therefore explicitly zero,
  and the legacy position controller is absent. There is one actuator plugin
  and one ROS publisher on the final effort topic.
- `ros2 run spot_bringup effort_smoke_test` applies at most 5 Nm for at most one
  second, requires no competing publisher, leaves position and velocity empty,
  and explicitly clears effort. In `effort_smoke_test.sdf`, +2 Nm and -2 Nm on
  `front_left_hip_x` produced +2.393 rad/s and -3.891 rad/s velocity changes,
  respectively. The completion re-audit then applied +1 Nm for 0.1 seconds to
  each of the 12 authoritative names in one zero-gravity run; all 12 tests
  passed with positive velocity changes (minimum +0.598 rad/s, maximum
  +6.178 rad/s), verifying order and sign across every actuator.

## Milestone 3: Effort interface and safe standing controller

Status: complete on 2026-07-26.

- Added `spot_effort_controller`. Its transport-independent PD and safety
  functions produce 12 torques in the authoritative order. A separate
  watchdog backend is the sole publisher of the Gazebo trajectory and always
  leaves position and velocity arrays empty.
- The backend rejects wrong dimensions/order, non-finite input, and mixed
  position/effort commands. It enforces a 60 Nm bound, a 2000 Nm/s rate bound,
  and a 0.1 simulated-second command watchdog.
- The standalone controller waits for ten valid full joint states, rejects
  stale/non-finite state, smoothly transitions to the original nominal
  posture, adds fixed-base Pinocchio gravity compensation, and applies joint
  PD. No OCS2 code is involved.
- Seven unit tests passed for joint mapping, dimensions, saturation, rate
  limiting, NaN rejection, gravity-plus-PD output, and the initialization
  gate. The node publishes no torque before state, still publishes none after
  nine valid samples, and enables only on the configured tenth sample.
- The repeatable `standing_smoke_test` passed a fresh 30.000 simulated seconds:
  minimum base height 0.478 m, maximum roll/pitch magnitude 0.007 rad, maximum
  planar drift 0.000 m, and maximum commanded torque 28.237 Nm.
- After the standing-controller process was stopped, the independent backend
  logged `command watchdog expired; ramping all efforts to zero`; the next
  observed 12-effort command was exactly all zeros and the backend remained
  alive.

## Milestone 4: State adapter

Status: complete on 2026-07-26.

- Added the replaceable `spot_state_estimator` package. Its ROS boundary is a
  24-value `/spot/ocs2_state` message, so neither OCS2 nor the WBC consumes
  Gazebo-specific odometry directly.
- The simulation adapter explicitly labels its Gazebo ground-truth odometry
  dependency in both logs and diagnostics. It combines joint states, IMU,
  odometry, and the four ordered foot-contact streams.
- Joint positions and velocities are reordered into the authoritative
  FL/FR/RL/RR sequence. Input quaternions are normalized; yaw, pitch, and roll
  are emitted in OCS2's Z/Y/X order. Pinocchio computes normalized centroidal
  momentum from the free-flyer pose, twist, and joint velocity.
- Source ages are bounded at 0.20 simulated seconds and inter-source skew at
  0.11 seconds (one 10 Hz odometry period plus tolerance). Missing, stale,
  dimensionally invalid, or non-finite data suppresses state output.
- Three unit tests passed for quaternion normalization/conversion and timestamp
  consistency. During the live headless run, diagnostics reported
  `ready:true`, contacts `[true,true,true,true]`, state dimension 24, and source
  stamps `[363.938,363.94,363.9]`; the printed state was finite and carried the
  documented layout label.

### Frame conventions

- `world` is Gazebo's inertial frame; `/spot/odometry` expresses the base pose
  in `world` and its twist in the child/base frame.
- `base_link` is the Pinocchio free-flyer root. `front_body_imu` supplies
  orientation/rates, while ground-truth odometry is authoritative for the
  initial simulation base pose and twist.
- OCS2 base orientation is `[yaw(Z), pitch(Y), roll(X)]`.
- The four contact frames and contact flags are ordered
  `front_left_ee`, `front_right_ee`, `rear_left_ee`, `rear_right_ee`.
- State order is normalized centroidal momentum (6), base XYZ (3), base
  yaw/pitch/roll (3), and the 12 authoritative joint positions.

## Milestone 5: Minimal Spot OCS2 model

Status: complete on 2026-07-26.

- Built the 18-package transitive chain through `ocs2_legged_robot_ros` in
  Release mode. The actual upstream SQP/HPIPM executable is used; solver
  behavior is not reimplemented in the Spot packages.
- Added Spot task, reference, and gait files for the SRBD centroidal
  formulation. The model has four ordered 3-DOF contacts, 12 contact-force
  inputs, 12 joint-velocity inputs, and a 24-state floating-base centroidal
  state with 12 joint positions. Initial/default mode is `STANCE` (mode 15),
  horizon is 0.6 s, and the observation/MPC cadence is 10 Hz.
- Upstream `ModelSettings` had hard-coded ANYmal names. The minimal three-line
  loader extension is isolated as dependency patch
  `0002-load-legged-model-names-from-task.patch`; clean-source patch
  applicability was verified. The Spot names remain exclusively in
  `spot_ocs2_mpc/config/task.info`.
- `spot_ocs2_mpc` keeps the estimator and MPC in separate processes. Its bridge
  resets OCS2 from the first valid live 24-state sample, publishes standard
  OCS2 observations, validates transient-local policy messages, and reports
  policy age, cadence, shape, contact order, trajectory length, and cost.
- Full headless `spot.mpc.launch.py` evidence after the conservative cadence
  correction: `initialized_from_live_state:true`, 168 repeated policies,
  23 trajectory knots, state/input dimensions 24/24, all values finite,
  observed rate 9.2--9.6 Hz, latest interval 106.84 ms, and policy age 0 ms in
  the sampled diagnostic. The independent effort loop held the robot at
  z=0.479 m during the run.
- OCS2 shutdown benchmarking over 168 solves reported mean times:
  LQ approximation 0.473 ms, QP solve 0.163 ms, line search 0.050 ms, and
  controller computation 0.068 ms (about 0.755 ms total). The failed
  real-time-priority request is a non-fatal unprivileged-container warning.
- Eleven package tests pass across model/policy contracts, gait selection and
  templates, reference construction, frame conversion, limiting, timeout,
  configurable posture, and non-finite rejection.

## Milestone 6: `/cmd_vel` reference generation

Status: complete on 2026-07-26.

- The `spot_ocs2_mpc` reference node consumes only `linear.x`, `linear.y`, and
  `angular.z`. It rotates limited body-frame planar velocity into `world` with
  the current estimator yaw and integrates continuous x/y/yaw references.
- Configurable defaults were reduced during integration to 0.05 m/s forward,
  0.03 m/s lateral, 0.10 rad/s yaw, 0.03/0.02 m/s² forward/lateral
  acceleration, 0.05 rad/s² yaw acceleration, a 0.5 s timeout, 0.48 m nominal
  height, and the authoritative nominal joints.
  Timeout selects a zero target while retaining the same acceleration limiter,
  so velocity returns smoothly rather than stepping.
- Five pure reference tests cover a 90-degree frame rotation, speed/acceleration
  saturation, smooth timeout deceleration, yaw integration, and target
  continuity. Together with the Milestone 5 contracts, all seven
  `spot_ocs2_mpc` tests pass.
- Before the final integration limits were reduced, the milestone live test
  command `(0.10, 0.05, 0.20)` at estimated yaw produced
  limited body velocity `[0.10,0.05,0.20]`, world velocity
  `[-0.01998,0.11000,0.20]`, a two-knot continuous 0.6 s trajectory, fixed
  0.48 m height, and the nominal 12-joint posture. OCS2 continued returning
  valid 24/24 policies while the target changed.
- After publication stopped, a live diagnostic reported
  `command_active:false` and all three limited velocities exactly zero.
  Once stopped, the target is re-anchored to the current live x/y/yaw. This
  prevents a later gait from walking back toward an early-startup pose while
  preserving reference continuity through the deceleration transient. Three
  gait tests and three contract tests bring the package total to 11.

## Milestone 7: Whole-body controller

Status: complete on 2026-07-26.

- Added `spot_wbc` with a 42-variable constrained QP:
  18 generalized accelerations, four ordered 3-D contact forces, and 12 joint
  torques. Pinocchio supplies the live free-flyer mass matrix, nonlinear
  effects, contact Jacobians, and contact drift acceleration.
- Equalities enforce full rigid-body dynamics, zero stance-foot acceleration,
  and zero force for planned swing feet. The objective tracks OCS2 contact
  forces and joint velocities, base acceleration, and planned swing-foot
  kinematics. Linear friction pyramids, nonnegative/maximum normal force,
  joint-position-derived acceleration bounds, acceleration bounds, and
  ±60 Nm torque bounds are explicit constraints.
- Feed-forward QP torque is combined with configured low-gain policy
  position/velocity feedback. The planned OCS2 mode schedule selects
  stance/swing constraints; the interface reserves measured contacts for a
  later estimator without silently changing the planned schedule.
- SciPy's optimizer initially blocked the torque executor for 5--8 ms. QP
  solution now runs in one background worker at 10 Hz for four-foot stance and
  50 Hz for swing modes while the torque/fallback publisher remains at
  200 Hz. The cache stores only the bounded policy correction, not an absolute
  high-gain torque, so live fallback feedback continues at 200 Hz between QP
  solutions. A 0.03 s moving-policy handoff and 2.0 s stance handoff limit
  transitions. Raw joint state starts the known-safe fixed-base fallback
  before odometry or OCS2 is ready.
- Six QP/safety tests pass: contact-bit/order and friction semantics, invalid
  dimension rejection, finite/bounded synthetic and real Spot problems,
  stale/non-finite policy fallback, and a regression test proving cached
  policy corrections preserve live 200 Hz feedback updates. The real problem
  converged in seven iterations with `3.55e-15` equality residual, positive
  friction margin, and 15.877 Nm maximum feed-forward torque.
- The final installed-source run reported 224 successful and 8 rejected QPs,
  9.81 ms worker time, `5.68e-14` dynamics/contact residual, 63.07 N friction
  margin, 20.84 Nm maximum feed-forward torque, and 88.05 N maximum force.
  There was exactly one publisher at each side of the effort backend.
- The full OCS2/WBC stack passed 30.001 simulated seconds of standing while
  the smoke test required an active mode-15 policy and a successful WBC QP:
  minimum height 0.480 m, maximum tilt 0.083 rad, drift 0.149 m, and maximum
  output 60.000 Nm.
- Deliberately terminating the MPC process changed diagnostics to
  `MPC policy stale; standalone fallback`. The fallback re-anchored the
  current posture and passed a further 10.001-second test at height >=0.481 m,
  tilt <=0.005 rad, zero measured drift, and maximum torque 23.299 Nm. The
  WBC/backend processes remained alive.

Integration added further conservative bounds without changing the QP
formulation:

- A HiGHS linear feasibility solve seeds every non-stance SLSQP problem.
  SLSQP output is accepted only when convergence, dynamics/contact equality,
  friction, normal-force, acceleration, joint, and torque bounds all pass.
- Moving-policy torque is limited to ±20 Nm relative to the known-safe
  fixed-base baseline and ±60 Nm overall. A failed or stale result immediately
  selects fallback. Planned swing degradation is independently bounded to
  20 Nm with 100/5 position/velocity gains.
- Four-foot stationary mode solves the same rigid-body WBC/QP at 10 Hz. Its
  correction is limited to ±2 Nm relative to the live fixed-base
  gravity-plus-PD safety baseline and blended over two seconds. This produces
  genuine OCS2/WBC contribution without freezing or bypassing the 200 Hz
  posture feedback.

The completion re-audit intentionally removed an earlier mode-15 bypass and
initially exposed repeated standing falls. Reducing the correction from 2 Nm
to 0.5, 0.05, and finally zero did not remove the failure. That isolation
showed the actual defect: the controller cached an absolute high-gain torque
between QP updates, freezing live posture feedback even at zero policy
correction. It now caches only `candidate - fallback` and adds that correction
to the freshly computed 200 Hz fallback. The dedicated regression test and the
fresh 30-second policy/WBC standing test both pass. These failed trials are
retained here because they materially changed the final safety design.

## Milestone 8: Locomotion and integration verification

Status: complete on 2026-07-26.

### Gaits, launch, and operator path

- Added the required `conservative_trot` schedule with alternating diagonal
  modes 9 and 6, 90% duty factor, and an intervening four-foot support phase.
  It is selectable at zero command with
  `stationary_gait:=conservative_trot`.
- A slower `conservative_crawl` schedule swings one leg at a time with
  four-foot support between swings. It is the default because live
  forward/lateral/yaw tests were more robust than with two simultaneous swing
  legs. Zero or timed-out command selects mode-15 four-foot stance.
- Gait schedules publish only on a stance/motion edge. Periodically resetting
  the OCS2 schedule was tested and rejected because it repeatedly shifted the
  phase origin.
- `spot.wbc.launch.py` starts bridge/controllers before the simulator through
  the bringup `simulator_delay`, avoiding a startup interval with uncontrolled
  joints, and exposes headless/GUI, world, policy, and gait arguments. README
  instructions include `teleop_twist_keyboard`, bounds, direction
  conventions, tuning, architecture, tests, and limitations.

Changed or added for this milestone:

- `spot_ocs2_mpc/config/gait.info`
- `spot_ocs2_mpc/config/task.info`
- `spot_ocs2_mpc/spot_ocs2_mpc/gait.py`
- `spot_ocs2_mpc/spot_ocs2_mpc/gait_node.py`
- `spot_ocs2_mpc/spot_ocs2_mpc/reference_node.py`
- `spot_wbc/config/wbc.yaml`
- `spot_wbc/launch/spot.wbc.launch.py`
- `spot_wbc/spot_wbc/locomotion_smoke_test.py`
- `spot_wbc/test/test_locomotion_smoke.py`
- `spot_bringup/launch/spot.gazebo.launch.py`
- `README.md`

### Final live evidence

A completion re-audit after restoring the stance WBC/QP path produced:

- OCS2-policy/WBC zero-command standing:
  `PASS duration=30.000`, minimum height 0.480 m, maximum tilt 0.083 rad,
  maximum planar drift 0.149 m, and maximum commanded torque 60.000 Nm.
  The smoke test required `policy_active:true`, mode 15, and at least one
  successful QP.
- Fresh forward test with +0.03 m/s command: +0.011366 m body-forward,
  -0.000372 m body-left, -0.001324 rad yaw, minimum height 0.479 m, maximum
  tilt 0.009 rad, and 33 rejected/infeasible QP samples handled safely.
- Fresh lateral test with +0.03 m/s command: +0.006925 m body-left
  (+0.019472 m forward), +0.019461 rad yaw, minimum height 0.478 m, maximum
  tilt 0.016 rad, and 27 rejected/infeasible QP samples handled safely.
- Fresh yaw test with +0.05 rad/s command: +0.014423 rad yaw
  (+0.007224 m forward, +0.004886 m left), minimum height 0.476 m, maximum
  tilt 0.022 rad, and 29 rejected/infeasible QP samples handled safely.
- After the 30-second standing dwell, a combined
  `(linear.x,linear.y,angular.z)=(0.02,0.01,0.03)` pulse passed with
  +0.009236 m forward, +0.000473 m left, +0.007420 rad yaw, minimum height
  0.476 m, maximum tilt 0.011 rad, and 23 bounded QP rejections.
- A separate zero-command
  `stationary_gait:=conservative_trot` launch published modes
  `[9,15,6,15]`. Live WBC diagnostics observed policy tracking in mode 6, and
  a 15.003-second physical smoke test passed with minimum height 0.474 m,
  maximum tilt 0.018 rad, drift 0.006 m, and maximum torque 50.339 Nm.

The smoke test intentionally reports QP failure-count deltas rather than
hiding them. These are isolated swing-sample optimization rejections; every
one took the configured bounded fallback/degraded path, output remained finite
and within ±60 Nm, and none caused a fall or loss of the command stream.

During the combined-pulse run, the last successful QP diagnostic reported
9.79 ms worker wall time, `1.24e-14` dynamics/contact residual, 68.38 N
minimum friction margin, 17.01 Nm maximum feed-forward torque, and 78.19 N
maximum contact force. The counters were 609 accepted and 30 rejected QPs;
policy age was within the configured 0.25 simulated-second bound. The torque
and backend loops are configured at 200 Hz, QP at 10 Hz in stance and 50 Hz in
swing modes, OCS2 observation/policy input at 10 Hz, and the backend watchdog
at 0.10 seconds.

The final post-build headless audit independently reported an active mode-15
policy at full blend, 2.0 Nm policy correction, 224 successful / 8 rejected
QPs, 9.81 ms solve wall time, `5.68e-14` residual, and one publisher on each
side of the effort backend. MPC diagnostics reported a valid 21-point 24/24
policy, 0.0 simulated-second age, 108.39 ms wall interval, and live-state
initialization; the state adapter simultaneously reported a finite 24-state
sample and four active contacts.

A post-fix 500-message rolling cadence sample under the complete Gazebo,
bridge, estimator, OCS2, and WBC load observed 184.46--190.14 Hz on
`/spot/joint_trajectory` (3--20 ms intervals) against the configured 200 Hz
loop. The simultaneous WBC sample remained active in mode 15 with a fresh
policy, 165 accepted / 8 rejected QPs, 12.61 ms worker wall time, and
`6.66e-15` dynamics/contact residual.

Terminating the MPC process during that still-upright run produced
`MPC policy stale; standalone fallback`, `policy_active:false`, and exactly
zero policy correction. A subsequent 10.001-second smoke test passed at
minimum height 0.481 m, maximum tilt 0.005 rad, zero measured drift, and
23.299 Nm maximum torque. Independently terminating the standalone standing
publisher made the backend log its watchdog transition and publish an
effort-only 12-vector of exact zeros while the backend remained alive.

### Automated coverage and residual limitations

- Added pure locomotion-test coverage for body-frame displacement projection
  and wrapped yaw differences. Together the four Spot packages contain 29
  passing tests: 7 effort/standing, 3 estimator, 11 MPC/reference/gait, and
  8 WBC/locomotion tests. They cover initialization inhibition,
  ordering/dimensions, transformations, limiting/timeout, finite and bounded
  QP output, stale/invalid fallback, the live-feedback cache regression, and
  locomotion measurements.
- The verified controller is a conservative simulation implementation. It
  uses ground-truth odometry, Python/SciPy is not hard-real-time, measured
  contacts are not yet fused into schedule selection, and the diagonal trot
  is selectable but is not the default verified directional gait.
- High-speed/aggressive locomotion, terrain perception, learned dynamics,
  self-collision optimization, production state estimation, and real hardware
  remain explicitly out of scope.

## Final verification audit

Run from a newly sourced shell on 2026-07-26:

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
colcon test --packages-select \
  spot_effort_controller spot_state_estimator spot_ocs2_mpc spot_wbc
colcon test-result --verbose
ros2 launch spot_wbc spot.wbc.launch.py --show-args
```

Results:

- Rosdep: `All system dependencies have been satisfied`. Pure-Python packages
  declare `python3-setuptools` as their build tool, and bringup declares the
  specific `ros_gz_sim`, `ros_gz_bridge`, and `ros_gz_interfaces` components
  it uses rather than the unused meta-package.
- Build: all 25 packages through `spot_wbc` finished. No package failed.
- Tests: 29 tests, zero errors, zero failures, zero skipped.
- `ament_flake8` checked 29 implementation/test files with no problems;
  Python byte-compilation and parsing of all seven YAML configuration files
  passed.
- `ign sdf -k` passed for the model and for both local test worlds after
  resolving their ROS `package://spot_description` URI to the installed share
  path (the standalone `ign sdf` CLI has no ROS package callback). Both worlds
  also passed the stronger live Fortress launch checks.
- The installed standing and WBC launch files expose the expected headless,
  world, policy-tracking, `stationary_gait`, and `moving_gait` arguments.
- A final installed headless launch started all controller, bridge, simulator,
  estimator, and OCS2 processes; live-state MPC initialized. Ctrl-C then
  cleanly stopped every custom Python and ROS C++ process. Fortress reported
  exit code -2, the documented result of SIGINT.
- The final controller configuration keeps the torque/backend loops at 200 Hz,
  OCS2 at 10 Hz, and WBC QP at 10 Hz in stance / 50 Hz in swing. The measured
  full-stack output cadence was 184.46--190.14 Hz over rolling 500-message
  windows. Runtime diagnostics expose all configured rates, policy age, solve
  wall time, bounds, residuals, force margins, maximum torque/force, and
  failure counts.
- WBC policy/dynamics tracking still requires ten ordered finite joint-state
  samples before torque output. Keeping this gate avoids acting on an
  unconditioned first velocity sample. The fixed-base gravity/PD fallback then
  exclusively owns a five-second startup posture transition; policy correction
  caching and QP tracking remain disabled until that transition is complete.
- A follow-up `empty_room.sdf` integration check found that the imported room
  mesh placed its floor at world z=-0.668251 while Spot spawned at z=-0.05,
  which was inconsistent with the controller's 0.48 m world-frame nominal
  height. The room mesh is translated so its visual floor is normalized to
  world z=0, gravity and friction are explicit, and the missing Gazebo contact
  system is loaded. The imported triangle mesh is visual-only: a plane supplies
  stable floor contact and four boxes preserve perimeter-wall collisions
  without the ODE trimesh/trimesh contact overflow seen between the original
  room floor and Spot's lower-leg collision meshes. The rendered floor is
  offset 15 mm below the physical plane so the lower-leg visuals do not appear
  embedded after contact settling.
- The installed `empty_room.sdf` now starts Spot in a symmetric seated crouch:
  base z=0.39 m, every hip-y joint=1.25 rad, and every knee=-2.10 rad. A small
  world-specific Gazebo system applies those joint positions during model
  configuration, leaving the shared Spot model and other worlds unchanged.
  A paused-server transport read verified all 12 configured joints before the
  first physics step. In the live controller run, body height rose from
  0.381 m at simulation second 1 to a stationary 0.479 m after the guarded
  transition. The corrected installed build then passed 30.000 simulated
  seconds of active-policy standing (0.479 m minimum height, 0.006 rad maximum
  tilt, zero drift, 24.426 Nm maximum torque) and a bounded forward crawl
  (+0.004020 m forward, 0.478 m minimum height, 0.014 rad maximum tilt).

The first final build attempt used `--symlink-install` against a build tree
previously produced in copy mode. CMake could not replace the generated
`ocs2_msgs/__init__.py` directory with its expected symlink. That single
generated artifact was moved, recoverably, to
`/tmp/ocs2_msgs_generated_20260726_0513`; no source or install content was
removed. The documented mode-neutral build command then passed twice,
including once after the final manifest and initialization-test changes.

### Slim OCS2 dependency follow-up

On 2026-07-27, the dependency import was reduced from two upstream
repositories to the pinned OCS2 repository alone. Reproducibility and runtime
were checked from an isolated directory that never contained
`ocs2_robotic_assets`:

- A fresh `import_ocs2.sh` run applied all four patches and matched the
  checked-in OCS2 source changes.
- The complete `--packages-up-to spot_wbc` closure built 28 packages at the
  time. After removal of the four legacy runtime packages, the current closure
  contains 24 packages.
- No installed package manifest, CMake export, launch file, or ROS package
  index entry in the production OCS2 closure referenced
  `ocs2_robotic_assets`.
- A second fresh import confirmed that `COLCON_IGNORE` excludes the three
  perceptive-ANYmal packages that otherwise depend on
  `elevation_mapping_cupy`. Removing that source repository left the Spot
  closure unchanged at 28 packages.
- The four Spot test packages passed 29 tests with zero errors, failures, or
  skips.
- The isolated headless `empty_room.sdf` run passed 30.000 simulated seconds
  of active-policy standing at 0.479 m minimum height, 0.006 rad maximum tilt,
  zero drift, and 24.434 Nm maximum torque.
- The isolated forward-crawl test passed with +0.009610 m signed forward
  displacement, 0.479 m minimum height, and 0.013 rad maximum tilt.

### Local Git reconstruction and repository-owned OCS2

On 2026-07-27, the broken `.git` indirection was replaced by a real local
clone of `https://github.com/justyx404/spot_gazebo_ros2.git`. The implementation
was reconstructed on an uncommitted local `ocs2` branch based on upstream
commit `23166d6efd67724edc1ce41e9ee4bb909cf9a2f9`. No implementation commit was
created or pushed, and no file was staged.

The pinned OCS2 checkout is now materialized below this repository at
`third_party/ocs2`. It is deliberately ignored because
`dependencies/import_ocs2.sh` recreates the exact source and applies the four
checked-in compatibility patches. There is no longer a second OCS2 checkout at
the workspace-level `src/ocs2` path.

The reconstructed branch then passed a 28-package build and all 29 automated
tests. A final installed `empty_room.sdf` launch exposed and corrected one
three-way merge mismatch: upstream's simulator bridge published `/imu/data`
while the Spot state adapter consumes `/spot/imu`. After restoring the
controller-facing bridge topic, live-state MPC initialized successfully.
Active-policy standing passed for 30.001 simulated seconds at 0.479 m minimum
height, 0.006 rad maximum tilt, zero drift, and 24.447 Nm maximum torque. The
subsequent bounded forward-crawl test passed with +0.004673 m signed forward
displacement, 0.479 m minimum height, 0.010 rad maximum tilt, and all 32
rejected/infeasible swing QP samples handled by the configured safe fallback.

Before reconstruction, durable copies of the original Spot tree, baseline, and
workspace-level OCS2 checkout were saved outside the workspace under
`/home/rosuser/ros2_ws_recovery/spot-ocs2-20260727`.

### Legacy controller removal

On 2026-07-27, the four obsolete position-controller packages were removed
from the OCS2 branch. Bringup no longer declares their runtime dependencies,
creates their node, exposes their launch switch, or passes that switch through
integrated launches. Current README instructions describe only the effort and
OCS2/WBC paths.

A repository guard test verifies that all four package directories stay absent,
active source and documentation contain no stale references, and the Gazebo
launch description still loads. The resulting `--packages-up-to spot_wbc`
closure built 24 packages. Tests across bringup and the four controller packages
reported 33 passing results with zero errors, failures, or skips. The installed
Gazebo launch argument list contains no legacy-controller switch.

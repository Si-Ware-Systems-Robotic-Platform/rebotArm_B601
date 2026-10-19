# SDK-compatible MoveToPose in MuJoCo

Status: implemented; 13 offline request/action-outcome checks passed. User colcon build and live MuJoCo checks remain pending. The user previously confirmed that a direct MoveIt terminal pose goal worked.

The simulation adapter exposes the same `/rebotarm/move_to_pose` action type as the hardware SDK: `rebotarm_msgs/action/MoveToPose`. It translates the pose into a MoveIt `/move_action` **planning-only** request, adjusts trajectory timing if necessary, and executes through MoveIt's `/execute_trajectory` action and the existing MuJoCo controllers. Direct `/move_action` requests and RViz remain available.

It also mirrors `/joint_states` to `/rebotarm/joint_states`, satisfying the existing SDK `MoveToPose` example's joint-state prerequisite. This adds a publisher on the alias topic, without adding a second publisher on `/joint_states`.

## Pose and duration semantics

- Position is in metres relative to `base_link`; the target link is `gripper_tcp`.
- Orientation is a quaternion in ROS `(x, y, z, w)` order. Finite nonzero quaternions are normalized. Invalid poses and nonpositive duration are rejected.
- The default planning region is a 5 mm position-radius tolerance and 0.05 rad orientation-component tolerance. Velocity/acceleration scaling defaults to 0.2.
- `duration` means **minimum moving-trajectory duration in simulation seconds**. If MoveIt's plan is shorter, timestamps are stretched and velocities/accelerations reduced consistently. A plan longer than the request retains its timing. Planning time and controller settling add to the overall action time. An already-satisfied pose with no planned trajectory succeeds only after checking measured TCP TF, without a timed hold.
- SDK feedback uses measured TCP TF and simulation elapsed time. `progress` is a trajectory-time estimate, reaching 1 only after successful execution. `final_pose` is measured URDF forward kinematics from joint states, not a copied goal or an independent MuJoCo-site measurement.
- Planning, execution and backend errors become an unsuccessful SDK result. Cancellation is forwarded to the active MoveIt action. If the backend cannot acknowledge a request or confirm an end state, the adapter rejects subsequent goals until the stack is restarted.
- The adapter accepts one of its own pose goals at a time. Use one motion client at a time: a concurrent RViz, direct MoveIt, or controller goal can interrupt an adapter trajectory.

The SDK action type has no frame header or speed fields. Defaults can be set with ROS parameters on `mujoco_pose_adapter`: `planning_group`, `base_frame`, `tip_frame`, `planning_time`, `position_tolerance`, `orientation_tolerance`, `velocity_scaling`, `acceleration_scaling`, and the wall-time watchdogs `result_timeout` (180 s), `response_timeout` (10 s), `cancel_timeout` (5 s). These values are read when the adapter starts. The result timeout remains active while simulation is paused.

## Build and launch — Terminal 1

Stop the previous combined simulation before starting the new launch. Use the same ROS domain settings in every terminal.

```bash
source /opt/ros/humble/setup.bash
cd /home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects
colcon build --symlink-install --packages-up-to rebotarm_mujoco_dm
source install/setup.bash
ros2 launch rebotarm_mujoco_dm simulation.launch.py scene:=empty use_moveit:=true
```

The adapter starts by default when MoveIt is enabled. `use_pose_adapter:=false` disables it. It does not start in the default Phase 2 mode (`use_moveit:=false`). Wait for MoveIt and the adapter to report readiness before sending a goal. Keep the physical SDK driver stopped so one action server owns `/rebotarm/move_to_pose`.

## Check the graph — Terminal 2

```bash
source /opt/ros/humble/setup.bash
cd /home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects
source install/setup.bash

ros2 action list -t
ros2 action info /rebotarm/move_to_pose
ros2 param get /mujoco_pose_adapter use_sim_time
ros2 topic info /joint_states -v
ros2 topic echo /rebotarm/joint_states --once
```

Expected: exactly one `/rebotarm/move_to_pose` server, the original `/move_action` and `/execute_trajectory` actions remain present, simulation time is true, `/joint_states` still has one publisher, and the alias topic contains measured joints.

## Unified SDK goal command

This pose previously worked in the pick/place demo and the user's direct MoveIt pose test:

```bash
ros2 action send_goal /rebotarm/move_to_pose rebotarm_msgs/action/MoveToPose \
  "{target_pose: {position: {x: 0.30, y: 0.12, z: 0.06}, orientation: {x: 0.0, y: 0.3826834324, z: 0.0, w: 0.9238795325}}, duration: 2.0}" \
  --feedback
```

Expected: accepted goal, motion in MuJoCo/RViz, `success: true`, measured `final_pose`, and `Goal finished with status: SUCCEEDED`. The launch log reports the actual planned trajectory duration, which may exceed 2 s. SDK `success: true` wraps MoveIt's success code 1; it does not expose a `MoveItErrorCodes` field.

Collect a controller-state snapshot after completion:

```bash
ros2 topic echo /rebotarm_controller/controller_state --once
```

## Existing SDK convenience command

The original client requires no source changes because the adapter provides its expected action and joint-state topic:

```bash
ros2 run rebotarmcontroller MoveToPose -- \
  --x 0.30 --y 0.12 --z 0.06 \
  --qy 0.3826834324 --qw 0.9238795325 --duration 2.0
```

Expected: `success=True` and the adapter's result message. Sending an already-reached goal may produce little or no movement. To visibly exercise another pose, the symmetric place target `(0.30, -0.12, 0.06)` with the same orientation was also used successfully by the demo; a new plan remains subject to the current scene.

The same command syntax works with the hardware SDK server when the physical stack is running. The simulation implementation uses MoveIt collision checking and the minimum-duration policy above; the hardware SDK keeps its existing backend and timing semantics. This addition implements the pose action and joint-state alias only. Hardware calibration and motor-mode services remain hardware operations.

## Live validation checklist

| Check | Status |
|---|---|
| Offline request conversion, quaternion validation, time/derivative scaling, cancellation forwarding, late-ack cancellation, failure/result handling | Passed (13 tests) |
| Launch construction with adapter enabled/disabled and with MoveIt disabled | Static checks passed |
| User colcon build and adapter startup | Pending |
| One SDK action server and correct joint-state alias | Pending |
| SDK `ros2 action send_goal` pose succeeds in MuJoCo | Pending |
| Existing SDK `MoveToPose` convenience client succeeds | Pending |

For a later cancellation check, the CLI exposes the action's standard ROS `CancelGoal` service at `/rebotarm/move_to_pose/_action/cancel_goal`. Cancel one deliberately long adapter motion through a ROS action client and verify that MoveIt and the controller terminate that motion. Ctrl+C in a CLI client alone does not demonstrate action cancellation. Offline cancellation checks exercise the forwarding and result logic; backend stop behaviour still needs a live check.

Reference: [MoveIt Humble time parameterization](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html).

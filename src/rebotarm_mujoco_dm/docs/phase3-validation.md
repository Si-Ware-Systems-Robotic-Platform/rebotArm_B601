# Phase 3 — MoveIt planning and execution in MuJoCo

Status: MoveIt startup and joint-goal execution passed from user-supplied results (12 October 2026). The user later confirmed a direct terminal end-effector pose goal worked. The separate RViz pose and named-home checks remain open. Phase 2 has already passed.

## What this provides

The command path is RViz or a MoveIt client → `move_group` → `FollowJointTrajectory` → the MuJoCo controllers → simulated motion. `/joint_states` supplies the measured simulation state to MoveIt and RViz. The existing DM SRDF, IK solver, joint limits, OMPL configuration and controller mapping are reused. All launched nodes use simulation time.

After validation you can plan/execute arm joint goals, end-effector pose goals and the SRDF named `home` state, and use the MoveIt gripper group. A pose goal specifies the final tool pose; ordinary OMPL planning does not promise a straight tool path. The existing `draw_square` and `pick_place` examples are the next phase. The physical SDK's `/rebotarm/safe_home`, `/rebotarm/set_zero` and `/rebotarm/move_to_pose` interfaces are not added by this launch.

MuJoCo geometry is not automatically imported into the MoveIt planning scene. The current MuJoCo scene contains a floor and no table or objects. For these first tests use small, above-floor motions; synchronized environment collision objects belong to the later scene work.

## 1. Build and launch — Terminal 1

Stop your previous Phase 2 launch with Ctrl+C. Close any mock `demo.launch.py` or hardware launch so this launch owns the controllers and joint-state publication. Leave the new launch running throughout the checks.

```bash
source /opt/ros/humble/setup.bash
cd /home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects
colcon build --symlink-install --packages-up-to rebotarm_mujoco_dm
source install/setup.bash
ros2 launch rebotarm_mujoco_dm simulation.launch.py scene:=empty use_moveit:=true
```

Expected: MuJoCo opens, the three controllers activate, then `move_group` and RViz start. RViz opens with MotionPlanning enabled, group `arm`, fixed frame `world`, and velocity/acceleration scaling 0.2. Wait until the robot is visible and MoveIt is ready to plan. If startup fails, report the first error and surrounding lines before attempting motion.

Launch options:

```bash
# Existing Phase 2 operation (default):
ros2 launch rebotarm_mujoco_dm simulation.launch.py

# MoveIt without either viewer, for later terminal-only use:
ros2 launch rebotarm_mujoco_dm simulation.launch.py scene:=empty use_moveit:=true use_rviz:=false headless:=true
```

These are alternatives, not additional launches to run together.

## 2. Confirm MoveIt is connected — Terminal 2

```bash
source /opt/ros/humble/setup.bash
cd /home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects
source install/setup.bash

ros2 node list
ros2 control list_controllers
ros2 action list -t
ros2 param get /move_group use_sim_time
ros2 param get /rviz2 use_sim_time
ros2 param get /robot_state_publisher use_sim_time
ros2 topic info /joint_states -v
```

Expected:

- `move_group`, `rviz2`, `robot_state_publisher`, and one controller manager are present.
- `joint_state_broadcaster`, `rebotarm_controller`, and `gripper_controller` are active.
- `/move_action [moveit_msgs/action/MoveGroup]`, `/execute_trajectory [moveit_msgs/action/ExecuteTrajectory]`, and both controller `follow_joint_trajectory` actions are present.
- All three queried `use_sim_time` parameters are `True`.
- `/joint_states` has exactly one publisher, `joint_state_broadcaster`. Multiple subscribers are expected now.

## 3. Plan and execute a joint target through MoveIt — Terminal 2

This sends a six-joint goal to MoveIt, which plans from the current simulation state and executes the trajectory. The target is the same near-home arm configuration previously exercised by topic control. A `plan_only: false` request tests both planning and execution.

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{
  request: {
    group_name: arm,
    pipeline_id: ompl,
    num_planning_attempts: 10,
    allowed_planning_time: 5.0,
    max_velocity_scaling_factor: 0.2,
    max_acceleration_scaling_factor: 0.2,
    start_state: {is_diff: true},
    goal_constraints: [{joint_constraints: [
      {joint_name: joint1, position: 0.2, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0},
      {joint_name: joint2, position: -0.1, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0},
      {joint_name: joint3, position: 0.0, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0},
      {joint_name: joint4, position: 0.0, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0},
      {joint_name: joint5, position: 0.0, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0},
      {joint_name: joint6, position: 0.0, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0}
    ]}]
  },
  planning_options: {
    plan_only: false,
    planning_scene_diff: {is_diff: true, robot_state: {is_diff: true}}
  }
}" --feedback
```

Expected: accepted goal, `Goal finished with status: SUCCEEDED`, and `error_code: {val: 1}` in the result. MoveIt uses success value **1**; the controller actions tested in Phase 2 use success value **0**. Both windows should show the actual arm moving. A moving translucent planned-path preview alone does not establish execution.

Once the action finishes, collect the measured state and controller error:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /rebotarm_controller/controller_state --once
```

Use `joint_names` to interpret each position array. The controller state's `reference`/`feedback` (or deprecated `desired`/`actual`) and `error.positions` show the final trajectory target, measured joints, and tracking error. Compare the measured state to the **returned trajectory's final point**: MoveIt may choose a goal within the requested ±0.005 rad region. Each arm joint must satisfy the configured 0.02 rad goal tolerance. The 0.15 rad path tolerance applies during motion; neither is a Cartesian TCP accuracy guarantee.

Paste the final action status/error code, its final trajectory point if practical, and the two state snapshots. If the action fails, paste its error and the relevant Terminal 1 logs.

## 4. Plan and execute an end-effector pose — RViz

1. Keep planning group `arm`, velocity scaling `0.2`, and acceleration scaling `0.2`.
2. Set the start state to `<current>` in the MotionPlanning panel. Set the goal to `<current>` before dragging, so the goal marker starts at the measured robot state.
3. Enable **Query Goal State** and use the **Interact** tool. Drag the tool marker a small distance, around 1–2 cm, while keeping its orientation initially unchanged. Choose a reachable target above the floor. If you already have a nearby pose that worked in the mock demo, you can use it instead.
4. Press **Plan**. Check that planning succeeds and inspect the path preview. Planning alone should leave the actual MuJoCo arm still.
5. Press **Execute**. Confirm success in RViz/Terminal 1 and that the measured arm moves in both RViz and MuJoCo.
6. After completion, repeat the two state snapshot commands from step 3. Report whether both views matched and paste the execution result/logs. A failed IK target can be a reachability issue; record the message before changing the integration.

Optional TCP readout:

```bash
ros2 run tf2_ros tf2_echo world gripper_tcp --ros-args -p use_sim_time:=true
```

Copy one transform and stop this readout with Ctrl+C. It is the URDF forward-kinematics pose computed from the reported joint states, not an independent measurement of MuJoCo's TCP site. Phase 1 already compared URDF and MJCF kinematics.

## 5. Return to the named home configuration — RViz

Set start state to `<current>`, choose `home` as the goal state, click **Plan**, then **Execute**. Confirm both windows return to the home configuration and collect the controller-state snapshot again. This uses the SRDF's all-zero arm joint goal through MoveIt; it does not call the hardware `/rebotarm/safe_home` service or change motor calibration.

## Acceptance and results

| Check | Status |
|---|---|
| Launch configuration: shared MuJoCo URDF, simulation time, controller action mapping, Humble planner adapters | Static checks passed |
| Controller activation gates MoveIt startup; Phase 2 mode and no-RViz mode construct correctly | Static checks passed |
| User colcon build and live MoveIt/RViz startup | Passed (MoveIt, RViz, MuJoCo and all controllers visible) |
| One joint-state publisher; simulation time confirmed on live nodes | Passed (one publisher; all three queried nodes reported `True`) |
| MoveIt joint-goal plan and execution succeed | Passed (action `SUCCEEDED`, MoveIt error code `1`; user also reports successful RViz execution) |
| Final arm tracking errors satisfy configured goal tolerances | Passed (measured joints were within the requested ±0.005 rad; largest reported controller tracking error was about 0.00541 rad) |
| Direct terminal end-effector pose plan and execution succeed | Passed by user report |
| RViz end-effector pose plan and execution succeed; measured motion agrees visually with MuJoCo | Pending |
| Named home plan and execution succeed | Pending |

The terminal joint goal requested `[0.2, -0.1, 0, 0, 0, 0]` rad. `/joint_states` later reported approximately `[0.20454, -0.10402, 0.00044, 0.00127, -0.00090, 0.00428]` rad in a different joint order; mapped by name, every joint was within the requested ±0.005 rad region. The controller state also reported near-zero final tracking error (largest absolute component approximately `0.00541` rad). The user separately confirmed successful RViz planning-GUI motion. `ros2 topic echo` showed a lost-message notice before each snapshot, but both later snapshots arrived. `ros2 node list` warned that exact node names were duplicated and listed `/move_group` twice; the successful action shows a MoveIt server responded, but the duplicate-name warning should be checked if it persists on a clean relaunch. End-effector pose execution and named-home execution have not yet been reported. The static checks construct launch actions and evaluate configuration without starting ROS nodes or GUI processes. Pause/reset behavior, the previously observed shutdown crash, demo scripts, and physical object grasping remain later integration work.

Reference: [MoveIt Humble controller configuration](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html) describes the controller/action mapping used here.

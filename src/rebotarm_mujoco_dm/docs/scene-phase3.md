# Scene Phase 3: five golf balls and two bowls

## Intended result

`scene:=two_bowls` starts a separate MuJoCo scene. Bowl 1 is centered at
`(0.30, +0.14) m` and begins with five free golf balls; bowl 2 is centered at
`(0.30, -0.14) m` and begins empty. The balls are native MuJoCo contact spheres
with the same 42.7 mm diameter and 45.9 g mass as the verified one-ball scene.
`ball_visual:=mesh` changes their appearance only. Both bowls use the same
existing visual mesh, wall collision pieces, bottom and friction settings.

The `bowl_transfer_mission` command reads the five measured free-joint poses. When one
bowl is empty, it selects the occupied bowl as source, uses MoveIt to approach one ball, closes the physical
DM fingers, verifies lift and hold, carries it to a free landing location in the
other bowl, releases it, and checks its measured resting location. It repeats
for every ball in the source bowl. Run it again **without restarting the launch**
to move the five balls back. A `--limit 1` option allows the first pickup and
release to be checked before a full batch run. It does not teleport balls, weld
them to the gripper or bypass collision checking.

## Build and run

In terminal 1:

```bash
source /opt/ros/humble/setup.bash
cd /home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects
colcon build --symlink-install --packages-select rebotarm_mujoco_dm
source install/setup.bash
export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1
ros2 launch rebotarm_mujoco_dm simulation.launch.py scene:=two_bowls use_moveit:=true
```

In terminal 2, use the same ROS domain and source the workspace:

```bash
source /opt/ros/humble/setup.bash
cd /home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects
source install/setup.bash
export ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1
python3 test/probes/scene_check.py
ros2 control list_controllers
ros2 topic echo /mujoco_scene/free_joint_states --once
ros2 topic echo /mujoco_scene/ready --once
```

Expect three active controllers, five named `loose_ball_1` through
`loose_ball_5` in the free-joint message, and scene readiness `true`. Wait for
the balls to settle before starting a transfer. For the first physical trial:

```bash
ros2 run rebotarm_mujoco_dm bowl_transfer_mission --limit 1
```

If that finishes with `PASS`, leave the simulation running. The default command
then moves the four balls still in bowl 1. Specify the source when both bowls
contain balls, so a retry after three placements cannot accidentally reverse direction:

```bash
ros2 run rebotarm_mujoco_dm bowl_transfer_mission --source bowl_1
```

Once all five are in bowl 2, run `ros2 run rebotarm_mujoco_dm bowl_transfer_mission`
to move all five back to bowl 1. On a fresh launch, this default command attempts
all five; after a complete pass the same command automatically reverses direction.

## What to inspect and report

1. In MuJoCo, check that five balls begin inside bowl 1 and bowl 2 is empty.
2. During each pickup, check whether the open fingers strike adjacent balls or
   the bowl rim. Check whether the selected ball rises with the gripper.
3. After each release, check the ball rests inside bowl 2. The command prints
   the remaining source/destination counts after each verified transfer.
4. Run the command again without restarting; check the direction reverses.
5. Paste the first error line and the surrounding launch/MoveIt logs if a motion
   is refused. Also report which numbered ball was being picked or placed and
   what moved in the MuJoCo and RViz viewers.

During pickup, the gripper first descends through the free space, then uses two
shorter stages near the ball at reduced requested speed. Each stage uses the
latest measured target-ball position. MoveIt permits finger
contact with the bowl and all five balls during this entry, while it continues
checking the rest of the arm, the table and robot self-collisions. MuJoCo
resolves the physical contact: nearby balls may move. The contact allowance is
removed after the measured lift and is also cleared on an error when the scene
bridge is responsive. A failed pickup does not count as a transferred ball.

If the gripper trajectory reports a path-tolerance stop while closing, the
command checks fresh finger positions and ball-in-TCP alignment. It accepts the
stop only when those measurements match a ball between the fingers, then still
requires a stable measured hold and lift. An unverified close or entry contact
triggers a retreat and a new measured antipodal approach angle, up to three
attempts. It stops if the selected ball leaves the bowl or the retreat fails.

The command stops on a planning failure, stale state, a lost grasp or an
unsettled/misplaced ball. It does **not** automatically open the fingers on a
failure. Inspect the scene before retrying. Continue a partial batch with
`--source bowl_1` or `--source bowl_2`. If a ball is outside both bowls, the
command refuses to start another batch.
Keep one motion client active at a time.

## Current validation

- The package builds and the new entry point is installed.
- The generated scene loads on MuJoCo 3.12.0 and 3.15.0. At all four model
  keyframes, the five balls settle inside bowl 1 after five simulated seconds
  without a MuJoCo warning; both engines report `nq=43`, `nv=38`, `nu=7`.
- Software checks cover ball classification, other-ball MoveIt obstacles, and
  existing single-ball failure paths.
- **15 October 2026 audit:** one physical bowl-to-bowl transfer passed in the
  headless ROS/MuJoCo stack. The next pickup stopped at 91.4% of its requested
  Cartesian descent. No descent was sent. The full batch and reverse batch
  therefore remain unvalidated.
- **Collision diagnosis:** a read-only probe from above ball 4 found that the
  unchecked descent was complete, while the checked descent first touched ball 3
  and then the bowl with both fingers. The staged, measured-contact entry now
  addresses that specific planning refusal; a live five-ball run is still required.
- **Next live observation:** the first-ball trial completed its three entry
  stages but a `FollowJointTrajectory` goal reported path-tolerance violation
  before a hold was verified. The earlier log did not identify which controller
  issued that result. Action errors now name the controller; measured finger
  contact is checked before considering a blocked close successful. This change
  has not yet been verified in a live pickup.

## Server and startup audit — 15 October 2026

The audit used a separate headless simulation on ROS domain 167. It did not
kill or restart individual nodes in the user's ROS graph. The package build
and 36 software tests passed, and the new `scene_check` reported:

- One publisher each for `/joint_states`, `/clock`, ball states, and scene readiness.
- One status publisher per required action: MoveGroup, ExecuteTrajectory,
  SDK-compatible MoveToPose, arm trajectory and gripper trajectory.
- Three active controllers, both bowls and all five balls in MoveIt's world.
- Controller list, planning-scene read, and state-validity responses in roughly
  0.8, 2.4 and 1.8 milliseconds in the measured sample.
- Advancing simulation time, fresh ball/TCP messages, no latched grasp fault,
  and a collision-free measured arm state after the startup adjustment.

This is a responsiveness check over the observation period, not proof that every
possible goal can execute. Planning rejection and contact failures remain distinct
from missing or unresponsive servers.

The first ball was lifted to Z = 0.1204 m and placed in bowl 2; the measured
counts were four source balls and one destination ball. The next selected grasp
was `loose_ball_3` with yaw -30 degrees. Its descent returned 91.4% and was
rejected before execution. The exact collision or kinematic constraint has not
been identified in this audit. No path
threshold was loosened to make that trial proceed. GUI-load behavior and a full
forward/reverse batch still need checks. The isolated launch was stopped after
the audit; its child processes reported exit, with SIGINT teardown errors that
are separate from the runtime transfer results.

An initial trial from home was rejected with MoveIt error -2: small measured
joint offsets led to reported `link2`/`link5` contact near the joint stops. The
two-bowl launch now initializes the robot from the existing `ready` keyframe
values in `config/initial_positions_two_bowls.yaml`; that configuration passed
the MoveIt state-validity check. No collision pair was disabled for this change.

The earlier `_set_parameters` callback-name collision with the ROS Node base
class is fixed by `_set_active_ball_parameters`. Further changes from this audit:

- Keep a dedicated executor attached to the grasp client through action/service
  waits. Report the exact stale signal or measured centre error on a stop.
- During cancellation, a terminal action result confirms the motion stopped;
  do not wait for a redundant cancellation acknowledgement after completion.
- Reject latest-TF fallback when it differs from the ball timestamp by more than
  50 ms. Old TF must not create apparently fresh grasp measurements.
- Print a warning if a planning-scene update remains unanswered beyond two seconds.
- Shut down the complete launch if a core MoveIt, scene-bridge or pose-adapter
  process exits. A replacement MoveIt process would otherwise lose its world and
  attachment state while the bridge believed the static scene was already sent.
- Reject approximate grasp candidates when none has the required rim and neighbor
  clearance. Keep the Cartesian collision check and measured hold verification.
- Require an explicit source bowl after a partial batch; the former fuller-bowl
  rule could reverse a failed batch once three of five balls had been placed.

`ROS_DOMAIN_ID=76` is simply the chosen ROS discovery domain; use the same value
in both terminals. `ROS_LOCALHOST_ONLY=1` restricts discovery to this computer.
They are optional for a single-computer simulation but must be consistent across
its processes. Restart the complete launch after changing them. Killing the ROS
CLI discovery daemon does not restart MoveIt or the controllers.

If `scene_check` fails, read its `errors` list before trying another motion. Stop
the simulation launch with Ctrl+C and restart it as a unit after a core-node
failure. Avoid starting a second launch in the same domain or running multiple
motion clients together. A healthy `scene_check` followed by an incomplete
Cartesian path points to a planning/geometry issue rather than a jammed server.

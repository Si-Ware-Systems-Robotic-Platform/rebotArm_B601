# B601 DM physical grasp adjustments — 14 October 2026

Status: the user verified `--stage close`, the full `--stage lift` check,
and a complete `--stage transfer` run. The latter printed
`PASS: measured lift, transfer, release and ball position inside bowl` after
the shorter retreat and bowl rolling-contact changes. This establishes the
one-ball, tabletop-to-bowl trial. Picking from a filled bowl is a separate
Phase 3 check.
The earlier ball-jump report still has no recorded trace pinpointing its cause.

## Changes

- Retained the actual DM finger meshes, their collision geometry and the 42.7 mm,
  45.9 g dynamic sphere. The optional Blender golf-ball mesh remains visual only.
- `dm_scene.xml` (bowl) and `dm_table.xml` now define explicit ball/inner-finger
  contact pairs using `finger_left_link_collision_0_3` and its right counterpart.
  These contacts use `condim=6`, sliding friction 1.2, torsional friction 0.001 m,
  rolling friction 0.0003 m, `solref="0.01 1"`, `solimp="0.95 0.99 0.001"`.
  The ball's general friction is restored to `0.6 0.0005 0.0001`. The explicit
  ball/table pair is preserved. Friction tuning is restricted to relevant pairs.
- Enabled elliptic friction cones, `impratio=10`, and one NoSlip iteration in
  `dm_arm.xml` and its generator. The NoSlip pass suppresses gradual contact
  creep observed during a long static grasp. This global solver setting also
  affects table, bowl and other contacts; the full scene was rechecked for
  settling, but the older rolling measurements are not a new calibration.
- Gripper actuator: `kp=800`, `kv=8`, force limited to ±6 N. This is the force
  of the driven slide actuator, not the force per finger and not a hardware rating.
  The original joint-level effort bounds remain; the actuator limit is tighter.
- Grasp command: close towards `finger_left=0.018 m` over three simulation seconds,
  instead of forcing the target to zero. The expected blocked position with this
  ball is about 0.0212 m. Normal gripper trajectory/goal tolerances are 10/2 mm;
  the grasp close alone permits 6 mm error to accommodate contact.
- Added a stationary check after closing. A valid candidate requires fresh finger
  measurements, opening consistent with the ball, near-symmetric fingers, and a
  stable ball centre near `[-0.014, 0, 0]` in the TCP frame. Before attachment it
  must also maintain a lift of at least 45 mm above the nominal table-rest height.
- Approach, lift, transfer and retreat use MoveIt's Cartesian-path service with
  collision checking, 2 mm steps, joint jump checks, and velocity/acceleration
  scaling 0.1. Partial paths are rejected. Returned trajectory timing is stretched
  to at least five seconds, never sped up. The initial move above the ball still
  uses the SDK-compatible pose action.
- Ball/TCP comparisons use matching simulation timestamps. A short queue allows
  TF to arrive after ball state; stale TF cannot be mistaken for a verified hold.
- During the first lift, the grasp client already monitors the ball/TCP offset
  and cancels if it departs by 8 mm or measurements become stale. The final
  stationary check now uses that same 8 mm centre limit; it still rejects more
  than 3 mm of recent relative motion, stale measurements, insufficient lift,
  or implausible finger opening. A centring failure reports the measured offset.
- After attachment, an offset change over 8 mm, missing matching TF for 0.5 wall
  seconds, or stale ball state latches a grasp fault. The bridge restores the
  measured world collision object and publishes scene readiness false. The grasp
  client cancels its active trajectory; the existing pose adapter also cancels its
  own active goal when readiness is lost. Restart after a fault.
- Deliberate release removes the planning attachment before opening the fingers.
  This avoids treating expected release as an accidental slip. Nothing welds,
  teleports, or sets the ball's physics state during the ROS grasp sequence.
- The attachment scene diff no longer explicitly removes the world ball:
  MoveIt removes it when processing the attached object. Steady scene updates
  no longer ask MoveIt to detach an object before one is attached. A slip or
  simulation reset still requests detachment once, then restores the measured
  world ball.
- The post-release vertical retreat target is now TCP Z = 0.16 m. The former
  Z = 0.20 m request stopped after 87.7% of its collision-checked path, after
  the ball had been released. The shorter target stays above the 48 mm rim and
  inside the previously returned path segment; the user verified it live.
- The bowl bottom now has a ball-specific six-dimensional contact pair with
  estimated rolling resistance 0.001 m. This affects ball/bowl contact only;
  it does not change gripper or table contact values.
- Final transfer success also requires fresh ball poses that move no more
  than 2 mm over the last simulation second. Position inside the bowl alone
  can no longer count as a settled ball.

The live hold check uses measured motion and finger opening. It does **not**
provide direct proof of two contact forces: the installed free-joint publisher
supplies object pose/twist, not a contact-force stream. The offline contact test
checks both actual MuJoCo finger contacts. Inspect contact/force visualization in
the viewer when diagnosing a failed live trial.

Fault monitoring is enforced by this grasp client and the pose adapter. It is not
a global interlock for unrelated RViz goals or direct controller commands. Run
one motion client at a time. A stopped trial leaves the gripper's command intact;
it does not automatically open and drop a held object.

## Reference comparison

The supplied MyCobot sorter uses dedicated box pads, friction `4.0 0.3 0.03`,
`condim=6`, elliptic cones, `impratio=10`, and three NoSlip iterations. Its URDF
contains matching pad boxes. Its balls have the same nominal diameter/mass as
ours, but start on raised narrow cradles. Its gripper uses a rotary linkage; its
actuator gains cannot be copied directly to DM's linear fingers. We retained the
DM geometry because static inspection found symmetric inner-face contacts.

All contact values are simulation estimates. The real finger material, tabletop
and golf-ball construction have not been measured. These results demonstrate a
numerically stable isolated grasp, not calibration to a real B601/golf-ball pair.

## Offline results

| Check | Result |
|---|---|
| Package `colcon build --symlink-install --packages-select rebotarm_mujoco_dm` | Passed |
| Software unit tests, including scene attachment, centring, slip and bowl rest handling | 29 passed |
| Native MuJoCo 3.12 isolated DM fingers: close, lift 100 mm, hold, release | Passed |
| Python MuJoCo 3.15 equivalent test at 1 ms | Passed |
| MuJoCo 3.15 test at 0.5 ms | Passed; hold drift essentially unchanged |
| MuJoCo 3.15 isolated 150 s static hold, then release | Passed; no measurable drift or warnings |
| Deliberate 120 mm miss | No lift, no finger/ball contact, no false grasp |
| Full scene at home, ready, pose_a and pose_b on 3.12 with one NoSlip iteration | Settled; zero warnings |
| Bowl drop with 0.15 m/s sideways velocity and corresponding spin on MuJoCo 3.12 | Ball settled inside bowl; zero warnings |
| Same 20 s bowl test on MuJoCo 3.15 | Ball settled inside bowl; zero warnings |
| Generator vs checked-in model option/gripper-actuator settings | Agree |
| Live close and first 100 mm lift before attachment | User confirmed |
| Fresh live lift through MoveIt attachment and three-second hold | User confirmed; ball Z = 0.1228 m, finger_left = 0.0212 m |
| Carry and release into the bowl | User verified |
| Complete transfer including retreat and settled-ball check | User verified; command printed `PASS` |

The isolated fixture uses the actual DM finger collision meshes, inertias, mimic
constraint, actuator and scene contact pairs. A driven vertical wrist slide
replaces the arm for this test; it supplies controlled motion while the ball is
fully dynamic. This isolates contact behaviour and does not validate arm IK,
MoveIt paths, the full robot's tracking error, or bowl transfer.

Measured at 1 ms on both engines with one NoSlip iteration: peak gripper
actuator force **2.5394 N**; peak finger/ball penetration **0.203 mm**;
bilateral contact throughout the measured hold interval; essentially zero
drift in the isolated **150-second** static hold; held ball centre about
**0.121 m**; released ball settles back on the table; zero warnings. Before
NoSlip, the isolated ball fell after roughly 120–150 simulated seconds,
matching the user's long-hold observation.

## Live attachment failure and correction

The user reported `--stage close` completed with ball Z = 0.0209 m and
`finger_left = 0.0212 m`. Separate fresh `--stage lift` and `--stage transfer`
trials each reached ball Z = 0.1221 m, with the same blocked finger position.
Both then returned `MoveIt refused the scene update` immediately after the
physical lift, before carrying toward the bowl. This confirms a real lift,
not a completed MoveIt attachment or transfer.

After that fix and one NoSlip iteration, the user again saw successful tabletop
close at ball Z = 0.0206 m with `finger_left = 0.0212 m`. Both lift and transfer
stopped after raising the TCP to 0.107 m with `Ball is not centred between the
fingers`. The grasp client accepted offsets below 8 mm throughout motion, but
the stationary check rejected offsets over 6 mm. Since the 1.5-second settle
check kept passing, an offset between those limits is a likely explanation;
the exact ball-in-TCP position was not logged, so this is not proof that the
ball was still securely held. The centre limits are now both 8 mm and any new
failure will print the measured error.

On the next fresh live `--stage lift` run, the tabletop hold passed at ball
Z = 0.0206 m and finger_left = 0.0212 m. The ball reached Z = 0.1228 m,
MoveIt's attachment transaction passed its scene verification, and the ball
remained at Z = 0.1228 m with the same finger position after a three-second
hold. The command printed `LIFT stage complete`. This validates this lift
stage, not carrying or releasing the ball into the bowl.

The corresponding `move_group` log at simulation timestamp 1791965258.632
reports: `Tried to remove world object 'loose_ball_1', but it does not exist
in this scene.` The bridge sent both an attached-object ADD and a world-object
REMOVE for the ball. MoveIt processes the attachment first and removes the
same world object automatically; the later explicit REMOVE fails the planning
scene service result. Repeated unattached-state updates also sent a needless
attached-object REMOVE, producing `Attached body ... not found` log messages.
Both redundant operations have been removed. The bridge now reads MoveIt's
scene after each attach/detach request and checks that the ball exists in
exactly the expected place. The attachment part passed in the subsequent
live lift trial; detachment still needs a live transfer/release check.

## Live transfer result and bowl contact

The user ran `--stage transfer` after restarting. The ball was lifted to
Z = 0.1228 m, carried to the bowl, and held at Z = 0.1057 m above it. The
gripper then released the ball, and the user saw it land in the bowl. The next
Cartesian request was a vertical TCP retreat from Z = 0.09 m to Z = 0.20 m.
MoveIt returned a partial path of **87.7%**, so no retreat motion was sent and
the final measured bowl-position check never ran. The user also observed the
ball continuing to move in the bowl. This is a partial pick/place success,
not a completed `grasp_test` transfer pass.

The scene had rolling resistance on the table but only default sliding
contact on the bowl bottom. A native MuJoCo 3.12 comparison starts the ball
at `(0.30, -0.14, 0.1057)` m with 0.15 m/s sideways speed and matching spin.
Without the new ball/bowl pair, its linear speed was still 0.00356 m/s after
20 seconds. With the pair, it reached practical rest at 0.433 seconds and
remained inside the bowl at `(0.30, -0.1058, 0.0250)` m, with zero warnings.
MuJoCo 3.15 gave the same final position and negligible speed. This numerical
test demonstrates controlled settling for that initial condition; actual
release motion and the complete ROS trial still need live validation.

Repeat the native comparison after building `test/validation/model_probe.cpp`:

```bash
python3 test/validation/validate_bowl_physics.py --probe /tmp/dm-model-probe-3.12
```

Reproduce from the package source directory:

```bash
python3 test/validation/validate_grasp_physics.py --export-xml /tmp/dm-grasp-fixture.xml
python3 test/validation/validate_grasp_physics.py --miss
python3 test/validation/validate_grasp_physics.py --timestep 0.0005
python3 test/validation/validate_grasp_physics.py --hold-seconds 150

g++ -std=c++17 -O2 test/validation/grasp_probe.cpp \
  -I/opt/ros/humble/opt/mujoco_vendor/include \
  -L/opt/ros/humble/opt/mujoco_vendor/lib \
  -Wl,-rpath,/opt/ros/humble/opt/mujoco_vendor/lib \
  -lmujoco -o /tmp/dm-grasp-probe-3.12
/tmp/dm-grasp-probe-3.12 /tmp/dm-grasp-fixture.xml
```

## Run the staged ROS checks

Stop the old simulation launch before starting the updated model. Restart the
launch between each stage so the ball and robot begin from their initial states.
A completed close/lift stage deliberately leaves the fingers holding their target.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
cd ~/si_ware/documents/rebot_arm_b601/ws_projects
colcon build --symlink-install --packages-select rebotarm_mujoco_dm
source install/setup.bash
ros2 launch rebotarm_mujoco_dm simulation.launch.py \
  scene:=bowl use_moveit:=true ball_visual:=mesh
```

Terminal 2 (source once in every new terminal):

```bash
source /opt/ros/humble/setup.bash
source ~/si_ware/documents/rebot_arm_b601/ws_projects/install/setup.bash
python3 test/probes/grasp_test.py --stage close
```

Expected: approach, slow close, stationary measurement check, then stop. The ball
should remain between the fingers on the table. No transfer occurs.

After restarting Terminal 1's launch:

```bash
python3 test/probes/grasp_test.py --stage lift
```

Expected: close, verify, lift 100 mm, verify the lifted object, then hold for three
simulation seconds. `grasp_test` without a stage now defaults to this lift test.

After a successful lift check and another fresh launch:

```bash
python3 test/probes/grasp_test.py --stage transfer
```

Expected: lift, carry above the bowl, release, retreat, and check the measured
ball position inside the bowl. An unreachable/incomplete Cartesian path produces
an error and sends no partial trajectory; paste that output instead of disabling
collision checking.

Optional deliberate miss on a fresh launch:

```bash
python3 test/probes/grasp_test.py --miss
```

Observe these while testing (separate terminal):

```bash
ros2 topic echo /mujoco_scene/grasp_fault
ros2 topic echo /mujoco_scene/grasp_held
ros2 topic echo /mujoco_scene/ball_in_tcp
```

Run one echo at a time, or use separate terminals. A successful trial has no
fault message, `grasp_held: true` only after verified lift/attachment, and a nearly
constant ball/TCP offset while held. `grasp_held` becomes false on deliberate
release. Send the stage output and say where any unexpected motion begins.

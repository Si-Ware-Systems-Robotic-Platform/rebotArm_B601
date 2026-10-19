# CAD slider scene and measured release mission

## What was corrected

The previous scene rendered the CAD slider but used 88 invisible boxes as its
physical channels. Those hand-written channels did not follow the CAD grooves,
especially around the bends. A ball could therefore rest inside the visible
surface or hit an invisible wall. The old "sliding" message only meant that the
gripper had opened; it did not verify the slide.

The revised scene uses convex collision parts decomposed from the normalized CAD
mesh. Both MuJoCo and MoveIt consume the **same inline vertices and faces**. The
visual remains the original detailed surface. The approximate decomposition must
be inspected and tested; a single convex hull of the entire slider would close
its grooves. There are no guessed box tracks or extra invisible base plate. The cache contains
1,081 convex parts with a 2 mm CoACD concavity threshold; this is an approximation,
not a claim of exact surface agreement everywhere.

## Coordinates and the V1.1 export

The slider is fixed at `(0.45, -0.04, 0.004451607)` in the robot base frame.
It has a 180-degree yaw and a **1.5-degree incline** raising its inlet, and its
low edge rests directly on the tabletop; there is no mount plate.
The MJCF wxyz quaternion is `(0, 0.013089596, 0, 0.999914328)`. The CAD mesh
itself is unchanged.

Why the incline: the CAD inlet grooves are nearly level. At the specified drop
markers, the faithful level collision model stalled one ball and let two roll
back out of the entrance. The small incline preserves your local CAD markers
while producing gravity-driven travel. It is a deliberate placement assumption,
not a change to the CAD surface. `mount_pitch_deg` in the YAML controls it; rerun
the generator to update the scene placement and all world drop targets.
Setting it to 0 makes the slider level, which does not guarantee rolling.

A blue catch tray (`slider_catch`) stands in front of the outlet. Balls leaving
the slider roll towards the robot base. A ball touching the base puts every arm
state in collision in MoveIt, which then cannot plan the next transfer. The
tray's position and size are under `slider.catch` in the YAML.

The exported `base_link.STL` is offset from the normalized visual mesh by
`(+0.120000, +0.02901822685, +0.15794752160)` metres. The Ball1/2/3 exports also
contain the **slider**, not a sphere. Applying each Ball joint transform to its
corresponding mesh reproduces the normalized visual to within 1 micrometre; the
collision builder checks this. The original CAD files are preserved. Do not
import their duplicate geometry or 5.44 kg inertias as golf balls.

The fixed Ball joints are ball-centre markers, transformed as:
with the same rigid transform as the visual/collision mesh. For incline angle a:
`world = (0.45 - cos(a)*x + sin(a)*z, -0.04 - y,
0.004451607 + sin(a)*x + cos(a)*z)`.

| Marker | World X (m) | World Y (m) | World Z (m) |
| --- | --- | --- | --- |
| Ball1 | 0.334175705 | 0.068525867 | 0.165486237 |
| Ball2 | 0.334175705 | -0.000518227 | 0.165486237 |
| Ball3 | 0.334175705 | -0.069018227 | 0.165486237 |

Sites `slider_entry_1..3` show these locations (site group 4). Table pickup balls
start at X=0.26, 0.20, 0.14 m, Y=**+0.18** m, Z=0.02135 m. Their physical shapes
are native spheres, radius 21.35 mm and mass 45.9 g. `ball_visual:=mesh` changes
appearance only.

## Contact settings

Slider collision geoms use priority 1, condim 6, and friction
`0.25 0.0001 0.00003` (sliding, torsional, rolling). This explicitly selects the
slider's contact parameters instead of unintentionally taking the ball's larger
sliding coefficient. Finger/ball and table/ball explicit contact pairs remain.
These are initial simulation estimates, not measured material properties.

Collision parts have translucent green colour in geom group 3. Enable that group
in MuJoCo's visualization controls to inspect them; disable visual group 2 if
needed to see the collision surfaces clearly. Other robot/table collision parts
also belong to group 3.

## Mission behaviour

1. Select a ball and verify fresh observations.
2. Pick it from the table, verify actual lift, then attach its planning shape.
3. Compute release TCP from the CAD ball-centre marker and **measured ball-in-TCP
   offset**, rotated to the commanded orientation (+90 degrees about Y, yaw 0).
4. Approach `approach_height` (15 mm) above the release TCP, then descend.
   A fixed high approach is unreachable: with the gripper horizontal, joint4
   reaches its +1.57 rad limit about 20 mm above the release pose.
   Release 15 mm above the marker. If the measured ball centre is more than
   3 mm (X/Y) or 6 mm (Z) off the release target, the TCP is shifted to cancel
   the miss, up to 3 times, before giving up.
5. Open, lift `approach_height` straight up, then retreat in joint space, with collision checking. Slider collision permissions are
   not disabled for the transfer/release. The bridge now restores ball–obstacle
   permissions as well as finger permissions when leaving contact-entry mode.
6. Observe travel while opening and retreating, then require a measured trip
   around the bend and through an outlet before reporting PASS.

The outlet gate is in slider-local coordinates: X >= 0.150 m, Y in [0.04, 0.19] m,
Z in [0.015, 0.09] m, after reaching X < -0.03 m around the bend. Five seconds with
less than 3 mm movement reports a stall. Leaving the track elsewhere, stale
observations, clock reset, or no exit within 30 simulation seconds reports a
failure. A failure stops the sequence; it does not teleport or force the ball.
A restart is appropriate before repeating a completed mission, because its balls
are now at the outlets rather than their initial pickup positions.

## Build and run

Stop the existing simulation normally before launching the updated scene.

```bash
source /opt/ros/humble/setup.bash
cd ~/si_ware/documents/rebot_arm_b601/ws_projects
colcon build --symlink-install --packages-select rebotarm_mujoco_dm
source install/setup.bash
ros2 launch rebotarm_mujoco_dm simulation.launch.py scene:=slider use_moveit:=true
```

RViz starts with **MotionPlanning → Scene Geometry → Show Scene Geometry** off.
Redrawing the slider's 1,081 collision pieces on every planning-scene update froze
RViz. Tick it to inspect the collision shapes. The scene bridge also sends MoveIt
an update only when a ball moves by 1 mm or more, rather than every 100 ms.
Add `use_rviz:=false` to the launch when RViz is not needed.

In terminal 2, use the same ROS domain as terminal 1:

```bash
source /opt/ros/humble/setup.bash
cd ~/si_ware/documents/rebot_arm_b601/ws_projects
source install/setup.bash
ros2 run rebotarm_mujoco_dm slider_mission --ball loose_ball_1
```

After the single-ball check, run `--ball loose_ball_2` and `--ball loose_ball_3`,
or restart the scene and run `ros2 run rebotarm_mujoco_dm slider_mission` for all
three. Paste any planning/controller failure or measured stall; a successful
pick/release alone is not evidence of successful slider travel.

## Source files and regeneration

- `tools/build_slider_collision.py`: offline CAD decomposition and frame checks;
  requires numpy, trimesh and CoACD (generated with trimesh 5.1.1 / CoACD 1.0.14). The cached result is included, so users do
  not need those decomposition dependencies to run the scene.
- `scene_assets/slider_collision.xml`: cached convex pieces and visual SHA256.
- `tools/generate_slider_scene.py`: embeds the cached pieces, CAD marker sites and
  catch tray into `models/scenes/dm_slider.xml`.
- `config/scenes/slider_params.yaml`: release marker positions and mission timing.
  The generator rewrites the positions; tolerances and timing you set are kept.
- `test/validation/validate_slider_physics.py`: gravity-only native MuJoCo drops, with the
  scene contact geometry and robot model's physics settings; starts no ROS nodes.

If the CAD surface changes, rebuild the collision cache, regenerate the scene,
and repeat validation. The generator refuses a stale collision cache when
the visual mesh hash changes. It also regenerates mission marker coordinates from
the CAD joints and the configured mounting angle. Editing the generated XML alone is not persistent.

```bash
python3 src/rebotarm_ros2/src/rebotarm_mujoco_dm/tools/build_slider_collision.py
python3 src/rebotarm_ros2/src/rebotarm_mujoco_dm/tools/generate_slider_scene.py
python3 src/rebotarm_ros2/src/rebotarm_mujoco_dm/test/validation/validate_slider_physics.py --perturbations
```

See [cad-to-mujoco-guide.md](cad-to-mujoco-guide.md) for the general pipeline when
adding a new CAD part.

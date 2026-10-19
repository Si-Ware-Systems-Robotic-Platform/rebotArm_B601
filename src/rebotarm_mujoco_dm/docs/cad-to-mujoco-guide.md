# Bringing a CAD part into MuJoCo and MoveIt

This is the pipeline that brought the SolidWorks slider into the simulation
without "ghosting", where balls pass through walls or ride on top of grooves.
Use it as the template for any new CAD part that balls or the arm must touch.
The slider files are the worked example throughout; see
[slider-scene.md](slider-scene.md) for the slider's own numbers.

## 1. Why a CAD mesh cannot be used directly

MuJoCo collides every mesh geom as its **convex hull**. A hollow or grooved
part, such as a track, chute, bowl or tray, therefore behaves like a solid
block. Balls rest on top of the grooves or stop against invisible faces.
Hand-placed boxes do not fix this; they never follow curved grooves or bends
exactly. The earlier slider used 88 such boxes and balls still stalled or
passed through the visible surface.

The fix is **convex decomposition**: split the surface into many small convex
pieces that together follow the grooves, walls and outlets. CoACD does this
automatically. The slider uses 1,081 pieces with a 2 mm concavity threshold.

The detailed CAD mesh stays as the **visual only** (`contype="0" conaffinity="0"`).
The convex pieces do all the contact (`contype="1" conaffinity="1"`) and are
also sent to MoveIt, so physics and planning see the same shape.

## 2. Export from SolidWorks / Inventor

1. Export with the URDF exporter (SolidWorks to URDF, or the Inventor
   equivalent) in **metres**.
2. Add a **reference frame/joint for every point the mission needs**: drop
   targets, ball-centre positions, sensor points. The slider export has
   fixed joints `Ball1`, `Ball2`, `Ball3` at the ball-centre release points.
   These joints are markers only; the generator reads their `origin xyz`.
3. Copy the export folder into the package unchanged (for the slider:
   `new scene/Slider_URDF&Meshes_V1.1/`). Keep the original; never edit it.
4. Copy the part's main STL to `scene_assets/<part>_base_link.stl`. This is the
   normalized visual mesh that everything else is measured against.

Check the export frame. The V1.1 slider export placed `base_link.STL` at an
offset of `(+0.12, +0.029018, +0.157948)` m from the normalized visual, and
each `BallN.STL` contained the whole slider expressed in that marker's frame.
`build_slider_collision.py` verifies that every `BallN.STL` plus its joint
offset reproduces the visual mesh to 1 µm and refuses to continue otherwise.
A new part needs the same kind of check, because a silent frame error moves
every drop target.

## 3. Build the collision parts (offline, once per CAD change)

```bash
pip install numpy trimesh coacd        # only needed for this step
python3 tools/build_slider_collision.py
```

The script:
- runs CoACD on the visual STL (`threshold=0.002`, `seed=0`, `merge=False`,
  so the result is reproducible);
- writes `scene_assets/slider_collision.xml` with one inline `<mesh>`
  (explicit `vertex`/`face` lists) and one `<geom>` per convex piece;
- stores the visual STL's **SHA256** in that file.

Inline vertices are required, not mesh files: the scene bridge converts the
same geoms into MoveIt `shape_msgs/Mesh` collision objects and rejects
file-based collision meshes.

The collision geoms carry the contact settings: `priority="1"` so the part's
friction wins over the ball's, `condim="6"` for rolling friction, group 3 and a
translucent green colour for inspection.

For a new part, copy `build_slider_collision.py`, change `VISUAL`, `OUTPUT`,
the geom name prefix and the export-frame check. Lower the threshold (for
example 0.001) if narrow grooves still close up, at the cost of more pieces.

## 4. Generate the scene

```bash
python3 tools/generate_slider_scene.py
```

`generate_slider_scene.py` builds `models/scenes/dm_slider.xml` from the
table template `models/scenes/dm_table.xml`:

1. **Hash check.** It compares the visual STL's SHA256 with the one stored in
   the collision file and refuses to run if they differ, so the visual and
   collision shapes can never drift apart.
2. **Placement.** One body holds both the visual mesh and all convex pieces,
   so they share a single transform (position, 180-degree yaw, 1.5-degree
   incline, low edge resting on the table).
3. **Markers to targets.** For each CAD marker joint it adds a site
   (`slider_entry_N`, group 4) and computes the marker's world position with
   the same transform. Those positions are written to
   `config/scenes/slider_params.yaml` as `rails.N.entry_x/y/z`; the mission
   reads them to compute its release TCP. Never type target coordinates by
   hand.
4. **Supporting geometry.** It adds the catch tray at the outlet and the
   pickup balls with their explicit table and finger contact pairs.
   **Collision bits:** the slider pieces get `contype/conaffinity="2"`, the
   balls `"3"`, and everything else keeps the default `"1"`. Balls collide
   with the slider, but the arm's 307 collision meshes are never checked
   against the 1,081 slider pieces. Without this filter those checks slowed
   the simulation near the release pose until ball poses went stale and the
   mission stopped. MoveIt still keeps the arm clear of the part.
5. **Parameters.** It rewrites only the generated values in the YAML; the
   tolerances and timing you set there are kept.

## 5. Inspect and validate offline

- **Look at it.** In the MuJoCo viewer, enable geom group 3 (collision) and
  disable group 2 (visual). The green pieces must follow the grooves, walls
  and outlets. Enable group 4 to see the drop-target sites.
- **Drop test, no ROS.** `test/validation/validate_slider_physics.py
  --perturbations` drops each ball through the real collision pieces at the
  target and at the edge of the release tolerances. It requires every ball to
  pass the bend, exit the outlet and come to rest in the tray, with no MuJoCo
  warnings.

Only then run the ROS mission.

## 6. Template checklist for a new CAD part

| Step | Slider example | New part |
| --- | --- | --- |
| CAD export with marker joints | `new scene/Slider_URDF&Meshes_V1.1/` | `new scene/<Part>/` |
| Normalized visual STL | `scene_assets/slider_base_link.stl` | `scene_assets/<part>_base_link.stl` |
| Collision builder | `tools/build_slider_collision.py` | `tools/build_<part>_collision.py` |
| Collision cache (+ SHA256) | `scene_assets/slider_collision.xml` | `scene_assets/<part>_collision.xml` |
| Scene generator | `tools/generate_slider_scene.py` | `tools/generate_<scene>_scene.py` |
| Generated scene | `models/scenes/dm_slider.xml` | `models/scenes/dm_<scene>.xml` |
| Generated + tuned parameters | `config/scenes/slider_params.yaml` | `config/scenes/<scene>_params.yaml` |
| Launch entry | `"slider": "dm_slider.xml"` in `launch/simulation.launch.py` | add `"<scene>": "dm_<scene>.xml"` |
| Offline physics check | `test/validation/validate_slider_physics.py` | `test/validation/validate_<scene>_physics.py` |
| Outcome monitor | `core/slider_checks.py` | `core/<scene>_checks.py` |
| Mission | `tasks/slider_mission.py` (+ `setup.py` entry point) | `tasks/<scene>_mission.py` |

Rules that prevented the slider's problems:
- Never use a concave CAD mesh for contact; always decompose it.
- Visual and collision geometry live in one body with one transform.
- Every target comes from a CAD marker through the generator, never by hand.
- Edit the generator or the YAML, never the generated XML; regeneration
  overwrites the XML.
- Filter contacts with collision bits so only the objects that must touch a
  decomposed part are checked against it.
- Give balls somewhere to stop. A ball that rolls into the robot base makes
  every MoveIt plan fail.
- Keep approach poses inside the arm's reach. With the gripper horizontal,
  joint4 hits its limit about 20 mm above the slider's release pose.

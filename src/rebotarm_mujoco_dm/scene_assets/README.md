# Scene assets

> **14 October 2026:** This document retains earlier measurements/history. The current ball is 42.7 mm / 45.9 g; the Blender table texture and optional golf-ball visual are installed. See [current grasp settings and validation](../docs/grasp-tuning.md) for the revised solver/contact parameters and staged tests.

`tableobj.obj` and `tableobj.mtl` are byte-for-byte copies of the user's
`ws_projects/src/scene obj/` exports, added for scene Phase 1 on 13 October 2026.
The original files are retained. No licence beyond the user's supplied files
is asserted here.

Coordinates are metres, +Z up and -Y forward, as specified by the user.
There is no rotation in the scene. Bounds are X ±1.25, Y ±0.75, Z [-0.8, 0].
The source tabletop is 0.032 m thick. The scene now scales X by 0.4 and Y by
0.5333333333333333, retaining Z: the displayed table is 1.0 × 0.8 × 0.8 m.
All collision boxes use identical scaling. Collision boxes enclose each of the nine
connected components (top, four legs, four supports); they approximate bevels.
The mesh is visual only. MJCF supplies its colour; the MTL is kept for provenance.

The source contains quads and eight-vertex polygons. Engine loader results
are recorded in `docs/scene-phase1.md`; conversion is only needed if loading
fails on a target engine.

Phase 1 uses a native sphere of diameter 0.045 m and mass 0.045 kg.
The user's `golf_ball2.obj` is reserved for the later mesh-ball stage, retaining
the same +Z-up, -Y-forward convention. Ball–table rolling resistance is now an
explicit estimated contact pair; see `docs/ball-physics.md`. Grasp tuning remains pending.
The chosen bowl for the next stage is
`ws_projects/src/scene obj/urdf/mycobot_pro_450/meshes_mujoco/wooden_bowl.obj`.
Its own orientation/scale must be checked separately from the table and golf ball.

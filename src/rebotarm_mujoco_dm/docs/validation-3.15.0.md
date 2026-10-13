# Phase 1 validation — MuJoCo 3.15.0

Run date: 2026-10-11. **PASS** (all assertions).

## URDF agreement

Eight joints, seven actuators; names, order, axes, limits and effort limits match. All supplied masses and COMs match. Source inertia entries are preserved exactly; compiled principal-axis tensors agree within 1e-8 kg m². Each finger uses the documented 0.04 kg estimate. Base remains fixed.

Independent URDF forward kinematics compared with compiled MuJoCo body and TCP transforms at the same joint positions:

| Pose | Largest body position error (m) | Largest body angle error (deg) | TCP error (mm) | TCP angle error (deg) |
|---|---:|---:|---:|---:|
| home | 2.78e-17 | 9.2e-15 | 1.99e-20 | 9.2e-15 |
| ready | 1.76e-16 | 2.19e-14 | 1.11e-13 | 1.98e-14 |
| pose_a | 3.35e-16 | 2.87e-14 | 2.31e-13 | 2.87e-14 |
| pose_b | 1.67e-16 | 2.47e-14 | 1.19e-13 | 2.47e-14 |

All four reference poses have zero contacts. This verifies the geometry against the URDF used by RViz; it is not a live ROS/RViz synchronization test.

## Dynamics

Hold runs: 10 simulated seconds each. Gripper: 1 s target ramp followed by 4 s settling. Unpowered gravity fall: 5 s. Zero MuJoCo warnings and zero actuator force-limit excess in every run.

| Test | Final max arm target error (rad) | Peak joint speed* | Peak mimic error (m) | Peak soft-limit excess* | Peak contact penetration (m) |
|---|---:|---:|---:|---:|---:|
| home hold | 0.00538105 | 0.181225 | 2.89859e-08 | 0.000564561 | 0 |
| ready hold | 0.00772165 | 0.164779 | 5.42136e-09 | 0 | 0 |
| pose_a hold | 0.00756521 | 0.161155 | 9.37227e-06 | 0 | 0 |
| pose_b hold | 0.00553727 | 0.117165 | 2.91151e-06 | 0 | 0 |
| gripper open | — | 0.164779 | 5.45466e-06 | 0 | 0 |
| gripper close | — | 0.164779 | 9.66774e-06 | 3.65954e-05 | 0 |
| unpowered fall | — | 6.74361 | 4.43595e-07 | 0.028425 | 0.00582037 |

*Maximum across hinge and slide coordinates: rad/s or m/s for speed, rad or m for limit excess. These are numerical diagnostics, not hardware safety limits.

Position servos have finite gravity sag. Joint limits and equality constraints are soft, so small overshoots occur. The unpowered fall is a physics stress check; its speed and impact are not an allowed commanded motion. Contact geometry and finger inertias remain approximations. Trajectory tuning, ROS timing and action execution belong to Phase 2 onward.

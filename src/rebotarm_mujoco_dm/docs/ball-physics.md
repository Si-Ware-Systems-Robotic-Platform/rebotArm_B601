# Ball physics: experiments and calibration

> **14 October 2026:** This document retains earlier measurements/history. The current ball is 42.7 mm / 45.9 g; the Blender table texture and optional golf-ball visual are installed. See [current grasp settings and validation](grasp-tuning.md) for the revised solver/contact parameters and staged tests.

The current bowl scene also has a separate ball/bowl-bottom rolling contact
(`condim=6`, rolling coefficient 0.001 m). The user observed continued bowl
motion with the earlier sliding-only contact. See the controlled bowl comparison
in [grasp-tuning.md](grasp-tuning.md#live-transfer-result-and-bowl-contact).

Updated 13 October 2026. The user has not yet chosen a real tabletop surface or
ball. All material/contact values here are **initial simulation estimates**.
These checks demonstrate numerical behaviour, not a calibrated real-world match.

## What changed

The table is now **1.0 × 0.8 m**, with its original 0.8 m height and 32 mm top.
The visual mesh and all collision boxes share the same XY scaling. Its centre
is `(0.35, 0, 0)`; table bounds are X [-0.15, 0.85], Y [-0.4, 0.4]. The robot
base stays at the origin. The entire tabletop is not guaranteed reachable:
reachability depends on tool orientation and approach clearance. Start near
the existing ball at `(0.30, 0.10)`; map feasible grasp poses during scene Phase 2.

The previous contacts inherited `condim=3` and sliding friction 0.8. That supports
sliding friction, but not rolling resistance. A sphere that rolls without slip
can continue moving on an ideal plane even with substantial sliding friction.
`condim=6` adds resistance to rolling. [MuJoCo contact dimensions](https://mujoco.readthedocs.io/en/stable/computation/index.html#contact).

The new explicit `ball_table` pair in `models/dm_scene.xml` applies only to
`loose_ball_1_geom` and `table_top`:

```xml
<pair name="ball_table" geom1="loose_ball_1_geom" geom2="table_top"
      condim="6" friction="0.8 0.8 0.005 0.0005 0.0005"
      solref="0.02 1" solimp="0.9 0.95 0.001" />
```

| Parameter | Meaning and how to tune |
|---|---|
| First two friction values, 0.8 / 0.8 | Sliding resistance in the two contact-plane directions; dimensionless |
| Third, 0.005 | Resistance to spinning around the contact normal; length units in metres |
| Last two, 0.0005 / 0.0005 | Resistance to rolling about the contact-plane axes; length units in metres |
| `solref="0.02 1"` | Contact response time constant and damping ratio; affects compliance and impact response |
| `solimp="0.9 0.95 0.001"` | Contact impedance shape; influences compliance/penetration |
| Timestep, 0.001 s | Numerical integration resolution, not a material property |
| Mass/inertia | 45 g and a uniform 45 mm solid sphere; assumptions awaiting measurements |

An explicit pair has **five** friction entries. A geom has three, with sliding
and rolling each expanded to two directions. Editing only a geom's friction
will not tune this explicit pair. [MuJoCo pair reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-pair).
The rotational coefficients have length units and must not be interpreted as
ordinary dimensionless Coulomb coefficients. [MuJoCo friction definitions](https://mujoco.readthedocs.io/en/stable/computation/index.html#contact).

Do not increase every parameter at once. Arbitrary free-joint damping would
also slow the ball in the air; use contact resistance for a surface effect.
Ball–finger contacts have not been tuned by this pair. Solver/contact compliance
must be assessed separately from friction. [MuJoCo solver parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters).

## Repeatable offline tests

Viewer dragging is useful for exploration, but mouse motion does not specify
a repeatable physical impulse. Release the perturbation before judging motion.
Use controlled initial velocity for comparisons. These tools run an independent
offline model; they do not command the ROS simulator or hardware.

Compile the probe against the same MuJoCo library used by ROS:

```bash
cd ~/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm
g++ -std=c++17 -O2 test/validation/model_probe.cpp \
  -I/opt/ros/humble/opt/mujoco_vendor/include \
  -L/opt/ros/humble/opt/mujoco_vendor/lib \
  -Wl,-rpath,/opt/ros/humble/opt/mujoco_vendor/lib \
  -lmujoco -lglfw -o /tmp/dm_ball_probe

python3 test/validation/validate_ball_physics.py --probe /tmp/dm_ball_probe
```

This settles the ball for two seconds, then tests a three-second interval:

1. Pure rolling at 0.1 m/s without rolling resistance (`condim=3`).
2. The same initial state with rolling resistance (`condim=6`).
3. Sliding start at 0.1 m/s with zero initial spin, using the new contact pair.

Pure rolling starts with angular speed `v/r = 4.4444 rad/s`. No continuing push
is applied. Results include travel, final linear/angular speeds, warnings, and
time to practical rest. Practical rest means both linear speed and `radius ×
angular speed` remain below 1 mm/s for at least 0.25 seconds. A null stop time
means this condition was not sustained at the end of the observation interval.
The tool checks that the ball remains supported by the tabletop and verifies
the active contact dimensionality.

Test a longer roll, a larger rolling coefficient, and timestep sensitivity:

```bash
python3 test/validation/validate_ball_physics.py --probe /tmp/dm_ball_probe \
  --case rolling --seconds 20

python3 test/validation/validate_ball_physics.py --probe /tmp/dm_ball_probe \
  --case rolling --rolling 0.001 --seconds 6

python3 test/validation/validate_ball_physics.py --probe /tmp/dm_ball_probe \
  --case rolling --timestep 0.0005
```

Overrides change only temporary copies. They do not alter `dm_scene.xml` or a
running simulation. The no-resistance baseline eventually rolls off the finite
table, so reserve long-duration tests for a case that stays on the tabletop.

## Recorded numerical results

Both 3.12.0 and 3.15.0 agreed in the three-second tests, with zero warnings:

| Initial condition | X travel after 3 s | Final linear speed |
|---|---:|---:|
| Pure roll, rolling resistance inactive | 299.586 mm | 99.862 mm/s |
| Pure roll, coefficient 0.0005 m | 192.490 mm | 38.146 mm/s |
| Sliding start, coefficient 0.0005 m | 137.667 mm | 27.230 mm/s |

Additional tests on 3.12.0:

- Current estimate 0.0005 m: practical rest begins at about **14.342 s**;
  travel after 20 s is **310.729 mm**. Small residual motion remains below the
  rest threshold. A long roll is still possible on this smooth-surface estimate.
- Alternative 0.001 m: practical rest begins at about **3.731 s**; travel after
  6 s is **87.388 mm**. This is an alternative experiment, not the saved default.
- Halving timestep to 0.0005 s changes the current estimate's three-second
  travel from 192.490 to 192.512 mm, about **0.011%**. This supports numerical
  consistency for this one experiment; it does not validate real materials.

The resized scene also passed all four saved-pose reset/settling checks on both
engines. The settled ball centre is Z = 22.409145 mm, with about 0.090855 mm of
compliant contact penetration, zero warnings, and only ball–table contact.

## How to make it match a real setup later

1. **Choose and measure geometry.** Record actual ball diameter, mass, surface,
   tabletop coating, and levelness. Replace the solid-sphere inertia assumption
   if the chosen ball's mass distribution needs a better model. Visual material
   colour or dimples do not automatically set physical properties.
2. **Measure rolling.** Film the ball against a ruler with known frame timing.
   Estimate speed from successive positions after the initial push and after
   slipping has ended. Repeat at several starting speeds and in both directions.
   Compare distance-versus-time and speed decay, not just eventual stopping.
   Change only the last two pair friction values first. Unequal directional
   results can indicate table tilt or surface differences, not just friction.
3. **Measure sliding separately.** A freely rolling ball on an inclined board
   does not directly measure sliding friction. Use a test that constrains
   rotation or otherwise identifies slip. In an ideal Coulomb sliding test,
   `mu ≈ deceleration/g`; do not apply that estimate to ordinary rolling.
4. **Measure impact.** Drop the real ball vertically from known bottom clearance
   above the table and record rebound height/timing. Repeat several heights.
   An ideal first estimate is `e ≈ sqrt(rebound_height/drop_height)`, measured
   consistently relative to the contact level. Compare a simulated drop with
   the same initial state; tune contact compliance/damping next. `solref` is
   not a direct restitution-coefficient input.
5. **Separate material fit from numerical stability.** Compare the same test
   at 1 ms and 0.5 ms timesteps. Watch maximum penetration, energy growth,
   persistent jitter, and warnings. Improve numerical convergence before
   interpreting solver artefacts as physical friction.
6. **Validate with unused trials.** Fit a small set of measurements, then test
   different speeds/heights/directions. Choose tolerances based on measurement
   uncertainty; a provisional 10% stopping-distance target is a project choice,
   not an engine guarantee. Grasping needs additional finger-material and force
   tests; table rolling calibration alone cannot establish a realistic grasp.

Until a real reference is selected, report these settings as stable, testable
simulation estimates. “Looks realistic” and “no warnings” are useful checks,
but neither establishes real-world accuracy.

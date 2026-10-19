# reBotArm B601-DM Documentation Index

This directory centralizes all design, physics validation, and task documentation for the `rebotarm_mujoco_dm` package.

---

## 1. Active Scenes & Tasks
* **[`slider-scene.md`](slider-scene.md)**
  * The CAD 3-rail slider resting on the table, catch tray, measured release and slide verification.
* **[`cad-to-mujoco-guide.md`](cad-to-mujoco-guide.md)**
  * Reference pipeline for bringing any SolidWorks/Inventor part into MuJoCo and MoveIt (CoACD collision, markers, generator, validation).
* **[`rail-scan-scene.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/rail-scan-scene.md)**
  * Details the 3-lane inclined sorting rails, optical scan station, and 6-ball material classification puzzle.
  * Coordinates, collision geometry, tolerances, and launch instructions.

---

## 2. Physics & Grasp Validation
* **[`ball-physics.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/ball-physics.md)**
  * Analysis of golf ball rolling physics, contact solref/solimp parameters, and incline dynamics.
* **[`grasp-tuning.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/grasp-tuning.md)**
  * Friction tuning, contact pair definitions, slip thresholds, and grip verification tests.

---

## 3. Middleware & Planning Adapters
* **[`pose-adapter.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/pose-adapter.md)**
  * Architecture of the MoveToPose action server bridging ROS 2 actions to MoveIt 2 Cartesian and joint trajectories.

---

## 4. Historical Milestones & Phase Reports
* **[`scene-phase1.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/scene-phase1.md)**: Phase 1 tabletop arm bringup.
* **[`scene-phase3.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/scene-phase3.md)**: Phase 3 bowl transfer setup.
* **[`phase3-validation.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/phase3-validation.md)**: Multi-ball transfer validation metrics.
* **[`validation-3.12.0.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/validation-3.12.0.md)** & **[`validation-3.15.0.md`](file:///home/ahmedmahmoud/si_ware/documents/rebot_arm_b601/ws_projects/src/rebotarm_ros2/src/rebotarm_mujoco_dm/docs/validation-3.15.0.md)**: MuJoCo engine compatibility notes.

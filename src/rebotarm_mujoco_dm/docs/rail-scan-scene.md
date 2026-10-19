# DM rail and scan scene (Sorting Puzzle)

`scene:=rail_scan` uses the B601 DM arm, its seven actuators, its gripper and
the existing wood table. The rail, six free golf balls (two per rail) and scan-station
geometry come from `ws_projects/src/scene obj/folders/mycobot_pro_450_ball_sorter.xml`.
`tools/generate_rail_scan_scene.py` extracts these parts into
`models/scenes/dm_rail_scan.xml`.

### Initial Puzzle Setup (Derangement)
Three lanes: HDPE ($Y=-0.10$ m), PP ($Y=0.00$ m), PET ($Y=+0.10$ m).
Each rail initially contains two balls that belong to the OTHER rails:
- **HDPE rail ($Y=-0.10$ m):** front ball `loose_ball_1` (PP), back ball `loose_ball_2` (PET)
- **PP rail ($Y=0.00$ m):** front ball `loose_ball_3` (PET), back ball `loose_ball_4` (HDPE)
- **PET rail ($Y=+0.10$ m):** front ball `loose_ball_5` (HDPE), back ball `loose_ball_6` (PP)

### Sorting Workflow
The `rail_scan_mission` command runs the full sorting puzzle:
1. **Pick from cradle:** Approaches the ball currently waiting at the cradle stop ($X \approx 0.374$ m), closes fingers with compliant high friction, and verifies physical lift.
2. **Roll forward:** As soon as the front ball is lifted, the second ball behind it automatically rolls down the $3^\circ$ cradle slope to the stop post, ready for the next pick.
3. **Scan in contact:** Carries the ball to the scan station box at $(0.12, -0.27)$ m, lowers it down to $Z_{\text{tcp}} = 0.038$ m so the ball is in contact with the sensor window, dwells 2 seconds, and publishes the simulated material reading (`/mujoco_scene/scan_result`).
4. **Sort & drop at rail start:** Moves to the high end ($X = 0.200$ m) of the matching target rail, lowers to $Z = 0.115$ m, detaches, and opens fingers to $0.040$ m. The ball drops onto the $24^\circ$ ramp and rolls down into the cradle!
5. **Repeat:** Executes for all 6 balls until all rails contain their matching sorted balls.

### Run Commands

Terminal 1:
```bash
ros2 launch rebotarm_mujoco_dm simulation.launch.py scene:=rail_scan use_moveit:=true
```

Terminal 2:
```bash
# Test 1 ball first
ros2 run rebotarm_mujoco_dm rail_scan_mission --limit 1

# Run the complete 6-ball sorting puzzle
ros2 run rebotarm_mujoco_dm rail_scan_mission
```

"""Backwards-compatible alias for rebotarm_mujoco_dm.tasks.rail_scan_mission."""
from .tasks.rail_scan_mission import RailScanMission, RailScanMission as RailScanCycle, main

if __name__ == "__main__":
    main()

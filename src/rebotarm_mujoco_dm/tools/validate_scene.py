#!/usr/bin/env python3
"""Offline scene Phase 1 checks using a compiled model_probe; no ROS graph."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rebotarm_mujoco_dm.model_paths import prepare_scene


def check_scene(probe, scene):
    results = []
    for key in ("home", "ready", "pose_a", "pose_b"):
        def run(seconds):
            return json.loads(subprocess.check_output(
                [str(probe), str(scene), key, str(seconds), "hold"], text=True))

        start, settled = run(0), run(5)
        assert (start["nq"], start["nv"], start["nu"]) == (15, 14, 7)
        ball = start["bodies"]["loose_ball_1"]
        assert all(abs(a-b) < 1e-12 for a, b in zip(ball["position"], [.3, .1, .03]))
        assert abs(ball["mass"] - .0459) < 1e-12
        assert all(abs(i - 8.3689011e-6) < 1e-12 for i in ball["diaginertia"])
        assert not start["contacts"], f"Initial contact in {key}: {start['contacts']}"
        assert settled["warnings"] == 0
        assert abs(settled["time"] - 5) < 1e-8
        position = settled["bodies"]["loose_ball_1"]["position"]
        assert abs(position[0] - .3) < .001 and abs(position[1] - .1) < .001
        assert abs(position[2] - .02135) < .001
        assert max(abs(v) for v in settled["qvel"][-6:]) < .001
        assert settled["max_force_excess"] < 1e-9
        assert max(abs(x) for x in settled["bodies"]["base_link"]["position"]) < 1e-12
        assert settled["contacts"] and all(
            {c["geom1"], c["geom2"]} == {"loose_ball_1_geom", "table_top"}
            for c in settled["contacts"])
        assert max(-c["distance"] for c in settled["contacts"]) < .001
        results.append({"key": key, "ball_position": position,
                        "warnings": settled["warnings"],
                        "max_ball_speed": max(abs(v) for v in settled["qvel"][-6:]),
                        "contact_depth": max(-c["distance"] for c in settled["contacts"])})
    return {"version": start["version"], "result": "PASS", "poses": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--dm-share", type=Path, default=ROOT)
    parser.add_argument("--bringup-share", type=Path, default=ROOT.parent / "rebotarm_bringup")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="dm_scene_validation_") as temp:
        scene = prepare_scene(args.dm_share, args.bringup_share, temp)
        result = check_scene(args.probe.resolve(), scene)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

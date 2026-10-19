#!/usr/bin/env python3
"""Compare golf-ball settling in the bowl using the native MuJoCo probe."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from rebotarm_mujoco_dm.model_paths import prepare_scene


def run(probe, scene):
    return json.loads(subprocess.check_output(
        [str(probe), str(scene), "home", "20", "bowl_roll"], text=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="dm_bowl_validation_") as temp:
        scene = prepare_scene(ROOT, ROOT.parent / "rebotarm_bringup", temp)
        tree = ET.parse(scene)
        pair = tree.find('contact/pair[@name="ball_bowl_bottom"]')
        if pair is None or pair.get("condim") != "6":
            raise AssertionError("Missing six-dimensional ball/bowl-bottom contact")
        with_pair = run(args.probe.resolve(), scene)
        tree.getroot().find("contact").remove(pair)
        without_pair = Path(temp) / "bowl_without_rolling.xml"
        tree.write(without_pair)
        baseline = run(args.probe.resolve(), without_pair)

    ball = with_pair["bodies"]["loose_ball_1"]["position"]
    result = with_pair["ball_test"]
    old = baseline["ball_test"]
    assert with_pair["warnings"] == baseline["warnings"] == 0
    assert result["stop_time"] is not None and result["stop_time"] < 5, result
    assert result["final_linear_speed"] < .001, result
    assert result["bowl_radial_distance"] < .045, result
    assert .020 < ball[2] < .045, ball
    assert old["stop_time"] is None or old["stop_time"] > result["stop_time"], old
    assert any({c["geom1"], c["geom2"]} ==
               {"loose_ball_1_geom", "bowl_1_bottom"} and c["dim"] == 6
               for c in with_pair["contacts"]), with_pair["contacts"]
    print(json.dumps({"version": with_pair["version"], "result": "PASS",
                      "with_bowl_rolling": result,
                      "without_bowl_rolling": old,
                      "settled_ball_position": ball}, indent=2))


if __name__ == "__main__":
    main()

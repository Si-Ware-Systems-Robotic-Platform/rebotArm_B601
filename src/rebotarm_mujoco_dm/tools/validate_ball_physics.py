#!/usr/bin/env python3
"""Compare repeatable ball tests offline; overrides affect temporary XML only."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rebotarm_mujoco_dm.model_paths import prepare_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--rolling", type=float, default=None, help="Override rolling coefficient in metres")
    parser.add_argument("--timestep", type=float, default=None)
    parser.add_argument("--case", choices=["all", "rolling", "sliding"], default="all")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    import math
    for value in (args.rolling, args.timestep, args.seconds):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error("Overrides must be positive and finite")
    results = []
    with tempfile.TemporaryDirectory(prefix="dm_ball_test_") as temp:
        scene = prepare_scene(ROOT, ROOT.parent / "rebotarm_bringup", temp)
        original = scene.read_text()
        for label, mode, dim in [("without_rolling_resistance", "roll", 3),
                                  ("with_rolling_resistance", "roll", 6),
                                  ("sliding_start", "slide", 6)]:
            if args.case == "rolling" and label != "with_rolling_resistance":
                continue
            if args.case == "sliding" and mode != "slide":
                continue
            tree = ET.ElementTree(ET.fromstring(original))
            pair = tree.find('contact/pair[@name="ball_table"]')
            pair.set("condim", str(dim))
            friction = list(map(float, pair.get("friction").split()))
            if args.rolling is not None:
                friction[3:] = [args.rolling]*2
                pair.set("friction", " ".join(map(str, friction)))
            if args.timestep is not None:
                ET.SubElement(tree.getroot(), "option", timestep=str(args.timestep))
            tree.write(scene)
            data = json.loads(subprocess.check_output(
                [str(args.probe.resolve()), str(scene), "home", str(args.seconds), mode], text=True))
            assert data["warnings"] == 0, data
            assert abs(data["time"]-args.seconds) < 1e-8
            ball = data["bodies"]["loose_ball_1"]["position"]
            assert abs(ball[2]-.02135) < .001, "Ball left the tabletop"
            assert data["contacts"] and all(
                {c["geom1"], c["geom2"]} == {"loose_ball_1_geom", "table_top"}
                and c["dim"] == dim for c in data["contacts"])
            results.append({"case": label, "version": data["version"],
                            "condim": dim, "elapsed_seconds": data["time"],
                            "rolling_coefficient_m": friction[3],
                            "ball_position": ball, "warnings": data["warnings"],
                            **data["ball_test"]})
    print(json.dumps({"results": results,
                      "note": "Numerical experiment; real-world calibration requires measured data."}, indent=2))


if __name__ == "__main__":
    main()

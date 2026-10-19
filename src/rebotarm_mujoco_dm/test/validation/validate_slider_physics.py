#!/usr/bin/env python3
"""Offline gravity-only drops using the scene's actual collision parts; no ROS nodes."""
import argparse
from copy import deepcopy
import json
import itertools
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE))
from rebotarm_mujoco_dm.core.slider_checks import SlideMonitor
from rebotarm_mujoco_dm.bridge.scene_geometry import read_scene, rotate


def physics_scene(scene_path):
    source = ET.parse(scene_path).getroot()
    root = ET.Element("mujoco", model="slider_offline_contact_check")
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false")
    robot = ET.parse(PACKAGE / "models/robot/dm_arm.xml")
    root.append(deepcopy(robot.find("option")))
    root.append(deepcopy(robot.find("default")))
    asset = deepcopy(source.find("asset"))
    for item in asset:
        if item.get("file"):
            item.set("file", str((PACKAGE / "scene_assets" / Path(item.get("file")).name).resolve()))
    root.append(asset)
    root.append(deepcopy(source.find("worldbody")))
    contact = ET.SubElement(root, "contact")
    for pair in source.findall("contact/pair"):
        if "finger" not in pair.get("name", ""):
            contact.append(deepcopy(pair))
    return ET.tostring(root, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/slider-validation.json"))
    parser.add_argument("--perturbations", action="store_true", help="Also check releases at the mission tolerance edge in X/Y/diagonal and Z")
    args = parser.parse_args()
    scene_path = PACKAGE / "models/scenes/dm_slider.xml"
    config = yaml.safe_load((PACKAGE / "config/scenes/slider_params.yaml").read_text())["slider_mission"]["ros__parameters"]["slider"]
    catch = config['catch']
    qw, qx, qy, qz = config['quat']
    inverse = [-qx, -qy, -qz, qw]
    model = mujoco.MjModel.from_xml_string(physics_scene(scene_path))
    objects, radius = read_scene(scene_path)
    slider = next(o for o in objects if o['id'] == 'dm_slider')
    assert len(slider['geometries']) == sum(n.startswith('slider_convex_') for n in
           [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '' for i in range(model.ngeom)])
    results = []
    offsets = [(0., 0., 0.)]
    if args.perturbations:
        # The mission releases once the measured miss is inside these limits.
        r, z = config['release_tolerance'], config['release_tolerance_z']
        d = r * np.sqrt(.5)
        offsets += [(-r, 0., 0.), (r, 0., 0.), (0., -r, 0.), (0., r, 0.),
                    (d, d, 0.), (d, -d, 0.), (-d, d, 0.), (-d, -d, 0.),
                    (0., 0., -z), (0., 0., z)]
    for rail, (dx, dy, dz) in itertools.product((1, 2, 3), offsets):
        data = mujoco.MjData(model)
        ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"loose_ball_{rail}")
        joint = model.body_jntadr[ball]
        adr = model.jnt_qposadr[joint]
        entry = config['rails'][rail]
        data.qpos[adr:adr+3] = [entry['entry_x'] + dx, entry['entry_y'] + dy, entry['entry_z'] + config['release_clearance'] + dz]
        monitor = SlideMonitor(timeout=config['slide_timeout'])
        path = []
        contact_names = set()
        worst_penetration = 0.
        exit_time = None
        # After the outlet, keep stepping so the ball's resting place is checked too.
        for step in range(int((config['slide_timeout'] + 3.) / model.opt.timestep) + 1):
            mujoco.mj_step(model, data)
            world = data.qpos[adr:adr+3].copy()
            local = np.array(rotate(inverse, (world - np.array(config['pos'])).tolist()))
            if step % 10 == 0:
                monitor.update(float(data.time), local.tolist())
                path.append([round(float(data.time), 4), *world.tolist()])
            for c in data.contact:
                if ball in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]):
                    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (c.geom1,c.geom2)]
                    contact_names.update(n for n in names if n.startswith('slider_'))
                    worst_penetration = max(worst_penetration, -float(c.dist))
            if monitor.exited and exit_time is None:
                exit_time = float(data.time)
            if monitor.failure or (exit_time is not None and data.time - exit_time >= 3.):
                break
        caught = (exit_time is not None
                  and catch['x_min'] - radius <= world[0] <= catch['x_max'] + .06
                  and abs(world[1] - catch['y_center']) <= catch['inner_width'] / 2
                  and float(np.linalg.norm(data.qvel[model.jnt_dofadr[joint]:model.jnt_dofadr[joint]+3])) < .01)
        result = dict(rail=rail, offset_m=[dx, dy, dz], exited=monitor.exited, failure=monitor.failure,
                      caught=caught,
                      elapsed_s=exit_time or float(data.time), final_world_m=world.tolist(),
                      max_penetration_m=worst_penetration, contacted_parts=sorted(contact_names),
                      warnings=int(sum(data.warning.number)), path=path)
        results.append(result)
        print(json.dumps({k:v for k,v in result.items() if k not in ('path','contacted_parts')}), flush=True)
    report = dict(mujoco_version=mujoco.__version__, collision_parts=len(slider['geometries']),
                  ball_radius_m=radius, trials=results)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    if any(not r['exited'] or not r['caught'] or r['warnings'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()

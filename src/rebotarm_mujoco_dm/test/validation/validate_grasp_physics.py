#!/usr/bin/env python3
"""Isolate the actual DM fingers with a prescribed wrist; ball remains fully dynamic.

Requires the Python mujoco binding. This checks contact/actuation, not ROS or arm IK.
A driven vertical wrist joint supplies test motion; no constraint attaches the ball.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from rebotarm_mujoco_dm.model_paths import prepare_scene


def build_fixture(prepared, output=None):
    arm = ET.parse(prepared.parent/'dm_arm.xml').getroot()
    scene = ET.parse(prepared).getroot()
    root = ET.Element('mujoco', model='DM finger contact isolation')
    for name in ['compiler', 'option', 'default', 'asset']:
        root.append(deepcopy(arm.find(name)))
    root.find('asset').extend(deepcopy(list(scene.find('asset'))))
    world = ET.SubElement(root, 'worldbody')
    wrist = deepcopy(arm.find('.//body[@name="end_link"]'))
    wrist.set('pos', '.3 .1 .00735')
    ET.SubElement(wrist, 'joint', name='test_wrist_lift', type='slide', axis='-1 0 0')
    wrist.set('quat', '.7071067811865476 0 .7071067811865476 0')
    world.append(wrist)
    world.append(deepcopy(scene.find('worldbody/body[@name="full_table"]')))
    world.append(deepcopy(scene.find('worldbody/body[@name="loose_ball_1"]')))
    root.append(deepcopy(arm.find('equality')))
    actuators = ET.SubElement(root, 'actuator')
    actuators.append(deepcopy(arm.find('actuator/position[@name="finger_left"]')))
    ET.SubElement(actuators, 'position', name='test_wrist_lift', joint='test_wrist_lift',
                  kp='20000', kv='200', forcerange='-100 100')
    contact = ET.SubElement(root, 'contact')
    for e in scene.findall('contact/pair'):
        contact.append(deepcopy(e))
    ET.SubElement(contact, 'exclude', body1='finger_left_link', body2='finger_right_link')
    if output:
        ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    return mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def trial(model, lateral_offset=0., timestep=None, hold_seconds=3.):
    if timestep:
        model.opt.timestep = timestep
    d = mj.MjData(model)
    left, right = [model.joint(n).qposadr[0] for n in ['finger_left', 'finger_right']]
    ball = model.body('loose_ball_1').id
    d.qpos[left], d.qpos[right] = .04, -.04
    model.body_pos[model.body('end_link').id,1] += lateral_offset
    maximum_force = maximum_depth = maximum_speed = 0.
    held_positions = []; bilateral = []; release = []
    checkpoints = {}
    release_start = 9. + hold_seconds
    end = release_start + 4.
    for step in range(round(end/model.opt.timestep)):
        t = step*model.opt.timestep
        # Smooth wrist lift from t=6 to t=9; hold, then open.
        a = np.clip((t-6)/3, 0, 1)
        d.ctrl[1] = .10*(3*a*a-2*a*a*a)
        if t < 1:
            d.ctrl[0] = .04
        elif t < 4:
            a = (t-1)/3; d.ctrl[0] = .04+(.018-.04)*(3*a*a-2*a*a*a)
        elif t < release_start:
            d.ctrl[0] = .018
        else:
            a = np.clip((t-release_start)/2,0,1); d.ctrl[0] = .018+(.04-.018)*(3*a*a-2*a*a*a)
        mj.mj_step(model,d)
        sides = set()
        for c in d.contact:
            names = [model.geom(c.geom1).name, model.geom(c.geom2).name]
            if 'loose_ball_1_geom' in names and any('finger_' in n for n in names):
                maximum_depth = max(maximum_depth, -c.dist)
                sides.add('left' if any('finger_left' in n for n in names) else 'right')
        maximum_force = max(maximum_force, abs(d.actuator_force[0]))
        maximum_speed = max(maximum_speed, np.linalg.norm(d.qvel[-6:-3]))
        if 10 <= t < release_start:
            held_positions.append(d.xpos[ball].copy()); bilateral.append(len(sides)==2)
        if t >= end-1:
            release.append(d.xpos[ball].copy())
        for at in [4,6,9,release_start,end]:
            if t < at <= t+model.opt.timestep+1e-9:
                checkpoints[str(at)] = {'ball':d.xpos[ball].tolist(), 'finger':float(d.qpos[left])}
    held = np.array(held_positions); released=np.array(release)
    result = {'mujoco':mj.__version__, 'timestep':model.opt.timestep,
              'hold_seconds':hold_seconds,
              'lateral_offset_m':lateral_offset, 'peak_actuator_force_N':maximum_force,
              'peak_ball_finger_penetration_m':maximum_depth, 'peak_ball_speed_m_s':maximum_speed,
              'held_height_m':float(held[:,2].min()),
              'hold_drift_m':float(np.linalg.norm(held[-1]-held[0])),
              'bilateral_contact_fraction':float(np.mean(bilateral)),
              'released_height_m':float(released[-1,2]),
              'warnings':int(sum(w.number for w in d.warning)), 'checkpoints':checkpoints}
    result['grasp_pass'] = bool(result['held_height_m']>.09 and result['hold_drift_m']<.003
                                and result['bilateral_contact_fraction']>.95 and result['warnings']==0
                                and result['peak_ball_finger_penetration_m']<.002
                                and abs(result['released_height_m']-.02135)<.002)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export-xml',type=Path)
    parser.add_argument('--miss', action='store_true')
    parser.add_argument('--timestep',type=float,default=.001)
    parser.add_argument('--hold-seconds',type=float,default=3.,
                        help='seconds between the end of lift and start of release')
    args=parser.parse_args()
    if args.hold_seconds < 3.:
        parser.error('--hold-seconds must be at least 3')
    with tempfile.TemporaryDirectory(prefix='dm_grasp_physics_') as temp:
        model=build_fixture(prepare_scene(ROOT,ROOT.parent/'rebotarm_bringup',temp),args.export_xml)
        result=trial(model,.12 if args.miss else 0.,args.timestep,args.hold_seconds)
    print(json.dumps(result,indent=2))
    if args.miss:
        if (result['grasp_pass'] or result['warnings'] or result['held_height_m']>.04
                or result['bilateral_contact_fraction'] or result['peak_ball_finger_penetration_m']>.0001):
            raise SystemExit(1)
    elif not result['grasp_pass']:
        raise SystemExit(1)


if __name__=='__main__':
    main()

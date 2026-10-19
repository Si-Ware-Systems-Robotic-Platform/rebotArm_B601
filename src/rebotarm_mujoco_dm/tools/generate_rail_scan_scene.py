#!/usr/bin/env python3
"""Use the supplied sorter rails/station with the B601 DM arm and table."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = Path(__file__).resolve().parents[4] / 'scene obj/folders/mycobot_pro_450_ball_sorter.xml'
OUTPUT_MODELS = PACKAGE / 'models/dm_rail_scan.xml'
OUTPUT_SCENES = PACKAGE / 'models/scenes/dm_rail_scan.xml'
BALLS = (
    # (name, x, y, z)
    ('loose_ball_1', 0.37315, -0.100, 0.04675),  # Lane HDPE front (pp)
    ('loose_ball_2', 0.32500, -0.100, 0.04930),  # Lane HDPE back (pet)
    ('loose_ball_3', 0.37315,  0.000, 0.04675),  # Lane PP front (pet)
    ('loose_ball_4', 0.32500,  0.000, 0.04930),  # Lane PP back (hdpe)
    ('loose_ball_5', 0.37315,  0.100, 0.04675),  # Lane PET front (hdpe)
    ('loose_ball_6', 0.32500,  0.100, 0.04930),  # Lane PET back (pp)
)


def generate():
    dm = ET.parse(PACKAGE / 'models/dm_table.xml')
    root = dm.getroot()
    root.set('model', 'B601_DM_rail_scan')
    source = ET.parse(SOURCE).getroot()
    asset = root.find('asset')
    world = root.find('worldbody')
    contact = root.find('contact')
    old_ball = world.find('body[@name="loose_ball_1"]')
    world.remove(old_ball)
    for pair in list(contact):
        contact.remove(pair)

    materials = ('slide_white', 'sensor_body', 'sensor_dark',
                 'label_pet', 'label_hdpe', 'label_pp')
    for name in materials:
        asset.append(deepcopy(source.find(f'asset/material[@name="{name}"]')))
    for name in ('slide_body_0', 'slide_body_1'):
        asset.append(deepcopy(source.find(f'asset/mesh[@name="{name}"]')))

    for name in ('lane_pet', 'lane_hdpe', 'lane_pp', 'slide_frame', 'sensor_station'):
        world.append(deepcopy(source.find(f'worldbody/body[@name="{name}"]')))
    for name, x, y, z in BALLS:
        body = ET.Element('body', name=name, pos=f'{x:.5f} {y:.5f} {z:.5f}')
        ET.SubElement(body, 'freejoint', name=f'{name}_joint')
        body.append(deepcopy(old_ball.find('inertial')))
        ET.SubElement(body, 'geom', name=f'{name}_geom', type='sphere',
                      size='0.02135', mass='0.0459', material='ball_material',
                      contype='1', conaffinity='1', condim='6',
                      friction='0.6 0.0005 0.0001', solref='0.006 0.6')
        world.append(body)
        for side in ('left', 'right'):
            for part in range(4):
                ET.SubElement(contact, 'pair', name=f'{name}_finger_{side}_{part}',
                    geom1=name+'_geom', geom2=f'finger_{side}_link_collision_0_{part}',
                    condim='6', friction='2.5 2.5 0.005 0.0005 0.0005',
                    solref='0.002 1', solimp='0.99 0.995 0.0005')
    ET.indent(dm, space='  ')
    dm.write(OUTPUT_MODELS, encoding='utf-8', xml_declaration=True)
    dm.write(OUTPUT_SCENES, encoding='utf-8', xml_declaration=True)
    return OUTPUT_SCENES


if __name__ == '__main__':
    print(generate())

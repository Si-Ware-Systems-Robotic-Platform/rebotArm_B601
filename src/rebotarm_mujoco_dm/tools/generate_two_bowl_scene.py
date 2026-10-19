#!/usr/bin/env python3
"""Build the five-ball/two-bowl scene from the validated single-bowl scene."""
from copy import deepcopy
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "models/dm_scene.xml"
OUTPUT = ROOT / "models/dm_two_bowls.xml"
BALL_COUNT = 5
BOWL_CENTERS = ((.30, .14), (.30, -.14))


def generate():
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    root.set("model", "B601_DM_two_bowls")
    world = root.find("worldbody")
    contact = root.find("contact")
    bowl1 = world.find('body[@name="bowl_1"]')
    bowl1.set("pos", f"{BOWL_CENTERS[0][0]} {BOWL_CENTERS[0][1]} 0")
    bowl2 = deepcopy(bowl1)
    bowl2.set("name", "bowl_2")
    bowl2.set("pos", f"{BOWL_CENTERS[1][0]} {BOWL_CENTERS[1][1]} 0")
    for geom in bowl2.findall("geom"):
        name = geom.get("name")
        geom.set("name", name.replace("bowl_1", "bowl_2", 1)
                 if name.startswith("bowl_1") else "bowl_2_"+name)
    world.append(bowl2)

    original = world.find('body[@name="loose_ball_1"]')
    template = deepcopy(original)
    for pair in list(contact):
        if pair.get("geom1") == "loose_ball_1_geom":
            contact.remove(pair)
    for index in range(1, BALL_COUNT+1):
        ball = original if index == 1 else deepcopy(template)
        ball.set("name", f"loose_ball_{index}")
        ball.find("freejoint").set("name", f"loose_ball_{index}_joint")
        ball.find("geom").set("name", f"loose_ball_{index}_geom")
        angle = 2*math.pi*(index-1)/BALL_COUNT
        x = BOWL_CENTERS[0][0] + .051*math.cos(angle)
        y = BOWL_CENTERS[0][1] + .051*math.sin(angle)
        ball.set("pos", f"{x:.12g} {y:.12g} .08")
        if index != 1:
            world.append(ball)
        geom = f"loose_ball_{index}_geom"
        ET.SubElement(contact, "pair", name=f"ball_{index}_table", geom1=geom,
                      geom2="table_top", condim="6",
                      friction="0.8 0.8 0.005 0.0005 0.0005",
                      solref="0.02 1", solimp="0.9 0.95 0.001")
        for bowl in (1, 2):
            ET.SubElement(contact, "pair", name=f"ball_{index}_bowl_{bowl}",
                          geom1=geom, geom2=f"bowl_{bowl}_bottom", condim="6",
                          friction="0.8 0.8 0.005 0.001 0.001",
                          solref="0.02 1", solimp="0.9 0.95 0.001")
        for side in ("left", "right"):
            for part in range(4):
                ET.SubElement(contact, "pair", name=f"ball_{index}_finger_{side}_{part}",
                              geom1=geom, geom2=f"finger_{side}_link_collision_0_{part}",
                              condim="6", friction="2.5 2.5 0.005 0.0005 0.0005",
                              solref="0.002 1", solimp="0.99 0.995 0.0005")
    ET.indent(tree, space="  ")
    tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)
    return OUTPUT


if __name__ == "__main__":
    print(generate())

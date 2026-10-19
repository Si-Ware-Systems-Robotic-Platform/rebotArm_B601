#!/usr/bin/env python3
"""Generate the DM slider scene using cached CAD-derived convex collision parts.

The normalized visual and collision meshes share the same frame. The original
CAD export is preserved. See docs/slider-scene.md for the export-frame correction.
Collision geoms use group 3 with visible alpha for optional inspection; sites in
group 4 mark the three URDF ball-centre targets. No guessed channel boxes.
"""
import argparse
from copy import deepcopy
from pathlib import Path
import hashlib
import math
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
TEMPLATE = PACKAGE / "models/dm_table.xml"
# "slider" stands on the angled mount plate; "slider_no_plate" keeps the same incline
# with its low edge resting directly on the table and no plate geometry.
VARIANTS = {
    "slider": dict(mount=True, scene="dm_slider.xml", config="slider_params.yaml"),
    "slider_no_plate": dict(mount=False, scene="dm_slider_no_plate.xml",
                            config="slider_no_plate_params.yaml"),
}
CATCH_DEFAULTS = dict(x_min=0.17, x_max=0.25, y_center=-0.156, inner_width=0.10,
                      wall_height=0.03, thickness=0.01)

# Pickup balls on the open table in front-left of the robot (+Y, away from chute exit at -Y)
BALLS = (
    # name, x, y, z
    ("loose_ball_1", 0.26, 0.18, 0.02135),
    ("loose_ball_2", 0.20, 0.18, 0.02135),
    ("loose_ball_3", 0.14, 0.18, 0.02135),
)

# Collision parts are built from the CAD surface by build_slider_collision.py.
# Keep the cached decomposition so normal scene generation needs no CoACD dependency.
COLLISION = PACKAGE / "scene_assets/slider_collision.xml"


def generate(variant="slider"):
    spec = VARIANTS[variant]
    output_scenes = PACKAGE / "models/scenes" / spec["scene"]
    output_models = PACKAGE / "models" / spec["scene"]
    config = PACKAGE / "config/scenes" / spec["config"]
    # A new variant starts from the plated scene's parameters.
    source = config if config.exists() else PACKAGE / "config/scenes/slider_params.yaml"
    parameters = yaml.safe_load(source.read_text())
    cfg = parameters["slider_mission"]["ros__parameters"]["slider"]
    angle = math.radians(cfg.get("mount_pitch_deg", 1.5))
    if not 0 <= angle <= math.radians(10):
        raise ValueError("Mount inclination must be between 0 and 10 degrees")
    sn, cs = math.sin(angle), math.cos(angle)
    length, width, minimum = 0.170, 0.170, 0.002
    height = minimum + math.tan(angle) * length
    px, py = cfg["pos"][:2]
    # Without the plate the slider's low edge rests on the table itself.
    base_z = height if spec["mount"] else height - minimum
    cfg["mount_pitch_deg"] = round(math.degrees(angle), 6)
    cfg["pos"] = [px, py, base_z]
    # Rz(pi) * Ry(-angle): the inlet (+local X) is raised slightly.
    cfg["quat"] = [0., math.sin(angle/2), 0., math.cos(angle/2)]
    tree = ET.parse(TEMPLATE)
    root = tree.getroot()
    root.set("model", "B601_DM_" + variant)
    
    asset = root.find("asset")
    world = root.find("worldbody")
    contact = root.find("contact")
    
    # Add slider asset mesh and materials
    ET.SubElement(asset, "mesh", name="slider_mesh", file="../scene_assets/slider_base_link.stl")
    ET.SubElement(asset, "material", name="slider_material", rgba="0.270588 0.270588 0.270588 1", specular="0.3", shininess="0.4")
    
    # Remove old ball from template
    old_ball = world.find('body[@name="loose_ball_1"]')
    ball_inertial = deepcopy(old_ball.find("inertial"))
    world.remove(old_ball)
    for pair in list(contact):
        contact.remove(pair)
    
    # Create slider body
    slider = ET.Element("body", name="slider", pos=" ".join(map(str, cfg["pos"])),
                        quat=" ".join(map(str, cfg["quat"])))

    if spec["mount"]:
        # A visible, collidable wedge accounts for the inclination physically.
        # Its top plane coincides with the normalized slider's local Z=0 plane.
        vertices = [(-length,-width,0), (length,-width,0), (length,width,0), (-length,width,0),
                    (-length,-width,minimum), (length,-width,2*height-minimum),
                    (length,width,2*height-minimum), (-length,width,minimum)]
        faces = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
                 (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
        ET.SubElement(asset, "mesh", name="slider_mount_mesh",
                      vertex=" ".join(str(v) for point in vertices for v in point),
                      face=" ".join(str(v) for face in faces for v in face))
        mount = ET.SubElement(world, "body", name="slider_mount", pos=f"{px} {py} 0", quat="0 0 0 1")
        ET.SubElement(mount, "geom", name="slider_mount", type="mesh", mesh="slider_mount_mesh",
                      rgba="0.55 0.32 0.13 1", group="0", friction="0.5 0.001 0.0001")

    # Finished balls leave the outlet towards the robot base. Left on the table, a ball
    # reaches the base and MoveIt then rejects every arm state, so catch them here.
    catch = {**CATCH_DEFAULTS, **cfg.get("catch", {})}
    cfg["catch"] = catch
    t, h, w, yc = catch["thickness"], catch["wall_height"], catch["inner_width"], catch["y_center"]
    x0, x1 = catch["x_min"], catch["x_max"]
    tray = ET.SubElement(world, "body", name="slider_catch", pos="0 0 0")
    for name, pos, size in (
            ("back", (x0 - t/2, yc, h/2), (t/2, w/2 + t, h/2)),
            ("left", ((x0 + x1)/2, yc + (w + t)/2, h/2), ((x1 - x0)/2, t/2, h/2)),
            ("right", ((x0 + x1)/2, yc - (w + t)/2, h/2), ((x1 - x0)/2, t/2, h/2))):
        ET.SubElement(tray, "geom", name=f"slider_catch_{name}", type="box",
                      pos=" ".join(f"{v:.6g}" for v in pos), size=" ".join(f"{v:.6g}" for v in size),
                      rgba="0.30 0.45 0.70 1", group="0", friction="0.5 0.001 0.0001")
    
    # CAD surface for rendering; matching convex pieces handle contact below
    ET.SubElement(slider, "geom", name="slider_visual", type="mesh", mesh="slider_mesh",
                  material="slider_material", contype="0", conaffinity="0", group="2")
    
    collision = ET.parse(COLLISION).getroot()
    visual_path = PACKAGE / "scene_assets/slider_base_link.stl"
    if collision.get("visual_sha256") != hashlib.sha256(visual_path.read_bytes()).hexdigest():
        raise ValueError("Slider visual changed: rebuild the CAD collision decomposition first")
    for mesh in collision.findall("asset/mesh"):
        asset.append(deepcopy(mesh))
    for geom in collision.findall("body/geom"):
        slider.append(deepcopy(geom))
    # URDF Ball joints are markers, not physical golf-ball meshes/inertias.
    urdf = ET.parse(PACKAGE / "new scene/Slider_URDF&Meshes_V1.1/urdf/Slider_URDF&Meshes_V1.1.urdf")
    for index in (1, 2, 3):
        origin = urdf.find(f"joint[@name='Ball{index}']/origin").get("xyz")
        x, y, z = map(float, origin.split())
        expected = (px - cs*x + sn*z, py - y, base_z + sn*x + cs*z)
        for key, value in zip(("entry_x", "entry_y", "entry_z"), expected):
            cfg["rails"][index][key] = value
        ET.SubElement(slider, "site", name=f"slider_entry_{index}", pos=origin,
                      size="0.003", rgba="1 0.3 0.1 1", group="4")

    world.append(slider)
    
    # 3 Balls on the table
    for name, bx, by, bz in BALLS:
        b_body = ET.Element("body", name=name, pos=f"{bx:.5f} {by:.5f} {bz:.5f}", quat="1 0 0 0")
        ET.SubElement(b_body, "freejoint", name=f"{name}_joint")
        b_body.append(deepcopy(ball_inertial))
        ET.SubElement(b_body, "geom", name=f"{name}_geom", type="sphere",
                      size="0.02135", mass="0.0459", material="ball_material",
                      contype="1", conaffinity="1", condim="6",
                      friction="0.6 0.0005 0.0001", solref="0.006 0.6")
        world.append(b_body)
        
        # Contact pairs
        ET.SubElement(contact, "pair", name=f"{name}_table",
                      geom1=f"{name}_geom", geom2="table_top", condim="6",
                      friction="0.8 0.8 0.005 0.0005 0.0005", solref="0.02 1", solimp="0.9 0.95 0.001")
        for side in ("left", "right"):
            for part in range(4):
                ET.SubElement(contact, "pair", name=f"{name}_finger_{side}_{part}",
                              geom1=f"{name}_geom", geom2=f"finger_{side}_link_collision_0_{part}",
                              condim="6", friction="2.5 2.5 0.005 0.0005 0.0005",
                              solref="0.002 1", solimp="0.99 0.995 0.0005")
    
    ET.indent(tree, space="  ")
    output_scenes.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_scenes, encoding="utf-8", xml_declaration=True)
    output_models.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_models, encoding="utf-8", xml_declaration=True)
    configs = [config]
    if variant == "slider":
        configs.append(PACKAGE / "config/slider_params.yaml")
    for path in configs:
        path.write_text(yaml.safe_dump(parameters, sort_keys=False))
    print(f"Generated {output_scenes}, {output_models} and {config}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=[*VARIANTS, "all"], default="all")
    variant = parser.parse_args().variant
    for name in VARIANTS if variant == "all" else [variant]:
        generate(name)

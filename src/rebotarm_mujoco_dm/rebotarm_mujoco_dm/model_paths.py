"""Resolve installed assets without changing installed files or regenerating hulls."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def extend_scene_keyframes(model, scene):
    """Give top-level free objects explicit reset poses on MuJoCo 3.12 too.

    3.12 fills omitted keyframe translations with zero; 3.15 uses body poses.
    Robot keyframes stay robot-only in the source model for the empty scene.
    """
    bodies = [b for b in scene.findall("worldbody/body") if b.find("freejoint") is not None]
    if len(bodies) != len(scene.findall(".//freejoint")) or scene.findall(".//joint"):
        raise ValueError("Scene keyframes currently support only top-level freejoint bodies")
    extra = []
    for body in bodies:
        if any(a in body.attrib for a in ("euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("Use an explicit wxyz quat for scene free-body reset poses")
        position = [float(x) for x in body.get("pos", "0 0 0").split()]
        quaternion = [float(x) for x in body.get("quat", "1 0 0 0").split()]
        norm = math.sqrt(sum(x*x for x in quaternion))
        if len(position) != 3 or len(quaternion) != 4 or not math.isfinite(norm) or norm <= 0 or not all(
                math.isfinite(x) for x in position + quaternion):
            raise ValueError("Invalid free-body reset pose")
        extra.extend(position + [x/norm for x in quaternion])
    for key in model.findall("keyframe/key"):
        if extra:
            key.set("qpos", key.get("qpos", "") + " " + " ".join(map(str, extra)))


def prepare_scene(dm_share, bringup_share, output, scene_file="dm_rail_scan.xml", ball_visual="native"):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    meshes = Path(bringup_share) / "description/DM/meshes"
    dm_share = Path(dm_share)

    robot_path = dm_share / "models/robot/dm_arm.xml"
    model = ET.parse(robot_path.resolve(strict=True))
    model.getroot().find("compiler").attrib.pop("meshdir", None)
    for mesh in model.findall("asset/mesh"):
        if mesh.get("file"):
            relative = mesh.get("file").split("description/DM/meshes/", 1)[-1]
            path = (meshes / relative).resolve(strict=True)
            mesh.set("file", str(path))

    # Scene models live in models/scenes/; an explicit path is also accepted.
    scene_path = None
    candidates = [
        dm_share / "models/scenes" / scene_file,
        Path(scene_file),
    ]
    for candidate in candidates:
        if candidate.exists():
            scene_path = candidate.resolve(strict=True)
            break
    if scene_path is None:
        raise FileNotFoundError(f"Scene file '{scene_file}' not found in {candidates}")
    scene = ET.parse(scene_path)

    if ball_visual not in ("native", "mesh"):
        raise ValueError("ball_visual must be native or mesh")
    if ball_visual == "mesh":
        for ball in scene.findall("worldbody/body"):
            name = ball.get("name", "")
            if name.startswith("loose_ball_") and ball.find("freejoint") is not None:
                geom = ball.find(f'geom[@name="{name}_geom"]')
                if geom is not None:
                    geom.set("group", "3")
                ET.SubElement(ball, "geom", name=f"{name}_visual", type="mesh",
                              mesh="golf_ball_mesh", material="ball_material",
                              contype="0", conaffinity="0", mass="0", group="2")

    extend_scene_keyframes(model, scene)
    model.write(output / "dm_arm.xml", encoding="utf-8", xml_declaration=True)

    for mesh in scene.findall("asset/mesh") + scene.findall("asset/texture"):
        file_attr = mesh.get("file")
        if file_attr:
            # Resolve relative to scene folder, or scene_assets, or models
            resolved_asset = None
            for asset_cand in [
                scene_path.parent / file_attr,
                dm_share / "scene_assets" / Path(file_attr).name,
                dm_share / "models" / file_attr,
                dm_share / file_attr,
            ]:
                if asset_cand.exists():
                    resolved_asset = asset_cand.resolve()
                    break
            if resolved_asset is None:
                raise FileNotFoundError(f"Asset '{file_attr}' in {scene_path} could not be resolved")
            mesh.set("file", str(resolved_asset))

    scene.write(output / "dm_scene.xml", encoding="utf-8", xml_declaration=True)
    return output / "dm_scene.xml"

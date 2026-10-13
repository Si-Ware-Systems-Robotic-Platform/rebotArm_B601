"""Resolve installed assets without changing installed files or regenerating hulls."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def prepare_scene(dm_share, bringup_share, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    meshes = Path(bringup_share) / "description/DM/meshes"
    model = ET.parse(Path(dm_share) / "models/dm_arm.xml")
    model.getroot().find("compiler").set("meshdir", str(meshes.absolute()))
    for mesh in model.findall("asset/mesh"):
        if mesh.get("file") and not (meshes / mesh.get("file")).is_file():
            raise FileNotFoundError(meshes / mesh.get("file"))
    model.write(output / "dm_arm.xml", encoding="utf-8", xml_declaration=True)
    shutil.copyfile(Path(dm_share) / "models/dm_scene.xml", output / "dm_scene.xml")
    return output / "dm_scene.xml"

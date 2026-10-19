#!/usr/bin/env python3
"""Offline CAD decomposition. Requires numpy, trimesh and coacd (not at runtime)."""
import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
VISUAL = PACKAGE / "scene_assets/slider_base_link.stl"
OUTPUT = PACKAGE / "scene_assets/slider_collision.xml"


def write_parts(parts):
    root = ET.Element("slider_collision", visual_sha256=hashlib.sha256(VISUAL.read_bytes()).hexdigest(),
                      method="CoACD", threshold_m="0.002", seed="0",
                      preprocess_resolution="60", merge="false")
    asset = ET.SubElement(root, "asset")
    body = ET.SubElement(root, "body")
    for i, (vertices, faces) in enumerate(parts):
        name = f"slider_convex_{i:03d}"
        # Explicit vertices/faces are also consumed by the existing MoveIt bridge.
        ET.SubElement(asset, "mesh", name=name,
                      vertex=" ".join(f"{x:.9g}" for x in vertices.ravel()),
                      face=" ".join(str(int(x)) for x in faces.ravel()))
        ET.SubElement(body, "geom", name=name, type="mesh", mesh=name,
                      group="3", rgba="0.1 0.8 0.6 0.35", contype="1", conaffinity="1",
                      priority="1", condim="6", friction="0.25 0.0001 0.00003",
                      solref="0.006 1", solimp="0.95 0.99 0.001")
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(OUTPUT, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {len(parts)} CAD collision parts: {OUTPUT}")


def main():
    import coacd
    import trimesh

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parts", type=Path, help="Reuse an NPZ produced with the settings below")
    args = parser.parse_args()
    # In this V1.1 export every Ball mesh is the slider in that marker's frame.
    # Verify the normalization against all three independent joint transforms.
    source = PACKAGE / "new scene/Slider_URDF&Meshes_V1.1"
    urdf = ET.parse(source / "urdf/Slider_URDF&Meshes_V1.1.urdf")
    visual = trimesh.load(VISUAL, process=False).vertices
    for index in (1, 2, 3):
        marker = f"Ball{index}"
        offset = np.fromstring(urdf.find(f"joint[@name='{marker}']/origin").get("xyz"), sep=" ")
        exported = trimesh.load(source / f"meshes/{marker}.STL", process=False).vertices
        if exported.shape != visual.shape or not np.allclose(exported + offset, visual, atol=1e-6, rtol=0):
            raise ValueError(f"{marker} no longer matches the V1.1 export-frame convention; review CAD alignment")
    if args.parts:
        data = np.load(args.parts, allow_pickle=False)
        parts = [(data[f"v{i}"], data[f"f{i}"]) for i in range(len(data.files) // 2)]
    else:
        mesh = trimesh.load(VISUAL, force="mesh")
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces),
                                threshold=0.002, real_metric=True, preprocess_resolution=60,
                                resolution=1500, mcts_iterations=20, mcts_nodes=10,
                                mcts_max_depth=2, seed=0, merge=False)
    write_parts(parts)


if __name__ == "__main__":
    main()

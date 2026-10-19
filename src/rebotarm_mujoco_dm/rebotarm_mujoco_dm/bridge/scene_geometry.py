"""Geometry shared with MoveIt, read from the actual prepared scene XML."""
import math
import xml.etree.ElementTree as ET


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotate(q, v):
    return multiply(multiply(q, [*v, 0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def compose(a, b):
    p = rotate(a[1], b[0])
    return ([x+y for x, y in zip(a[0], p)], multiply(a[1], b[1]))


def relative(parent, child):
    q = [-parent[1][0], -parent[1][1], -parent[1][2], parent[1][3]]
    return (rotate(q, [a-b for a, b in zip(child[0], parent[0])]), multiply(q, child[1]))


def xml_pose(element):
    if any(k in element.attrib for k in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Planning-scene conversion requires explicit wxyz quaternions")
    p = list(map(float, element.get("pos", "0 0 0").split()))
    w, x, y, z = map(float, element.get("quat", "1 0 0 0").split())
    n = math.sqrt(x*x+y*y+z*z+w*w)
    return (p, [x/n, y/n, z/n, w/n])


def read_scene(path):
    root = ET.parse(path).getroot()
    meshes = {m.get("name"): m for m in root.findall("asset/mesh")}
    objects = []

    def geometry(geom, parent):
        if geom.get("contype") == "0" and geom.get("conaffinity") == "0":
            return None
        kind = geom.get("type", "sphere")
        pose = compose(parent, xml_pose(geom))
        size = list(map(float, geom.get("size", "").split()))
        if kind == "plane":
            kind, size = "box", [6., 6., .05]
            pose[0][2] -= .025
        elif kind == "box":
            size = [2*x for x in size]
        elif kind == "cylinder":
            size = [2*size[1], size[0]]  # MoveIt height, radius
        elif kind == "mesh":
            mesh = meshes[geom.get("mesh")]
            if mesh.get("file"):
                raise ValueError("Collision meshes must contain explicit vertices/faces")
            vertices = list(map(float, mesh.get("vertex").split()))
            faces = list(map(int, mesh.get("face").split()))
            return {"kind": kind, "pose": pose, "vertices": vertices, "faces": faces}
        elif kind != "sphere":
            raise ValueError(f"Unsupported collision geometry: {kind}")
        return {"kind": kind, "pose": pose, "size": size}

    identity = ([0., 0., 0.], [0., 0., 0., 1.])
    for geom in root.findall("worldbody/geom"):
        item = geometry(geom, identity)
        if item:
            objects.append({"id": "dm_"+geom.get("name"), "geometries": [item]})
    for body in root.findall("worldbody/body"):
        if body.find("freejoint") is not None:
            continue
        if body.findall("body"):
            raise ValueError("Nested static scene bodies need explicit transform support")
        items = [geometry(g, xml_pose(body)) for g in body.findall("geom")]
        objects.append({"id": "dm_"+body.get("name"),
                        "geometries": [g for g in items if g]})
    ball = root.find('worldbody/body[@name="loose_ball_1"]')
    radius = float(ball.find('geom[@name="loose_ball_1_geom"]').get("size"))
    return objects, radius

#!/usr/bin/env python3
"""Check compiled DM geometry and dynamics independently against the source URDF.

Requires numpy/scipy and the compiled model_probe. Does not use ROS or hardware.
"""
import argparse
from datetime import date
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def vector(text):
    return np.fromstring(text, sep=" ")


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def forward_kinematics(urdf, positions):
    transforms = {"base_link": np.eye(4)}
    remaining = list(urdf.findall("joint"))
    while remaining:
        progressed = False
        for joint in list(remaining):
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            origin = joint.find("origin")
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = Rotation.from_euler("xyz", vector(origin.get("rpy", "0 0 0"))).as_matrix()
                transform[:3, 3] = vector(origin.get("xyz", "0 0 0"))
            motion = np.eye(4)
            if joint.get("type") != "fixed":
                axis = vector(joint.find("axis").get("xyz"))
                axis /= np.linalg.norm(axis)
                q = positions[joint.get("name")]
                if joint.get("type") == "revolute":
                    motion[:3, :3] = Rotation.from_rotvec(axis*q).as_matrix()
                else:
                    motion[:3, 3] = axis*q
            transforms[joint.find("child").get("link")] = transforms[parent] @ transform @ motion
            remaining.remove(joint)
            progressed = True
        check(progressed, "Disconnected URDF tree")
    return transforms


def main(args):
    urdf = ET.parse(args.description / "urdf/ReBot_Arm_DM.urdf").getroot()
    source_joints = [j for j in urdf.findall("joint") if j.get("type") != "fixed"]
    moveit = ET.parse(ROOT.parent / "rebotarm_moveit_config/config/rebotarm.urdf.xacro").getroot()
    tcp_joint = moveit.find('joint[@name="gripper_tcp_joint"]')
    check(tcp_joint is not None and tcp_joint.get("type") == "fixed", "Missing fixed MoveIt TCP")
    check(tcp_joint.find("parent").get("link") == "end_link", "MoveIt TCP parent changed")
    check(tcp_joint.find("child").get("link") == "gripper_tcp", "MoveIt TCP name changed")
    origin = tcp_joint.find("origin")
    if origin is not None:
        check(np.all(vector(origin.get("xyz", "0 0 0")) == 0) and
              np.all(vector(origin.get("rpy", "0 0 0")) == 0), "MoveIt TCP offset changed")
    # Robot-only regression tests deliberately omit dynamic scene objects.
    scene = ROOT / "models/dm_empty.xml"
    model = ET.parse(ROOT / "models/dm_arm.xml").getroot()
    keys = {k.get("name"): vector(k.get("qpos")) for k in model.findall("keyframe/key")}
    results = []

    def probe(pose, seconds=0, mode="hold"):
        data = json.loads(subprocess.check_output(
            [str(args.probe.resolve()), str(scene), pose, str(seconds), mode], text=True))
        check(data["warnings"] == 0, f"MuJoCo warning in {pose}/{mode}")
        check(np.all(np.isfinite(data["qpos"] + data["qvel"])), "Nonfinite state")
        check(abs(data["time"] - seconds) < 1e-8, "Simulation reset or incomplete run")
        check(data["max_force_excess"] < 1e-10, "Actuator effort limit exceeded")
        check(np.linalg.norm(data["bodies"]["base_link"]["position"]) < 1e-12, "Base moved")
        return data

    static = {name: probe(name) for name in keys}
    reference = static["home"]
    check((reference["nq"], reference["nv"], reference["nu"]) == (8, 8, 7), "Joint/actuator count")
    check([j["name"] for j in reference["joints"]] == [j.get("name") for j in source_joints], "Joint order")
    for actual, expected in zip(reference["joints"], source_joints):
        limit = expected.find("limit")
        check(np.allclose(actual["axis"], vector(expected.find("axis").get("xyz")), atol=1e-12), "Joint axis")
        check(np.allclose(actual["range"], [float(limit.get("lower")), float(limit.get("upper"))], atol=1e-12), "Joint limits")
        check(actual["type"] == (3 if expected.get("type") == "revolute" else 2), "Joint type")
    commanded = [j for j in source_joints if j.find("mimic") is None]
    for actual, expected in zip(reference["actuators"], commanded):
        limit = expected.find("limit")
        check(actual["joint"] == actual["name"] == expected.get("name"), "Actuator mapping")
        check(np.allclose(actual["ctrlrange"], [float(limit.get("lower")), float(limit.get("upper"))]), "Control limits")
        effort = float(limit.get("effort"))
        check(np.allclose(actual["forcerange"], [-effort, effort]), "Effort limits")
    for link in urdf.findall("link"):
        expected = link.find("inertial")
        actual = reference["bodies"][link.get("name")]
        if expected is None:
            check(link.get("name") in ("finger_left_link", "finger_right_link"), "Unexpected missing inertia")
            check(actual["mass"] == 0.04, "Finger mass assumption changed")
            continue
        check(abs(actual["mass"] - float(expected.find("mass").get("value"))) < 1e-12, "Mass mismatch")
        check(np.allclose(actual["inertial_position"], vector(expected.find("origin").get("xyz")), atol=1e-12), "COM mismatch")
        tensor = expected.find("inertia")
        ixx, iyy, izz, ixy, ixz, iyz = [float(tensor.get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        source_tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        q = actual["inertial_quaternion"]
        rotation = Rotation.from_quat(q[1:] + q[:1]).as_matrix()
        compiled_tensor = rotation @ np.diag(actual["diaginertia"]) @ rotation.T
        # MJCF preserves all six URDF entries; the compiled principal-axis
        # eigensolver in MuJoCo 3.12 introduces about 2e-9 kg m^2 here.
        mjcf_tensor = model.find(f'.//body[@name="{link.get("name")}"]/inertial').get("fullinertia")
        check(np.array_equal(vector(mjcf_tensor), [ixx, iyy, izz, ixy, ixz, iyz]), "Source inertia changed")
        check(np.max(np.abs(compiled_tensor-source_tensor)) < 1e-8, "Compiled inertia tensor mismatch")

    fk_rows = []
    for name, data in static.items():
        positions = {j["name"]: data["qpos"][j["qposadr"]] for j in data["joints"]}
        transforms = forward_kinematics(urdf, positions)
        max_position, max_angle = 0, 0
        for link, transform in transforms.items():
            body = data["bodies"][link]
            max_position = max(max_position, np.linalg.norm(transform[:3, 3]-body["position"]))
            max_angle = max(max_angle, Rotation.from_matrix(transform[:3, :3].T @ np.array(body["rotation"]).reshape(3, 3)).magnitude())
        tcp = transforms["end_link"]  # MoveIt gripper_tcp is fixed at end_link origin.
        tcp_position = np.linalg.norm(tcp[:3, 3]-data["tcp_position"])
        tcp_angle = Rotation.from_matrix(tcp[:3, :3].T @ np.array(data["tcp_rotation"]).reshape(3, 3)).magnitude()
        check(max_position < 1e-9 and max_angle < 1e-9, "URDF/MJCF body transform mismatch")
        check(tcp_position < 0.002 and tcp_angle < np.deg2rad(2), "TCP tolerance exceeded")
        check(len(data["contacts"]) == 0, f"Unexpected reference-pose collision: {name}")
        fk_rows.append(f"| {name} | {max_position:.3g} | {np.rad2deg(max_angle):.3g} | {tcp_position*1000:.3g} | {np.rad2deg(tcp_angle):.3g} |")
        held = probe(name, 10)
        error = np.max(np.abs(np.array(held["qpos"][:6])-keys[name][:6]))
        check(error < 0.012, f"Excessive static arm tracking error: {name}")
        check(held["max_velocity"] < 1.0, f"Excessive hold velocity: {name}")
        check(np.max(np.abs(held["qvel"])) < 0.02, f"Hold did not settle: {name}")
        check(held["max_limit_error"] < 0.001, "Hold joint-limit overshoot")
        check(held["max_mimic_error"] < 2e-5, "Hold mimic error")
        results.append((name+" hold", held, error))
    for mode, target in (("open", 0.04), ("close", 0.0)):
        data = probe("ready", 5, mode)
        check(abs(data["qpos"][6]-target) < 1e-4, "Finger target not reached")
        check(data["max_mimic_error"] < 2e-5, "Moving mimic error")
        check(data["max_limit_error"] < 1e-4, "Gripper limit overshoot")
        results.append(("gripper "+mode, data, None))
    released = probe("ready", 5, "release")
    check(np.linalg.norm(np.array(released["qpos"][:6])-keys["ready"][:6]) > 0.1, "Gravity had no effect")
    check(len(released["contacts"]) > 0, "Expected self-contact in unpowered fall")
    check(released["max_velocity"] < 30 and released["max_limit_error"] < 0.05, "Unbounded unpowered fall")
    check(released["max_penetration"] < 0.01, "Excessive impact penetration")
    check(released["max_mimic_error"] < 1e-4, "Mimic failed during fall")
    results.append(("unpowered fall", released, None))

    lines = [f"# Phase 1 validation — MuJoCo {reference['version']}", "",
             f"Run date: {date.today().isoformat()}. **PASS** (all assertions).", "",
             "## URDF agreement", "",
             "Eight joints, seven actuators; names, order, axes, limits and effort limits match. All supplied masses and COMs match. Source inertia entries are preserved exactly; compiled principal-axis tensors agree within 1e-8 kg m². Each finger uses the documented 0.04 kg estimate. Base remains fixed.", "",
             "Independent URDF forward kinematics compared with compiled MuJoCo body and TCP transforms at the same joint positions:", "",
             "| Pose | Largest body position error (m) | Largest body angle error (deg) | TCP error (mm) | TCP angle error (deg) |",
             "|---|---:|---:|---:|---:|", *fk_rows, "",
             "All four reference poses have zero contacts. This verifies the geometry against the URDF used by RViz; it is not a live ROS/RViz synchronization test.", "",
             "## Dynamics", "",
             "Hold runs: 10 simulated seconds each. Gripper: 1 s target ramp followed by 4 s settling. Unpowered gravity fall: 5 s. Zero MuJoCo warnings and zero actuator force-limit excess in every run.", "",
             "| Test | Final max arm target error (rad) | Peak joint speed* | Peak mimic error (m) | Peak soft-limit excess* | Peak contact penetration (m) |",
             "|---|---:|---:|---:|---:|---:|"]
    for name, data, error in results:
        error_text = "—" if error is None else f"{error:.6g}"
        lines.append(f"| {name} | {error_text} | {data['max_velocity']:.6g} | {data['max_mimic_error']:.6g} | {data['max_limit_error']:.6g} | {data['max_penetration']:.6g} |")
    lines += ["", "*Maximum across hinge and slide coordinates: rad/s or m/s for speed, rad or m for limit excess. These are numerical diagnostics, not hardware safety limits.", "",
              "Position servos have finite gravity sag. Joint limits and equality constraints are soft, so small overshoots occur. The unpowered fall is a physics stress check; its speed and impact are not an allowed commanded motion. Contact geometry and finger inertias remain approximations. Trajectory tuning, ROS timing and action execution belong to Phase 2 onward.", ""]
    report = "\n".join(lines)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(report)
    print(report)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--description", type=Path, default=ROOT.parent / "rebotarm_bringup/description/DM")
    parser.add_argument("--report", type=Path)
    main(parser.parse_args())

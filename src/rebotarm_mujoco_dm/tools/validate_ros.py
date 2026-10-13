#!/usr/bin/env python3
"""Exercise the running DM MuJoCo controllers and record measured ROS state.

Sends real ROS goals, but requires the DM MuJoCo hardware plugin before doing so.
Run with the same ROS_DOMAIN_ID as mujoco_dm.launch.py.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, ListHardwareInterfaces
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("dm_phase2_validation")
    latest = {}
    history = []
    clocks = []

    def state_callback(message):
        latest.clear()
        latest.update(zip(message.name, message.position))
        history.append(dict(latest))

    subscriptions = [
        node.create_subscription(JointState, "/joint_states", state_callback, qos_profile_sensor_data),
        node.create_subscription(Clock, "/clock", lambda m: clocks.append(m.clock.sec + m.clock.nanosec*1e-9), 10),
    ]

    def wait_until(predicate, timeout=15):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        require(predicate(), "Timed out waiting for ROS data/result")

    def service(kind, name, request=None):
        client = node.create_client(kind, name)
        require(client.wait_for_service(timeout_sec=15), f"Missing service {name}")
        future = client.call_async(request or kind.Request())
        wait_until(future.done)
        result = future.result()
        node.destroy_client(client)
        return result

    try:
        hardware = service(ListHardwareComponents, "/controller_manager/list_hardware_components")
        require(len(hardware.component) == 1, "Expected one simulation hardware component")
        component = hardware.component[0]
        require(component.name == "RebotArmMujocoDMSystem" and
                component.class_type == "mujoco_ros2_control/MujocoSystemInterface",
                "Refusing motion: this is not the DM MuJoCo hardware system")
        controllers = service(ListControllers, "/controller_manager/list_controllers")
        states = {c.name: c.state for c in controllers.controller}
        require(states == {n: "active" for n in ("joint_state_broadcaster", "rebotarm_controller", "gripper_controller")},
                f"Unexpected controllers: {states}")
        interfaces = service(ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces")
        commands = {i.name for i in interfaces.command_interfaces}
        expected = [f"joint{i}" for i in range(1, 7)] + ["finger_left"]
        require(commands == {n+"/position" for n in expected}, "Command interface mismatch")
        require(all(i.is_claimed and i.is_available for i in interfaces.command_interfaces), "Unclaimed interface")
        require({i.name for i in interfaces.state_interfaces} ==
                {n+"/"+v for n in expected+["finger_right"] for v in ("position", "velocity")},
                "State interface mismatch")
        sim_time_nodes = ["controller_manager", "robot_state_publisher", "world_to_base", *states]
        for name in sim_time_nodes:
            request = GetParameters.Request(names=["use_sim_time"])
            response = service(GetParameters, f"/{name}/get_parameters", request)
            require(response.values[0].bool_value, f"{name} is not using simulation time")
        wait_until(lambda: len(history) > 10 and len(clocks) > 10)
        publishers = node.get_publishers_info_by_topic("/joint_states")
        require(len(publishers) == 1 and publishers[0].node_name == "joint_state_broadcaster",
                "Expected exactly one joint_state_broadcaster publisher")
        require(set(latest) == set(expected+["finger_right"]), "JointState inventory mismatch")
        require(clocks[-1] > clocks[0], "Clock is not advancing")
        initial = dict(latest)
        measurements = []

        def move(controller, joint, target, duration, tolerance):
            client = ActionClient(node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
            require(client.wait_for_server(timeout_sec=10), "Missing trajectory action")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [joint]
            point = JointTrajectoryPoint(positions=[target], velocities=[0.0])
            point.time_from_start.sec = duration
            goal.trajectory.points = [point]
            start = len(history)
            sent = client.send_goal_async(goal)
            wait_until(sent.done)
            handle = sent.result()
            require(handle.accepted, "Trajectory rejected")
            result = handle.get_result_async()
            wait_until(result.done, duration+15)
            response = result.result()
            require(response.status == 4 and response.result.error_code == 0,
                    f"Trajectory failed: {response.result.error_code} {response.result.error_string}")
            wait_until(lambda: len(history) > start+2)
            error = abs(latest[joint]-target)
            require(error < tolerance, f"Measured {joint} error {error}")
            measurements.append({"joint": joint, "target": target, "measured": latest[joint],
                                 "error": error, "action_error_code": response.result.error_code,
                                 "samples_during_goal": len(history)-start})
            print(json.dumps(measurements[-1]), flush=True)
            client.destroy()

        move("rebotarm_controller", "joint1", 0.2, 4, 0.02)
        require(abs(latest["joint1"]-initial["joint1"]) > 0.15, "Joint did not move")
        move("rebotarm_controller", "joint1", 0.0, 4, 0.02)
        move("gripper_controller", "finger_left", 0.04, 3, 0.001)
        require(abs(latest["finger_right"]+0.04) < 0.001, "Passive finger did not open")
        move("gripper_controller", "finger_left", 0.0, 3, 0.001)
        mimic_error = max(abs(s["finger_left"]+s["finger_right"]) for s in history)
        require(mimic_error < 0.0001, f"Excessive gripper mimic error: {mimic_error}")
        require(all(b >= a for a, b in zip(clocks, clocks[1:])), "Simulation time went backwards")
        report = {"result": "PASS", "hardware_plugin": component.class_type,
                  "controllers": states, "command_interfaces": sorted(commands),
                  "state_interface_count": len(interfaces.state_interfaces),
                  "joint_state_publishers": len(publishers), "use_sim_time_nodes": sim_time_nodes,
                  "clock_start": clocks[0], "clock_end": clocks[-1],
                  "state_samples": len(history), "peak_finger_mimic_error_m": mimic_error,
                  "goals": measurements}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

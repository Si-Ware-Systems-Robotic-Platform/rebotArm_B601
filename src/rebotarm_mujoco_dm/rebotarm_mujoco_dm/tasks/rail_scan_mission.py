"""High-level task mission: pick each rail ball, scan at the sensor box, and return to matching lane."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from ..core.arm_interface import ArmInterface, lifted
from ..core.grasp_checks import CLOSE_TARGET


def load_yaml_config():
    """Load default scene parameters from config YAML if available."""
    try:
        pkg_dir = Path(get_package_share_directory("rebotarm_mujoco_dm"))
        candidates = [
            pkg_dir / "config/scenes/rail_scan_params.yaml",
            pkg_dir / "config/rail_scan_params.yaml",
        ]
        for c in candidates:
            if c.exists():
                data = yaml.safe_load(c.read_text())
                return data.get("rail_scan", {}).get("ros__parameters", {})
    except Exception:
        pass
    return {}


class RailScanMission(ArmInterface):
    """Orchestrates picking balls from rail cradle stops, scanning, and sorting by material."""

    def __init__(self):
        super().__init__("rail_scan_mission")
        cfg = load_yaml_config()

        # Mission parameters (with robust defaults)
        self.scan_xy = (
            cfg.get("scan_station", {}).get("x", 0.120),
            cfg.get("scan_station", {}).get("y", -0.270),
        )
        self.scan_approach_z = cfg.get("scan_station", {}).get("approach_z", 0.130)
        self.scan_contact_z = cfg.get("scan_station", {}).get("contact_z", 0.038)
        self.scan_dwell = cfg.get("scan_station", {}).get("dwell_time", 2.0)
        self.scan_yaw = cfg.get("scan_station", {}).get("yaw", -0.785398)

        self.rail_start_x = cfg.get("rails", {}).get("start_x", 0.200)
        self.rail_drop_z = cfg.get("rails", {}).get("drop_z", 0.115)
        self.cradle_x = cfg.get("rails", {}).get("cradle_x", 0.3737)
        self.settle_time = cfg.get("rails", {}).get("settle_time", 3.0)

        self.target_lane_y = cfg.get("rails", {}).get(
            "lanes", {"hdpe": -0.10, "pp": 0.00, "pet": 0.10}
        )
        self.ball_names = tuple(
            cfg.get(
                "puzzle", {}
            ).get(
                "balls",
                [
                    "loose_ball_1",
                    "loose_ball_2",
                    "loose_ball_3",
                    "loose_ball_4",
                    "loose_ball_5",
                    "loose_ball_6",
                ],
            )
        )
        self.materials = cfg.get(
            "puzzle", {}
        ).get(
            "material_map",
            {
                "loose_ball_1": "pp",
                "loose_ball_2": "pet",
                "loose_ball_3": "pet",
                "loose_ball_4": "hdpe",
                "loose_ball_5": "hdpe",
                "loose_ball_6": "pp",
            },
        )

        self.poses = {}
        self.poses_received = 0.0

        self.create_subscription(
            FreeJointStateArray,
            "/mujoco_scene/free_joint_states",
            self.observe_all,
            qos_profile_sensor_data,
        )
        self.select_client = self.create_client(
            SetParametersAtomically, "/mujoco_scene_bridge/set_parameters_atomically"
        )
        self.scan_pub = self.create_publisher(String, "/mujoco_scene/scan_result", 10)

    def observe_all(self, message):
        values = {
            entry.name: deepcopy(entry.pose)
            for entry in message.free_joints
            if entry.name in self.ball_names
        }
        if len(values) == len(self.ball_names):
            self.poses = values
            self.poses_received = time.monotonic()

    def select(self, name):
        if not self.select_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("Scene bridge parameter service unavailable")
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(
                name="active_ball_id",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=name
                ),
            )
        ]
        deadline = time.monotonic() + 8.0
        while True:
            response = self.wait(
                self.select_client.call_async(request), timeout=12.0, check_state=False
            )
            if response.result.successful:
                break
            if time.monotonic() > deadline or "Finish the current" not in response.result.reason:
                raise RuntimeError(f"Cannot select {name}: {response.result.reason}")
            self.spin_once(timeout_sec=0.05)
        self.ball = None
        self.ready = False
        self.history.clear()
        self.ball_trace.clear()
        self.require_geometry = False
        self.require_held = False
        self.held = False
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.02)
            if self.fresh() and self.ball is not None:
                p = self.ball.pose.position
                q = self.poses.get(name, self.ball).pose.position
                if math.dist((p.x, p.y, p.z), (q.x, q.y, q.z)) < 0.003:
                    return
        raise RuntimeError(f"{name} did not become ready")

    def cycle_one(self, name):
        self.select(name)
        # Wait until target ball is settled at cradle stop (X ~ cradle_x)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.05)
            if self.fresh() and self.ball is not None:
                p = self.ball.pose.position
                if abs(p.x - self.cradle_x) < 0.025 and 0.035 < p.z < 0.060:
                    break
        start = deepcopy(self.ball.pose.position)
        if abs(start.x - self.cradle_x) > 0.025 or not (0.035 < start.z < 0.060):
            raise RuntimeError(f"{name} is not settled at its rail end: {start}")
        x, y, tcp_z = start.x, start.y, start.z - 0.014
        self.get_logger().info(
            f"Picking {name} at rail end ({x:.3f}, {y:.3f}, {start.z:.3f}) m"
        )
        self.fingers(0.048)
        self.move(x, y, tcp_z + 0.10, yaw=0.0)
        self.service(self.touch, SetBool.Request(data=True))
        self.service(self.enter, SetBool.Request(data=True))
        for offset in (0.035, 0.0):
            if not self.fresh():
                raise RuntimeError(self.state_error())
            p = self.ball.pose.position
            if math.hypot(p.x - x, p.y - y) > 0.020:
                raise RuntimeError(f"{name} moved away during rail entry")
            self.move(p.x, p.y, p.z - 0.014 + offset, linear=True, speed=0.2, duration=1.2, yaw=0.0)
        self.fingers(CLOSE_TARGET, closing=True)
        self.settle(1.5)
        self.verify_hold(0.025)
        self.require_geometry = True
        p = self.ball.pose.position
        x, y, tcp_z = p.x, p.y, p.z - 0.014
        self.move(x, y, tcp_z + 0.08, linear=True, yaw=0.0)
        self.settle(1.5)
        if not lifted(start, self.ball.pose.position):
            raise RuntimeError(f"{name} did not physically lift")
        self.verify_hold(start.z + 0.045)
        self.service(self.enter, SetBool.Request(data=False))
        self.service(self.attach, Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not self.held and time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.05)
        if not self.held:
            raise RuntimeError("No measured grasp acknowledgement")
        self.require_held = True

        # Move to scan station and lower into contact with sensor window
        self.move(x, y, self.scan_approach_z, linear=True, duration=1.2, yaw=0.0)
        self.move(*self.scan_xy, self.scan_approach_z, linear=False, duration=2.0, yaw=self.scan_yaw)
        self.service(self.enter, SetBool.Request(data=True))
        self.move(*self.scan_xy, self.scan_contact_z, linear=True, speed=0.2, duration=1.2, yaw=self.scan_yaw)
        self.settle(self.scan_dwell)
        self.verify_hold(0.045)
        material = self.materials.get(name, "unknown")
        reading = f"{name}: {material} (simulated reading)"
        self.scan_pub.publish(String(data=reading))
        self.get_logger().info(f"Scan contact complete: {reading}")

        # Move to START (high end) of the matching destination rail
        self.move(*self.scan_xy, self.scan_approach_z, linear=True, duration=1.2, yaw=self.scan_yaw)
        self.service(self.enter, SetBool.Request(data=False))
        target_y = self.target_lane_y[material]
        self.get_logger().info(
            f"Sorting {name} ({material}) -> start of rail Y={target_y:.2f} m"
        )
        self.move(self.rail_start_x, target_y, 0.150, linear=False, duration=2.0, yaw=0.0)
        self.service(self.enter, SetBool.Request(data=True))
        self.move(self.rail_start_x, target_y, self.rail_drop_z, linear=True, speed=0.2, duration=1.2, yaw=0.0)
        self.verify_hold(0.080)
        self.require_held = False
        self.require_geometry = False
        self.service(self.detach, Trigger.Request())
        self.fingers(0.040)
        self.move(self.rail_start_x, target_y, 0.150, linear=True, duration=1.2, yaw=0.0)
        self.service(self.enter, SetBool.Request(data=False))
        self.service(self.touch, SetBool.Request(data=False))
        self.get_logger().info(
            f"Dropped {name} at rail start; rolling down to cradle..."
        )
        self.settle(self.settle_time)
        self.get_logger().info(f"Sorted and settled {name} in {material} rail")

    def is_sorted(self, name):
        if name not in self.poses:
            return False
        pos = self.poses[name].pose.position
        material = self.materials.get(name)
        if material not in self.target_lane_y:
            return False
        target_y = self.target_lane_y[material]
        return (
            abs(pos.y - target_y) < 0.025
            and 0.170 <= pos.x <= 0.420
            and 0.035 < pos.z < 0.070
        )

    def run_cycle(self, limit):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.1)
            if self.fresh() and time.monotonic() - self.poses_received < 0.75:
                break
        if not self.fresh() or len(self.poses) != len(self.ball_names):
            raise RuntimeError("Launch scene:=rail_scan use_moveit:=true first")
        processed = 0
        for name in self.ball_names:
            if processed >= limit:
                break
            if self.is_sorted(name):
                self.get_logger().info(
                    f"{name} is already in its matching {self.materials[name]} rail; skipping"
                )
                continue
            self.cycle_one(name)
            processed += 1
        self.get_logger().info(
            f"PASS: sorted {processed} measured balls into matching rails"
        )


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, choices=range(1, 7), default=6)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = RailScanMission()
    try:
        node.run_cycle(options.limit)
    except (Exception, KeyboardInterrupt) as error:
        try:
            node.cancel_motion()
            for client in (node.enter, node.touch):
                if client.service_is_ready():
                    node.service(client, SetBool.Request(data=False))
        except Exception as cleanup_error:
            node.get_logger().warning(
                f"Cleanup incomplete: {cleanup_error}; restart scene"
            )
        node.get_logger().error(f"Rail scan stopped: {error}")
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

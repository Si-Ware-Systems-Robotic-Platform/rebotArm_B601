"""High-level task mission: pick each ball from the table and test sliding down each rail."""
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
from ..core.slider_checks import SlideMonitor, release_correction, release_tcp
from ..bridge.scene_geometry import rotate


def load_slider_config():
    """Load the parameters generated alongside the slider scene."""
    try:
        path = Path(get_package_share_directory("rebotarm_mujoco_dm")) / "config/scenes/slider_params.yaml"
        if path.exists():
            data = yaml.safe_load(path.read_text())
            return data.get("slider_mission", {}).get("ros__parameters", {})
    except Exception as error:
        raise RuntimeError(f"Cannot load slider configuration: {error}") from error
    raise RuntimeError("Slider configuration missing; rebuild rebotarm_mujoco_dm")


class SliderMission(ArmInterface):
    """Picks each of the 3 balls from the table and places each at the start of its rail."""

    def __init__(self):
        super().__init__("slider_mission")
        cfg = load_slider_config()

        self.slider_pos = tuple(cfg.get("slider", {}).get("pos", [0.450, -0.040, 0.000]))
        # Configuration follows MJCF wxyz ordering; rotate() uses ROS xyzw.
        w, x, y, z = cfg["slider"]["quat"]
        self.slider_inverse = [-x, -y, -z, w]
        self.rails_cfg = cfg.get("slider", {}).get("rails", {
            1: {"entry_x": 0.3300, "entry_y":  0.0685, "entry_z": 0.1580},
            2: {"entry_x": 0.3300, "entry_y": -0.0005, "entry_z": 0.1580},
            3: {"entry_x": 0.3300, "entry_y": -0.0690, "entry_z": 0.1580},
        })
        self.release_clearance = cfg["slider"]["release_clearance"]
        self.release_tolerance = cfg["slider"]["release_tolerance"]
        self.release_tolerance_z = cfg["slider"].get("release_tolerance_z", 0.006)
        # Approach and retreat height above the release TCP. With the gripper horizontal,
        # joint4 reaches its +1.57 rad limit ~20 mm above the release pose, so a fixed
        # high approach (the former 0.200 m) has no IK solution.
        self.approach_height = cfg["slider"].get("approach_height", 0.015)
        self.slide_timeout = cfg["slider"]["slide_timeout"]
        self.slide_monitor = None
        self.slide_ball = None
        self.slide_received = 0.0

        self.ball_names = tuple(
            cfg.get("puzzle", {}).get("balls", ["loose_ball_1", "loose_ball_2", "loose_ball_3"])
        )
        self.rail_map = cfg.get("puzzle", {}).get("rail_map", {
            "loose_ball_1": 1,
            "loose_ball_2": 2,
            "loose_ball_3": 3,
        })
        self.approach_lift_z = cfg.get("puzzle", {}).get("approach_lift_z", 0.140)

        self.poses = {}
        self.poses_received = 0.0

        self.create_subscription(
            FreeJointStateArray,
            "/mujoco_scene/free_joint_states",
            self._observe_all,
            qos_profile_sensor_data,
        )
        self.select_client = self.create_client(
            SetParametersAtomically, "/mujoco_scene_bridge/set_parameters_atomically"
        )
        self.status_pub = self.create_publisher(String, "/mujoco_scene/slider_status", 10)

    def _observe_all(self, message):
        values = {
            entry.name: deepcopy(entry.pose)
            for entry in message.free_joints
            if entry.name in self.ball_names
        }
        if len(values) == len(self.ball_names):
            self.poses = values
            self.poses_received = time.monotonic()
        if self.slide_monitor is not None and self.slide_ball in values:
            pose = values[self.slide_ball]
            p = pose.pose.position
            local = rotate(self.slider_inverse, [v - origin for v, origin in
                           zip((p.x, p.y, p.z), self.slider_pos)])
            stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
            if stamp != self.slide_monitor.last_stamp:
                self.slide_received = time.monotonic()
            self.slide_monitor.update(stamp, local)

    def measured_release_tcp(self, center):
        if not self.history or time.monotonic() - self.relative_received > 1.5:
            raise RuntimeError("Fresh ball-in-TCP measurement required for release")
        # All slider transfers command yaw=0: a +90 degree rotation about Y.
        s = math.sqrt(0.5)
        offset = rotate([0., s, 0., s], self.history[-1][1])
        return release_tcp(center, offset, self.release_clearance)

    def align_release(self, tcp, expected, attempts=3):
        """Servo on the measured ball centre: the loaded arm settles a few mm off its command."""
        tcp = list(tcp)
        for attempt in range(attempts + 1):
            self.settle(0.5)
            p = self.ball.pose.position
            actual = (p.x, p.y, p.z)
            shift = release_correction(actual, expected, self.release_tolerance,
                                       self.release_tolerance_z)
            error_mm = math.dist(actual, expected) * 1000
            if shift is None:
                self.get_logger().info(f"Release aligned within {error_mm:.1f} mm "
                                       f"after {attempt} correction(s)")
                return
            if attempt == attempts:
                break
            self.get_logger().info(f"Release miss {error_mm:.1f} mm; correcting TCP")
            tcp = [t + s for t, s in zip(tcp, shift)]
            self.move(*tcp, linear=True, speed=0.1, duration=0.8, yaw=0.0)
        raise RuntimeError(f"Measured release error {error_mm:.1f} mm still exceeds "
                           f"{self.release_tolerance*1000:.1f} mm XY / "
                           f"{self.release_tolerance_z*1000:.1f} mm Z; keeping grip")

    def clear_contact_permissions(self):
        """Drop finger/ball contact permissions left by an aborted run.

        The scene bridge refuses to switch balls while they are active."""
        for client in (self.enter, self.touch):
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("Scene contact-permission service unavailable")
            result = self.wait(client.call_async(SetBool.Request(data=False)),
                               timeout=12.0, check_state=False)
            if not result.success:
                self.get_logger().warn(f"Could not clear contact permissions: {result.message}")

    def verify_slide(self):
        # Observe during retreat too: the outlet may be crossed before retreat ends.
        deadline = time.monotonic() + max(30., self.slide_timeout * 4)
        while rclpy.ok():
            monitor = self.slide_monitor
            if monitor.failure:
                raise RuntimeError(monitor.failure)
            if monitor.exited:
                self.status_pub.publish(String(data=f"PASS: {self.slide_ball} exited the CAD slider"))
                self.get_logger().info(f"PASS: measured {self.slide_ball} travelling around the bend and through the outlet")
                return
            if time.monotonic() - self.slide_received > 2.0:
                raise RuntimeError("Ball observations stale while verifying slide")
            if time.monotonic() > deadline:
                raise RuntimeError("Wall-clock timeout waiting for slider outcome; check simulation clock")
            self.spin_once(timeout_sec=0.02)
        raise RuntimeError("ROS shutdown during slider verification")

    def select(self, name):
        """Select active ball for tracking and relative pose measurement."""
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
            # Safe attribute checking for SetParametersAtomically response
            res = getattr(response, "result", response)
            if getattr(res, "successful", False):
                break
            reason = getattr(res, "reason", str(res))
            if time.monotonic() > deadline or "Finish the current" not in reason:
                raise RuntimeError(f"Cannot select {name}: {reason}")
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
                if math.dist((p.x, p.y, p.z), (q.x, q.y, q.z)) < 0.005:
                    return
        raise RuntimeError(f"{name} did not become ready")

    def test_rail(self, ball_name):
        """Pick a ball from the table and place it at the upstream entry of its rail."""
        rail_id = self.rail_map.get(ball_name, 1)
        rail_cfg = self.rails_cfg.get(rail_id, self.rails_cfg.get(str(rail_id)))
        if not rail_cfg:
            raise ValueError(f"No rail configuration found for Rail {rail_id}")

        self.get_logger().info(f"--- Starting test for {ball_name} -> Rail {rail_id} ---")
        self.slide_monitor = None
        self.select(ball_name)

        # Wait for ball pose to settle on table
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.05)
            if self.fresh() and self.ball is not None:
                break
        if not self.fresh() or self.ball is None:
            raise RuntimeError(f"No fresh pose available for {ball_name}")

        start_pos = deepcopy(self.ball.pose.position)
        bx, by, bz = start_pos.x, start_pos.y, start_pos.z
        tcp_z = bz - 0.014
        self.get_logger().info(
            f"Picking {ball_name} from table at ({bx:.3f}, {by:.3f}, {bz:.3f}) m"
        )

        # Open gripper fingers to clear ball
        self.fingers(0.048)

        # Move arm above ball at approach height
        self.move(bx, by, tcp_z + self.approach_lift_z, linear=False, duration=2.0, yaw=0.0)

        # Notify scene bridge of ball proximity and entry
        self.service(self.touch, SetBool.Request(data=True))
        self.service(self.enter, SetBool.Request(data=True))

        # Descend linearly onto the ball
        for offset in (0.040, 0.0):
            if not self.fresh():
                raise RuntimeError(self.state_error())
            p = self.ball.pose.position
            self.move(p.x, p.y, p.z - 0.014 + offset, linear=True, speed=0.2, duration=1.2)

        # Close fingers to grasp ball securely
        self.fingers(CLOSE_TARGET, closing=True)
        self.settle(1.5)
        self.verify_hold(0.020)
        self.require_geometry = True

        # Lift ball off the table
        p = self.ball.pose.position
        self.move(p.x, p.y, p.z - 0.014 + self.approach_lift_z, linear=True)
        self.settle(1.2)
        if not lifted(start_pos, self.ball.pose.position):
            raise RuntimeError(f"{ball_name} did not physically lift off the table")
        self.verify_hold(start_pos.z + 0.045)

        # Attach ball collision model to gripper for obstacle avoidance
        self.service(self.enter, SetBool.Request(data=False))
        self.service(self.attach, Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not self.held and time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.05)
        if not self.held:
            raise RuntimeError(f"Grasp hold for {ball_name} not acknowledged")
        self.require_held = True

        # Target entry coordinates for this rail (user CAD fixed joints)
        entry_x = rail_cfg["entry_x"]
        entry_y = rail_cfg["entry_y"]
        entry_z = rail_cfg["entry_z"]
        center = (entry_x, entry_y, entry_z)
        drop_x, drop_y, drop_z = self.measured_release_tcp(center)
        approach_z = drop_z + self.approach_height

        self.get_logger().info(
            f"Transferring {ball_name} to upstream start of Rail {rail_id} (ball midpoint target: {entry_x:.3f}, {entry_y:.3f}, {entry_z:.3f} m)"
        )

        # Transfer above the upstream rail entry
        self.move(drop_x, drop_y, approach_z, linear=False, duration=2.5, yaw=0.0)

        # Descend gently to drop height
        self.move(drop_x, drop_y, drop_z, linear=True, speed=0.15, duration=2.0, yaw=0.0)
        expected = (entry_x, entry_y, entry_z + self.release_clearance)
        self.align_release((drop_x, drop_y, drop_z), expected)

        # Release ball into the rail
        self.require_held = False
        self.require_geometry = False
        self.service(self.detach, Trigger.Request())
        self.slide_ball = ball_name
        self.slide_monitor = SlideMonitor(timeout=self.slide_timeout)
        self.slide_received = time.monotonic()
        self.fingers(0.045)
        self.get_logger().info(
            f"Released {ball_name} at Rail {rail_id}; observing actual ball travel"
        )

        # Lift straight out of the rail mouth; the joint-space move completes the retreat.
        self.move(drop_x, drop_y, approach_z, linear=True, speed=0.15, duration=1.0)
        # Retract to safe standby pose clear of the slider
        self.move(0.240, 0.120, 0.200, linear=False, duration=1.8)
        self.service(self.enter, SetBool.Request(data=False))
        self.service(self.touch, SetBool.Request(data=False))

        self.verify_slide()

    def run_all(self):
        """Run the full sequence for all 3 balls and rails."""
        self.get_logger().info("=== Commencing 3-Rail Slider Mission ===")
        # Initial settle
        time.sleep(1.5)
        for ball_name in self.ball_names:
            self.test_rail(ball_name)
        self.get_logger().info("=== 3-Rail Slider Mission Successfully Completed! ===")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ball",
        choices=["loose_ball_1", "loose_ball_2", "loose_ball_3", "all"],
        default="all",
        help="Test specific ball or all 3 balls",
    )
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)

    mission = SliderMission()
    try:
        mission.clear_contact_permissions()
        if options.ball == "all":
            mission.run_all()
        else:
            mission.test_rail(options.ball)
    except (RuntimeError, TimeoutError) as error:
        mission.get_logger().error(f"Slider mission stopped: {error}. Inspect the scene before retrying.")
        try:
            mission.cancel_motion()
            mission.clear_contact_permissions()
        except (RuntimeError, TimeoutError) as cleanup_error:
            mission.get_logger().warn(f"Cleanup after failure incomplete: {cleanup_error}")
        raise SystemExit(1)
    finally:
        mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

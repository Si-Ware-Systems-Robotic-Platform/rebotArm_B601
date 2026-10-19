"""Clean, reusable ROS 2 interface for B601-DM arm motion, gripper, and grasp checking."""
from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import rclpy
from rebotarm_msgs.action import MoveToPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .grasp_checks import (
    CENTER_TOLERANCE,
    CLOSE_TARGET,
    EXPECTED_OFFSET,
    fingers_error,
    hold_error,
)
from ..bridge.pose_requests import stretch_trajectory


def lifted(start, current):
    """Check if object lifted by at least 45 mm."""
    return current.z - start.z >= 0.045


class PathToleranceViolation(RuntimeError):
    """A trajectory controller stopped because measured motion diverged."""


class GraspVerificationError(RuntimeError):
    """Fresh measurements show that the selected ball is not held."""


class ArmInterface(Node):
    """Reusable high-level interface for moving the robot arm, gripper, and verifying grasps."""

    def __init__(self, node_name="rebotarm_interface"):
        super().__init__(node_name, automatically_declare_parameters_from_overrides=True)
        self.ball = None
        self.received = 0.0
        self.ready = False
        self.ready_received = 0.0
        self.active = None
        self.active_result = None
        self.pending_goal = None
        self.history = []
        self.ball_trace = []
        self.finger_positions = None
        self.fingers_received = 0.0
        self.relative_received = 0.0
        self.require_geometry = False
        self.require_held = False
        self.held = False
        self.held_received = 0.0
        self.last_target = None

        # Subscriptions
        self.create_subscription(PoseStamped, "/mujoco_scene/ball_in_tcp", self._on_relative, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Bool, "/mujoco_scene/grasp_held", self._on_held_status, 10)
        self.create_subscription(PoseStamped, "/mujoco_scene/ball_pose", self._on_observe, 10)
        self.create_subscription(Bool, "/mujoco_scene/ready", self._on_status, 10)

        # Action clients
        self.arm = ActionClient(self, MoveToPose, "/rebotarm/move_to_pose")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        # Service clients
        self.touch = self.create_client(SetBool, "/mujoco_scene/touch_ball")
        self.enter = self.create_client(SetBool, "/mujoco_scene/enter_bowl")
        self.attach = self.create_client(Trigger, "/mujoco_scene/attach_ball")
        self.detach = self.create_client(Trigger, "/mujoco_scene/detach_ball")

        self._spin_executor = SingleThreadedExecutor(context=self.context)
        self._spin_executor.add_node(self)

    def spin_once(self, timeout_sec=0.05):
        self._spin_executor.spin_once(timeout_sec=timeout_sec)

    def destroy_node(self):
        self._spin_executor.shutdown()
        return super().destroy_node()

    def _on_relative(self, message):
        if self.ball is None:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        p = message.pose.position
        self.relative_received = time.monotonic()
        self.history.append((stamp, [p.x, p.y, p.z], self.ball.pose.position.z))
        self.history = [v for v in self.history if stamp - v[0] <= 1.0]

    def _on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(n in values for n in ("finger_left", "finger_right")):
            self.finger_positions = (values["finger_left"], values["finger_right"])
            self.fingers_received = time.monotonic()

    def _on_held_status(self, message):
        self.held = message.data
        self.held_received = time.monotonic()

    def _on_observe(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.ball_trace and stamp < self.ball_trace[-1][0]:
            self.ball_trace.clear()
        if not self.ball_trace or stamp > self.ball_trace[-1][0]:
            p = message.pose.position
            self.ball_trace.append((stamp, [p.x, p.y, p.z]))
            self.ball_trace = [sample for sample in self.ball_trace if stamp - sample[0] <= 2.0]
        if self.ball is None or message.header.stamp != self.ball.header.stamp:
            self.received = time.monotonic()
        self.ball = message

    def _on_status(self, message):
        self.ready = message.data
        self.ready_received = time.monotonic()

    # Backwards-compatible subscriber aliases
    relative = _on_relative
    joints = _on_joints
    held_status = _on_held_status
    observe = _on_observe
    status = _on_status

    def fresh(self):
        return self.state_error() is None

    def state_error(self):
        now = time.monotonic()
        if self.ball is None or now - self.received >= 1.5:
            return f"Ball pose missing or stale (age {now-self.received:.3f} s)"
        if now - self.ready_received >= 2.0:
            return f"Scene readiness publisher stale (age {now-self.ready_received:.3f} s)"
        if not self.ready:
            return "Scene bridge reports not ready; inspect /mujoco_scene/grasp_fault"
        if self.require_geometry:
            if not self.history or now - self.relative_received >= 1.5:
                return f"Ball/TCP measurements missing or stale (age {now-self.relative_received:.3f} s)"
            error = math.dist(self.history[-1][1], EXPECTED_OFFSET)
            if not math.isfinite(error) or error >= CENTER_TOLERANCE:
                return (
                    f"Ball moved out of the grip: centre error {error*1000:.1f} mm "
                    f"(limit {CENTER_TOLERANCE*1000:.1f} mm), offset {self.history[-1][1]} m"
                )
        if self.require_held and (not self.held or now - self.held_received >= 1.5):
            return f"Measured hold lost or stale (age {now-self.held_received:.3f} s)"
        return None

    def wait(self, future, timeout=180.0, check_state=True):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            self.spin_once(timeout_sec=0.02)
            if time.monotonic() > deadline:
                raise TimeoutError("ROS request timed out")
            if check_state and not self.fresh():
                raise RuntimeError(self.state_error())
        if not future.done():
            raise RuntimeError("ROS shutdown")
        return future.result()

    def settle(self, seconds):
        if self.ball is None:
            time.sleep(seconds)
            return
        start = self.ball.header.stamp.sec + self.ball.header.stamp.nanosec * 1e-9
        deadline = time.monotonic() + max(15.0, seconds * 10)
        while rclpy.ok():
            self.spin_once(timeout_sec=0.02)
            if not self.fresh():
                raise RuntimeError(self.state_error())
            now = self.ball.header.stamp.sec + self.ball.header.stamp.nanosec * 1e-9
            if now < start or time.monotonic() > deadline:
                raise RuntimeError("Simulation reset or stalled during hold")
            if now - start >= seconds:
                return

    def verify_hold(self, minimum_height):
        stamp = self.ball.header.stamp.sec + self.ball.header.stamp.nanosec * 1e-9
        error = hold_error(self.history, stamp, minimum_height)
        if time.monotonic() - self.relative_received > 1.5 or error:
            if time.monotonic() - self.relative_received > 1.5:
                raise RuntimeError("Ball/TCP measurements are stale")
            raise GraspVerificationError(error)
        if self.finger_positions is None or time.monotonic() - self.fingers_received > 1.5:
            raise RuntimeError("Finger measurements are stale")
        error = fingers_error(*self.finger_positions)
        if error:
            raise GraspVerificationError(error)
        self.get_logger().info(
            f"Hold verified: ball z={self.ball.pose.position.z:.4f} m, "
            f"finger_left={self.finger_positions[0]:.4f} m"
        )

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("Scene service unavailable")
        result = self.wait(client.call_async(request), timeout=12.0, check_state=False)
        if not result.success:
            raise RuntimeError(result.message)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=0.05)
            if self.fresh():
                return
        raise RuntimeError("Scene did not become ready after its update")

    def action(self, client, goal, label="motion"):
        if not self.fresh() or not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Scene/action server unavailable")
        sent = client.send_goal_async(goal)
        self.pending_goal = sent
        handle = self.wait(sent, timeout=10.0)
        self.pending_goal = None
        if not handle.accepted:
            raise RuntimeError("Motion goal rejected")
        self.active = handle
        self.active_result = handle.get_result_async()
        result = self.wait(self.active_result)
        self.active = None
        self.active_result = None
        code = getattr(result.result, "error_code", None)
        value = getattr(code, "val", code)
        if isinstance(result.result, FollowJointTrajectory.Result) and value == -4:
            raise PathToleranceViolation(f"{label} path tolerance violation: {result.result}")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{label} failed: {result.result}")
        if hasattr(result.result, "success") and not result.result.success:
            raise RuntimeError(f"{label} failed: {result.result.message}")

    def cancel_motion(self):
        if self.pending_goal is not None:
            handle = self.wait(self.pending_goal, timeout=5.0, check_state=False)
            self.pending_goal = None
            if handle and handle.accepted:
                self.active = handle
                self.active_result = handle.get_result_async()
        if self.active is not None:
            if self.active_result is None:
                self.active_result = self.active.get_result_async()
            if not self.active_result.done():
                self.active.cancel_goal_async()
                self.wait(self.active_result, timeout=5.0, check_state=False)
            self.active = None
            self.active_result = None

    def move(self, x, y, z, linear=False, min_fraction=0.999, yaw=None, speed=0.5, duration=None):
        if yaw is None:
            yaw = getattr(self, "_current_yaw", 0.0)
        self._current_yaw = float(yaw)
        self.get_logger().info(
            f"TCP target: {x:.3f}, {y:.3f}, {z:.3f} m"
            + (f", yaw {math.degrees(yaw):.0f} deg" if yaw else "")
        )
        if duration is None:
            if self.last_target is not None:
                dist = math.dist((float(x), float(y), float(z)), self.last_target)
                duration = 1.2 if dist <= 0.08 else 2.0
            else:
                duration = 1.2 if linear else 2.0
        self.last_target = (float(x), float(y), float(z))
        goal = MoveToPose.Goal(duration=duration)
        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)
        s, half = math.sqrt(0.5), float(yaw) / 2
        goal.target_pose.orientation.x = -s * math.sin(half)
        goal.target_pose.orientation.y = s * math.cos(half)
        goal.target_pose.orientation.z = s * math.sin(half)
        goal.target_pose.orientation.w = s * math.cos(half)
        if not linear:
            self.action(self.arm, goal, "arm pose")
            return
        if not self.cartesian.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("Cartesian path service unavailable")
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.start_state.is_diff = True
        request.group_name = "arm"
        request.link_name = "gripper_tcp"
        request.waypoints = [goal.target_pose]
        request.max_step = 0.002
        request.jump_threshold = 2.0
        request.revolute_jump_threshold = 0.15
        request.prismatic_jump_threshold = 0.005
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = speed
        request.max_acceleration_scaling_factor = speed
        result = self.wait(self.cartesian.call_async(request), timeout=15.0)
        if result.error_code.val != 1 or result.fraction < min_fraction:
            raise RuntimeError(f"Cartesian path incomplete ({result.fraction:.1%}); no motion sent")
        trajectory = stretch_trajectory(result.solution, duration)
        self.action(self.execute, ExecuteTrajectory.Goal(trajectory=trajectory), "arm Cartesian trajectory")

    def fingers(self, opening, closing=False, duration=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["finger_left"]
        point = JointTrajectoryPoint(positions=[opening])
        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        goal.trajectory.points = [point]
        if closing:
            goal.path_tolerance = [JointTolerance(name="finger_left", position=0.010)]
            goal.goal_tolerance = [JointTolerance(name="finger_left", position=0.010)]
        try:
            self.action(self.gripper, goal, "gripper trajectory")
        except PathToleranceViolation:
            if not closing:
                raise
            now = time.monotonic()
            stamp = self.ball.header.stamp.sec + self.ball.header.stamp.nanosec * 1e-9 if self.ball else 0.0
            aligned = (
                self.history
                and stamp - self.history[-1][0] < 0.15
                and math.dist(self.history[-1][1], EXPECTED_OFFSET) < CENTER_TOLERANCE
            )
            if (
                self.finger_positions is None
                or now - self.fingers_received > 1.5
                or now - self.relative_received > 1.5
                or not aligned
                or fingers_error(*self.finger_positions)
            ):
                raise
            self.get_logger().info("Gripper stopped against the measured ball; checking hold before lift")


# Alias for backwards compatibility
GraspTest = ArmInterface

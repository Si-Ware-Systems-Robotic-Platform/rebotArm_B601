"""SDK MoveToPose action backed by MoveIt and MuJoCo, without a motor driver."""
from copy import deepcopy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rebotarm_msgs.action import MoveToPose
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from .pose_requests import build_plan_goal, pose_matches, stretch_trajectory, validate_goal


class PoseAdapter(Node):
    def __init__(self):
        super().__init__("mujoco_pose_adapter")
        defaults = {
            "planning_group": "arm", "base_frame": "base_link", "tip_frame": "gripper_tcp",
            "planning_time": 5.0, "position_tolerance": 0.005, "orientation_tolerance": 0.05,
            "velocity_scaling": 0.2, "acceleration_scaling": 0.2,
            "result_timeout": 180.0, "response_timeout": 10.0, "cancel_timeout": 5.0,
        }
        self.settings = {name: self.declare_parameter(name, default).value
                         for name, default in defaults.items()}
        for name, value in self.settings.items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")
        if any(self.settings[name] > 1 for name in ("velocity_scaling", "acceleration_scaling")):
            raise ValueError("Velocity and acceleration scaling must be in (0, 1]")
        self._lock = threading.Lock()
        self._busy = False
        self._blocked = False
        self._child = None
        self._child_result = None
        self._execution_start = None
        self._trajectory_duration = 0.0
        self._last_feedback = 0.0
        self._group = ReentrantCallbackGroup()
        self._buffer = Buffer(node=self)
        self._listener = TransformListener(self._buffer, self)
        self._states = self.create_publisher(JointState, "/rebotarm/joint_states", qos_profile_sensor_data)
        self._state_subscription = self.create_subscription(
            JointState, "/joint_states", self._states.publish, qos_profile_sensor_data,
            callback_group=self._group)
        self._planner = ActionClient(self, MoveGroup, "/move_action", callback_group=self._group)
        self._executor_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory", callback_group=self._group)
        self._server = ActionServer(
            self, MoveToPose, "/rebotarm/move_to_pose", execute_callback=self._execute,
            goal_callback=self._accept_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._group)
        self.get_logger().info(
            f"Simulation MoveToPose interface ready: {self.settings['tip_frame']} poses in "
            f"{self.settings['base_frame']}; waiting for MoveIt servers before accepting goals. "
            "duration is a minimum trajectory time; MoveIt limits may require longer.")

    def _accept_goal(self, goal):
        try:
            validate_goal(goal)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        with self._lock:
            if self._busy or self._blocked:
                self.get_logger().warning("Rejecting pose goal: adapter busy or awaiting restart after a backend timeout")
                return GoalResponse.REJECT
            if not self._planner.server_is_ready() or not self._executor_client.server_is_ready():
                self.get_logger().warning("Rejecting pose goal: MoveIt action servers are not ready")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _pose(self):
        try:
            transform = self._buffer.lookup_transform(
                self.settings["base_frame"], self.settings["tip_frame"], Time())
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = deepcopy(transform.transform.rotation)
        return pose

    def _feedback(self, handle, complete=False):
        now = time.monotonic()
        if not complete and now - self._last_feedback < 0.2:
            return
        self._last_feedback = now
        pose = self._pose()
        if pose is None:
            return
        feedback = MoveToPose.Feedback()
        feedback.current_pose = pose
        feedback.time_elapsed = max(0.0, (self.get_clock().now().nanoseconds - self._started) / 1e9)
        feedback.progress = 0.0
        if self._execution_start is not None and self._trajectory_duration > 0:
            elapsed = (self.get_clock().now().nanoseconds - self._execution_start) / 1e9
            feedback.progress = min(0.99, max(0.0, elapsed / self._trajectory_duration))
        if complete:
            feedback.progress = 1.0
        handle.publish_feedback(feedback)

    def _wait(self, future, handle, timeout):
        # A worker waits while other executor threads process action, TF and state callbacks.
        deadline = time.monotonic() + timeout
        cancel_sent = False
        while rclpy.ok() and not future.done():
            if handle.is_cancel_requested and self._child is not None and not cancel_sent:
                self._child.cancel_goal_async()
                cancel_sent = True
                deadline = min(deadline, time.monotonic() + self.settings["cancel_timeout"])
            if time.monotonic() > deadline:
                raise TimeoutError("MoveIt did not finish or confirm cancellation before the timeout")
            self._feedback(handle)
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError("ROS shutdown interrupted the pose goal")
        return future.result()

    def _run_child(self, client, request, handle):
        if handle.is_cancel_requested:
            return None
        if not client.server_is_ready():
            raise RuntimeError("MoveIt action server disappeared")
        sent = client.send_goal_async(request)
        try:
            child = self._wait(sent, handle, self.settings["response_timeout"])
        except Exception:
            # A late acknowledgement must not leave an unnoticed active goal.
            self._blocked = True
            def cancel_late(future):
                try:
                    goal = future.result()
                    if goal is not None and goal.accepted:
                        goal.cancel_goal_async()
                except Exception:
                    pass
            sent.add_done_callback(cancel_late)
            raise
        if child is None or not child.accepted:
            raise RuntimeError("MoveIt rejected the request")
        self._child = child
        self._child_result = child.get_result_async()
        if handle.is_cancel_requested:
            child.cancel_goal_async()
        result = self._wait(self._child_result, handle,
                            self.settings["cancel_timeout"] if handle.is_cancel_requested
                            else self.settings["result_timeout"])
        self._child = None
        self._child_result = None
        return result

    def _stop_child(self):
        if self._child is None:
            return
        try:
            self._child.cancel_goal_async()
            deadline = time.monotonic() + self.settings["cancel_timeout"]
            while rclpy.ok() and self._child_result is not None and not self._child_result.done():
                if time.monotonic() > deadline:
                    break
                time.sleep(0.02)
            if self._child_result is None or not self._child_result.done():
                self._blocked = True
        except Exception:
            self._blocked = True
        self._child = None
        self._child_result = None

    @staticmethod
    def _check_child(response, label):
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt {label} failed: status={response.status}, code={response.result.error_code.val}")

    def _execute(self, handle):
        result = MoveToPose.Result()
        self._started = self.get_clock().now().nanoseconds
        self._execution_start = None
        self._last_feedback = 0.0
        try:
            plan = self._run_child(self._planner, build_plan_goal(handle.request, self.settings), handle)
            if handle.is_cancel_requested:
                handle.canceled()
                result.message = "Pose goal canceled during planning"
            else:
                self._check_child(plan, "planning")
                if not plan.result.planned_trajectory.joint_trajectory.points:
                    pose = self._pose()
                    if not pose_matches(pose, handle.request.target_pose,
                                        self.settings["position_tolerance"], self.settings["orientation_tolerance"]):
                        raise RuntimeError("MoveIt returned no trajectory and measured TCP is outside the pose tolerance")
                    result.final_pose = pose
                    result.success = True
                    result.message = "Measured TCP is already within pose tolerance; no motion required"
                    self._feedback(handle, complete=True)
                    handle.succeed()
                    return result
                trajectory = stretch_trajectory(plan.result.planned_trajectory, handle.request.duration)
                last = trajectory.joint_trajectory.points[-1].time_from_start
                self._trajectory_duration = last.sec + last.nanosec / 1e9
                self.get_logger().info(f"Executing pose trajectory: {self._trajectory_duration:.3f} s "
                                       f"(requested minimum {handle.request.duration:.3f} s)")
                self._execution_start = self.get_clock().now().nanoseconds
                execution = self._run_child(
                    self._executor_client, ExecuteTrajectory.Goal(trajectory=trajectory), handle)
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = "Pose goal canceled during execution"
                else:
                    self._check_child(execution, "execution")
                    pose = self._pose()
                    if pose is None:
                        raise RuntimeError("Execution completed, but measured TCP TF is unavailable")
                    result.final_pose = pose
                    result.success = True
                    result.message = f"MoveIt pose execution succeeded; trajectory duration {self._trajectory_duration:.3f} s"
                    self._feedback(handle, complete=True)
                    handle.succeed()
        except Exception as error:
            self._stop_child()
            result.message = str(error)
            if self._blocked:
                result.message += "; restart the simulation stack before another adapter goal"
            self.get_logger().error(result.message)
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            if not result.success:
                pose = self._pose()
                if pose is not None:
                    result.final_pose = pose
            with self._lock:
                self._busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PoseAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_child()
        executor.shutdown(timeout_sec=6.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

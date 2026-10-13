"""Offline checks of request, timing and action outcomes; no ROS nodes are started."""
from concurrent.futures import Future
from copy import deepcopy
import math
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import RobotTrajectory
from rebotarm_msgs.action import MoveToPose
from trajectory_msgs.msg import JointTrajectoryPoint

from rebotarm_mujoco_dm.pose_adapter import PoseAdapter
from rebotarm_mujoco_dm.pose_requests import build_plan_goal, stretch_trajectory, validate_goal


SETTINGS = {"planning_group": "arm", "base_frame": "base_link", "tip_frame": "gripper_tcp",
            "planning_time": 5.0, "position_tolerance": 0.005, "orientation_tolerance": 0.05,
            "velocity_scaling": 0.2, "acceleration_scaling": 0.2, "response_timeout": 10.0}


def goal():
    result = MoveToPose.Goal(duration=2.0)
    result.target_pose.position.x = 0.3
    result.target_pose.position.y = 0.12
    result.target_pose.position.z = 0.06
    result.target_pose.orientation.w = 2.0
    return result


def trajectory():
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ["joint1"]
    for value, seconds in [(0.0, 0), (0.2, 1)]:
        point = JointTrajectoryPoint(positions=[value], velocities=[0.2], accelerations=[0.4])
        point.time_from_start.sec = seconds
        result.joint_trajectory.points.append(point)
    return result


class RequestTests(unittest.TestCase):
    def test_pose_constraints_and_current_start(self):
        original = goal()
        request = build_plan_goal(original, SETTINGS)
        self.assertTrue(request.planning_options.plan_only)
        self.assertTrue(request.request.start_state.is_diff)
        self.assertTrue(request.planning_options.planning_scene_diff.robot_state.is_diff)
        constraints = request.request.goal_constraints[0]
        position = constraints.position_constraints[0]
        self.assertEqual(position.header.frame_id, "base_link")
        self.assertEqual(position.link_name, "gripper_tcp")
        self.assertEqual(list(position.constraint_region.primitives[0].dimensions), [0.005])
        self.assertAlmostEqual(position.constraint_region.primitive_poses[0].position.y, 0.12)
        self.assertEqual(constraints.orientation_constraints[0].orientation.w, 1.0)
        self.assertEqual(original.target_pose.orientation.w, 2.0)

    def test_invalid_goals(self):
        for value in [0.0, -1.0, math.nan, math.inf]:
            invalid = goal()
            invalid.duration = value
            with self.subTest(duration=value), self.assertRaises(ValueError):
                validate_goal(invalid)
        invalid = goal()
        invalid.target_pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            validate_goal(invalid)
        invalid.target_pose.position.x = math.nan
        with self.assertRaises(ValueError):
            validate_goal(invalid)

    def test_duration_scales_derivatives_and_preserves_path(self):
        original = trajectory()
        slowed = stretch_trajectory(original, 4.0)
        self.assertEqual(slowed.joint_trajectory.points[-1].time_from_start.sec, 4)
        self.assertEqual(list(slowed.joint_trajectory.points[-1].positions), [0.2])
        self.assertAlmostEqual(slowed.joint_trajectory.points[-1].velocities[0], 0.05)
        self.assertAlmostEqual(slowed.joint_trajectory.points[-1].accelerations[0], 0.025)
        self.assertEqual(original.joint_trajectory.points[-1].time_from_start.sec, 1)

    def test_duration_never_speeds_up_plan(self):
        result = stretch_trajectory(trajectory(), 0.5)
        self.assertEqual(result.joint_trajectory.points[-1].time_from_start.sec, 1)
        self.assertEqual(list(result.joint_trajectory.points[-1].velocities), [0.2])

    def test_stationary_plan_becomes_timed_hold(self):
        source = trajectory()
        source.joint_trajectory.points = source.joint_trajectory.points[:1]
        result = stretch_trajectory(source, 2.5)
        point = result.joint_trajectory.points[0]
        self.assertEqual((point.time_from_start.sec, point.time_from_start.nanosec), (2, 500_000_000))
        self.assertEqual(list(point.velocities), [0.0])

    def test_empty_or_nonincreasing_plan_is_rejected(self):
        with self.assertRaises(ValueError):
            stretch_trajectory(RobotTrajectory(), 2.0)
        invalid = trajectory()
        invalid.joint_trajectory.points[-1].time_from_start.sec = 0
        with self.assertRaises(ValueError):
            stretch_trajectory(invalid, 2.0)


class ActionOutcomeTests(unittest.TestCase):
    def adapter(self):
        # Use the execute method with fake clients, avoiding Node construction/DDS.
        measured = Pose()
        measured.position.x = 0.299
        measured.position.y = 0.12
        measured.position.z = 0.06
        measured.orientation.w = 1.0
        return SimpleNamespace(
            settings=SETTINGS, _lock=threading.Lock(), _busy=True, _blocked=False,
            _planner=object(), _executor_client=object(),
            _run_child=Mock(), _stop_child=Mock(), _feedback=Mock(),
            _pose=lambda: deepcopy(measured), get_logger=lambda: Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
            _check_child=PoseAdapter._check_child)

    def handle(self):
        return SimpleNamespace(request=goal(), is_cancel_requested=False,
                               canceled=Mock(), abort=Mock(), succeed=Mock())

    def response(self, kind, code=1, status=GoalStatus.STATUS_SUCCEEDED):
        result = kind.Result()
        result.error_code.val = code
        if kind is MoveGroup:
            result.planned_trajectory = trajectory()
        return SimpleNamespace(status=status, result=result)

    def test_success_uses_measured_final_pose(self):
        adapter, handle = self.adapter(), self.handle()
        adapter._run_child.side_effect = [self.response(MoveGroup), self.response(ExecuteTrajectory)]
        result = PoseAdapter._execute(adapter, handle)
        self.assertTrue(result.success)
        self.assertAlmostEqual(result.final_pose.position.x, 0.299)
        handle.succeed.assert_called_once()
        self.assertFalse(adapter._busy)
        sent = adapter._run_child.call_args_list[1].args[1]
        self.assertEqual(sent.trajectory.joint_trajectory.points[-1].time_from_start.sec, 2)

    def test_planning_failure_does_not_execute(self):
        adapter, handle = self.adapter(), self.handle()
        adapter._run_child.return_value = self.response(MoveGroup, code=-31)
        result = PoseAdapter._execute(adapter, handle)
        self.assertFalse(result.success)
        self.assertIn("code=-31", result.message)
        handle.abort.assert_called_once()
        self.assertEqual(adapter._run_child.call_count, 1)

    def test_already_at_goal_requires_measured_pose(self):
        adapter, handle = self.adapter(), self.handle()
        response = self.response(MoveGroup)
        response.result.planned_trajectory = RobotTrajectory()
        adapter._run_child.return_value = response
        self.assertTrue(PoseAdapter._execute(adapter, handle).success)
        self.assertEqual(adapter._run_child.call_count, 1)
        adapter, handle = self.adapter(), self.handle()
        adapter._run_child.return_value = response
        adapter._pose = lambda: None
        self.assertFalse(PoseAdapter._execute(adapter, handle).success)
        handle.abort.assert_called_once()

    def test_execution_failure_is_not_reported_as_success(self):
        adapter, handle = self.adapter(), self.handle()
        adapter._run_child.side_effect = [self.response(MoveGroup), self.response(ExecuteTrajectory, code=-4)]
        result = PoseAdapter._execute(adapter, handle)
        self.assertFalse(result.success)
        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()

    def test_cancel_during_planning_does_not_execute(self):
        adapter, handle = self.adapter(), self.handle()
        def canceled(*_):
            handle.is_cancel_requested = True
            return self.response(MoveGroup, code=-7, status=GoalStatus.STATUS_CANCELED)
        adapter._run_child.side_effect = canceled
        result = PoseAdapter._execute(adapter, handle)
        self.assertFalse(result.success)
        handle.canceled.assert_called_once()
        self.assertEqual(adapter._run_child.call_count, 1)

    def test_late_goal_acknowledgement_is_canceled_after_timeout(self):
        sent, client = Future(), Mock()
        client.server_is_ready.return_value = True
        client.send_goal_async.return_value = sent
        adapter = SimpleNamespace(settings=SETTINGS, _blocked=False,
                                  _wait=Mock(side_effect=TimeoutError("no acknowledgement")))
        with self.assertRaises(TimeoutError):
            PoseAdapter._run_child(adapter, client, object(), self.handle())
        self.assertTrue(adapter._blocked)
        child = Mock(accepted=True)
        sent.set_result(child)
        child.cancel_goal_async.assert_called_once()

    def test_cancellation_is_forwarded_to_active_moveit_goal(self):
        result, child, handle = Future(), Mock(), self.handle()
        handle.is_cancel_requested = True
        child.cancel_goal_async.side_effect = lambda: result.set_result("stopped")
        adapter = SimpleNamespace(_child=child, settings={"cancel_timeout": 0.1}, _feedback=Mock())
        with patch("rebotarm_mujoco_dm.pose_adapter.rclpy.ok", return_value=True):
            self.assertEqual(PoseAdapter._wait(adapter, result, handle, 1.0), "stopped")
        child.cancel_goal_async.assert_called_once()


if __name__ == "__main__":
    unittest.main()

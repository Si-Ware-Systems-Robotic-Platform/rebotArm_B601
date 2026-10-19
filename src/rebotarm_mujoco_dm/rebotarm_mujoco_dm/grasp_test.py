"""Explicit one-ball physical grasp trial; no welds, teleports or simulated success."""
import argparse
from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from .pose_requests import stretch_trajectory
from .grasp_checks import (hold_error, fingers_error, bowl_rest_error,
                           CLOSE_TARGET, EXPECTED_OFFSET, CENTER_TOLERANCE)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rebotarm_msgs.action import MoveToPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def lifted(start, current):
    return current.z-start.z >= .045


class PathToleranceViolation(RuntimeError):
    """A trajectory controller stopped because measured motion diverged."""


class GraspVerificationError(RuntimeError):
    """Fresh measurements show that the selected ball is not held."""


class GraspTest(Node):
    def __init__(self):
        super().__init__('mujoco_grasp_test')
        self.ball = None
        self.received = 0.
        self.ready = False
        self.ready_received = 0.
        self.active = None
        self.active_result = None
        self.pending_goal = None
        self.history = []
        self.ball_trace = []
        self.finger_positions = None
        self.fingers_received = 0.
        self.relative_received = 0.
        self.require_geometry = False
        self.require_held = False
        self.held = False
        self.held_received = 0.
        self.create_subscription(PoseStamped, '/mujoco_scene/ball_in_tcp', self.relative, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Bool, '/mujoco_scene/grasp_held', self.held_status, 10)
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.create_subscription(PoseStamped, '/mujoco_scene/ball_pose', self.observe, 10)
        self.create_subscription(Bool, '/mujoco_scene/ready', self.status, 10)
        self.arm = ActionClient(self, MoveToPose, '/rebotarm/move_to_pose')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.touch = self.create_client(SetBool, '/mujoco_scene/touch_ball')
        self.attach = self.create_client(Trigger, '/mujoco_scene/attach_ball')
        self.detach = self.create_client(Trigger, '/mujoco_scene/detach_ball')
        self._spin_executor = SingleThreadedExecutor(context=self.context)
        self._spin_executor.add_node(self)

    def spin_once(self, timeout_sec=.05):
        self._spin_executor.spin_once(timeout_sec=timeout_sec)

    def destroy_node(self):
        self._spin_executor.shutdown()
        return super().destroy_node()

    def relative(self, message):
        if self.ball is None:
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        p = message.pose.position
        self.relative_received = time.monotonic()
        self.history.append((stamp, [p.x, p.y, p.z], self.ball.pose.position.z))
        self.history = [v for v in self.history if stamp-v[0] <= 1.]

    def joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(n in values for n in ('finger_left', 'finger_right')):
            self.finger_positions = (values['finger_left'], values['finger_right'])
            self.fingers_received = time.monotonic()

    def held_status(self, message):
        self.held = message.data
        self.held_received = time.monotonic()

    def observe(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if self.ball_trace and stamp < self.ball_trace[-1][0]:
            self.ball_trace.clear()
        if not self.ball_trace or stamp > self.ball_trace[-1][0]:
            p = message.pose.position
            self.ball_trace.append((stamp, [p.x, p.y, p.z]))
            self.ball_trace = [sample for sample in self.ball_trace if stamp-sample[0] <= 2.]
        if self.ball is None or message.header.stamp != self.ball.header.stamp:
            self.received = time.monotonic()
        self.ball = message

    def status(self, message):
        self.ready = message.data
        self.ready_received = time.monotonic()

    def fresh(self):
        return GraspTest.state_error(self) is None

    def state_error(self):
        now = time.monotonic()
        if self.ball is None or now-self.received >= 1.5:
            return f'Ball pose missing or stale (age {now-self.received:.3f} s)'
        if now-self.ready_received >= 2.0:
            return f'Scene readiness publisher stale (age {now-self.ready_received:.3f} s)'
        if not self.ready:
            return 'Scene bridge reports not ready; inspect /mujoco_scene/grasp_fault'
        if self.require_geometry:
            if not self.history or now-self.relative_received >= 1.5:
                return f'Ball/TCP measurements missing or stale (age {now-self.relative_received:.3f} s)'
            error = math.dist(self.history[-1][1], EXPECTED_OFFSET)
            if not math.isfinite(error) or error >= CENTER_TOLERANCE:
                return (f'Ball moved out of the grip: centre error {error*1000:.1f} mm '
                        f'(limit {CENTER_TOLERANCE*1000:.1f} mm), offset {self.history[-1][1]} m')
        if self.require_held and (not self.held or now-self.held_received >= 1.5):
            return f'Measured hold lost or stale (age {now-self.held_received:.3f} s)'
        return None

    def wait(self, future, timeout=180., check_state=True):
        deadline = time.monotonic()+timeout
        while rclpy.ok() and not future.done():
            self.spin_once(timeout_sec=.02)
            if time.monotonic() > deadline:
                raise TimeoutError('ROS request timed out')
            if check_state and not self.fresh():
                raise RuntimeError(self.state_error())
        if not future.done():
            raise RuntimeError('ROS shutdown')
        return future.result()

    def settle(self, seconds):
        start = self.ball.header.stamp.sec+self.ball.header.stamp.nanosec*1e-9
        deadline = time.monotonic()+max(15., seconds*10)
        while rclpy.ok():
            self.spin_once(timeout_sec=.02)
            if not self.fresh():
                raise RuntimeError(self.state_error())
            now = self.ball.header.stamp.sec+self.ball.header.stamp.nanosec*1e-9
            if now < start or time.monotonic()>deadline:
                raise RuntimeError('Simulation reset or stalled during hold')
            if now-start>=seconds:
                return

    def verify_hold(self, minimum_height):
        stamp = self.ball.header.stamp.sec+self.ball.header.stamp.nanosec*1e-9
        error = hold_error(self.history, stamp, minimum_height)
        if time.monotonic()-self.relative_received>1.5 or error:
            if time.monotonic()-self.relative_received>1.5:
                raise RuntimeError('Ball/TCP measurements are stale')
            raise GraspVerificationError(error)
        if self.finger_positions is None or time.monotonic()-self.fingers_received>1.5:
            raise RuntimeError('Finger measurements are stale')
        error = fingers_error(*self.finger_positions)
        if error:
            raise GraspVerificationError(error)
        self.get_logger().info(f'Hold verified: ball z={self.ball.pose.position.z:.4f} m, '
                               f'finger_left={self.finger_positions[0]:.4f} m')

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Scene service unavailable')
        result = self.wait(client.call_async(request), timeout=12., check_state=False)
        if not result.success:
            raise RuntimeError(result.message)
        # Wait for fresh readiness after the bridge finishes the transition.
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=.05)
            if self.fresh():
                return
        raise RuntimeError('Scene did not become ready after its update')

    def action(self, client, goal, label='motion'):
        if not self.fresh() or not client.wait_for_server(timeout_sec=3.):
            raise RuntimeError('Scene/action server unavailable')
        sent = client.send_goal_async(goal)
        self.pending_goal = sent
        handle = self.wait(sent, timeout=10.)
        self.pending_goal = None
        if not handle.accepted:
            raise RuntimeError('Motion goal rejected')
        self.active = handle
        self.active_result = handle.get_result_async()
        result = self.wait(self.active_result)
        self.active = None
        self.active_result = None
        code = getattr(result.result, 'error_code', None)
        value = getattr(code, 'val', code)
        if isinstance(result.result, FollowJointTrajectory.Result) and value == -4:
            raise PathToleranceViolation(f'{label} path tolerance violation: {result.result}')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'{label} failed: {result.result}')
        if hasattr(result.result, 'success') and not result.result.success:
            raise RuntimeError(f'{label} failed: {result.result.message}')
        if hasattr(result.result, 'error_code'):
            if (hasattr(code, 'val') and code.val != 1) or (not hasattr(code, 'val') and code != 0):
                raise RuntimeError(f'{label} failed: {result.result}')

    def cancel_motion(self):
        if self.pending_goal is not None:
            handle = self.wait(self.pending_goal, timeout=5., check_state=False)
            self.pending_goal = None
            if handle and handle.accepted:
                self.active = handle
                self.active_result = handle.get_result_async()
        if self.active is not None:
            # A terminal result also proves the motion stopped, even if the
            # cancellation response is delayed or the goal just completed.
            if self.active_result is None:
                self.active_result = self.active.get_result_async()
            if not self.active_result.done():
                self.active.cancel_goal_async()
                self.wait(self.active_result, timeout=5., check_state=False)
            self.active = None
            self.active_result = None

    def move(self, x, y, z, linear=False, min_fraction=.999, yaw=0., speed=.1):
        self.get_logger().info(f'TCP target: {x:.3f}, {y:.3f}, {z:.3f} m'
                               + (f', yaw {math.degrees(yaw):.0f} deg' if yaw else ''))
        goal = MoveToPose.Goal(duration=5.)
        goal.target_pose.position.x = float(x)
        goal.target_pose.position.y = float(y)
        goal.target_pose.position.z = float(z)
        # Tool pointing down, turned by yaw about world Z (yaw 0 is the original
        # orientation). The fingers close along (-sin yaw, cos yaw) in world XY.
        s, half = math.sqrt(.5), yaw/2
        goal.target_pose.orientation.x = -s*math.sin(half)
        goal.target_pose.orientation.y = s*math.cos(half)
        goal.target_pose.orientation.z = s*math.sin(half)
        goal.target_pose.orientation.w = s*math.cos(half)
        if not linear:
            self.action(self.arm, goal, 'arm pose')
            return
        if not self.cartesian.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Cartesian path service unavailable')
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.start_state.is_diff = True
        request.group_name = 'arm'
        request.link_name = 'gripper_tcp'
        request.waypoints = [goal.target_pose]
        request.max_step = .002
        request.jump_threshold = 2.
        request.revolute_jump_threshold = .15
        request.prismatic_jump_threshold = .005
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = speed
        request.max_acceleration_scaling_factor = speed
        result = self.wait(self.cartesian.call_async(request), timeout=15.)
        if result.error_code.val != 1 or result.fraction < min_fraction:
            raise RuntimeError(f'Cartesian path incomplete ({result.fraction:.1%}); no motion sent')
        trajectory = stretch_trajectory(result.solution, 5.)
        self.action(self.execute, ExecuteTrajectory.Goal(trajectory=trajectory), 'arm Cartesian trajectory')

    def fingers(self, opening, closing=False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['finger_left']
        point = JointTrajectoryPoint(positions=[opening])
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        if closing:
            # Contact legitimately prevents reaching zero. Lift/state checks prove grasp.
            goal.path_tolerance = [JointTolerance(name='finger_left', position=.010)]
            goal.goal_tolerance = [JointTolerance(name='finger_left', position=.010)]
        try:
            self.action(self.gripper, goal, 'gripper trajectory')
        except PathToleranceViolation:
            if not closing:
                raise
            # A ball blocks closure before the requested 15 mm half-gap.
            # Accept that contact only when the measured fingers and ball/TCP
            # geometry match a grasp. settle() and verify_hold() still follow.
            now = time.monotonic()
            stamp = self.ball.header.stamp.sec+self.ball.header.stamp.nanosec*1e-9 if self.ball else 0.
            aligned = (self.history and stamp-self.history[-1][0] < .15 and
                       math.dist(self.history[-1][1], EXPECTED_OFFSET) < CENTER_TOLERANCE)
            if (self.finger_positions is None or now-self.fingers_received > 1.5 or
                    now-self.relative_received > 1.5 or not aligned or
                    fingers_error(*self.finger_positions)):
                raise
            self.get_logger().info('Gripper stopped against the measured ball; '
                                   'checking hold before lift')

    def run(self, miss=False, stage="lift"):
        deadline = time.monotonic()+15.
        while not self.fresh() and time.monotonic() < deadline:
            self.spin_once(timeout_sec=.1)
        if not self.fresh():
            raise RuntimeError('Launch scene:=bowl use_moveit:=true first')
        start = deepcopy(self.ball.pose.position)
        if abs(start.z-.02135) > .005:
            raise RuntimeError('Start with the ball settled on the table; restart the scene')
        x, y = start.x, start.y + (.12 if miss else 0.)
        tcp_z = start.z-.014  # ball centre 14 mm behind the downward-pointing fingertips (around equator)
        self.fingers(.04)
        self.move(x, y, tcp_z+.12)
        self.service(self.touch, SetBool.Request(data=True))
        self.move(x, y, tcp_z, linear=True)
        self.fingers(CLOSE_TARGET, closing=True)
        self.settle(1.5)
        if not miss:
            self.verify_hold(.015)
            self.require_geometry = True
        if stage == 'close':
            self.get_logger().info('CLOSE stage complete; fingers remain commanded to hold. No lift requested.')
            return
        self.move(x, y, tcp_z+.10, linear=True)
        self.settle(1.5)
        held = lifted(start, self.ball.pose.position)
        if miss:
            if held:
                raise RuntimeError('Unexpected ball lift in deliberate-miss trial')
            self.service(self.touch, SetBool.Request(data=False))
            self.get_logger().info('PASS: deliberate miss detected; no planning attachment created')
            return
        if not held:
            raise RuntimeError('Ball did not lift: physical grasp failed; no attachment or transfer performed')
        self.verify_hold(start.z+.045)
        self.service(self.attach, Trigger.Request())
        deadline = time.monotonic()+2.
        while not self.held and time.monotonic()<deadline:
            self.spin_once(timeout_sec=.05)
        if not self.held:
            raise RuntimeError('No measured grasp acknowledgement')
        self.require_held = True
        if stage == 'lift':
            self.settle(3.)
            self.verify_hold(start.z+.045)
            self.get_logger().info('LIFT stage complete; ball held for 3 simulation seconds. Restart before the next trial.')
            return
        self.move(x, y, .18, linear=True)
        self.move(.30, -.14, .18, linear=True)
        # Keep fingertips and the ball above the 48 mm bowl rim during release.
        self.move(.30, -.14, .09, linear=True)
        self.verify_hold(.07)
        # Deliberate release ends hold monitoring before fingers open.
        self.require_held = False
        self.require_geometry = False
        self.service(self.detach, Trigger.Request())
        self.fingers(.04)
        # The longer 0.20 m Cartesian retreat stopped at 87.7% after release.
        # Clear the 48 mm rim without requesting the unreachable final segment.
        self.move(.30, -.14, .16, linear=True)
        self.service(self.touch, SetBool.Request(data=False))
        self.settle(3.)
        position = self.ball.pose.position
        if math.hypot(position.x-.30, position.y+.14) > .045 or not .020 < position.z < .045:
            raise RuntimeError(f'Release did not settle in the bowl: {position}')
        stamp = self.ball.header.stamp.sec+self.ball.header.stamp.nanosec*1e-9
        error = bowl_rest_error(self.ball_trace, stamp)
        if error:
            raise RuntimeError(error)
        self.get_logger().info('PASS: measured lift, transfer, release and ball position inside bowl')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['close', 'lift', 'transfer'], default='lift',
                        help='stop after closing, lifting (default), or complete transfer')
    parser.add_argument('--miss', action='store_true', help='deliberately miss; verify no false attachment')
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = GraspTest()
    try:
        node.run(miss=options.miss, stage=options.stage)
    except (Exception, KeyboardInterrupt) as error:
        try:
            node.cancel_motion()
        except Exception as cancellation_error:
            node.get_logger().error(f'Cancellation unconfirmed: {cancellation_error}; stop the simulation launch.')
        node.get_logger().error(f'Trial stopped: {error}. Inspect the scene; restart before retrying. Gripper was not automatically opened.')
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

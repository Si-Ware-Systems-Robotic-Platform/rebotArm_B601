"""Build SDK-compatible MoveIt requests and slow trajectories without speeding up."""
from copy import deepcopy
import math

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive


def validate_goal(goal):
    pose = goal.target_pose
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
              goal.duration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Pose and duration must be finite")
    if goal.duration <= 0:
        raise ValueError("duration must be positive (minimum trajectory duration in seconds)")
    norm = math.hypot(*values[3:7])
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Orientation quaternion must have a finite, nonzero norm")


def build_plan_goal(goal, settings):
    validate_goal(goal)
    pose = deepcopy(goal.target_pose)
    q = pose.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    q.x, q.y, q.z, q.w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    request = MoveGroup.Goal()
    request.request.group_name = settings["planning_group"]
    request.request.pipeline_id = "ompl"
    request.request.num_planning_attempts = 10
    request.request.allowed_planning_time = settings["planning_time"]
    request.request.max_velocity_scaling_factor = settings["velocity_scaling"]
    request.request.max_acceleration_scaling_factor = settings["acceleration_scaling"]
    request.request.start_state.is_diff = True
    position = PositionConstraint()
    position.header.frame_id = settings["base_frame"]
    position.link_name = settings["tip_frame"]
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE,
                            dimensions=[settings["position_tolerance"]])
    position.constraint_region.primitives = [sphere]
    position.constraint_region.primitive_poses = [pose]
    position.weight = 1.0
    orientation = OrientationConstraint()
    orientation.header.frame_id = settings["base_frame"]
    orientation.link_name = settings["tip_frame"]
    orientation.orientation = pose.orientation
    orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
    orientation.absolute_x_axis_tolerance = settings["orientation_tolerance"]
    orientation.absolute_y_axis_tolerance = settings["orientation_tolerance"]
    orientation.absolute_z_axis_tolerance = settings["orientation_tolerance"]
    orientation.weight = 1.0
    request.request.goal_constraints = [Constraints(
        position_constraints=[position], orientation_constraints=[orientation])]
    request.planning_options.plan_only = True
    request.planning_options.planning_scene_diff.is_diff = True
    request.planning_options.planning_scene_diff.robot_state.is_diff = True
    return request


def pose_matches(measured, target, position_tolerance, orientation_tolerance):
    """Conservative check for an already-satisfied pose when MoveIt returns no path."""
    if measured is None:
        return False
    distance = math.hypot(measured.position.x - target.position.x,
                          measured.position.y - target.position.y,
                          measured.position.z - target.position.z)
    a, b = measured.orientation, target.orientation
    a_values, b_values = (a.x, a.y, a.z, a.w), (b.x, b.y, b.z, b.w)
    a_norm, b_norm = math.hypot(*a_values), math.hypot(*b_values)
    if a_norm < 1e-9 or b_norm < 1e-9:
        return False
    dot = abs(sum((x / a_norm) * (y / b_norm) for x, y in zip(a_values, b_values)))
    angle = 2 * math.acos(min(1.0, dot))
    return distance <= position_tolerance and angle <= orientation_tolerance


def stretch_trajectory(trajectory, minimum_seconds):
    """Scale time, velocity and acceleration together; preserve the geometric path."""
    if not math.isfinite(minimum_seconds) or minimum_seconds <= 0:
        raise ValueError("Minimum trajectory duration must be finite and positive")
    result = deepcopy(trajectory)
    points = result.joint_trajectory.points
    if not points or result.multi_dof_joint_trajectory.points:
        raise ValueError("Expected a nonempty arm joint trajectory")
    times = [p.time_from_start.sec * 1_000_000_000 + p.time_from_start.nanosec for p in points]
    if times[0] < 0 or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("MoveIt trajectory timestamps must increase")
    original = times[-1] / 1_000_000_000
    if original == 0:
        # A stationary one-point plan can be sent as a timed hold.
        stamp = round(minimum_seconds * 1_000_000_000)
        points[0].time_from_start.sec, points[0].time_from_start.nanosec = divmod(stamp, 1_000_000_000)
        points[0].velocities = [0.0] * len(points[0].positions)
        points[0].accelerations = [0.0] * len(points[0].positions)
    elif minimum_seconds > original:
        scale = minimum_seconds / original
        for point, stamp in zip(points, times):
            sec, nanosec = divmod(round(stamp * scale), 1_000_000_000)
            point.time_from_start.sec, point.time_from_start.nanosec = sec, nanosec
            point.velocities = [value / scale for value in point.velocities]
            point.accelerations = [value / scale ** 2 for value in point.accelerations]
    # Execution is relative to now, independent of a planning timestamp.
    result.joint_trajectory.header.stamp.sec = 0
    result.joint_trajectory.header.stamp.nanosec = 0
    return result

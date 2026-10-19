"""Read-only diagnosis of a bowl descent; never executes a trajectory."""
import argparse
import json
import math
import time

from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def diagnose(node, executor, ball_name):
    observations = {}

    def balls(message):
        for entry in message.free_joints:
            if entry.name == ball_name:
                observations['ball'] = (time.monotonic(), entry.pose)

    def joints(message):
        observations['joints'] = (time.monotonic(), message)

    subscriptions = [
        node.create_subscription(FreeJointStateArray, '/mujoco_scene/free_joint_states',
                                 balls, qos_profile_sensor_data),
        node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data),
    ]
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + 8.
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.05)
        if all(key in observations and time.monotonic()-observations[key][0] < .5
               for key in ('ball', 'joints')) and buffer.can_transform(
                   'base_link', 'gripper_tcp', Time()):
            break
    else:
        raise RuntimeError('Fresh ball/joint states or base_link -> gripper_tcp TF unavailable')

    ball = observations['ball'][1].pose.position
    tcp = buffer.lookup_transform('base_link', 'gripper_tcp', Time()).transform
    target_z = ball.z - .014
    approach_z = target_z + .12
    xy_error = math.hypot(tcp.translation.x-ball.x, tcp.translation.y-ball.y)
    z_error = abs(tcp.translation.z-approach_z)
    if xy_error > .015 or z_error > .025:
        raise RuntimeError('Arm is not at the failed pregrasp pose above '
                           f'{ball_name}: XY error {xy_error:.3f} m, '
                           f'Z error {z_error:.3f} m. Reproduce the 91.5% stop first.')

    path_client = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    validity_client = node.create_client(GetStateValidity, '/check_state_validity')
    for client, name in ((path_client, '/compute_cartesian_path'),
                         (validity_client, '/check_state_validity')):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError(f'{name} unavailable')

    def call(client, name, request):
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=15.)
        if not future.done() or future.exception() is not None:
            raise RuntimeError(f'{name} did not respond: {future.exception() if future.done() else "timeout"}')
        return future.result()

    request = GetCartesianPath.Request()
    request.header.frame_id = 'base_link'
    request.start_state.is_diff = True
    request.group_name = 'arm'
    request.link_name = 'gripper_tcp'
    waypoint = tcp_to_pose(tcp, ball.x, ball.y, target_z)
    request.waypoints = [waypoint]
    request.max_step = .002
    request.jump_threshold = 2.
    request.revolute_jump_threshold = .15
    request.prismatic_jump_threshold = .005
    request.max_velocity_scaling_factor = .1
    request.max_acceleration_scaling_factor = .1
    request.avoid_collisions = True
    guarded = call(path_client, '/compute_cartesian_path', request)
    request.avoid_collisions = False
    unguarded = call(path_client, '/compute_cartesian_path', request)

    trajectory = unguarded.solution.joint_trajectory
    measured = observations['joints'][1]
    measured_positions = dict(zip(measured.name, measured.position))
    contacts = []
    first_invalid = None
    # Inspect every returned waypoint. This is a query only: no trajectory is sent.
    for index, point in enumerate(trajectory.points):
        positions = measured_positions.copy()
        positions.update(zip(trajectory.joint_names, point.positions))
        check = GetStateValidity.Request()
        check.group_name = 'arm'
        check.robot_state.joint_state.name = list(positions)
        check.robot_state.joint_state.position = list(positions.values())
        result = call(validity_client, '/check_state_validity', check)
        if not result.valid:
            pairs = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2)))
                            for c in result.contacts})
            if first_invalid is None:
                first_invalid = {'step': index, 'fraction': round(index/max(1, len(trajectory.points)-1), 3),
                                 'contacts': pairs}
            for pair in pairs:
                if pair not in contacts:
                    contacts.append(pair)

    return {
        'ball': ball_name,
        'target_xyz_m': [round(ball.x, 4), round(ball.y, 4), round(target_z, 4)],
        'pregrasp_xyz_m': [round(tcp.translation.x, 4), round(tcp.translation.y, 4),
                           round(tcp.translation.z, 4)],
        'collision_checked_fraction': round(guarded.fraction, 4),
        'collision_checked_error_code': guarded.error_code.val,
        'collision_disabled_fraction': round(unguarded.fraction, 4),
        'collision_disabled_error_code': unguarded.error_code.val,
        'collision_disabled_waypoints': len(trajectory.points),
        'first_invalid_waypoint': first_invalid,
        'contact_pairs_on_collision_disabled_path': contacts,
        'motion_sent': False,
        'planning_scene_changed': False,
    }


def tcp_to_pose(transform, x, y, z):
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation = transform.rotation
    return pose


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ball', default='loose_ball_4')
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Node('mujoco_descent_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        print(json.dumps(diagnose(node, executor, options.ball), indent=2))
    except Exception as error:
        print(json.dumps({'error': str(error), 'motion_sent': False,
                          'planning_scene_changed': False}, indent=2))
        raise SystemExit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

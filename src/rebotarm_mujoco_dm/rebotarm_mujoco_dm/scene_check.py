"""Read-only two-bowl ROS health check; sends no motion or scene changes."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String


def main(args=None):
    rclpy.init(args=args)
    node = Node('mujoco_scene_check')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    samples, latest, last, gaps = {}, {}, {}, {}
    errors = []

    def callback(topic):
        def observe(message):
            now = time.monotonic()
            if topic in last:
                gaps[topic] = max(gaps.get(topic, 0.), now-last[topic])
            samples[topic] = samples.get(topic, 0)+1
            latest[topic] = message
            last[topic] = now
        return observe

    topics = [('/clock', Clock), ('/mujoco_scene/ready', Bool),
              ('/mujoco_scene/grasp_fault', String),
              ('/mujoco_scene/ball_in_tcp', PoseStamped),
              ('/mujoco_scene/free_joint_states', FreeJointStateArray)]
    subscriptions = [node.create_subscription(kind, topic, callback(topic), qos_profile_sensor_data)
                     for topic, kind in topics]
    report = {'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0')}

    def query(kind, name, request):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('service not discovered')
            started = time.monotonic()
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=3.)
            if not future.done():
                raise RuntimeError('no response within 3 seconds')
            report.setdefault('service_response_ms', {})[name] = round((time.monotonic()-started)*1000, 1)
            return future.result()
        except Exception as error:
            errors.append(f'{name}: {error}')
        finally:
            node.destroy_client(client)

    try:
        deadline = time.monotonic()+6.
        clock_start = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if clock_start is None and '/clock' in latest:
                stamp = latest['/clock'].clock
                clock_start = stamp.sec+stamp.nanosec*1e-9
        report['sample_count'] = samples
        report['max_message_gap_s'] = {k: round(v, 3) for k, v in gaps.items()}
        for topic, _ in topics:
            if topic not in last or time.monotonic()-last[topic] > .75:
                errors.append(f'{topic}: missing or stale messages')
            limit = .5 if topic == '/mujoco_scene/ball_in_tcp' else .75
            if gaps.get(topic, 0.) > limit:
                errors.append(f'{topic}: message gap exceeded {limit} seconds during observation')
        if clock_start is not None:
            stamp = latest['/clock'].clock
            advance = stamp.sec+stamp.nanosec*1e-9-clock_start
            report['simulation_seconds_observed'] = round(advance, 3)
            if advance <= 0:
                errors.append('Simulation clock is not advancing')
        report['ready'] = getattr(latest.get('/mujoco_scene/ready'), 'data', False)
        report['grasp_fault'] = getattr(latest.get('/mujoco_scene/grasp_fault'), 'data', '')
        if not report['ready'] or report['grasp_fault']:
            errors.append('Scene bridge is not ready or has a latched grasp fault')
        raw = latest.get('/mujoco_scene/free_joint_states')
        report['physics_balls'] = sorted(s.name for s in raw.free_joints) if raw else []
        expected_balls = {f'loose_ball_{i}' for i in range(1, 6)}
        if set(report['physics_balls']) != expected_balls:
            errors.append('Expected five free golf-ball states')

        publishers = ['/joint_states', '/clock', '/mujoco_scene/free_joint_states', '/mujoco_scene/ready']
        actions = ['/move_action', '/execute_trajectory', '/rebotarm/move_to_pose',
                   '/rebotarm_controller/follow_joint_trajectory',
                   '/gripper_controller/follow_joint_trajectory']
        report['publisher_counts'] = {topic: node.count_publishers(topic) for topic in publishers}
        report['action_status_publisher_counts'] = {
            action: node.count_publishers(action+'/_action/status') for action in actions}
        for topic, count in {**report['publisher_counts'], **report['action_status_publisher_counts']}.items():
            if count != 1:
                errors.append(f'{topic}: expected one publisher/server, discovered {count}')

        result = query(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        if result is not None:
            report['controllers'] = {c.name: c.state for c in result.controller}
            for name in ('joint_state_broadcaster', 'rebotarm_controller', 'gripper_controller'):
                if report['controllers'].get(name) != 'active':
                    errors.append(f'{name}: controller not active')
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        result = query(GetPlanningScene, '/get_planning_scene', request)
        if result is not None:
            world = {o.id for o in result.scene.world.collision_objects}
            held = {o.object.id for o in result.scene.robot_state.attached_collision_objects}
            report['world_objects'], report['attached_objects'] = sorted(world), sorted(held)
            if not {'dm_bowl_1', 'dm_bowl_2'}.issubset(world) or not expected_balls.issubset(world | held):
                errors.append('MoveIt is missing bowls or balls; restart the complete launch')
        request = GetStateValidity.Request()
        request.group_name = 'arm'
        request.robot_state.is_diff = True
        result = query(GetStateValidity, '/check_state_validity', request)
        if result is not None:
            report['arm_state_valid'] = result.valid
            report['contacts'] = [[c.contact_body_1, c.contact_body_2] for c in result.contacts]
            if not result.valid:
                errors.append('Measured arm state is in collision or invalid')
        report['errors'] = errors
        report['result'] = 'FAIL' if errors else 'PASS'
        print(json.dumps(report, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if errors:
        raise SystemExit(1)

"""Failure-path tests without ROS nodes or a running simulator."""
from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch
import threading
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (AllowedCollisionEntry, AllowedCollisionMatrix,
                             AttachedCollisionObject, CollisionObject, PlanningScene, RobotTrajectory)
from rebotarm_mujoco_dm.grasp_checks import hold_error, fingers_error, bowl_rest_error
from rebotarm_mujoco_dm.scene_bridge import BALL, SceneBridge
from rebotarm_mujoco_dm.grasp_test import GraspTest, PathToleranceViolation
from rebotarm_mujoco_dm.bowl_transfer import classify, destination_point, grasp_plan, transfer_direction
from rebotarm_mujoco_dm.pose_adapter import PoseAdapter


def history(z=.12):
    return [(i*.02, [-.014,0.,0.], z) for i in range(51)]


class GraspChecks(TestCase):
    def test_stable_lift_and_blocked_fingers(self):
        self.assertIsNone(hold_error(history(),1.,.066))
        self.assertIsNone(fingers_error(.0212,-.0212))

    def test_tabletop_is_not_a_lift(self):
        self.assertIsNotNone(hold_error(history(.02135),1.,.066))

    def test_miss_drift_staleness_and_mimic_failure(self):
        samples=history(); samples[-1]=(1.,[-.014,.009,0.],.12)
        self.assertIsNotNone(hold_error(samples,1.,.066))
        samples=history(); samples[-1]=(1.,[-.014,.004,0.],.12)
        self.assertIsNotNone(hold_error(samples,1.,.066))
        self.assertIsNotNone(hold_error(history(),2.,.066))
        self.assertIsNotNone(fingers_error(.018,-.018))
        self.assertIsNotNone(fingers_error(.021,-.018))

    def test_stable_reseating_within_motion_limit_is_accepted(self):
        samples=[(t,[-.014,.007,0.],z) for t,_,z in history()]
        self.assertIsNone(hold_error(samples,1.,.066))
        samples=[(t,[-.014,.013,0.],z) for t,_,z in history()]
        self.assertIn('13.0 mm',hold_error(samples,1.,.066))

    def test_bowl_rest_rejects_continuing_roll(self):
        stable=[(i*.02,[.3,-.14,.025]) for i in range(51)]
        rolling=[(i*.02,[.3,-.14+i*.02*.004,.025]) for i in range(51)]
        self.assertIsNone(bowl_rest_error(stable,1.))
        self.assertIn('still moving',bowl_rest_error(rolling,1.))

    def test_slip_latches_fault_and_clears_attachment(self):
        bridge=SimpleNamespace(_ready=Mock(),_held_pub=Mock(),_fault_pub=Mock(),
                               _attached=True,_attachment=object(),_needs_detach=False,
                               get_logger=lambda:Mock())
        SceneBridge._lose_grasp(bridge,'slipped')
        self.assertFalse(bridge._attached)
        self.assertIsNone(bridge._attachment)
        self.assertTrue(bridge._needs_detach)
        self.assertIn('slipped',bridge._fault)
        self.assertFalse(bridge._ready.publish.call_args.args[0].data)
        self.assertFalse(bridge._held_pub.publish.call_args.args[0].data)

    def test_unheld_scene_removes_attachment_only_after_a_slip(self):
        ball=CollisionObject(id=BALL,operation=CollisionObject.ADD)
        bridge=SimpleNamespace(_static=[],_attached=False,_needs_detach=False,
                               _ball_names=(BALL,),_ball_id=BALL,
                               _ball_object=lambda name=None:ball)
        scene=SceneBridge._scene(bridge)
        self.assertEqual([obj.operation for obj in scene.world.collision_objects],
                         [CollisionObject.ADD])
        self.assertEqual(scene.robot_state.attached_collision_objects,[])
        bridge._needs_detach=True
        scene=SceneBridge._scene(bridge)
        self.assertEqual(scene.robot_state.attached_collision_objects[0].object.operation,
                         CollisionObject.REMOVE)

    def test_multiball_scene_keeps_other_balls_as_obstacles(self):
        ids=("loose_ball_1", "loose_ball_2")
        bridge=SimpleNamespace(_static=[],_attached=False,_needs_detach=False,
                               _ball_names=ids,_ball_id=ids[0],
                               _ball_object=lambda name=None:CollisionObject(
                                   id=name or ids[0],operation=CollisionObject.ADD))
        scene=SceneBridge._scene(bridge)
        self.assertEqual({obj.id for obj in scene.world.collision_objects},set(ids))
        bridge._attached=True
        bridge._attachment=AttachedCollisionObject()
        bridge._attachment.object.id=ids[0]
        scene=SceneBridge._scene(bridge)
        self.assertEqual([obj.id for obj in scene.world.collision_objects],[ids[1]])
        self.assertEqual(scene.robot_state.attached_collision_objects[0].object.id,ids[0])

    def test_entry_permissions_only_allow_finger_contacts_with_balls_and_bowl(self):
        names=['finger_left_link','finger_right_link','loose_ball_1','loose_ball_2',
               'dm_bowl_1','dm_full_table']
        acm=AllowedCollisionMatrix(entry_names=names,
            entry_values=[AllowedCollisionEntry(enabled=[False]*len(names)) for _ in names])
        bridge=SimpleNamespace(_get=Mock(),_wait=lambda _:SimpleNamespace(
            scene=SimpleNamespace(allowed_collision_matrix=acm)),
            _ball_names=('loose_ball_1','loose_ball_2'),_ball_id='loose_ball_1',
            _static=[CollisionObject(id='dm_bowl_1')])
        scene, _, _ = SceneBridge._contact_permissions(bridge,True,True)()
        matrix=scene.allowed_collision_matrix
        for obj in ('loose_ball_1','loose_ball_2','dm_bowl_1'):
            for finger in ('finger_left_link','finger_right_link'):
                i,j=map(matrix.entry_names.index,(obj,finger))
                self.assertTrue(matrix.entry_values[i].enabled[j])
                self.assertTrue(matrix.entry_values[j].enabled[i])
        for finger in ('finger_left_link','finger_right_link'):
            i,j=map(matrix.entry_names.index,('dm_full_table',finger))
            self.assertFalse(matrix.entry_values[i].enabled[j])
        scene, _, _ = SceneBridge._contact_permissions(bridge,True,False)()
        matrix=scene.allowed_collision_matrix
        i,j=map(matrix.entry_names.index,('loose_ball_2','finger_left_link'))
        self.assertFalse(matrix.entry_values[i].enabled[j])
        i,j=map(matrix.entry_names.index,('loose_ball_1','finger_left_link'))
        self.assertTrue(matrix.entry_values[i].enabled[j])

    def test_ball_selection_resets_grasp_history_and_requires_new_scene(self):
        ids=("loose_ball_1", "loose_ball_2")
        selected=PoseStamped()
        bridge=SimpleNamespace(_lock=threading.RLock(),_ball_names=ids,
                               _ball_id=ids[0],_ball_states={ids[1]:selected},
                               _attached=False,_needs_detach=False,_transition=False,
                               _touch_active=False,_entry_active=False,
                               _pending=None,_fault="",_history=[(1.,[0,0,0],.1)],
                               _pose_queue=deque([selected]),_last_relative_pose=object(),
                               _last_relative_stamp=1.,_last_tcp=1.,_applied=True,
                               _ready=Mock(),_pose_pub=Mock())
        request=Parameter("active_ball_id",value=ids[1])
        result=SceneBridge._set_active_ball_parameters(bridge,[request])
        self.assertTrue(result.successful)
        self.assertEqual(bridge._ball_id,ids[1])
        self.assertEqual(bridge._history,[])
        self.assertFalse(bridge._applied)
        bridge._attached=True
        result=SceneBridge._set_active_ball_parameters(bridge,[Parameter("active_ball_id",value=ids[0])])
        self.assertFalse(result.successful)
        self.assertEqual(bridge._ball_id,ids[1])

    def test_bowl_classification_uses_measured_positions(self):
        from rebotarm_mujoco_dm.bowl_transfer import BALLS
        centers={"bowl_1":(.3,.14),"bowl_2":(.3,-.14)}
        poses={}
        for index,name in enumerate(BALLS):
            pose=PoseStamped()
            pose.pose.position.x=.3+.02*index/5
            pose.pose.position.y=.14
            pose.pose.position.z=.025
            poses[name]=pose
        counts=classify(poses,centers)
        self.assertEqual(counts["bowl_1"],list(BALLS))
        self.assertEqual(counts["bowl_2"],[])
        poses[BALLS[0]].pose.position.y=-.14
        counts=classify(poses,centers)
        self.assertEqual(counts["bowl_2"],[BALLS[0]])
        point=destination_point(centers["bowl_2"],[])
        self.assertAlmostEqual(point[0],.339)
        poses[BALLS[1]].pose.position.x=1.
        with self.assertRaisesRegex(RuntimeError,"outside"):
            classify(poses,centers)

    def test_attach_diff_does_not_remove_world_object_twice(self):
        import time
        ball=CollisionObject(id=BALL,operation=CollisionObject.ADD)
        bridge=SimpleNamespace(_lock=threading.RLock(),_history=history(),_stamp=1.,
            radius=.02135,_fingers=(.0212,-.0212),_fingers_received=time.monotonic(),
            _last_relative_pose=([-.014,0.,0.],[0.,0.,0.,1.]),
            _ball_object=lambda:ball,_scene=lambda **_:PlanningScene(is_diff=True),
            _transition_call=lambda response,operation:operation()[0])
        scene=SceneBridge._attach(bridge,None,None)
        self.assertFalse(any(obj.id==BALL for obj in scene.world.collision_objects))
        attached=scene.robot_state.attached_collision_objects[0]
        self.assertEqual(attached.object.operation,CollisionObject.ADD)
        self.assertEqual(attached.object.pose.orientation.w,1.)

    def test_attachment_acknowledgement_checks_moveit_scene(self):
        attached=AttachedCollisionObject();attached.object.id=BALL
        scene=PlanningScene()
        scene.robot_state.attached_collision_objects=[attached]
        node=SimpleNamespace(_get=Mock(),_ball_id=BALL,
                             _wait=lambda future:SimpleNamespace(scene=scene))
        SceneBridge._verify_ball_scene(node,True)
        with self.assertRaisesRegex(RuntimeError,'did not match'):
            SceneBridge._verify_ball_scene(node,False)
        scene.robot_state.attached_collision_objects=[]
        scene.world.collision_objects=[CollisionObject(id=BALL)]
        SceneBridge._verify_ball_scene(node,False)

    def test_pose_adapter_stops_waiting_when_scene_faults(self):
        adapter=SimpleNamespace(_require_scene=True,_scene_ok=lambda:False)
        with patch('rebotarm_mujoco_dm.pose_adapter.rclpy.ok',return_value=True):
            with self.assertRaisesRegex(RuntimeError,'stale'):
                PoseAdapter._wait(adapter,Future(),Mock(),1.)

    def test_partial_cartesian_path_never_executes(self):
        node=SimpleNamespace(get_logger=lambda:Mock(),cartesian=Mock(),wait=Mock(),action=Mock())
        node.cartesian.wait_for_service.return_value=True
        node.wait.return_value=SimpleNamespace(error_code=SimpleNamespace(val=1),fraction=.9)
        with self.assertRaisesRegex(RuntimeError,'incomplete'):
            GraspTest.move(node,.3,.1,.12,linear=True)
        node.action.assert_not_called()
        request=node.cartesian.call_async.call_args.args[0]
        self.assertTrue(request.avoid_collisions)
        self.assertEqual(request.link_name,'gripper_tcp')

    def test_gripper_contact_stop_requires_measured_ball_alignment(self):
        import time
        now=time.monotonic()
        ball=PoseStamped();ball.header.stamp.sec=1
        node=SimpleNamespace(action=Mock(side_effect=PathToleranceViolation('blocked')),
            gripper=Mock(),
            ball=ball,history=[(1.,[-.014,0.,0.],.025)],
            finger_positions=(.021,-.021),fingers_received=now,
            relative_received=now,get_logger=lambda:Mock())
        GraspTest.fingers(node,.015,closing=True)
        node.history=[(1.,[-.014,.02,0.],.025)]
        with self.assertRaises(PathToleranceViolation):
            GraspTest.fingers(node,.015,closing=True)

    def test_delayed_tf_uses_older_matching_sample_once(self):
        def sample(sec):
            p=PoseStamped();p.header.stamp.sec=sec;p.pose.position.z=.12;return p
        node=SimpleNamespace(_lock=threading.RLock(),_pose_queue=deque([sample(1),sample(2)]),
            _stamp=2.,_last_relative_stamp=None,_tcp_relative=Mock(),_relative_pub=Mock(),
            _history=[],_attached=False)
        node._publish_relative=lambda *a: SceneBridge._publish_relative(node,*a)
        # Keep candidates within the 150 ms window.
        node._pose_queue[0].header.stamp.sec=1
        node._pose_queue[0].header.stamp.nanosec=900000000
        node._tcp_relative.side_effect=[RuntimeError('TF pending'),([-.014,0.,0.],[0.,0.,0.,1.])]
        SceneBridge._update_relative(node)
        self.assertAlmostEqual(node._last_relative_stamp,1.9)
        self.assertEqual(len(node._history),1)
        self.assertEqual(node._relative_pub.publish.call_args.args[0].header.stamp.nanosec,900000000)

    def test_first_lift_rejects_ball_departure_before_attachment(self):
        import time
        now=time.monotonic()
        node=SimpleNamespace(ball=object(),received=now,ready=True,ready_received=now,
                             require_geometry=True,history=history(),relative_received=now,
                             require_held=False)
        self.assertTrue(GraspTest.fresh(node))
        node.history[-1]=(1.,[-.014,.02,0.],.12)
        self.assertFalse(GraspTest.fresh(node))

    def test_cancel_waits_for_active_trajectory_result(self):
        handle=Mock();result=Future();node=SimpleNamespace(
            pending_goal=None,active=handle,active_result=result,wait=Mock())
        GraspTest.cancel_motion(node)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(node.wait.call_args_list[-1].args[0],result)
        self.assertIsNone(node.active)

    def test_completed_motion_does_not_wait_for_redundant_cancel(self):
        result=Future();result.set_result(object())
        node=SimpleNamespace(pending_goal=None,active=Mock(),active_result=result,wait=Mock())
        handle=node.active
        GraspTest.cancel_motion(node)
        handle.cancel_goal_async.assert_not_called()
        node.wait.assert_not_called()
        self.assertIsNone(node.active)

    def test_grasp_plan_refuses_fingers_outside_bowl(self):
        with self.assertRaisesRegex(RuntimeError,'No bowl-interior grasp'):
            grasp_plan(['ball'],{'ball':(.40,.14)},(.30,.14))

    def test_grasp_plan_can_enter_when_neighbour_blocks_open_finger(self):
        import math
        positions={'ball':(.30,.14)}
        positions.update({f'neighbour_{i}':(.30+.053*math.cos(i*math.pi/12),
                                            .14+.053*math.sin(i*math.pi/12))
                          for i in range(24)})
        name, _, reach, gap = grasp_plan(['ball'],
            positions,(.30,.14))
        self.assertEqual(name,'ball')
        self.assertLess(reach,.070)
        self.assertLess(gap,0.)

    def test_partial_batch_does_not_reverse_after_three_placements(self):
        counts={'bowl_1':['a','b'],'bowl_2':['c','d','e']}
        with self.assertRaisesRegex(RuntimeError,'Both bowls'):
            transfer_direction(counts)
        self.assertEqual(transfer_direction(counts,'bowl_1'),('bowl_1','bowl_2'))
        counts={'bowl_1':[],'bowl_2':['a','b','c','d','e']}
        self.assertEqual(transfer_direction(counts),('bowl_2','bowl_1'))

    def test_stale_latest_tf_does_not_refresh_grasp_measurement(self):
        from geometry_msgs.msg import TransformStamped
        pose=PoseStamped();pose.header.stamp.sec=10
        old_tf=TransformStamped();old_tf.header.stamp.sec=9
        node=SimpleNamespace(_lock=threading.RLock(),_pose_queue=deque([pose]),
            _stamp=10.,_last_relative_stamp=None,_tcp_relative=Mock(side_effect=RuntimeError()),
            _buffer=Mock(),_publish_relative=Mock(),get_logger=lambda:Mock())
        node._buffer.lookup_transform.return_value=old_tf
        SceneBridge._update_relative(node)
        node._publish_relative.assert_not_called()

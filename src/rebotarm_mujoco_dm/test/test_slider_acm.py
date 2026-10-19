"""Contact permission must be restored before carrying a ball above the slider."""
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene
from rebotarm_mujoco_dm.bridge.scene_bridge import SceneBridge


def test_entry_false_restores_ball_and_finger_obstacle_checks():
    scene = PlanningScene()
    scene.allowed_collision_matrix = AllowedCollisionMatrix()
    fake = SimpleNamespace(
        _static=[SimpleNamespace(id='dm_slider')],
        _ball_names=['loose_ball_1', 'loose_ball_2'], _ball_id='loose_ball_1',
        _get=SimpleNamespace(call_async=lambda request: SimpleNamespace(scene=scene)),
        _wait=lambda response: response,
    )
    opened, _, _ = SceneBridge._contact_permissions(fake, True, True)()
    scene.allowed_collision_matrix = opened.allowed_collision_matrix
    restored, _, _ = SceneBridge._contact_permissions(fake, True, False)()
    acm = restored.allowed_collision_matrix

    def allowed(a, b):
        return acm.entry_values[acm.entry_names.index(a)].enabled[acm.entry_names.index(b)]

    assert not allowed('loose_ball_1', 'dm_slider')
    assert not allowed('dm_slider', 'loose_ball_1')
    assert not allowed('finger_left_link', 'dm_slider')
    assert not allowed('loose_ball_1', 'loose_ball_2')
    assert allowed('finger_left_link', 'loose_ball_1')

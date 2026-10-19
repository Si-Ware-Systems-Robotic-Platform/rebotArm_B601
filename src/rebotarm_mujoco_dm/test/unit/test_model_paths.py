"""Check installed assets when DM and bringup have unrelated install prefixes."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from rebotarm_mujoco_dm.model_paths import prepare_scene, extend_scene_keyframes


class ModelPathsTest(unittest.TestCase):
    def test_free_body_reset_pose_is_explicit(self):
        model = ET.ElementTree(ET.fromstring(
            '<mujoco><keyframe><key name="home" qpos="0 0 0 0 0 0 0 0" '
            'ctrl="0 0 0 0 0 0 0"/></keyframe></mujoco>'))
        scene = ET.ElementTree(ET.fromstring(
            '<mujoco><worldbody><body pos="0.3 0.1 0.03" quat="2 0 0 0">'
            '<freejoint/></body></worldbody></mujoco>'))
        extend_scene_keyframes(model, scene)
        key = model.find("keyframe/key")
        self.assertEqual(list(map(float, key.get("qpos").split())),
                         [0]*8 + [.3, .1, .03, 1, 0, 0, 0])
        self.assertEqual(key.get("ctrl"), "0 0 0 0 0 0 0")

    def test_isolated_prefixes_and_missing_scene_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dm = root / "dm_install/share/rebotarm_mujoco_dm"
            bringup = root / "bringup_install/share/rebotarm_bringup"
            (dm / "models/robot").mkdir(parents=True)
            (dm / "models/scenes").mkdir()
            (dm / "scene_assets").mkdir()
            (bringup / "description/DM/meshes/visual").mkdir(parents=True)
            arm_mesh = bringup / "description/DM/meshes/visual/arm.stl"
            table_mesh = dm / "scene_assets/table.obj"
            arm_mesh.write_text("arm fixture")
            table_mesh.write_text("table fixture")
            (dm / "models/robot/dm_arm.xml").write_text(
                '<mujoco><compiler inertiafromgeom="false"/><asset><mesh name="arm" '
                'file="../../rebotarm_bringup/description/DM/meshes/visual/arm.stl"/>'
                '</asset></mujoco>')
            original = ('<mujoco><include file="dm_arm.xml"/><asset><mesh name="table" '
                        'file="../../scene_assets/table.obj"/></asset></mujoco>')
            (dm / "models/scenes/dm_rail_scan.xml").write_text(original)
            output = root / "prepared"
            scene = prepare_scene(dm, bringup, output)
            self.assertEqual(ET.parse(scene).find("asset/mesh").get("file"), str(table_mesh))
            self.assertEqual(ET.parse(output / "dm_arm.xml").find("asset/mesh").get("file"), str(arm_mesh))
            self.assertEqual((dm / "models/scenes/dm_rail_scan.xml").read_text(), original)
            table_mesh.unlink()
            with self.assertRaises(FileNotFoundError):
                prepare_scene(dm, bringup, output)


if __name__ == "__main__":
    unittest.main()

from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_mujoco_dm"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz") + glob("config/*.xacro")),
        ("share/" + package_name + "/config/urdf", glob("config/urdf/*")),
        ("share/" + package_name + "/config/scenes", glob("config/scenes/*")),
        ("share/" + package_name + "/models", glob("models/*.xml")),
        ("share/" + package_name + "/models/robot", glob("models/robot/*.xml")),
        ("share/" + package_name + "/models/scenes", glob("models/scenes/*.xml")),
        ("share/" + package_name + "/scene_assets", glob("scene_assets/*")),
        ("share/" + package_name + "/docs", glob("docs/*.md") + glob("docs/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="B601-DM simulation using MuJoCo and ros2_control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_bridge = rebotarm_mujoco_dm.bridge.scene_bridge:main",
            "pose_adapter = rebotarm_mujoco_dm.bridge.pose_adapter:main",
            "wait_for_scene = rebotarm_mujoco_dm.bridge.wait_for_scene:main",
            "slider_mission = rebotarm_mujoco_dm.tasks.slider_mission:main",
            "rail_scan_mission = rebotarm_mujoco_dm.tasks.rail_scan_mission:main",
            "bowl_transfer_mission = rebotarm_mujoco_dm.tasks.bowl_transfer_mission:main",
        ]
    },
)

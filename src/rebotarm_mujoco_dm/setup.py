from glob import glob
from setuptools import find_packages, setup

package_name = "rebotarm_mujoco_dm"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/models", glob("models/*.xml")),
        ("share/" + package_name + "/docs", glob("docs/*.md") + glob("docs/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="B601-DM simulation using MuJoCo and ros2_control.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "pose_adapter = rebotarm_mujoco_dm.pose_adapter:main",
    ]},
)

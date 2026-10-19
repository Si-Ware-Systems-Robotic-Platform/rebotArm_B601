"""Convenience launch file for the rail and scan sorter scene with MoveIt."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dm = Path(get_package_share_directory("rebotarm_mujoco_dm"))
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Run without the MuJoCo viewer"),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"],
                              description="Start RViz when use_moveit is true"),
        DeclareLaunchArgument("ball_visual", default_value="mesh", choices=["native", "mesh"],
                              description="Sphere or Blender golf mesh appearance; identical sphere physics"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(dm / "launch/mujoco_dm.launch.py")),
            launch_arguments={
                "scene": "rail_scan",
                "use_moveit": "true",
                "headless": LaunchConfiguration("headless"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "ball_visual": LaunchConfiguration("ball_visual"),
            }.items(),
        ),
    ])

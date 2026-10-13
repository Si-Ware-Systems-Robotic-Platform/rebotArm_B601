"""DM MuJoCo controllers, with optional MoveIt planning and RViz."""
import importlib.util
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from rebotarm_mujoco_dm.model_paths import prepare_scene


def moveit_configuration(dm, moveit, mappings):
    """Reuse the DM planning configuration and the existing Humble/RViz fixes."""
    from moveit_configs_utils import MoveItConfigsBuilder

    spec = importlib.util.spec_from_file_location(
        "dm_moveit_launch_common", moveit / "launch/moveit_launch_common.py")
    common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(common)
    config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path=str(dm / "config/rebotarm_mujoco_dm.urdf.xacro"),
                           mappings=mappings)
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config = common.apply_rviz_urdf_compat(config)
    parameters = common.moveit_parameters(config)
    parameters["use_sim_time"] = True
    return config.robot_description["robot_description"], parameters


def launch_setup(context):
    dm = Path(get_package_share_directory("rebotarm_mujoco_dm"))
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    moveit = Path(get_package_share_directory("rebotarm_moveit_config"))
    # Each launch owns a private model copy, so installed files stay read-only
    # and concurrent runs do not overwrite one another's asset paths.
    temporary = tempfile.TemporaryDirectory(prefix="rebotarm_mujoco_dm_")
    try:
        scene = prepare_scene(dm, bringup, temporary.name)
        mappings = {
            "mujoco_scene_path": str(scene),
            "headless": LaunchConfiguration("headless").perform(context),
            "initial_positions_file": str(moveit / "config/initial_positions.yaml"),
        }
        moveit_params = None
        if LaunchConfiguration("use_moveit").perform(context) == "true":
            description, moveit_params = moveit_configuration(dm, moveit, mappings)
        else:
            description = xacro.process_file(
                str(dm / "config/rebotarm_mujoco_dm.urdf.xacro"), mappings=mappings).toxml()
        controllers = yaml.safe_load((moveit / "config/ros2_controllers.yaml").read_text())
        overrides = yaml.safe_load((dm / "config/controllers_sim.yaml").read_text())
        for name, params in overrides.items():
            controllers[name]["ros__parameters"].update(params["ros__parameters"])
        for params in controllers.values():
            params["ros__parameters"]["use_sim_time"] = True
        controllers.setdefault("joint_state_broadcaster", {"ros__parameters": {}})["ros__parameters"]["use_sim_time"] = True
        controller_file = Path(temporary.name) / "controllers.yaml"
        controller_file.write_text(yaml.safe_dump(controllers))
    except Exception:
        temporary.cleanup()
        raise

    robot = Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": ParameterValue(description, value_type=str), "use_sim_time": True}],
                 output="screen")
    world = Node(package="tf2_ros", executable="static_transform_publisher", name="world_to_base",
                 arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                            "--frame-id", "world", "--child-frame-id", "base_link"],
                 parameters=[{"use_sim_time": True}], output="screen")
    control = Node(package="mujoco_ros2_control", executable="ros2_control_node",
                   parameters=[str(controller_file), {"use_sim_time": True}],
                   remappings=[("~/robot_description", "/robot_description")], output="screen")
    spawners = [Node(package="controller_manager", executable="spawner",
                     arguments=[name, "--controller-manager", "/controller_manager",
                                "--controller-manager-timeout", "60", "--param-file", str(controller_file)],
                     parameters=[{"use_sim_time": True}], output="screen")
                for name in ("joint_state_broadcaster", "rebotarm_controller", "gripper_controller")]

    planning_nodes = []
    if moveit_params is not None:
        # Pass exactly the same description to MoveIt, RViz and the state publisher.
        moveit_params["robot_description"] = ParameterValue(description, value_type=str)
        planning_nodes.append(Node(
            package="moveit_ros_move_group", executable="move_group", name="move_group",
            parameters=[moveit_params], output="screen"))
        if LaunchConfiguration("use_pose_adapter").perform(context) == "true":
            planning_nodes.append(Node(
                package="rebotarm_mujoco_dm", executable="pose_adapter", name="mujoco_pose_adapter",
                parameters=[{"use_sim_time": True}], output="screen"))
        if LaunchConfiguration("use_rviz").perform(context) == "true":
            planning_nodes.append(Node(
                package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", str(dm / "config/moveit_mujoco.rviz")],
                parameters=[moveit_params], output="screen"))

    def after_spawner(next_actions):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                return [LogInfo(msg="ERROR: DM controller activation failed; stopping simulation."),
                        EmitEvent(event=Shutdown(reason="Controller spawner failed"))]
            return next_actions
        return callback

    def control_exited(event, context):
        return [] if context.is_shutdown else [EmitEvent(event=Shutdown(reason="MuJoCo control node exited"))]

    def cleanup(context):
        temporary.cleanup()
        return []

    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=after_spawner(
        [spawners[i+1]] if i+1 < len(spawners) else planning_nodes)))
        for i, node in enumerate(spawners)]
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            RegisterEventHandler(OnProcessExit(target_action=control, on_exit=control_exited)),
            *handlers, robot, world, control, spawners[0]]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Run without the MuJoCo viewer"),
        DeclareLaunchArgument("use_moveit", default_value="false", choices=["true", "false"],
                              description="Start MoveIt after the simulation controllers activate"),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"],
                              description="Start RViz when use_moveit is true"),
        DeclareLaunchArgument("use_pose_adapter", default_value="true", choices=["true", "false"],
                              description="Expose SDK MoveToPose through MoveIt when use_moveit is true"),
        OpaqueFunction(function=launch_setup),
    ])

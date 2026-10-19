"""Unified simulation launch for reBotArm B601-DM in MuJoCo with MoveIt 2."""
import importlib.util
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

from rebotarm_mujoco_dm.model_paths import prepare_scene


def moveit_configuration(dm, moveit, mappings, urdf_path):
    """Build MoveIt configuration dictionary and parameter tree."""
    from moveit_configs_utils import MoveItConfigsBuilder

    spec = importlib.util.spec_from_file_location(
        "dm_moveit_launch_common", moveit / "launch/moveit_launch_common.py"
    )
    common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(common)
    config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path=str(urdf_path), mappings=mappings)
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config = common.apply_rviz_urdf_compat(config)
    parameters = common.moveit_parameters(config)
    parameters["use_sim_time"] = True
    return config.robot_description["robot_description"], parameters


def launch_setup(context):
    scene_name = LaunchConfiguration("scene").perform(context)
    dm = Path(get_package_share_directory("rebotarm_mujoco_dm"))
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    moveit = Path(get_package_share_directory("rebotarm_moveit_config"))

    scene_map = {
        "slider": "dm_slider.xml",
        "dm_slider": "dm_slider.xml",
        "rail_scan": "dm_rail_scan.xml",
        "bowl": "dm_bowl.xml",
        "one_bowl": "dm_bowl.xml",
        "two_bowls": "dm_two_bowls.xml",
        "table": "dm_table.xml",
        "empty": "dm_empty.xml",
    }
    if scene_name not in scene_map:
        raise ValueError(
            f"Unknown scene '{scene_name}'. Available scenes: {list(scene_map.keys())}"
        )

    urdf_path = dm / "config/urdf/rebotarm_dm.urdf.xacro"
    controllers_override_path = dm / "config/controllers.yaml"
    rviz_path = dm / "config/rviz.rviz"

    temporary = tempfile.TemporaryDirectory(prefix="rebotarm_mujoco_dm_")
    try:
        scene = prepare_scene(
            dm,
            bringup,
            temporary.name,
            scene_file=scene_map[scene_name],
            ball_visual=LaunchConfiguration("ball_visual").perform(context),
        )

        initial_positions_file = moveit / "config/initial_positions.yaml"

        mappings = {
            "mujoco_scene_path": str(scene),
            "headless": LaunchConfiguration("headless").perform(context),
            "initial_positions_file": str(initial_positions_file),
        }

        moveit_params = None
        if LaunchConfiguration("use_moveit").perform(context) == "true":
            description, moveit_params = moveit_configuration(
                dm, moveit, mappings, urdf_path
            )
        else:
            description = xacro.process_file(
                str(urdf_path), mappings=mappings
            ).toxml()

        controllers = yaml.safe_load(
            (moveit / "config/ros2_controllers.yaml").read_text()
        )
        overrides = yaml.safe_load(controllers_override_path.read_text())
        for name, params in overrides.items():
            controllers[name]["ros__parameters"].update(params["ros__parameters"])
        for params in controllers.values():
            params["ros__parameters"]["use_sim_time"] = True
        controllers.setdefault("joint_state_broadcaster", {"ros__parameters": {}})[
            "ros__parameters"
        ]["use_sim_time"] = True

        controller_file = Path(temporary.name) / "controllers.yaml"
        controller_file.write_text(yaml.safe_dump(controllers))
    except Exception:
        temporary.cleanup()
        raise

    robot = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(description, value_type=str),
                "publish_frequency": 100.0,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world", "--child-frame-id", "base_link",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    plugin_params = {}
    if scene_name != "empty":
        ball_names = [
            body.get("name")
            for body in ET.parse(scene).getroot().findall("worldbody/body")
            if body.get("name", "").startswith("loose_ball_")
            and body.find("freejoint") is not None
        ]
        plugin_params = {
            "mujoco_plugins": {
                "free_joint_state_publisher": {
                    "type": "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin",
                    "frame_id": "base_link",
                    "body_names": ball_names,
                    "topic": "/mujoco_scene/free_joint_states",
                    "publish_rate": 50.0,
                }
            }
        }

    control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[str(controller_file), {"use_sim_time": True}, plugin_params],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
                "--param-file", str(controller_file),
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
        for name in (
            "joint_state_broadcaster",
            "rebotarm_controller",
            "gripper_controller",
        )
    ]

    planning_nodes = []
    core_planning_nodes = []
    if moveit_params is not None:
        moveit_params["robot_description"] = ParameterValue(
            description, value_type=str
        )
        planning_nodes.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                parameters=[moveit_params],
                output="screen",
            )
        )
        core_planning_nodes.append(planning_nodes[-1])

        clients = []
        if scene_name != "empty":
            planning_nodes.append(
                Node(
                    package="rebotarm_mujoco_dm",
                    executable="scene_bridge",
                    parameters=[{"use_sim_time": False, "scene_path": str(scene)}],
                    output="screen",
                )
            )
            core_planning_nodes.append(planning_nodes[-1])

        if LaunchConfiguration("use_pose_adapter").perform(context) == "true":
            clients.append(
                Node(
                    package="rebotarm_mujoco_dm",
                    executable="pose_adapter",
                    name="mujoco_pose_adapter",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "require_scene_ready": scene_name != "empty",
                            "position_tolerance": 0.002
                            if scene_name != "empty"
                            else 0.005,
                        }
                    ],
                    output="screen",
                )
            )
            core_planning_nodes.append(clients[-1])

        if LaunchConfiguration("use_rviz").perform(context) == "true":
            clients.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=["-d", str(rviz_path)],
                    parameters=[moveit_params],
                    output="screen",
                )
            )

        if scene_name != "empty":
            waiter = Node(
                package="rebotarm_mujoco_dm",
                executable="wait_for_scene",
                output="screen",
            )

            def after_scene(event, context):
                if context.is_shutdown:
                    return []
                if event.returncode != 0:
                    return [
                        EmitEvent(
                            event=Shutdown(
                                reason="Scene not ready; inspect scene bridge logs"
                            )
                        )
                    ]
                return clients

            planning_nodes.extend(
                [
                    RegisterEventHandler(
                        OnProcessExit(target_action=waiter, on_exit=after_scene)
                    ),
                    waiter,
                ]
            )
        else:
            planning_nodes.extend(clients)

    def after_spawner(next_actions):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                return [
                    LogInfo(
                        msg="ERROR: DM controller activation failed; stopping simulation."
                    ),
                    EmitEvent(event=Shutdown(reason="Controller spawner failed")),
                ]
            return next_actions

        return callback

    def planning_exited(event, context):
        if context.is_shutdown:
            return []
        return [
            LogInfo(
                msg="ERROR: A core planning node exited; stopping simulation stack."
            ),
            EmitEvent(
                event=Shutdown(
                    reason="Core planning node exited; restart complete launch"
                )
            ),
        ]

    def control_exited(event, context):
        return (
            []
            if context.is_shutdown
            else [EmitEvent(event=Shutdown(reason="MuJoCo control node exited"))]
        )

    def cleanup(context):
        temporary.cleanup()
        return []

    handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=after_spawner(
                    [spawners[i + 1]] if i + 1 < len(spawners) else planning_nodes
                ),
            )
        )
        for i, node in enumerate(spawners)
    ]

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(
            OnProcessExit(target_action=control, on_exit=control_exited)
        ),
        robot,
        world,
        control,
        spawners[0],
        *handlers,
        *[
            RegisterEventHandler(
                OnProcessExit(target_action=node, on_exit=planning_exited)
            )
            for node in core_planning_nodes
        ],
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene",
                default_value="rail_scan",
                description="Scene to simulate: rail_scan (default), bowl, two_bowls, table, empty",
            ),
            DeclareLaunchArgument(
                "use_moveit",
                default_value="true",
                description="Launch MoveIt move_group planning stack",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with MoveIt visualizer",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Render physics without MuJoCo GUI window",
            ),
            DeclareLaunchArgument(
                "use_pose_adapter",
                default_value="true",
                description="Expose /rebotarm/move_to_pose action adapter",
            ),
            DeclareLaunchArgument(
                "ball_visual",
                default_value="mesh",
                description="Appearance of golf balls: mesh or native",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

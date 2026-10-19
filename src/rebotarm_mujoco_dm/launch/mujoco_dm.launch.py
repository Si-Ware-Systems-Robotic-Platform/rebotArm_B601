"""DM MuJoCo controllers, with optional MoveIt planning and RViz."""
import importlib.util
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

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
    scene_name = LaunchConfiguration("scene").perform(context)
    dm = Path(get_package_share_directory("rebotarm_mujoco_dm"))
    bringup = Path(get_package_share_directory("rebotarm_bringup"))
    moveit = Path(get_package_share_directory("rebotarm_moveit_config"))
    # Each launch owns a private model copy, so installed files stay read-only
    # and concurrent runs do not overwrite one another's asset paths.
    temporary = tempfile.TemporaryDirectory(prefix="rebotarm_mujoco_dm_")
    try:
        scene = prepare_scene(dm, bringup, temporary.name,
                              {"bowl": "dm_scene.xml", "two_bowls": "dm_two_bowls.xml",
                               "rail_scan": "dm_rail_scan.xml",
                               "table": "dm_table.xml", "empty": "dm_empty.xml"}[scene_name],
                              ball_visual=LaunchConfiguration("ball_visual").perform(context))
        mappings = {
            "mujoco_scene_path": str(scene),
            "headless": LaunchConfiguration("headless").perform(context),
            "initial_positions_file": str(dm / "config/initial_positions_two_bowls.yaml"
                                          if scene_name in ("two_bowls", "rail_scan")
                                          else moveit / "config/initial_positions.yaml"),
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

    # Humble throttles moving-joint TF to publish_frequency (default 20 Hz). The
    # ball/TCP grasp measurement needs TF at each 50 Hz ball sample; at 20 Hz its
    # gaps reached 0.3 s and intermittently failed the hold checks.
    robot = Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": ParameterValue(description, value_type=str),
                              "publish_frequency": 100.0, "use_sim_time": True}],
                 output="screen")
    world = Node(package="tf2_ros", executable="static_transform_publisher", name="world_to_base",
                 arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0",
                            "--frame-id", "world", "--child-frame-id", "base_link"],
                 parameters=[{"use_sim_time": True}], output="screen")
    plugin_params = {}
    if scene_name != "empty":
        ball_names = [body.get('name') for body in
                      ET.parse(scene).getroot().findall('worldbody/body')
                      if body.get('name', '').startswith('loose_ball_')
                      and body.find('freejoint') is not None]
        plugin_params = {"mujoco_plugins": {"free_joint_state_publisher": {
            "type": "mujoco_ros2_control_plugins/FreeJointStatePublisherPlugin",
            "frame_id": "base_link", "body_names": ball_names,
            "topic": "/mujoco_scene/free_joint_states", "publish_rate": 50.0}}}
    control = Node(package="mujoco_ros2_control", executable="ros2_control_node",
                   parameters=[str(controller_file), {"use_sim_time": True}, plugin_params],
                   remappings=[("~/robot_description", "/robot_description")], output="screen")
    spawners = [Node(package="controller_manager", executable="spawner",
                     arguments=[name, "--controller-manager", "/controller_manager",
                                "--controller-manager-timeout", "60", "--param-file", str(controller_file)],
                     parameters=[{"use_sim_time": True}], output="screen")
                for name in ("joint_state_broadcaster", "rebotarm_controller", "gripper_controller")]

    planning_nodes = []
    core_planning_nodes = []
    if moveit_params is not None:
        # Pass exactly the same description to MoveIt, RViz and the state publisher.
        moveit_params["robot_description"] = ParameterValue(description, value_type=str)
        planning_nodes.append(Node(
            package="moveit_ros_move_group", executable="move_group", name="move_group",
            parameters=[moveit_params], output="screen"))
        core_planning_nodes.append(planning_nodes[-1])
        clients = []
        if scene_name != "empty":
            planning_nodes.append(Node(
                package="rebotarm_mujoco_dm", executable="scene_bridge",
                # Wall time on purpose: it only uses message stamps. Sim time wakes its
                # executor for every 1 kHz /clock tick, which starves the ball/TCP
                # measurement stream (it stalled for seconds in the two-bowl scene).
                parameters=[{"use_sim_time": False, "scene_path": str(scene)}], output="screen"))
            core_planning_nodes.append(planning_nodes[-1])
        if LaunchConfiguration("use_pose_adapter").perform(context) == "true":
            clients.append(Node(
                package="rebotarm_mujoco_dm", executable="pose_adapter", name="mujoco_pose_adapter",
                parameters=[{"use_sim_time": True, "require_scene_ready": scene_name != "empty",
                             "position_tolerance": 0.002 if scene_name != "empty" else 0.005}], output="screen"))
            core_planning_nodes.append(clients[-1])
        if LaunchConfiguration("use_rviz").perform(context) == "true":
            clients.append(Node(
                package="rviz2", executable="rviz2", name="rviz2",
                arguments=["-d", str(dm / "config/moveit_mujoco.rviz")],
                parameters=[moveit_params], output="screen"))

        if scene_name != "empty":
            waiter = Node(package="rebotarm_mujoco_dm", executable="wait_for_scene", output="screen")
            def after_scene(event, context):
                if context.is_shutdown:
                    return []
                if event.returncode != 0:
                    return [EmitEvent(event=Shutdown(reason="Scene not ready; inspect scene bridge logs"))]
                return clients
            planning_nodes.extend([RegisterEventHandler(OnProcessExit(
                target_action=waiter, on_exit=after_scene)), waiter])
        else:
            planning_nodes.extend(clients)

    def after_spawner(next_actions):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                return [LogInfo(msg="ERROR: DM controller activation failed; stopping simulation."),
                        EmitEvent(event=Shutdown(reason="Controller spawner failed"))]
            return next_actions
        return callback

    # A new MoveIt process would lose the acknowledged world/attachment state.
    # Stop this launch if a core planning node exits instead of leaving a partial stack.
    def planning_exited(event, context):
        if context.is_shutdown:
            return []
        return [LogInfo(msg="ERROR: A core planning node exited; stopping the simulation stack."),
                EmitEvent(event=Shutdown(reason="Core planning node exited; restart the complete launch"))]

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
            *[RegisterEventHandler(OnProcessExit(target_action=node, on_exit=planning_exited))
              for node in core_planning_nodes],
            *handlers, robot, world, control, spawners[0]]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="bowl", choices=["bowl", "two_bowls", "rail_scan", "table", "empty"],
                              description="One bowl, two bowls, rail/scan sorter, table, or empty scene"),
        DeclareLaunchArgument("ball_visual", default_value="mesh", choices=["native", "mesh"],
                              description="Sphere or Blender golf mesh appearance; identical sphere physics"),
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

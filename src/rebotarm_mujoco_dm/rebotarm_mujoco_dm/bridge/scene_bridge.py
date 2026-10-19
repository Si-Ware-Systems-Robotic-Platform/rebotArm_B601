"""Mirror MuJoCo scene objects into MoveIt; never change the physics state."""
from copy import deepcopy
from collections import deque
import math
import threading
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                            CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from .scene_geometry import read_scene, relative
from ..core.grasp_checks import hold_error, fingers_error, CENTER_TOLERANCE

BALL = "loose_ball_1"
FINGERS = ["finger_left_link", "finger_right_link"]


def pose_message(value):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = value[0]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = value[1]
    return pose


def pose_value(pose):
    return ([pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


def static_objects(path):
    definitions, radius = read_scene(path)
    objects = []
    for definition in definitions:
        obj = CollisionObject(id=definition["id"], operation=CollisionObject.ADD)
        obj.header.frame_id = "base_link"
        for item in definition["geometries"]:
            pose = pose_message(item["pose"])
            if item["kind"] == "mesh":
                mesh = Mesh()
                v, f = item["vertices"], item["faces"]
                mesh.vertices = [Point(x=v[i], y=v[i+1], z=v[i+2]) for i in range(0, len(v), 3)]
                mesh.triangles = [MeshTriangle(vertex_indices=f[i:i+3]) for i in range(0, len(f), 3)]
                obj.meshes.append(mesh); obj.mesh_poses.append(pose)
            else:
                kind = {"box": SolidPrimitive.BOX, "sphere": SolidPrimitive.SPHERE,
                        "cylinder": SolidPrimitive.CYLINDER}[item["kind"]]
                obj.primitives.append(SolidPrimitive(type=kind, dimensions=item["size"]))
                obj.primitive_poses.append(pose)
        objects.append(obj)
    return objects, radius


class SceneBridge(Node):
    def __init__(self):
        super().__init__("mujoco_scene_bridge")
        path = self.declare_parameter("scene_path", "").value
        self._static, self.radius = static_objects(path)
        self._ball_names = tuple(body.get("name") for body in
                                 ET.parse(path).getroot().findall("worldbody/body")
                                 if body.get("name", "").startswith("loose_ball_")
                                 and body.find("freejoint") is not None)
        if not self._ball_names:
            raise ValueError("Scene has no free golf balls")
        self._ball_id = self.declare_parameter("active_ball_id", self._ball_names[0]).value
        if self._ball_id not in self._ball_names:
            raise ValueError("active_ball_id is not a ball in this scene")
        self._lock = threading.RLock()
        self.add_on_set_parameters_callback(self._set_active_ball_parameters)
        self._group = ReentrantCallbackGroup()
        self._state = None
        self._ball_states = {}
        self._stamp = None
        self._last_progress = 0.
        self._pending = None
        self._applied = False
        self._transition = False
        self._blocked = False
        self._fault = ""
        self._last_tcp = 0.
        self._last_relative_pose = None
        self._last_relative_stamp = None
        self._pose_queue = deque(maxlen=50)
        self._fingers = None
        self._fingers_received = 0.
        self._attached = False
        self._touch_active = False
        self._entry_active = False
        self._attachment = None
        self._needs_detach = False
        self._pending_detach = False
        self._pending_static = False
        self._static_sent = False
        self._history = []
        self._apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene", callback_group=self._group)
        self._get = self.create_client(GetPlanningScene, "/get_planning_scene", callback_group=self._group)
        self._buffer = Buffer(node=self)
        self._listener = TransformListener(self._buffer, self)
        self._tf = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._ready = self.create_publisher(Bool, "/mujoco_scene/ready", qos)
        self._held_pub = self.create_publisher(Bool, "/mujoco_scene/grasp_held", qos)
        self._fault_pub = self.create_publisher(String, "/mujoco_scene/grasp_fault", qos)
        self._relative_pub = self.create_publisher(PoseStamped, "/mujoco_scene/ball_in_tcp", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state, qos_profile_sensor_data)
        self._pose_pub = self.create_publisher(PoseStamped, "/mujoco_scene/ball_pose", 10)
        self.create_subscription(FreeJointStateArray, "/mujoco_scene/free_joint_states",
                                 self._observe, qos_profile_sensor_data)
        self.create_service(SetBool, "/mujoco_scene/touch_ball", self._touch, callback_group=self._group)
        self.create_service(SetBool, "/mujoco_scene/enter_bowl", self._enter_bowl, callback_group=self._group)
        self.create_service(Trigger, "/mujoco_scene/attach_ball", self._attach, callback_group=self._group)
        self.create_service(Trigger, "/mujoco_scene/detach_ball", self._detach, callback_group=self._group)
        # Wall timer detects a frozen clock or a dead state publisher.
        from rclpy.clock import Clock, ClockType
        self.create_timer(.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.02, self._update_relative, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _fresh(self):
        return (self._state is not None and len(self._ball_states) == len(self._ball_names)
                and time.monotonic()-self._last_progress < 1.5)

    def _set_active_ball_parameters(self, parameters):
        with self._lock:
            for parameter in parameters:
                if parameter.name != "active_ball_id":
                    continue
                if (parameter.type_ != Parameter.Type.STRING or
                        parameter.value not in self._ball_names):
                    return SetParametersResult(successful=False, reason="Unknown ball ID")
                if parameter.value == self._ball_id:
                    continue
                if (self._attached or self._needs_detach or self._transition or
                        self._pending is not None or self._fault or self._touch_active or
                        self._entry_active):
                    return SetParametersResult(successful=False, reason="Finish the current grasp/scene update first")
                self._ball_id = parameter.value
                self._state = deepcopy(self._ball_states.get(self._ball_id))
                self._history.clear()
                self._pose_queue.clear()
                self._last_relative_pose = None
                self._last_relative_stamp = None
                self._last_tcp = 0.
                self._applied = False
                self._ready.publish(Bool(data=False))
                if self._state is not None:
                    self._pose_pub.publish(self._state)
            return SetParametersResult(successful=True)

    def _joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(n in positions for n in ("finger_left", "finger_right")):
            self._fingers = (positions["finger_left"], positions["finger_right"])
            self._fingers_received = time.monotonic()

    def _lose_grasp(self, reason):
        self._fault = reason + "; restart the scene before another trial"
        self._needs_detach = self._needs_detach or self._attached
        self._attached = False
        self._attachment = None
        self._ready.publish(Bool(data=False))
        self._held_pub.publish(Bool(data=False))
        self._fault_pub.publish(String(data=self._fault))
        self.get_logger().error(self._fault)

    def _tcp_relative(self, pose, stamp=None):
        # Match the ball timestamp: latest-TF lag can look like slip during motion.
        t = self._buffer.lookup_transform("base_link", "gripper_tcp",
                                          Time.from_msg(stamp if stamp is not None else self._state.header.stamp))
        v = t.transform
        parent = ([v.translation.x, v.translation.y, v.translation.z],
                  [v.rotation.x, v.rotation.y, v.rotation.z, v.rotation.w])
        return relative(parent, pose_value(pose))

    def _observe(self, message):
        entries = {s.name: s for s in message.free_joints if s.name in self._ball_names}
        if len(entries) != len(self._ball_names) or message.header.frame_id not in ("", "world", "base_link"):
            return
        for state in entries.values():
            if state.pose.header.frame_id not in ("", "world", "base_link") or not all(
                    math.isfinite(x) for x in sum(pose_value(state.pose.pose), [])):
                return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        with self._lock:
            if stamp == self._stamp:
                return
            if stamp != self._stamp:
                if self._stamp is not None and stamp < self._stamp:
                    self._history.clear()
                    self._pose_queue.clear()
                    self._last_relative_stamp = None
                    if self._attached:
                        self._lose_grasp("Simulation reset during grasp")
                    self._attached = False  # simulator reset: restore a measured world object
                    self._attachment = None
                self._last_progress = time.monotonic()
            self._stamp = stamp
            for name, state in entries.items():
                pose = deepcopy(state.pose)
                pose.header = deepcopy(message.header)
                pose.header.frame_id = "base_link"
                self._ball_states[name] = pose
                # Only the active ball needs a TF frame. Broadcasting all five at 50 Hz
                # put 250 extra /tf messages/s in front of every TF listener (bridge,
                # MoveIt, RViz), which starved the grasp measurements under GUI load.
                if name != self._ball_id:
                    continue
                tf = TransformStamped(header=pose.header, child_frame_id=name)
                tf.transform.translation.x = pose.pose.position.x
                tf.transform.translation.y = pose.pose.position.y
                tf.transform.translation.z = pose.pose.position.z
                tf.transform.rotation = pose.pose.orientation
                self._tf.sendTransform(tf)
            self._state = self._ball_states[self._ball_id]
            self._pose_pub.publish(self._state)
            self._pose_queue.append(deepcopy(self._state))
            self._update_relative()

    def _publish_relative(self, sample, stamp, relative_pose):
        offset = relative_pose[0]
        self._last_tcp = time.monotonic()
        self._last_relative_stamp = stamp
        self._last_relative_pose = deepcopy(relative_pose)
        relative_message = PoseStamped()
        relative_message.header = deepcopy(sample.header)
        relative_message.header.frame_id = "gripper_tcp"
        relative_message.pose = pose_message(relative_pose)
        self._relative_pub.publish(relative_message)
        self._history.append((stamp, list(offset), sample.pose.position.z))
        self._history = [v for v in self._history if stamp-v[0] <= 1.0]
        if self._attached and self._attachment:
            expected = pose_value(self._attachment.object.primitive_poses[0])[0]
            if math.dist(expected, offset) > CENTER_TOLERANCE:
                self._lose_grasp(f"Ball slipped more than {CENTER_TOLERANCE*1000:.0f} mm from its held position")

    def _update_relative(self):
        with self._lock:
            for sample in reversed(self._pose_queue):
                stamp = sample.header.stamp.sec+sample.header.stamp.nanosec*1e-9
                if self._last_relative_stamp is not None and stamp <= self._last_relative_stamp:
                    break
                if self._stamp-stamp > .30:
                    break
                try:
                    relative_pose = self._tcp_relative(sample.pose, sample.header.stamp)
                except Exception:
                    continue  # TF for the newest sample may still be in flight.
                self._publish_relative(sample, stamp, relative_pose)
                return
            # A latest-TF fallback must still be close to the ball timestamp.
            # A stopped TF publisher must never appear as a fresh grasp sample.
            if not self._pose_queue:
                return
            sample = self._pose_queue[-1]
            stamp = sample.header.stamp.sec+sample.header.stamp.nanosec*1e-9
            if self._last_relative_stamp is not None and stamp <= self._last_relative_stamp:
                return
            try:
                t = self._buffer.lookup_transform("base_link", "gripper_tcp", Time())
            except Exception as error:
                self.get_logger().warning(f"TCP TF unavailable: {error}", throttle_duration_sec=2.)
                return
            tf_stamp = t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if abs(stamp-tf_stamp) > .20:
                self.get_logger().warning(
                    f"TCP TF is {stamp-tf_stamp:.3f} simulation seconds away from ball pose; "
                    "waiting for a matching sample", throttle_duration_sec=2.)
                return
            v = t.transform
            relative_pose = relative(
                ([v.translation.x, v.translation.y, v.translation.z],
                 [v.rotation.x, v.rotation.y, v.rotation.z, v.rotation.w]), pose_value(sample.pose))
            self.get_logger().warning("TCP TF at the ball sample time unavailable; using latest TF",
                                      throttle_duration_sec=2.)
            self._publish_relative(sample, stamp, relative_pose)

    def _ball_object(self, name=None):
        name = name or self._ball_id
        obj = CollisionObject(id=name, operation=CollisionObject.ADD)
        obj.header.frame_id = "base_link"
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[self.radius])]
        obj.primitive_poses = [deepcopy(self._ball_states[name].pose)]
        return obj

    def _scene(self, include_static=True):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        # The static bowl/table meshes are large; resending them every tick starves the
        # grasp-measurement path in the two-bowl scene, so periodic updates skip them.
        scene.world.collision_objects = deepcopy(self._static) if include_static else []
        scene.world.collision_objects.extend(self._ball_object(name) for name in self._ball_names
                                             if name != self._ball_id)
        if self._attached:
            # MoveIt removes the world object when the attachment is added.
            scene.robot_state.attached_collision_objects = [deepcopy(self._attachment)]
        else:
            scene.world.collision_objects.append(self._ball_object())
            if self._needs_detach:
                removal = AttachedCollisionObject(link_name="gripper_tcp")
                removal.object.id = self._ball_id; removal.object.operation = CollisionObject.REMOVE
                scene.robot_state.attached_collision_objects = [removal]
        for obj in scene.world.collision_objects:
            color = ObjectColor(id=obj.id)
            color.color.r, color.color.g, color.color.b, color.color.a = (
                (1., 1., 1., 1.) if obj.id in self._ball_names else (.55, .32, .14, 1.))
            scene.object_colors.append(color)
        return scene

    def _tick(self):
        with self._lock:
            if self._attached and (time.monotonic()-self._last_tcp > 1.5 or
                                   not self._fresh()):
                self._lose_grasp("Lost measured ball/TCP state while holding")
            self._held_pub.publish(Bool(data=self._attached and not self._fault))
            self._fault_pub.publish(String(data=self._fault))
            self._ready.publish(Bool(data=not self._fault and self._fresh() and self._applied and not self._transition and not self._blocked))
            if self._pending is not None:
                if self._pending.done():
                    try:
                        self._applied = bool(self._pending.result().success)
                    except Exception:
                        self._applied = False
                    if self._pending_static:
                        self._static_sent = self._applied
                    if self._applied and self._pending_detach:
                        self._needs_detach = False
                    self._pending = None
                    self._pending_detach = False
                elif time.monotonic()-self._pending_since > 2.:
                    self._applied = False
                    self.get_logger().warning(
                        "Waiting for /apply_planning_scene response; scene is not ready", 
                        throttle_duration_sec=2.)
                return
            if self._blocked or self._transition or not self._fresh() or not self._apply.service_is_ready():
                return
            self._pending_detach = self._needs_detach and not self._attached
            self._pending_static = not self._static_sent
            self._pending = self._apply.call_async(
                ApplyPlanningScene.Request(scene=self._scene(self._pending_static)))
            self._pending_since = time.monotonic()

    def _wait(self, future):
        deadline = time.monotonic()+5
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        if not future.done():
            raise TimeoutError("Planning-scene service timed out; restart before motion")
        return future.result()

    def _verify_ball_scene(self, expected_attached):
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                     PlanningSceneComponents.WORLD_OBJECT_NAMES)
        current = self._wait(self._get.call_async(req)).scene
        attached = any(obj.object.id == self._ball_id for obj in
                       current.robot_state.attached_collision_objects)
        world = any(obj.id == self._ball_id for obj in current.world.collision_objects)
        if attached != expected_attached or world == expected_attached:
            raise RuntimeError("MoveIt ball scene did not match the requested "
                               f"attachment state (attached={attached}, world={world})")

    def _transition_call(self, response, operation):
        with self._lock:
            if self._fault or self._blocked or self._transition or not self._fresh():
                response.success = False
                response.message = "Scene busy or state stale; retry after fresh state"
                return response
            self._transition = True
            pending = self._pending
        try:
            if pending is not None:
                self._wait(pending)
                with self._lock:
                    self._pending = None
            scene, commit, expected_attached = operation()
            result = self._wait(self._apply.call_async(ApplyPlanningScene.Request(scene=scene)))
            if not result.success:
                raise RuntimeError("MoveIt refused the scene update")
            if expected_attached is not None:
                self._verify_ball_scene(expected_attached)
            # MoveIt's scene update can briefly delay TF; wait (unlocked) for a
            # current ball/TCP sample instead of judging a momentarily stale one.
            deadline = time.monotonic()+1.
            while time.monotonic() < deadline:
                with self._lock:
                    if (self._stamp is not None and self._last_relative_stamp is not None
                            and self._stamp-self._last_relative_stamp <= .15):
                        break
                time.sleep(.01)
            with self._lock:
                commit()
                self._applied = True
            response.success = True
            response.message = "MoveIt scene updated; MuJoCo physics was not modified"
        except Exception as error:
            with self._lock:
                self._applied = False
                if isinstance(error, TimeoutError):
                    self._blocked = True
            response.success = False; response.message = str(error)
        finally:
            with self._lock:
                self._transition = False
        return response

    def _contact_permissions(self, touch, entry):
        def operation():
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            acm = self._wait(self._get.call_async(req)).scene.allowed_collision_matrix
            contacts = [obj.id for obj in self._static
                        if obj.id.startswith(('dm_bowl_', 'dm_lane_', 'dm_slide_frame', 'dm_sensor_station', 'dm_slider', 'slider'))]
            for name in [*self._ball_names, *contacts, *FINGERS]:
                if name not in acm.entry_names:
                    acm.entry_names.append(name)
                    for value in acm.entry_values:
                        value.enabled.append(False)
                    acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
            for obj in [*self._ball_names, *contacts]:
                allowed = entry or (touch and obj == self._ball_id)
                obj_index = acm.entry_names.index(obj)
                for finger in FINGERS:
                    finger_index = acm.entry_names.index(finger)
                    acm.entry_values[obj_index].enabled[finger_index] = allowed
                    acm.entry_values[finger_index].enabled[obj_index] = allowed
                if self._ball_id in acm.entry_names and obj != self._ball_id:
                    ball_index = acm.entry_names.index(self._ball_id)
                    # Restore object collision checks when leaving contact mode.
                    # Previously True entries survived entry=False indefinitely.
                    acm.entry_values[obj_index].enabled[ball_index] = bool(entry)
                    acm.entry_values[ball_index].enabled[obj_index] = bool(entry)
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            scene.allowed_collision_matrix = acm
            return scene, lambda: None, None
        return operation

    def _touch(self, request, response):
        def operation():
            scene, _, attached = self._contact_permissions(request.data, self._entry_active)()
            return scene, lambda: setattr(self, '_touch_active', request.data), attached
        return self._transition_call(response, operation)

    def _enter_bowl(self, request, response):
        def operation():
            scene, _, attached = self._contact_permissions(self._touch_active, request.data)()
            return scene, lambda: setattr(self, '_entry_active', request.data), attached
        return self._transition_call(response, operation)

    def _attach(self, request, response):
        def operation():
            with self._lock:
                error = hold_error(self._history, self._stamp, self.radius+.045)
                if error:
                    raise RuntimeError(error)
                if self._fingers is None or time.monotonic()-self._fingers_received > 1.5:
                    raise RuntimeError("Finger measurements are stale")
                error = fingers_error(*self._fingers)
                if error:
                    raise RuntimeError(error)
                attached = AttachedCollisionObject(link_name="gripper_tcp", touch_links=FINGERS)
                attached.object = self._ball_object()
                attached.object.header.frame_id = "gripper_tcp"
                attached.object.pose.orientation.w = 1.
                attached.object.primitive_poses = [pose_message(self._last_relative_pose)]
                # Static meshes are already in MoveIt; resending them stalls the
                # ball/TCP measurements that commit() checks below.
                scene = self._scene(include_static=False)
                scene.world.collision_objects = [o for o in scene.world.collision_objects if o.id != self._ball_id]
                scene.robot_state.attached_collision_objects = [attached]
            def commit():
                if not self._fresh() or self._stamp-self._last_relative_stamp > .15:
                    self._lose_grasp("State lost during attachment acknowledgement")
                    raise RuntimeError(self._fault)
                offset = self._last_relative_pose[0]
                if math.dist(offset, pose_value(attached.object.primitive_poses[0])[0]) > .003:
                    self._lose_grasp("Ball moved during attachment acknowledgement")
                    raise RuntimeError(self._fault)
                self._attachment = attached; self._attached = True
                self._held_pub.publish(Bool(data=True))
            return scene, commit, True
        return self._transition_call(response, operation)

    def _detach(self, request, response):
        def operation():
            with self._lock:
                scene = PlanningScene(is_diff=True); scene.robot_state.is_diff = True
                removal = AttachedCollisionObject(link_name="gripper_tcp")
                removal.object.id = self._ball_id; removal.object.operation = CollisionObject.REMOVE
                scene.robot_state.attached_collision_objects = [removal]
                scene.world.collision_objects = [self._ball_object()]
            def commit():
                self._attachment = None; self._attached = False
                self._held_pub.publish(Bool(data=False))
            return scene, commit, False
        return self._transition_call(response, operation)


def main(args=None):
    rclpy.init(args=args)
    node = SceneBridge(); executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

"""Move measured golf balls between the two MuJoCo bowls, one at a time."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger

from ..core.grasp_checks import bowl_rest_error, CLOSE_TARGET
from ..core.arm_interface import ArmInterface, GraspVerificationError, PathToleranceViolation, lifted
GraspTest = ArmInterface

BALLS = tuple(f"loose_ball_{i}" for i in range(1, 6))
BOWL_RADIUS = .065
# Pointing straight down, the arm cannot reach 0.18 m above the outer bowl
# positions (~0.37 m from the base): the climb stopped at 89.2%. At 0.13 m the
# fingertips still clear the 48 mm bowl rims and neighbouring balls by >75 mm.
CARRY_Z = .13


def bowl_centers(path):
    world = ET.parse(path).getroot().find("worldbody")
    return {name: tuple(float(x) for x in world.find(f'body[@name="{name}"]').get("pos").split()[:2])
            for name in ("bowl_1", "bowl_2")}


def classify(poses, centers):
    result = {name: [] for name in centers}
    for ball in BALLS:
        if ball not in poses:
            raise RuntimeError(f"No measured pose for {ball}")
        p = poses[ball].pose.position
        matches = [name for name, (x, y) in centers.items()
                   if math.hypot(p.x-x, p.y-y) < BOWL_RADIUS and .015 < p.z < .055]
        if len(matches) != 1:
            raise RuntimeError(f"{ball} is outside both settled bowls; inspect scene")
        result[matches[0]].append(ball)
    return result


def transfer_direction(counts, source=None):
    if source is None:
        occupied = [name for name, balls in counts.items() if balls]
        if len(occupied) != 1:
            raise RuntimeError('Both bowls contain balls. Use --source bowl_1 or '
                               '--source bowl_2 to continue the intended direction.')
        source = occupied[0]
    if source not in counts or not counts[source]:
        raise RuntimeError(f'Source {source} is empty or unknown')
    return source, next(name for name in counts if name != source)


# finger_left equals the open half-gap. 0.048 is the value that lifted ball 1 in the
# live trials; 0.034 made the descent abort with a path tolerance violation.
OPENING = .048
FINGER_OFFSET = OPENING+.005   # finger centre distance from the ball centre
FINGER_RADIUS = .004
BALL_RADIUS = .02135
# Open fingers must stay inside the bowl: ball 1 (fingers 0.065 m from the bowl
# centre) descended fine, a ball whose fingers reached 0.081 m stopped on the rim.
MAX_REACH = .070


def finger_posts(ball, yaw):
    axis = (-math.sin(yaw), math.cos(yaw))
    return [(ball[0]+side*FINGER_OFFSET*axis[0], ball[1]+side*FINGER_OFFSET*axis[1])
            for side in (1, -1)]


def grasp_metrics(ball, yaw, others, center):
    """(largest finger distance from the bowl centre, smallest gap to a neighbour ball)."""
    posts = finger_posts(ball, yaw)
    reach = max(math.dist(post, center) for post in posts)
    gap = min((math.dist(post, other)-BALL_RADIUS-FINGER_RADIUS
               for post in posts for other in others), default=1.)
    return reach, gap


def grasp_plan(candidates, positions, center, excluded=()):
    """Choose (ball, yaw): keep finger centres near the bowl interior.

    The fingers close along world Y at yaw 0 and the gripper is symmetric under
    180 deg, so -90..90 deg covers every closing direction. Yaw 0 is kept whenever it
    is acceptable (that is the verified motion).
    """
    best = None
    for name in candidates:
        others = [xy for other, xy in positions.items() if other != name]
        for degrees in sorted(range(-90, 91, 15), key=abs):
            if (name, degrees) in excluded:
                continue
            yaw = math.radians(degrees)
            reach, gap = grasp_metrics(positions[name], yaw, others, center)
            ok = reach <= MAX_REACH
            # Prefer a clear, near-zero yaw. If none fits, choose the least
            # intrusive contact: nearby balls may move under MuJoCo physics.
            score = (ok, gap >= 0., -abs(degrees) if gap >= 0. else gap, -reach)
            if best is None or score > best[0]:
                best = (score, name, yaw, reach, gap)
            if ok and gap >= 0. and degrees == 0:
                break
    if best is None:
        raise RuntimeError('No untried antipodal grasp angle remains')
    _, name, yaw, reach, gap = best
    if reach > MAX_REACH:
        raise RuntimeError(f'No bowl-interior grasp: best candidate {name} has finger reach '
                           f'{reach*1000:.1f} mm (limit {MAX_REACH*1000:.1f})')
    return name, yaw, reach, gap


def destination_point(center, existing):
    # Five spaced resting locations within the measured bowl footprint.
    slots = [(center[0]+.039*math.cos(2*math.pi*i/5),
              center[1]+.039*math.sin(2*math.pi*i/5)) for i in range(5)]
    if not existing:
        return slots[0]
    ranked = sorted(slots, key=lambda xy: min(
        math.hypot(xy[0]-p.pose.position.x, xy[1]-p.pose.position.y) for p in existing),
        reverse=True)
    point = ranked[0]
    clearance = min(math.hypot(point[0]-p.pose.position.x, point[1]-p.pose.position.y)
                    for p in existing)
    if clearance < .043:
        raise RuntimeError(f"No safe nominal landing slot ({clearance:.3f} m clearance)")
    return point


class BowlTransfer(GraspTest):
    def __init__(self):
        super().__init__()
        self.poses = {}
        self.raw_received = 0.
        self.raw_stamp = None
        self.create_subscription(FreeJointStateArray, "/mujoco_scene/free_joint_states",
                                 self.observe_all, qos_profile_sensor_data)
        self.select_client = self.create_client(SetParametersAtomically,
                                                "/mujoco_scene_bridge/set_parameters_atomically")
        self.enter = self.create_client(SetBool, "/mujoco_scene/enter_bowl")
        scene = Path(get_package_share_directory("rebotarm_mujoco_dm")) / "models/scenes/dm_two_bowls.xml"
        self.centers = bowl_centers(scene)

    def observe_all(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if stamp == self.raw_stamp:
            return
        values = {entry.name: deepcopy(entry.pose) for entry in message.free_joints
                  if entry.name in BALLS}
        if len(values) == len(BALLS):
            self.poses = values
            self.raw_received = time.monotonic()
            self.raw_stamp = stamp

    def state(self):
        if time.monotonic()-self.raw_received > .75:
            raise RuntimeError("Five fresh free-joint poses are unavailable")
        return classify(self.poses, self.centers)

    def select(self, name):
        if not self.select_client.wait_for_service(timeout_sec=3.):
            raise RuntimeError("Scene bridge parameter service unavailable")
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name="active_ball_id", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=name))]
        deadline = time.monotonic()+8.
        while True:
            result = self.wait(self.select_client.call_async(request), timeout=12., check_state=False)
            if result.result.successful:
                break
            if time.monotonic() > deadline or "Finish the current" not in result.result.reason:
                raise RuntimeError(f"Could not select {name}: {result.result.reason}")
            self.spin_once(timeout_sec=.05)
        self.ball = None
        self.ready = False
        self.history.clear()
        self.ball_trace.clear()
        self.require_geometry = False
        self.require_held = False
        self.held = False
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=.05)
            if self.fresh() and self.ball is not None:
                p, q = self.ball.pose.position, self.poses[name].pose.position
                if math.dist((p.x,p.y,p.z), (q.x,q.y,q.z)) < .003:
                    return
        raise RuntimeError(f"Selected ball {name} did not become ready")

    def open_for_retry(self):
        """Open in smaller increments after a failed physical close."""
        for target in (.035, OPENING):
            try:
                self.fingers(target)
            except PathToleranceViolation as error:
                if (self.finger_positions is None or
                        time.monotonic()-self.fingers_received > .5 or
                        self.finger_positions[0] < target-.010 or
                        abs(sum(self.finger_positions)) > .002):
                    raise RuntimeError(f'Cannot reopen fingers for another grasp: {error}') from error
                self.get_logger().warning(
                    f'Opening stopped near {self.finger_positions[0]:.3f} m; '
                    'continuing measured recovery')
        if self.finger_positions is None or self.finger_positions[0] < .038:
            raise RuntimeError('Fingers did not open enough to retry')

    def transfer_one(self, name, destination, yaw=0.):
        self.select(name)
        start = deepcopy(self.ball.pose.position)
        target_x, target_y = destination
        tried = set()
        for attempt in range(3):
            if attempt:
                counts = self.state()
                if name not in counts[self.source]:
                    raise RuntimeError(f'{name} left {self.source}; cannot retry its grasp')
                positions = {ball: (self.poses[ball].pose.position.x,
                                    self.poses[ball].pose.position.y)
                             for ball in counts[self.source]}
                _, yaw, reach, gap = grasp_plan([name], positions,
                    self.centers[self.source], excluded=tried)
                self.get_logger().info(f'Retry {attempt+1}/3: {name}, yaw '
                    f'{math.degrees(yaw):.0f} deg, finger reach {reach*1000:.0f} mm, '
                    f'neighbour gap {gap*1000:.0f} mm')
            tried.add((name, round(math.degrees(yaw))))
            p = self.ball.pose.position
            x, y, tcp_z = p.x, p.y, p.z-.014
            self.get_logger().info(f"Picking {name} from ({x:.3f}, {y:.3f})")
            if attempt == 0:
                self.fingers(OPENING)
            self.move(x, y, tcp_z+.12, yaw=yaw)
            if attempt == 0:
                self.service(self.touch, SetBool.Request(data=True))
                # Only finger/ball/bowl contact is permitted. MuJoCo resolves
                # physical contact; other robot and table checks remain active.
                self.service(self.enter, SetBool.Request(data=True))
            last_xy = (x, y)
            try:
                # One longer move above the contacts, then shorter measured
                # steps around the balls. This reduces visible stop-start motion.
                for offset in (.04, .02, 0.):
                    if not self.fresh():
                        raise RuntimeError(self.state_error())
                    p = self.ball.pose.position
                    current_xy = (p.x, p.y)
                    if math.dist(current_xy, last_xy) > .025:
                        raise RuntimeError(f'{name} moved more than 25 mm during entry; replan from above')
                    center = self.centers[self.source]
                    if math.dist(current_xy, center) > BOWL_RADIUS:
                        raise RuntimeError(f'{name} left {self.source} during entry')
                    self.move(p.x, p.y, p.z-.014+offset, linear=True, yaw=yaw, speed=.03)
                    last_xy = current_xy
                self.fingers(CLOSE_TARGET, closing=True)
                self.settle(1.5)
                self.verify_hold(.015)
                p = self.ball.pose.position
                x, y, tcp_z = p.x, p.y, p.z-.014
                break
            except (GraspVerificationError, PathToleranceViolation) as error:
                if attempt == 2:
                    raise RuntimeError(f'{name} not grasped after 3 measured approaches: {error}') from error
                self.get_logger().warning(f'{name} approach {attempt+1}/3 did not grasp: {error}')
                # Open and return above the measured ball before changing yaw.
                # If this retreat cannot execute, stop instead of forcing motion.
                self.open_for_retry()
                p = self.ball.pose.position
                self.move(p.x, p.y, p.z-.014+.12, linear=True, yaw=yaw, speed=.03)
        self.require_geometry = True
        self.move(x, y, tcp_z+.10, linear=True, yaw=yaw)
        self.settle(1.5)
        if not lifted(start, self.ball.pose.position):
            raise RuntimeError(f"{name} did not physically lift")
        self.verify_hold(start.z+.045)
        self.service(self.enter, SetBool.Request(data=False))
        self.service(self.attach, Trigger.Request())
        deadline = time.monotonic()+2.
        while not self.held and time.monotonic()<deadline:
            self.spin_once(timeout_sec=.05)
        if not self.held:
            raise RuntimeError("No measured grasp acknowledgement")
        self.require_held = True
        self.move(x, y, CARRY_Z, linear=True, yaw=yaw)
        self.move(target_x, target_y, CARRY_Z, linear=True, yaw=yaw)
        self.move(target_x, target_y, .09, linear=True, yaw=yaw)
        self.verify_hold(.07)
        self.require_held = False
        self.require_geometry = False
        self.service(self.detach, Trigger.Request())
        self.fingers(.04)
        self.move(target_x, target_y, CARRY_Z, linear=True, yaw=yaw)
        self.service(self.touch, SetBool.Request(data=False))
        self.settle(3.)
        p = self.ball.pose.position
        center = self.centers[self.destination]
        if math.hypot(p.x-center[0],p.y-center[1]) > BOWL_RADIUS or not .015 < p.z < .055:
            raise RuntimeError(f"{name} did not settle in {self.destination}: {p}")
        stamp = self.ball.header.stamp.sec+self.ball.header.stamp.nanosec*1e-9
        error = bowl_rest_error(self.ball_trace, stamp)
        if error:
            raise RuntimeError(error)
        self.get_logger().info(f"Placed {name} in {self.destination}")

    def run_batch(self, limit=5, source=None):
        deadline = time.monotonic()+15.
        while time.monotonic()<deadline:
            self.spin_once(timeout_sec=.1)
            if self.fresh() and time.monotonic()-self.raw_received < .75:
                break
        if not self.fresh():
            raise RuntimeError("Launch scene:=two_bowls use_moveit:=true first")
        counts = self.state()
        self.source, self.destination = transfer_direction(counts, source)
        total = min(len(counts[self.source]), limit)
        self.get_logger().info(f"Moving {total} of {len(counts[self.source])} balls: "
                               f"{self.source} -> {self.destination}")
        for index in range(total):
            counts = self.state()
            candidates = counts[self.source]
            if not candidates:
                raise RuntimeError("Source bowl became empty unexpectedly")
            # Prefer a clear finger entry, but permit a measured neighbouring
            # ball to move when no clear approach remains. Recompute after release.
            positions = {ball: (self.poses[ball].pose.position.x, self.poses[ball].pose.position.y)
                         for ball in BALLS if ball in counts[self.source]}
            name, yaw, reach, gap = grasp_plan(candidates, positions, self.centers[self.source])
            self.get_logger().info(f"Grasp plan: {name}, yaw {math.degrees(yaw):.0f} deg, finger reach "
                                   f"{reach*1000:.0f} mm, neighbour gap {gap*1000:.0f} mm")
            target = destination_point(self.centers[self.destination],
                                       [self.poses[ball] for ball in counts[self.destination]])
            self.transfer_one(name, target, yaw)
            counts = self.state()
            if name not in counts[self.destination]:
                raise RuntimeError(f"{name} is not measured in the destination bowl")
            self.get_logger().info(f"Progress {index+1}/{total}: "
                                   f"{len(counts[self.source])} source, {len(counts[self.destination])} destination")
        self.get_logger().info(f"PASS: moved {total} measured balls from {self.source} to {self.destination}")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=5, choices=range(1,6),
                        help="number of balls to attempt (default: all remaining in source)")
    parser.add_argument("--source", choices=['bowl_1', 'bowl_2'],
                        help="source bowl for resuming a partial batch; otherwise one bowl must be empty")
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = BowlTransfer()
    try:
        node.run_batch(options.limit, options.source)
    except (Exception, KeyboardInterrupt) as error:
        try:
            node.cancel_motion()
        except Exception as cancellation_error:
            node.get_logger().error(f"Cancellation unconfirmed: {cancellation_error}")
        for client in (node.enter, node.touch):
            try:
                if client.service_is_ready():
                    node.service(client, SetBool.Request(data=False))
            except Exception as cleanup_error:
                node.get_logger().warning(f"Contact permissions may remain active: {cleanup_error}; restart the scene")
        node.get_logger().error(f"Transfer stopped: {error}. Inspect the scene before retrying.")
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Exit successfully only after the bridge confirms the measured scene is installed."""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main(args=None):
    rclpy.init(args=args)
    node = Node("wait_for_mujoco_scene")
    ready = [False]
    node.create_subscription(Bool, "/mujoco_scene/ready", lambda m: ready.__setitem__(0, m.data), 10)
    deadline = time.monotonic()+60.
    try:
        while rclpy.ok() and not ready[0] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not ready[0]:
            node.get_logger().error("No measured scene acknowledgement within 60 seconds")
            raise SystemExit(1)
        node.get_logger().info("Measured MuJoCo scene applied to MoveIt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

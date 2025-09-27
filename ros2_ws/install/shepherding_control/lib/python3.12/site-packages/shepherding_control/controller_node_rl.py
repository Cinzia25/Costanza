import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Assumed interface (adjust here if your function lives elsewhere or has a different signature):
# learning_controller(H: np.ndarray, T: np.ndarray, logger) -> (np.ndarray, np.ndarray)
#   H: shape (n_herder, 3) rows are [x, y, yaw]
#   T: shape (n_target, 3) rows are [x, y, yaw]
#   returns:
#     herder_cmds: shape (n_herder, 2) rows are [v, omega]
#     target_cmds: shape (n_target, 2) rows are [v, omega]
from shepherding_control.my_control_library.learning_control import learning_controller


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Convert quaternion to yaw (Z axis rotation).
    Formula consistent with ROS REP-103 / standard yaw extraction.
    """
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ControllerNodeRL(Node):
    def __init__(self, n_herder: int, n_target: int):
        super().__init__('controller_node_rl')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Team sizes
        self.n = n_herder
        self.nt = n_target

        # Pose buffers (store geometry_msgs/Pose for each robot; keyed by 1-based IDs)
        self.H = {i: None for i in range(1, n_herder + 1)}
        self.T = {j: None for j in range(1, n_target + 1)}

        # Publishers (('herder'|'target'), id) -> Publisher(Twist)
        self.cmd_publishers = {}

        # Subscriptions & publishers for herders
        for i in self.H:
            sub_topic = f'/model/herder{i}/odometry'
            self.create_subscription(Odometry, sub_topic, self._make_callback('herder', i), 10)
            pub_topic = f'/model/herder{i}/cmd_vel'
            self.cmd_publishers[('herder', i)] = self.create_publisher(Twist, pub_topic, 10)

        # Subscriptions & publishers for targets
        for j in self.T:
            sub_topic = f'/model/target{j}/odometry'
            self.create_subscription(Odometry, sub_topic, self._make_callback('target', j), 10)
            pub_topic = f'/model/target{j}/cmd_vel'
            self.cmd_publishers[('target', j)] = self.create_publisher(Twist, pub_topic, 10)

        # Control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

    def _make_callback(self, kind: str, idx: int):
        def callback(msg: Odometry):
            pose = msg.pose.pose
            if kind == 'herder':
                self.H[idx] = pose
            else:
                self.T[idx] = pose
        return callback

    def _poses_to_array(self, poses_dict, expected_len: int) -> np.ndarray:
        """
        Convert a dict of geometry_msgs/Pose (1-based contiguous keys) into
        an array of shape (expected_len, 3) with rows [x, y, yaw].
        """
        arr = np.zeros((expected_len, 3), dtype=np.float64)
        for k in range(1, expected_len + 1):
            pose = poses_dict[k]
            position = pose.position
            orient = pose.orientation
            yaw = quat_to_yaw(orient.x, orient.y, orient.z, orient.w)
            arr[k - 1, :] = [position.x, position.y, yaw]
        return arr

    def control_loop(self):
        # Check readiness
        not_ready_H = [i for i, p in self.H.items() if p is None]
        not_ready_T = [j for j, p in self.T.items() if p is None]
        if not_ready_H or not_ready_T:
            self.get_logger().info(f"Waiting for poses: H{not_ready_H}, T{not_ready_T}")
            return

        # Build numpy arrays of poses: H -> (n_herder, 3), T -> (n_target, 3)
        try:
            H_arr = self._poses_to_array(self.H, self.n)
            T_arr = self._poses_to_array(self.T, self.nt)
        except Exception as e:
            self.get_logger().error(f"Failed converting poses to arrays: {e}")
            return

        # Call learning controller to get velocity commands
        try:
            herder_cmds, target_cmds = learning_controller(H_arr, T_arr, self.get_logger())
        except Exception as e:
            self.get_logger().error(f"learning_controller raised an exception: {e}")
            return

        # Validate outputs
        if herder_cmds is None or target_cmds is None:
            self.get_logger().warn("learning_controller returned None commands; skipping this cycle.")
            return

        herder_cmds = np.asarray(herder_cmds, dtype=float)
        target_cmds = np.asarray(target_cmds, dtype=float)

        if herder_cmds.shape != (self.n, 2):
            self.get_logger().error(f"Expected herder_cmds shape {(self.n, 2)}, got {herder_cmds.shape}")
            return
        if target_cmds.shape != (self.nt, 2):
            self.get_logger().error(f"Expected target_cmds shape {(self.nt, 2)}, got {target_cmds.shape}")
            return

        # Publish herder commands
        for i in range(1, self.n + 1):
            v, omega = herder_cmds[i - 1, 0], herder_cmds[i - 1, 1]
            twist = Twist()
            twist.linear.x = float(v)
            twist.angular.z = float(omega)
            self.cmd_publishers[('herder', i)].publish(twist)

        # Publish target commands
        for j in range(1, self.nt + 1):
            v, omega = target_cmds[j - 1, 0], target_cmds[j - 1, 1]
            twist = Twist()
            twist.linear.x = float(v)
            twist.angular.z = float(omega)
            self.cmd_publishers[('target', j)].publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # Configure number of robots (targets = Osoyoo, herders = TurtleBot)
    osoyoo_count = 12
    turtlebot_count = 4

    node = ControllerNodeRL(n_herder=turtlebot_count, n_target=osoyoo_count)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

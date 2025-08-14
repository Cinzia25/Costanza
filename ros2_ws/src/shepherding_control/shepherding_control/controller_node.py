import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
##lama_control_obs for obstacle avoidance, lama_contro otherwise
from shepherding_control.my_control_library.lama_control_obs import compute_cmd

class ControllerNode(Node):
    def __init__(self, n_herder, n_target):
        super().__init__('controller_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Number of herders (TurtleBots) and targets (Osoyoo)
        self.n = n_herder
        self.nt = n_target

        # Pose buffers (Odometry.pose.pose) for herders and targets, keyed by 1-based IDs
        self.H = {i: None for i in range(1, n_herder + 1)}
        self.T = {j: None for j in range(1, n_target + 1)}

        # Publishers dictionary keyed by ('herder'|'target', id) → Publisher(Twist)
        self.cmd_publishers = {}

        # --- Subscriptions & publishers for herders ---
        for i in self.H:
            # Subscribe to each herder's odom
            sub_topic = f'/model/herder{i}/odometry'
            self.create_subscription(Odometry, sub_topic, self._make_callback('herder', i), 10)
            # Publisher for velocity commands
            pub_topic = f'/model/herder{i}/cmd_vel'
            self.cmd_publishers[('herder', i)] = self.create_publisher(Twist, pub_topic, 10)

        # --- Subscriptions & publishers for targets ---
        for j in self.T:
            sub_topic = f'/model/target{j}/odometry'
            self.create_subscription(Odometry, sub_topic, self._make_callback('target', j), 10)
            pub_topic = f'/model/target{j}/cmd_vel'
            self.cmd_publishers[('target', j)] = self.create_publisher(Twist, pub_topic, 10)

        # Periodic control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)
 
    def _make_callback(self, kind, idx):
        """
        Factory for Odometry callbacks.
        Stores the incoming pose into the appropriate dictionary (H or T).
        """
        def callback(msg):
            if kind == 'herder':
                # Save herder pose (geometry_msgs/Pose)
                self.H[idx] = msg.pose.pose
            else:
                # Save target pose
                self.T[idx] = msg.pose.pose
        return callback

    def control_loop(self):
        """
        Main control loop:
        - Ensures all poses are available.
        - Computes commands for each herder and target via compute_cmd().
        - Publishes geometry_msgs/Twist to the respective /cmd_vel topics.
        """
        # Check readiness: list IDs whose pose hasn't arrived yet
        not_ready_H = [i for i, p in self.H.items() if p is None]
        not_ready_T = [j for j, p in self.T.items() if p is None]

        if not_ready_H or not_ready_T:
            # If any pose is missing, wait and log which ones
            self.get_logger().info(f"Waiting for poses: H{not_ready_H}, T{not_ready_T}")
            return

        # Compute and publish commands for all herders
        for i in self.H:
            # is_herder=True → herder behavior in compute_cmd()
            cmd = compute_cmd(i, self.H, self.T, self.get_logger(), is_herder=True)
            if cmd:
                self.cmd_publishers[('herder', i)].publish(cmd)

        # Compute and publish commands for all targets
        for j in self.T:
            # is_herder=False → target behavior in compute_cmd()
            cmd = compute_cmd(j, self.H, self.T, self.get_logger(), is_herder=False)
            if cmd:
                self.cmd_publishers[('target', j)].publish(cmd)
 

def main(args=None):
    # Standard ROS 2 entry point
    rclpy.init(args=args)

    # Configure number of robots (targets = Osoyoo, herders = TurtleBot)
    osoyoo_count = 5
    turtlebot_count = 2

    # Create controller node with the requested team sizes
    node = ControllerNode(n_herder=turtlebot_count, n_target=osoyoo_count)

    # Spin until shutdown, then clean up
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


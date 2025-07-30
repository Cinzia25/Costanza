import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from shepherding_control.my_control_library.control import compute_cmd

class ControllerNode(Node):
    def __init__(self, n_herder, n_target):
        super().__init__('controller_node')

        self.n = n_herder
        self.nt = n_target

        self.H = {i: None for i in range(1, n_herder + 1)}
        self.T = {j: None for j in range(1, n_target + 1)}

        self.cmd_publishers = {}

        # Herders
        for i in self.H:
            sub_topic = f'/model/herder{i}/odometry'
            self.create_subscription(Odometry, sub_topic, self._make_callback('herder', i), 10)
            pub_topic = f'/model/herder{i}/cmd_vel'
            self.cmd_publishers[('herder', i)] = self.create_publisher(Twist, pub_topic, 10)

        #Targets
        for j in self.T:
            sub_topic = f'/model/target{j}/odometry'
            self.create_subscription(Odometry, sub_topic, self._make_callback('target', j), 10)
            pub_topic = f'/model/target{j}/cmd_vel'
            self.cmd_publishers[('target', j)] = self.create_publisher(Twist, pub_topic, 10)


        self.create_timer(0.1, self.control_loop)
 
    def _make_callback(self, kind, idx):
        def callback(msg):
            if kind == 'herder':
                self.H[idx] = msg.pose.pose
            else:
                self.T[idx] = msg.pose.pose
        return callback

    def control_loop(self):
        for i in self.H:
            cmd = compute_cmd(i, self.H, self.T, is_herder=True)
            if cmd:
                self.cmd_publishers[('herder', i)].publish(cmd)
        for j in self.T:
            cmd = compute_cmd(j, self.H, self.T, is_herder=False)
            if cmd:
                self.cmd_publishers[('target', j)].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    osoyoo_count = 10
    turtlebot_count = 5
    node = ControllerNode(n_herder=osoyoo_count, n_target=turtlebot_count)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


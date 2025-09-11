import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from geometry_msgs.msg import Twist
##lama_control_obs for obstacle avoidance, lama_control otherwise
from shepherding_control.my_control_library.robot_lama_control_obs import compute_cmd

import csv
import os
from datetime import datetime
try:
    from zoneinfo import ZoneInfo
    _BERLIN_TZ = ZoneInfo("Europe/Berlin")
except Exception:
    _BERLIN_TZ = None

turtlebot_ids = [4, 5]
osoyoo_ids = [1, 2, 3]
obstacle_ids = [1]

class LamaRobotControllerNode(Node):
    def __init__(self, n_herder, n_target, n_obstacle):
        super().__init__('lama_robot_controller_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.n = n_herder
        self.nt = n_target
        self.no = n_obstacle

        self.H = {i: None for i in range(1, n_herder + 1)}
        self.T = {j: None for j in range(1, n_target + 1)}
        self.O = {k: None for k in range(1, n_obstacle + 1)}

        self.cmd_publishers = {}

        # Herders
        for i in self.H:
            sub_topic = f'/vicon/turtlebot_{str(turtlebot_ids[i-1])}/turtlebot_{str(turtlebot_ids[i-1])}'
            self.create_subscription(Position, sub_topic, self._make_callback('herder', i), 10)
            pub_topic = f'/turtlebot4_{str(turtlebot_ids[i-1])}/cmd_vel_unstamped'
            self.cmd_publishers[('herder', i)] = self.create_publisher(Twist, pub_topic, 10)

        # Targets
        for j in self.T:
            sub_topic = f'/vicon/osoyoo_{str(osoyoo_ids[j-1])}/osoyoo_{str(osoyoo_ids[j-1])}'
            self.create_subscription(Position, sub_topic, self._make_callback('target', j), 10)
            pub_topic = f"/model/target{str(osoyoo_ids[j-1])}/cmd_vel"
            self.cmd_publishers[('target', j)] = self.create_publisher(Twist, pub_topic, 10)

        # Obstacles
        for k in self.O:
            sub_topic = f'/vicon/obstacle_{str(obstacle_ids[k-1])}/obstacle_{str(obstacle_ids[k-1])}'
            self.create_subscription(Position, sub_topic, self._make_callback('obstacle', k), 10)

        # CSV setup
        self._setup_csv_logging()

        self.create_timer(0.02, self.control_loop)

    def _setup_csv_logging(self):
        now = datetime.now(_BERLIN_TZ) if _BERLIN_TZ else datetime.now()
        date_str = now.strftime("%Y%m%d")
        time_str = now.strftime("%H%M%S")
        self.csv_filename = f"{self.n}_{self.nt}_{date_str}_{time_str}.csv"

        header = ["timestamp"]
        for i in range(1, self.n + 1):
            header += [f"H{i}_x", f"H{i}_y", f"H{i}_theta"]
        for j in range(1, self.nt + 1):
            header += [f"T{j}_x", f"T{j}_y", f"T{j}_theta"]
        for k in range(1, self.no + 1):
            header += [f"O{k}_x", f"O{k}_y", f"O{k}_theta"]

        self.csv_file = open(self.csv_filename, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(header)
        self.csv_file.flush()
        self.get_logger().info(f"CSV logging to: {os.path.abspath(self.csv_filename)}")

    def _make_callback(self, kind, idx):
        def callback(msg):
            if kind == 'herder':
                self.H[idx] = msg
            elif kind == 'target':
                self.T[idx] = msg
            else:
                self.O[idx] = msg
        return callback

    def _extract_xyz(self, msg: Position):
        """Extract (x, y, orientation) from Position msg."""
        if msg is None:
            return None, None, None
        x = msg.x_trans
        y = msg.y_trans
        theta = msg.z_rot_euler  # third quaternion component
        return x, y, theta

    def _log_current_row(self):
        now = datetime.now(_BERLIN_TZ) if _BERLIN_TZ else datetime.now()
        ts = now.isoformat(timespec='milliseconds')
        row = [ts]
        for i in range(1, self.n + 1):
            row += list(self._extract_xyz(self.H[i]))
        for j in range(1, self.nt + 1):
            row += list(self._extract_xyz(self.T[j]))
        for k in range(1, self.no + 1):
            row += list(self._extract_xyz(self.O[k]))
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def control_loop(self):
        not_ready_H = [i for i, p in self.H.items() if p is None]
        not_ready_T = [j for j, p in self.T.items() if p is None]
        not_ready_O = [k for k, p in self.O.items() if p is None]

        if not_ready_H or not_ready_T or not_ready_O:
            self.get_logger().info(f"Waiting for poses: H{not_ready_H}, T{not_ready_T}, O{not_ready_O}")
            return

        # Log positions
        self._log_current_row()

        # Herders
        for i in self.H:
            cmd = compute_cmd(i, self.H, self.T, self.O, self.get_logger(), is_herder=True)
            if cmd:
                self.cmd_publishers[('herder', i)].publish(cmd)

        # Targets
        for j in self.T:
            cmd = compute_cmd(j, self.H, self.T, self.O, self.get_logger(), is_herder=False)
            if cmd:
                self.cmd_publishers[('target', j)].publish(cmd)

    def destroy_node(self):
        try:
            if hasattr(self, "csv_file") and self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception as e:
            self.get_logger().error(f"Error closing CSV file: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    osoyoo_count = 3
    turtlebot_count = 2
    obstacle_count = 1

    node = LamaRobotControllerNode(
        n_herder=turtlebot_count, n_target=osoyoo_count, n_obstacle=obstacle_count
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

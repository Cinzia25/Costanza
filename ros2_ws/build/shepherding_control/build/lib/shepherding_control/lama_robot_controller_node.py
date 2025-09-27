import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from shepherding_control.my_control_library.robot_lama_control import compute_cmd

import numpy as np
import asyncio
import xml.etree.ElementTree as ET
from threading import Thread
import qtm_rt as qtm
import csv
import os
from datetime import datetime

# IDs of physical robots
turtlebot_ids = [3, 4]
osoyoo_ids = [3, 7, 8, 10]

# Names in QTM (these must match your rigid body names in the QTM project)
herder_names = ['turtlebot_3', 'turtlebot_4']
target_names = ['target_3', 'target_7', 'target_9', 'target_10']


class QtmWrapper(Thread):
    """Minimal QTM client running in a background thread. Keeps most recent 6D pose for each body."""
    def __init__(self, body_names):
        super().__init__()
        self.body_names = body_names
        self.qtm_6DoF_labels = []
        self.pose = {name: None for name in body_names}
        self.connection = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while self._stay_open:
            await asyncio.sleep(0.1)
        await self._close()

    async def _connect(self):
        qtm_instance = await self._discover()
        host = qtm_instance.host
        host = "10.111.20.20"
        print(f"[QtmWrapper] Connecting to QTM on {host}")
        self.connection = await qtm.connect(host)

        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip()
                                for label in xml.findall('*/Body/Name')]

        await self.connection.stream_frames(
            components=['6D', '6dEuler'],
            on_packet=self._on_packet)

    async def _discover(self):
        async for qtm_instance in qtm.Discover('0.0.0.0'):
            return qtm_instance

    def _on_packet(self, packet):
        _, bodies = packet.get_6d()
        _, bodies_euler = packet.get_6d_euler()
        if bodies is None:
            return

        for body in self.body_names:
            if body not in self.qtm_6DoF_labels:
                print(f"[QtmWrapper] Body {body} not found in QTM.")
                continue
            idx = self.qtm_6DoF_labels.index(body)
            pos, rot = bodies[idx]
            _, euler = bodies_euler[idx]
            if pos is None:
                continue
            x, y, z = np.array(pos) / 1000.0
            yaw = np.deg2rad(euler[2]) if euler is not None else 0.0
            self.pose[body] = (x, y, yaw)

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()


class LamaRobotControllerNode(Node):
    def __init__(self, n_herder, n_target):
        super().__init__('lama_robot_controller_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.n = n_herder
        self.nt = n_target

        # Pose buffers
        self.H = {i: None for i in range(1, n_herder + 1)}
        self.T = {j: None for j in range(1, n_target + 1)}

        # Publishers
        self.cmd_publishers = {}
        for i in self.H:
            pub_topic = f'/turtlebot4_{turtlebot_ids[i-1]}/cmd_vel_unstamped'
            self.cmd_publishers[('herder', i)] = self.create_publisher(Twist, pub_topic, 10)
        for j in self.T:
            pub_topic = f"/osoyoo_{osoyoo_ids[j-1]}/cmd_vel"
            self.cmd_publishers[('target', j)] = self.create_publisher(Twist, pub_topic, 10)

        # QTM wrapper (herders + targets)
        self.qtm_wrapper = QtmWrapper(herder_names + target_names)

        # CSV logging
        log_name = datetime.now().strftime("poses_log_%Y%m%d_%H%M%S.csv")
        self.log_path = os.path.join(os.getcwd(), log_name)
        self.csv_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "type", "id", "x", "y", "yaw"])
        self.get_logger().info(f"Logging poses to {self.log_path}")

        # Timer for control loop
        self.create_timer(0.02, self.control_loop)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Update poses from QTM
        for i, name in enumerate(herder_names, start=1):
            if self.qtm_wrapper.pose[name] is not None:
                x, y, yaw = self.qtm_wrapper.pose[name]
                self.H[i] = (x, y, yaw)
                # log pose
                self.csv_writer.writerow([now, "herder", i, x, y, yaw])

        for j, name in enumerate(target_names, start=1):
            if self.qtm_wrapper.pose[name] is not None:
                x, y, yaw = self.qtm_wrapper.pose[name]
                self.T[j] = (x, y, yaw)
                # log pose
                self.csv_writer.writerow([now, "target", j, x, y, yaw])

        # Check readiness
        not_ready_H = [i for i, p in self.H.items() if p is None]
        not_ready_T = [j for j, p in self.T.items() if p is None]
        if not_ready_H or not_ready_T:
            self.get_logger().info(f"Waiting for poses: H{not_ready_H}, T{not_ready_T}")
            return

        # Compute and publish commands
        for i in self.H:
            cmd = compute_cmd(i, self.H, self.T, self.get_logger(), is_herder=True)
            if cmd:
                self.cmd_publishers[('herder', i)].publish(cmd)

        for j in self.T:
            cmd = compute_cmd(j, self.H, self.T, self.get_logger(), is_herder=False)
            if cmd:
                self.cmd_publishers[('target', j)].publish(cmd)

    def destroy_node(self):
        # Close CSV file cleanly
        if hasattr(self, "csv_file"):
            self.csv_file.close()
        self.qtm_wrapper.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LamaRobotControllerNode(n_herder=2, n_target=4)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import numpy as np
import asyncio
import xml.etree.ElementTree as ET
from threading import Thread
from datetime import datetime
import csv
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# === CONFIG: ID fisici e nomi in QTM ===
turtlebot_ids = [3, 4]                 # herders (fisici)
osoyoo_ids    = [4, 3, 7, 9, 10]       # targets (fisici)

# turtlebot_ids = [3]                 # herders (fisici)
# osoyoo_ids    = [7]       # targets (fisici)

# I nomi DEVONO combaciare con i rigid body nel progetto QTM
herder_names  = ['turtlebot_3', 'turtlebot_4']
target_names  = ['target_4', 'target_3', 'target_7', 'target_9', 'target_10']

# herder_names  = ['turtlebot_3']
# target_names  = ['target_7']

# === Controller RL (come nel tuo primo nodo) ===
# learning_controller(H: (nH,3), T: (nT,3), logger) -> (herder_cmds:(nH,2), target_cmds:(nT,2))
from shepherding_control.my_control_library.learning_control import learning_controller


# ---------------- QTM wrapper identico alla tua versione ----------------
import qtm_rt as qtm

class QtmWrapper(Thread):
    """QTM client in thread dedicato, mantiene l'ultima pose 6DoF per ogni body."""
    def __init__(self, body_names):
        super().__init__()
        self.body_names = body_names
        self.qtm_6DoF_labels = []
        self.pose = {name: None for name in body_names}  # name -> (x, y, yaw)
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
        host = qtm_instance.host if qtm_instance else "10.111.20.20"
        # Se vuoi forzare un IP QTM specifico:
        host = "10.111.20.20"
        print(f"[QtmWrapper] Connecting to QTM on {host}")
        self.connection = await qtm.connect(host)

        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip()
                                for label in xml.findall('*/Body/Name')]

        await self.connection.stream_frames(
            components=['6D', '6dEuler'],
            on_packet=self._on_packet
        )

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
                # Body non presente: salta
                continue
            idx = self.qtm_6DoF_labels.index(body)
            pos, _ = bodies[idx]
            _, euler = bodies_euler[idx]
            if pos is None:
                continue
            x, y, z = np.array(pos) / 1000.0
            yaw = np.deg2rad(euler[2]) if euler is not None else 0.0
            self.pose[body] = (float(x), float(y), float(yaw))

    async def _close(self):
        try:
            await self.connection.stream_frames_stop()
        finally:
            self.connection.disconnect()


# ---------------- Nodo che usa QTM + learning_controller ----------------
class ControllerNodeRLReal(Node):
    """
    - Legge pose reali da QTM (herders + targets).
    - Converte in array H (nH,3), T (nT,3) -> chiama learning_controller.
    - Pubblica Twist su topic reali dei robot.
    - Logga CSV delle pose.
    """
    CONTROL_DT = 0.02  # 50 Hz

    def __init__(self):
        super().__init__('controller_node_rl_real')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.nH = len(turtlebot_ids)
        self.nT = len(osoyoo_ids)

        # Buffer pose (indice base-1 come nel tuo codice)
        self.H = {i: None for i in range(1, self.nH + 1)}  # i -> (x,y,yaw)
        self.T = {j: None for j in range(1, self.nT + 1)}  # j -> (x,y,yaw)

        # Publisher verso i topic fisici
        self.cmd_publishers = {}
        for i in range(1, self.nH + 1):
            tb_id = turtlebot_ids[i - 1]
            topic = f'/turtlebot4_{tb_id}/cmd_vel_unstamped'
            self.cmd_publishers[('herder', i)] = self.create_publisher(Twist, topic, 10)

        for j in range(1, self.nT + 1):
            os_id = osoyoo_ids[j - 1]
            topic = f'/osoyoo_{os_id}/cmd_vel'
            self.cmd_publishers[('target', j)] = self.create_publisher(Twist, topic, 10)

        # QTM wrapper (nomi = herder_names + target_names)
        self.qtm_wrapper = QtmWrapper(herder_names + target_names)

        # CSV logging pose
        log_name = datetime.now().strftime("poses_log_%Y%m%d_%H%M%S.csv")
        self.log_path = os.path.join(os.getcwd(), log_name)
        self.csv_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "type", "id", "x", "y", "yaw"])
        self.get_logger().info(f"Logging poses to {self.log_path}")

        # Timer controllo
        self.create_timer(self.CONTROL_DT, self.control_loop)

    def _fill_pose_buffers_from_qtm(self):
        """Aggiorna H/T dai nomi QTM e logga su CSV."""
        now = self.get_clock().now().nanoseconds / 1e9

        for i, name in enumerate(herder_names, start=1):
            p = self.qtm_wrapper.pose.get(name)
            if p is not None:
                x, y, yaw = p
                self.H[i] = (x, y, yaw)
                self.csv_writer.writerow([now, "herder", i, x, y, yaw])

        for j, name in enumerate(target_names, start=1):
            p = self.qtm_wrapper.pose.get(name)
            if p is not None:
                x, y, yaw = p
                self.T[j] = (x, y, yaw)
                self.csv_writer.writerow([now, "target", j, x, y, yaw])

    @staticmethod
    def _dict_to_array(dct, expected_len):
        """(1..N)->(x,y,yaw)  -> np.array (N,3)"""
        arr = np.zeros((expected_len, 3), dtype=np.float64)
        for k in range(1, expected_len + 1):
            x, y, yaw = dct[k]
            arr[k - 1, :] = [x, y, yaw]
        return arr

    def control_loop(self):
        # Aggiorna pose da QTM
        self._fill_pose_buffers_from_qtm()

        # Prontezza
        not_ready_H = [i for i, p in self.H.items() if p is None]
        not_ready_T = [j for j, p in self.T.items() if p is None]
        if not_ready_H or not_ready_T:
            self.get_logger().info(f"Waiting for poses: H{not_ready_H}, T{not_ready_T}")
            return

        # Costruisci H, T per il controller RL
        try:
            H_arr = self._dict_to_array(self.H, self.nH)  # (nH,3)
            T_arr = self._dict_to_array(self.T, self.nT)  # (nT,3)
        except Exception as e:
            self.get_logger().error(f"Pose -> array failed: {e}")
            return

        # Chiama il controller RL
        try:
            herder_cmds, target_cmds = learning_controller(H_arr, T_arr, self.get_logger())
        except Exception as e:
            self.get_logger().error(f"learning_controller exception: {e}")
            return

        # Validazione forme
        if herder_cmds is None or target_cmds is None:
            self.get_logger().warn("learning_controller returned None; skipping this cycle.")
            return

        herder_cmds = np.asarray(herder_cmds, dtype=float)  # (nH,2)
        target_cmds = np.asarray(target_cmds, dtype=float)  # (nT,2)

        if herder_cmds.shape != (self.nH, 2):
            self.get_logger().error(f"Expected herder_cmds shape {(self.nH, 2)}, got {herder_cmds.shape}")
            return
        if target_cmds.shape != (self.nT, 2):
            self.get_logger().error(f"Expected target_cmds shape {(self.nT, 2)}, got {target_cmds.shape}")
            return

        # Pubblica comandi
        for i in range(1, self.nH + 1):
            v, omega = float(herder_cmds[i - 1, 0]), float(herder_cmds[i - 1, 1])
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = omega
            self.cmd_publishers[('herder', i)].publish(msg)

        for j in range(1, self.nT + 1):
            v, omega = float(target_cmds[j - 1, 0]), float(target_cmds[j - 1, 1])
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = omega
            self.cmd_publishers[('target', j)].publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self, "csv_file"):
                self.csv_file.close()
        finally:
            try:
                self.qtm_wrapper.close()
            finally:
                super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNodeRLReal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

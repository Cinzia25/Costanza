import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

from robot_controller.controller_library import move_robot, stopcar


class OnboardController(Node):
    """
    ROS 2 node che riceve comandi Twist e li inoltra al controller dei motori.

    - Callback del subscriber: salva solo l'ultimo comando.
    - Timer a 50 Hz: applica periodicamente i comandi ai motori.
    - Watchdog: se non arrivano messaggi per TIMEOUT_S secondi, ferma i motori.
    """

    TIMEOUT_S: float = 2.0
    CONTROL_HZ: float = 50.0  # frequenza ciclo di controllo (Hz)

    def __init__(self) -> None:
        # Legge ID robot
        with open("/home/pi/ssm_ws/id.txt", "r") as f:
            self_id = f.read().strip()

        super().__init__(f"{self_id}_onboard_controller")

        self.last_msg_time = self.get_clock().now()
        self.stopped_due_to_timeout = False
        self.latest_cmd = Twist()  # ultimo comando ricevuto

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            f"/{self_id}/cmd_vel",
            self.listener_callback,
            10,
        )

        # Timer di controllo (es. 50 Hz)
        self.control_timer = self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)

        # Timer watchdog (1 Hz)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info(
            f"OnboardController avviato. "
            f"Topic: /{self_id}/cmd_vel. "
            f"Loop: {self.CONTROL_HZ} Hz. "
            f"Timeout: {self.TIMEOUT_S} s."
        )

    def listener_callback(self, msg: Twist) -> None:
        """Salva l'ultimo comando ricevuto senza bloccare ROS."""
        self.latest_cmd = msg
        self.last_msg_time = self.get_clock().now()

        if self.stopped_due_to_timeout:
            self.get_logger().info("Ricevuto nuovo cmd_vel dopo timeout → ripresa movimento.")
            self.stopped_due_to_timeout = False

    def _control_loop(self) -> None:
        """Applica periodicamente l'ultimo comando salvato ai motori."""
        if not self.stopped_due_to_timeout:
            move_robot(self.latest_cmd.linear.x, self.latest_cmd.angular.z)

    def _watchdog_check(self) -> None:
        """Ferma i motori se non arrivano comandi da troppo tempo."""
        now = self.get_clock().now()
        if (now - self.last_msg_time) > Duration(seconds=self.TIMEOUT_S):
            if not self.stopped_due_to_timeout:
                self.get_logger().warn(
                    f"Nessun cmd_vel da {self.TIMEOUT_S} s → stop di sicurezza."
                )
                self.stop_car()
                self.stopped_due_to_timeout = True

    def stop_car(self) -> None:
        """Invoca la funzione di stop basso livello."""
        stopcar()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OnboardController()
    executor = MultiThreadedExecutor()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown richiesto (CTRL+C).")
    finally:
        node.get_logger().info("Arresto: stop motori.")
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

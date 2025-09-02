import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from robot_controller.controller_library import move_robot, stopcar


class OnboardController(Node):
    """
    ROS 2 node that subscribes to a Twist command stream and forwards
    linear/angular velocities to the low-level motion controller.

    The node includes a watchdog: if no new Twist message is received
    for more than TIMEOUT_S seconds, it triggers a safety stop by calling
    `stopcar()` once. As soon as a new message arrives, normal operation resumes.

    Attributes
    ----------
    TIMEOUT_S : float
        Inactivity timeout in seconds before a safety stop is triggered.
    last_msg_time : rclpy.time.Time
        Timestamp of the most recently received cmd_vel message.
    stopped_due_to_timeout : bool
        Whether a safety stop has been issued due to a timeout.
    subscription : rclpy.subscription.Subscription
        Subscription to the cmd_vel topic for this robot instance.
    watchdog_timer : rclpy.timer.Timer
        Periodic timer that checks for message inactivity.
    """

    TIMEOUT_S: float = 20.0

    def __init__(self) -> None:
        """Initialize the node, set up the subscriber and the watchdog timer."""
        super().__init__("onboard_controller")

        # Read the robot ID to build the namespaced topic
        with open("/home/pi/ssm_ws/id.txt", "r") as f:
            self_id = f.read().strip()

        self.last_msg_time = self.get_clock().now()
        self.stopped_due_to_timeout = False

        # Subscribe to the velocity command topic for this robot instance
        self.subscription = self.create_subscription(
            Twist,
            f"/model/{self_id}/cmd_vel",
            self.listener_callback,
            10,
        )

        # Check every second whether the inactivity timeout has elapsed
        self.watchdog_timer = self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info(
            f"SpeedReceiver started. Subscribed to /model/{self_id}/cmd_vel. "
            f"Watchdog timeout set to {self.TIMEOUT_S} s."
        )

    def listener_callback(self, msg: Twist) -> None:
        """
        Handle incoming Twist messages.

        Parameters
        ----------
        msg : geometry_msgs.msg.Twist
            The commanded linear and angular velocities.
        """
        self.last_msg_time = self.get_clock().now()

        # If we had stopped due to timeout, inform that we're resuming
        if self.stopped_due_to_timeout:
            self.get_logger().info("Received new cmd_vel after timeout. Resuming motion control.")
            self.stopped_due_to_timeout = False

        v_lin = msg.linear.x    # m/s
        v_ang = msg.angular.z   # rad/s
        move_robot(v_lin, v_ang)

    def _watchdog_check(self) -> None:
        """
        Periodic watchdog callback.

        If no Twist has arrived within TIMEOUT_S seconds, call `stop_car()` once
        and set the internal flag to avoid repeated stops. Logging occurs only on the transition.
        """
        now = self.get_clock().now()
        if (now - self.last_msg_time) > Duration(seconds=self.TIMEOUT_S):
            if not self.stopped_due_to_timeout:
                self.get_logger().warn(
                    f"No cmd_vel received for > {self.TIMEOUT_S} s. Issuing safety stop."
                )
                self.stop_car()
                self.stopped_due_to_timeout = True

    def stop_car(self) -> None:
        """
        Invoke the low-level stop routine.

        This is a thin wrapper around `robot_controller.controller_library.stopcar()`
        to centralize the stopping behavior within the node.
        """
        stopcar()


def main(args=None) -> None:
    """
    Entry point for the SpeedReceiver node.

    Initializes rclpy, spins the node, and ensures the robot is stopped on shutdown.
    """
    rclpy.init(args=args)
    node = OnboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (KeyboardInterrupt).")
    finally:
        # Safety: ensure the robot is stopped and clean up the node
        node.get_logger().info("Shutting down. Ensuring the robot is stopped.")
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

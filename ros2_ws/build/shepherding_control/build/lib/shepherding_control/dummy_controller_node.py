import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from vicon_receiver.msg import Position
##lama_control_obs for obstacle avoidance, lama_control otherwise
from shepherding_control.my_control_library.control import compute_cmd

import numpy as np

class ControllerNode(Node):
    def __init__(self):
        super().__init__('dummy_controller_node')

        sub_topic = f'vicon/turtlebot_5/turtlebot_5'
        self.create_subscription(Position, sub_topic, self.listener_callback, 10)
        pub_topic = f'/turtlebot4_5/cmd_vel_unstamped'
        self.vel_pub = self.create_publisher(Twist, pub_topic, 10)

        # Periodic control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.robot_pos = None
 
    def listener_callback(self, msg):
        
        # Update the current position with the received data
        self.robot_pos = np.array([float(msg.x_trans), float(msg.y_trans), 
                                float( msg.z_rot_euler)])
            

    def control_loop(self):
        """
        Main control loop:
        - Ensures all poses are available.
        - Computes commands for each herder and target via compute_cmd().
        - Publishes geometry_msgs/Twist to the respective /cmd_vel topics.
        """

        # Compute and publish commands for all herders

        GOAL_X = 0
        GOAL_Y = 0
        goal = np.array([GOAL_X, GOAL_Y])

        cmd = compute_cmd(self.robot_pos, goal)
        self.vel_pub.publish(cmd)


def main(args=None):
    # Standard ROS 2 entry point
    rclpy.init(args=args)

    # Create controller node with the requested team sizes
    node = ControllerNode()

    # Spin until shutdown, then clean up
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


from geometry_msgs.msg import Twist
import numpy as np
import math


linear_velocity_gain = 0.001
angular_velocity_gain = 0.001

max_linear_vel = 0.10
max_angular_vel = 1

def compute_cmd(robot_pos, goal):
    """
    Comando per dirigersi verso la goal region (solo per gli herder).
    I target restano passivi.
    """

    cmd = Twist()

    if robot_pos is None:
        print("NO POSITION RECEIVED FROM VICON")
        return Twist()

    pos_error = goal - robot_pos[:2]
    rot_error = np.arctan2(pos_error[1], pos_error[0])
    dist = np.linalg.norm(pos_error, axis=0)

    linear_vel = linear_velocity_gain * dist * np.cos(rot_error - robot_pos[2])
    angular_vel = angular_velocity_gain * dist * np.sin(rot_error - robot_pos[2])

    linear_vel = np.clip(linear_vel, -max_linear_vel, max_linear_vel)
    angular_vel = np.clip (angular_vel, -max_angular_vel, max_angular_vel)

    cmd.linear.x = linear_vel
    cmd.angular.z = angular_vel
    
    print("robot_pos = ", robot_pos)
    print("linear vel = ", cmd.linear.x)
    print("angular vel = ", cmd.angular.z)

    return cmd

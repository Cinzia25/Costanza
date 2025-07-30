from geometry_msgs.msg import Twist
import math

GOAL_X = 5.0
GOAL_Y = 5.0

def compute_cmd(idx, H, T, is_herder):
    """
    Comando per dirigersi verso la goal region (solo per gli herder).
    I target restano passivi.
    """
    if not is_herder:
        return None  
    pose = H.get(idx)
    if pose is None:
        return None  


    x = pose.position.x
    y = pose.position.y
    q = pose.orientation

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    dx = GOAL_X - x
    dy = GOAL_Y - y
    dist = math.hypot(dx, dy)

    if dist < 0.2:
        return Twist()  

    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - yaw

    while angle_error > math.pi:
        angle_error -= 2 * math.pi
    while angle_error < -math.pi:
        angle_error += 2 * math.pi

    cmd = Twist()
    Kp_ang = 1.5
    Kp_lin = 0.5
    angle_thresh = 0.2  # rad

    cmd.angular.z = Kp_ang * angle_error

    if abs(angle_error) < angle_thresh:
        cmd.linear.x = Kp_lin * dist
    else:
        cmd.linear.x = 0.0


    cmd.linear.x = max(min(cmd.linear.x, 0.5), 0.0)
    cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

    return cmd


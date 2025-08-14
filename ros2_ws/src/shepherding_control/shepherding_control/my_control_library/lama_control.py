from geometry_msgs.msg import Twist
import math
import numpy as np
import rclpy

# === Constants and parameters ===
TURTLEBOT_RADIUS = 0.15     # Physical radius of herder (TurtleBot)
OSOYOO_RADIUS = 0.1         # Physical radius of target (Osoyoo)
GOAL_X = 0.0                # X coordinate of the goal region center
GOAL_Y = 0.0                # Y coordinate of the goal region center
FACTOR = 1/10               # Global scaling factor for distances
XI = 200.0*FACTOR + TURTLEBOT_RADIUS + TURTLEBOT_RADIUS  # Herder sensing range 
LAMBDA = 2.5*FACTOR + OSOYOO_RADIUS + TURTLEBOT_RADIUS   # Target sensing range
LAMBDA_DELTA = 2.5*0.5*FACTOR+ OSOYOO_RADIUS + TURTLEBOT_RADIUS  # Reduced distance threshold for pushing behaviour
V_H = 0.46                  # Max linear speed for herders
RG = 10*FACTOR              # Goal region radius 
BETA = 3                    # Repulsion scaling factor for targets
ALPHA = 3                   # Gain for pushing component
THRESHOLD = 0.5             # Distance tolerance for switching behaviours
EPSILON = 0.2               # Small offset parameter
R_REP = 2*TURTLEBOT_RADIUS + 0.1  # Global repulsion radius
K_REP = 1.0                 # Global repulsion gain 
D = 0.1                     # Scaling factor for noise in targets
sigma_v = 1                 # Std dev of linear velocity noise for targets
sigma_w = 1                 # Std dev of angular velocity noise for targets
mu_w = 0.0                  # Mean angular velocity noise for targets
p = 2                       # Repulsion exponent

def compute_cmd(idx, H, T, logger, is_herder):
    # Build a combined array of all robot positions (herders + targets)
    positions = []
    for h in H.values():
        if h is not None:
            positions.append([h.position.x, h.position.y])
    for t in T.values():
        if t is not None:
            positions.append([t.position.x, t.position.y])
    positions = np.array(positions)

    if is_herder:
        # === HERDER behavior ===
        pose_h = H.get(idx)
        if pose_h is None:
            return None

        # Extract position and orientation (yaw) of the herder
        xh, yh = pose_h.position.x, pose_h.position.y
        q = pose_h.orientation 
        herder_pos = np.array([xh, yh])
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)        
        
        goal = np.array([GOAL_X, GOAL_Y])
        
        if not T:
            # No targets → no movement
            dX = np.zeros(2)
        else:
            # Build arrays of target and herder positions
            T_array = np.array([[t.position.x, t.position.y] for t in T.values()])
            H_array = np.array([[h.position.x, h.position.y] for h in H.values()])
            
            # Distances from this herder to all targets
            d_HT = np.linalg.norm(T_array - herder_pos, axis=1)
            idx_visible = np.where(d_HT <= XI)[0]  # IDs of visible targets
            selected_target = None
            max_dist_from_goal = -np.inf

            # Target selection: choose visible target farthest from the goal 
            # among those for which this herder is the closest
            for i in idx_visible:
                target = T_array[i]
                d_target_to_H = np.linalg.norm(H_array - target, axis=1)
                idx_H_near = np.where(d_target_to_H <= XI)[0]
                min_dist = np.min(d_target_to_H[idx_H_near])
                dist_current = np.linalg.norm(herder_pos - target)

                if abs(dist_current - min_dist) < 1e-3:
                    d_goal = np.linalg.norm(target - goal)
                    if d_goal > max_dist_from_goal:
                        selected_target = target
                        selected_target_id = list(T.keys())[i]  
                        max_dist_from_goal = d_goal

            if selected_target is not None:
                # Vector from target to goal
                a = herder_pos - selected_target
                b = selected_target - goal
                norm_a = np.linalg.norm(a)
                norm_b = np.linalg.norm(b)
                direction = b / norm_b if norm_b > 1e-6 else np.zeros(2)

                # Desired herder position slightly toward the goal from the target
                desired_pos = selected_target + LAMBDA_DELTA * direction
                dX_dir = desired_pos - herder_pos
                dist = np.linalg.norm(dX_dir) + 1e-6  # Avoid division by zero

                # Compute cosine of angle between herder-target and target-goal
                b = desired_pos - selected_target
                norm_b = np.linalg.norm(b)
                cos_alpha = np.dot(a, b) / (norm_a * norm_b + 1e-6)
                cos_threshold = math.cos(math.radians(10))  # Tight alignment
                cos_orbit = math.cos(math.radians(45))      # Wide angle

                # Distance-based blending factor gamma (approach → orbit transition)
                if norm_a >= LAMBDA + THRESHOLD:
                    gamma = 0.0  # Only approaching
                elif norm_a <= LAMBDA_DELTA:
                    gamma = 1.0  # Only orbiting
                else:
                    gamma = (LAMBDA + THRESHOLD - norm_a) / (LAMBDA + THRESHOLD - LAMBDA_DELTA)

                # Angle-based blending factor beta (reduce orbiting if aligned)
                if cos_alpha >= cos_threshold:
                    beta = 1.0
                elif cos_alpha <= cos_orbit:
                    beta = 0.0
                else:
                    beta = (cos_alpha - cos_orbit) / (cos_threshold - cos_orbit)
                    
                # Determine orbiting rotation direction (perpendicular vector)
                vett = desired_pos - selected_target
                cross = a[0] * vett[1] - a[1] * vett[0]
                if cross > 0:
                    R = np.array([[0, -1], [1, 0]])  # CCW rotation
                else:
                    R = np.array([[0, 1], [-1, 0]])  # CW rotation

                perp_dir = R @ a / (norm_a + 1e-6)         # Orbiting direction
                dir_radial = a / (norm_a + 1e-6)           # Pushing direction

                # Compute pushing and orbiting velocity components
                pushing_component = ALPHA * dX_dir / dist
                orbiting_component = 4.5 * perp_dir + ALPHA * dir_radial * (1 - norm_a / LAMBDA)
                
                # Blend components according to gamma (distance) and beta (angle)
                dX = (1 - gamma) * pushing_component + gamma * orbiting_component * (1 - beta)
            else:
                # No target selected → return toward goal if far
                dist_goal = np.linalg.norm(herder_pos - goal)
                if dist_goal >= RG:
                    dX = -V_H * (herder_pos - goal) / dist_goal
                else:
                    dX = np.zeros(2)

    else:
        # === TARGET behavior ===
        pose_t = T.get(idx)
        if pose_t is None:
            return None

        # Extract position and yaw of the target
        xt, yt = pose_t.position.x, pose_t.position.y
        target_pos = np.array([xt, yt])
        q = pose_t.orientation 
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) 

        if not H:
            # No herders → no movement
            dX = np.zeros(2)
        else:
            # Repel from herders within LAMBDA
            H_array = np.array([[h.position.x, h.position.y] for h in H.values()])
            d_TH = np.linalg.norm(H_array - target_pos, axis=1)
            idx_near = np.where(d_TH <= LAMBDA)[0]

            repulsion = np.zeros(2)
            for i in idx_near:
                diff = target_pos - H_array[i]
                dist_th = np.linalg.norm(diff)
                if dist_th > 1e-2:  # Avoid division by zero
                    repulsion += BETA * diff
            dX = repulsion

    # === Global collision avoidance (all robots) ===
    if is_herder:
        current_pos = np.array([H[idx].position.x, H[idx].position.y])
    else:
        current_pos = np.array([T[idx].position.x, T[idx].position.y])

    repulsion_global = np.zeros(2)
    for pos_other in positions:
        diff = current_pos - pos_other
        dist = np.linalg.norm(diff)
        if 1e-3 < dist < R_REP:
            strength = K_REP * (1.0 / dist**p - 1.0 / R_REP**p)
            repulsion_global += strength * (diff / dist)
    dX += repulsion_global

    # === Convert desired displacement into Twist command ===
    dist = np.linalg.norm(dX)
    angle_to_goal = math.atan2(dX[1], dX[0])
    angle_error = angle_to_goal - yaw

    # Normalize angle to [-pi, pi]
    while angle_error > math.pi:
        angle_error -= 2 * math.pi
    while angle_error < -math.pi:
        angle_error += 2 * math.pi

    Kp_ang = 1.5  # Angular speed gain
    Kp_lin = 0.5  # Linear speed gain
    
    vel_threshold = 1e-3 # Min linear speed considered as movement

    cmd = Twist()

    if dist > vel_threshold:  # If movement is needed
        cmd.angular.z = Kp_ang * angle_error
        cmd.linear.x = Kp_lin * dist * math.exp(-5 * abs(angle_error))  # Soft stop if rotated
    else:
        cmd.angular.z = 0.0
        cmd.linear.x = 0.0
        
    if is_herder == 0:
        # Inject noise into target motion
        v = D * np.random.normal(0, sigma_v)
        omega = np.random.normal(mu_w, sigma_w)
        cmd.linear.x += v
        cmd.angular.z += omega

    # === Saturation limits ===
    cmd.linear.x = max(min(cmd.linear.x, V_H), -V_H)
    cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)

    return cmd


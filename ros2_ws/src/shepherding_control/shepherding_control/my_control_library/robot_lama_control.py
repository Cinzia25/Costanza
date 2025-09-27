from geometry_msgs.msg import Twist
import math
import numpy as np

# === Constants and parameters ===
TURTLEBOT_RADIUS = 0.15     
OSOYOO_RADIUS = 0.1         
GOAL_X = 0.0                
GOAL_Y = 0.0                
FACTOR = 1/10             
XI = 200.0*FACTOR + TURTLEBOT_RADIUS + TURTLEBOT_RADIUS  
LAMBDA = 2.5*FACTOR + OSOYOO_RADIUS + TURTLEBOT_RADIUS   
LAMBDA_DELTA = 2.5*0.5*FACTOR+ OSOYOO_RADIUS + TURTLEBOT_RADIUS  
V_H = 0.3    
RG = 20*FACTOR              
BETA = 3                    
ALPHA = 3                   
THRESHOLD = 0.5             
EPSILON = 0.2               
R_REP = 2*TURTLEBOT_RADIUS + 0.1  
K_REP = 1.0                 
D = 0                       
sigma_v = 1                 
sigma_w = 1                 
mu_w = 0.0                  
p = 2                       

def compute_cmd(idx, H, T, logger, is_herder):
    # Build combined array of all robot positions (herders + targets)
    positions = []
    for h in H.values():
        if h is not None:
            positions.append([h[0], h[1]])
    for t in T.values():
        if t is not None:
            positions.append([t[0], t[1]])
    positions = np.array(positions)

    if is_herder:
        pose_h = H.get(idx)
        if pose_h is None:
            return None

        # QTM tuple: (x, y, yaw)
        xh, yh, yaw = pose_h
        herder_pos = np.array([xh, yh])
        goal = np.array([GOAL_X, GOAL_Y])
        
        if not T:
            dX = np.zeros(2)
        else:
            T_array = np.array([[t[0], t[1]] for t in T.values()])
            H_array = np.array([[h[0], h[1]] for h in H.values()])
            
            d_HT = np.linalg.norm(T_array - herder_pos, axis=1)
            idx_visible = np.where(d_HT <= XI)[0]
            selected_target = None
            max_dist_from_goal = -np.inf

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
                a = herder_pos - selected_target
                b = selected_target - goal
                norm_a = np.linalg.norm(a)
                norm_b = np.linalg.norm(b)
                direction = b / norm_b if norm_b > 1e-6 else np.zeros(2)

                desired_pos = selected_target + LAMBDA_DELTA * direction
                dX_dir = desired_pos - herder_pos
                dist = np.linalg.norm(dX_dir) + 1e-6  

                b = desired_pos - selected_target
                norm_b = np.linalg.norm(b)
                cos_alpha = np.dot(a, b) / (norm_a * norm_b + 1e-6)
                cos_threshold = math.cos(math.radians(10))
                cos_orbit = math.cos(math.radians(45))

                if norm_a >= LAMBDA + THRESHOLD:
                    gamma = 0.0
                elif norm_a <= LAMBDA_DELTA:
                    gamma = 1.0
                else:
                    gamma = (LAMBDA + THRESHOLD - norm_a) / (LAMBDA + THRESHOLD - LAMBDA_DELTA)

                if cos_alpha >= cos_threshold:
                    beta = 1.0
                elif cos_alpha <= cos_orbit:
                    beta = 0.0
                else:
                    beta = (cos_alpha - cos_orbit) / (cos_threshold - cos_orbit)
                    
                vett = desired_pos - selected_target
                cross = a[0] * vett[1] - a[1] * vett[0]
                if cross > 0:
                    R = np.array([[0, -1], [1, 0]])
                else:
                    R = np.array([[0, 1], [-1, 0]])

                perp_dir = R @ a / (norm_a + 1e-6)
                dir_radial = a / (norm_a + 1e-6)

                pushing_component = ALPHA * dX_dir / dist
                orbiting_component = 4.5 * perp_dir + ALPHA * dir_radial * (1 - norm_a / LAMBDA)
                
                dX = (1 - gamma) * pushing_component + gamma * orbiting_component * (1 - beta)
            else:
                dist_goal = np.linalg.norm(herder_pos - goal)
                if dist_goal >= RG:
                    dX = -V_H * (herder_pos - goal) / dist_goal
                else:
                    dX = np.zeros(2)

    else:
        pose_t = T.get(idx)
        if pose_t is None:
            return None

        xt, yt, yaw = pose_t
        target_pos = np.array([xt, yt])
        
        if not H:
            dX = np.zeros(2)
        else:
            H_array = np.array([[h[0], h[1]] for h in H.values()])
            d_TH = np.linalg.norm(H_array - target_pos, axis=1)
            idx_near = np.where(d_TH <= LAMBDA)[0]

            repulsion = np.zeros(2)
            for i in idx_near:
                diff = target_pos - H_array[i]
                dist_th = np.linalg.norm(diff)
                if dist_th > 1e-2:
                    repulsion += BETA * diff
            dX = repulsion

    if is_herder:
        current_pos = np.array([H[idx][0], H[idx][1]])
    else:
        current_pos = np.array([T[idx][0], T[idx][1]])

    repulsion_global = np.zeros(2)
    for pos_other in positions:
        diff = current_pos - pos_other
        dist = np.linalg.norm(diff)
        if 1e-3 < dist < R_REP:
            strength = K_REP * (1.0 / dist**p - 1.0 / R_REP**p)
            repulsion_global += strength * (diff / dist)
    
    dX += repulsion_global

    if is_herder:
        projection_distance = 0.3
    else:
        projection_distance = 0.05

    cs = np.cos(yaw)
    ss = np.sin(yaw)

    linear_vel = cs * dX[0] + ss * dX[1]
    angular_vel = (1 / projection_distance) * (-ss*dX[0] + cs*dX[1])
    
    if is_herder:
        d = TURTLEBOT_RADIUS*angular_vel
        den1 = abs(linear_vel + d)
        den2 = abs(linear_vel - d)
        s = 1.0
        if den1 > 0:
            s = min(s, V_H/den1)
        if den2 > 0:
            s = min(s, V_H/den2)
        s = max(0.0, min(1.0, s))
        linear_vel = s * linear_vel
        angular_vel = s * angular_vel

    cmd = Twist()
    cmd.linear.x = linear_vel
    cmd.angular.z = angular_vel

    print('is herder:', is_herder, 'linear_vel:', linear_vel, 'angular_vel:', angular_vel)

    return cmd

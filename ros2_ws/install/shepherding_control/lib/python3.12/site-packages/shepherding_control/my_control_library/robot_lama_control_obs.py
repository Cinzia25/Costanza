from shapely.geometry import Point, Polygon
from shapely import affinity
from geometry_msgs.msg import Twist
import math
import numpy as np

# === Constants and parameters (meters / radians) ===
TURTLEBOT_RADIUS = 0.15            # Herder (TurtleBot) radius
OSOYOO_RADIUS = 0.10               # Target (Osoyoo) radius
GOAL_X = 0.0
GOAL_Y = 0.0
FACTOR = 1/10

XI = 200.0 * FACTOR + TURTLEBOT_RADIUS + TURTLEBOT_RADIUS        # Herder sensing range
LAMBDA = 2.5 * FACTOR + OSOYOO_RADIUS + TURTLEBOT_RADIUS         # Target "push/orbit" range
LAMBDA_DELTA = 2.5 * 0.5 * FACTOR + OSOYOO_RADIUS + TURTLEBOT_RADIUS
V_H = 0.3                                                        # Max linear speed (herders)
RG = 10 * FACTOR                                                 # Goal region radius
BETA = 3                                                         # Target repulsion gain
ALPHA = 3                                                        # Pushing gain
THRESHOLD = 0.5
EPSILON = 0.2
R_REP = 0.1 + 2 * TURTLEBOT_RADIUS                               # Global inter-robot repulsion radius
p = 2                                                            # Repulsion exponent
K_REP = 1                                                        # Global inter-robot repulsion gain

# Single polygonal obstacle parameters (rotated box)
lx = 2.00                         # Rectangle width  (m)
ly = 0.30                         # Rectangle height (m)
O_REP = TURTLEBOT_RADIUS + 0.05   # Obstacle repulsion radius (m)
K_O = 3.0                         # Obstacle repulsion gain
min_value = 0.0                   # (kept for compatibility)
max_value = 0.9                   # (kept for compatibility)

def rotated_box(center, size, angle_rad):
    """Create a rotated rectangular Polygon centered at 'center', rotated by angle_rad (radians)."""
    cx, cy = center
    Lx, Ly = size
    box = Polygon([[-Lx/2, -Ly/2], [Lx/2, -Ly/2], [Lx/2, Ly/2], [-Lx/2, Ly/2]])
    box = affinity.rotate(box, angle_rad, origin=(0, 0), use_radians=True)
    box = affinity.translate(box, cx, cy)
    return box

def compute_cmd(idx, H, T, O, logger, is_herder):
    """
    QTM-compliant version.
    Inputs:
      H, T, O: dict[int] -> (x, y, yaw)   # meters, radians
      is_herder: bool
    Returns:
      geometry_msgs.msg.Twist
    """
    # Build combined array of all robot positions (herders + targets) for global collision term
    positions = []
    for h in H.values():
        if h is not None:
            positions.append([h[0], h[1]])
    for t in T.values():
        if t is not None:
            positions.append([t[0], t[1]])
    positions = np.array(positions) if len(positions) else np.zeros((0, 2))

    # === Multiple rotated rectangular obstacles ===
    obs_list = []
    obs_centers = []
    for pose_o in O.values():
        if pose_o is None:
            continue
        xc, yc, theta_rad = pose_o
        theta_rad = - theta_rad
        obs_i = rotated_box(center=(xc, yc), size=(lx, ly), angle_rad=theta_rad)
        obs_list.append(obs_i)
        obs_centers.append(np.array([xc, yc]))

    if is_herder:
        # === HERDER behavior ===
        pose_h = H.get(idx)
        if pose_h is None:
            return None

        xh, yh, yaw = pose_h
        herder_pos = np.array([xh, yh])
        point_h = Point(xh, yh)
        goal = np.array([GOAL_X, GOAL_Y])

        if not T:
            dX = np.zeros(2)
            G_point = goal
        else:
            T_array = np.array([[t[0], t[1]] for t in T.values()])
            H_array = np.array([[h[0], h[1]] for h in H.values()])

            d_HT = np.linalg.norm(T_array - herder_pos, axis=1)
            idx_visible = np.where(d_HT <= XI)[0]
            selected_target = None
            max_dist_from_goal = -np.inf

            # Target selection: farthest from goal among those for which current herder is the closest within XI
            for i_vis in idx_visible:
                target = T_array[i_vis]
                d_target_to_H = np.linalg.norm(H_array - target, axis=1)
                idx_H_near = np.where(d_target_to_H <= XI)[0]
                min_dist = np.min(d_target_to_H[idx_H_near]) if idx_H_near.size > 0 else np.min(d_target_to_H)
                dist_current = np.linalg.norm(herder_pos - target)
                if abs(dist_current - min_dist) < 1e-3:
                    d_goal = np.linalg.norm(target - goal)
                    if d_goal > max_dist_from_goal:
                        selected_target = target
                        selected_target_id = list(T.keys())[i_vis]
                        max_dist_from_goal = d_goal

            if selected_target is not None:
                point_t = Point(selected_target[0], selected_target[1])
                G_point = selected_target

                a = herder_pos - selected_target
                b_vec = selected_target - goal
                norm_a = np.linalg.norm(a)
                norm_b = np.linalg.norm(b_vec)
                direction = b_vec / norm_b if norm_b > 1e-6 else np.zeros(2)

                desired_pos = selected_target + LAMBDA_DELTA * direction
                dX_dir = desired_pos - herder_pos
                dist = np.linalg.norm(dX_dir) + 1e-6

                # If target is close to ANY obstacle, steer tangentially wrt the closest one
                flag = 0
                if len(obs_list) > 0:
                    dists_t = [point_t.distance(ob) for ob in obs_list]
                    j_min = int(np.argmin(dists_t)) if len(dists_t) > 0 else None
                    if j_min is not None and dists_t[j_min] < O_REP + EPSILON/2:
                        flag = 1
                        obs_sel = obs_list[j_min]
                        obs_center = obs_centers[j_min]
                        closest_point_t = obs_sel.exterior.interpolate(obs_sel.exterior.project(point_t))
                        diff = selected_target - np.array(closest_point_t.coords[0])
                        v1 = selected_target - obs_center
                        v2 = goal - obs_center
                        cross = v1[0] * v2[1] - v1[1] * v2[0]
                        R_obs_t = np.array([[0, -1], [1, 0]]) if cross < 0 else np.array([[0, 1], [-1, 0]])
                        direction = R_obs_t @ diff / (np.linalg.norm(diff) + 1e-6)
                        desired_pos = selected_target + LAMBDA_DELTA * direction
                        dX_dir = desired_pos - herder_pos
                        dist = np.linalg.norm(dX_dir) + 1e-6

                # Blending terms
                b_vec2 = desired_pos - selected_target
                norm_b2 = np.linalg.norm(b_vec2)
                cos_alpha = np.dot(a, b_vec2) / (norm_a * norm_b2 + 1e-6)
                cos_threshold = math.cos(math.radians(10))
                cos_orbit = math.cos(math.radians(45))

                # Distance-based blending gamma
                if norm_a >= LAMBDA + THRESHOLD:
                    gamma = 0.0
                elif norm_a <= LAMBDA_DELTA:
                    gamma = 1.0
                else:
                    gamma = (LAMBDA + THRESHOLD - norm_a) / (LAMBDA + THRESHOLD - LAMBDA_DELTA)

                # Angle-based blending beta
                if cos_alpha >= cos_threshold:
                    beta = 1.0
                elif cos_alpha <= cos_orbit:
                    beta = 0.0
                else:
                    beta = (cos_alpha - cos_orbit) / (cos_threshold - cos_orbit)

                # Orbit direction
                vett = desired_pos - selected_target
                cross = a[0] * vett[1] - a[1] * vett[0]
                R = np.array([[0, -1], [1, 0]]) if cross > 0 else np.array([[0, 1], [-1, 0]])

                if flag == 1:
                    # Optional correction to orbit sense if obstacle-driven direction conflicts
                    # (use same 'diff' from above scope safely only when flag==1)
                    vett2 = -(selected_target - np.array(closest_point_t.coords[0]))
                    if np.dot(vett2, a) > 0 and cos_alpha < 0:
                        R = -R

                perp_dir = R @ a / (norm_a + 1e-6)
                dir_radial = a / (norm_a + 1e-6)

                # Pushing and orbiting components
                pushing_component = ALPHA * dX_dir / dist
                orbiting_component = 4.5 * perp_dir + ALPHA * dir_radial * (1 - norm_a / LAMBDA)

            else:
                # No target selected → move toward goal if far away
                G_point = goal
                dist_goal = np.linalg.norm(herder_pos - goal)
                if dist_goal >= RG:
                    dX = -V_H * (herder_pos - goal) / (dist_goal + 1e-6)
                else:
                    dX = np.zeros(2)

        # === Obstacle avoidance blending for herder (sum over obstacles) ===
        repulsive_component_total = np.zeros(2)
        for obs, obs_center in zip(obs_list, obs_centers):
            v1 = herder_pos - obs_center
            v2 = (G_point if 'G_point' in locals() else goal) - obs_center
            cross = v1[0]*v2[1] - v1[1]*v2[0]
            R_obs_h = np.array([[0, -1], [1, 0]]) if cross > 0 else np.array([[0, 1], [-1, 0]])

            ph = Point(herder_pos[0], herder_pos[1])
            dist = max(ph.distance(obs), 1e-6)
            closest_point = obs.exterior.interpolate(obs.exterior.project(ph))
            diff = herder_pos - np.array(closest_point.coords[0])

            raw = (1.0 / (dist**p)) - (1.0 / ((O_REP + EPSILON)**p))
            strength = K_O * max(raw, 0.0)

            VALUE = 0.3   # your radial/tangential blend weight
            sigma_obs = 1.0
            repulsive_component = (
                VALUE * strength * (diff / dist)
                + (1 - VALUE) * strength * (R_obs_h @ (diff / dist))
            )
            repulsive_component_total += repulsive_component

        if 'selected_target' in locals() and selected_target is not None:
            dX = (1 - gamma) * pushing_component + (1 - beta) * gamma * orbiting_component + sigma_obs * repulsive_component_total
        else:
            dX = dX + sigma_obs * repulsive_component_total

    else:
        # === TARGET behavior ===
        pose_t = T.get(idx)
        if pose_t is None:
            return None

        xt, yt, yaw = pose_t
        target_pos = np.array([xt, yt])
        point_t = Point(xt, yt)

        if not H:
            dX = np.zeros(2)
        else:
            # Repel from herders within LAMBDA
            H_array = np.array([[h[0], h[1]] for h in H.values()])
            d_TH = np.linalg.norm(H_array - target_pos, axis=1)
            idx_near = np.where(d_TH <= LAMBDA)[0]
            repulsion = np.zeros(2)
            for i_near in idx_near:
                diff = target_pos - H_array[i_near]
                dist_th = np.linalg.norm(diff)
                if dist_th > 1e-2:
                    repulsion += BETA * diff
            dX = repulsion

        # Obstacle repulsion for target (sum over obstacles)
        for obs in obs_list:
            dist = point_t.distance(obs)
            if dist < O_REP:
                dist = max(dist, 1e-6)
                closest_point = obs.exterior.interpolate(obs.exterior.project(point_t))
                diff = target_pos - np.array(closest_point.coords[0])
                raw = (1.0 / (dist**p)) - (1.0 / (O_REP**p))
                strength = K_O * max(raw, 0.0)
                dX += strength * (diff / dist)

    # === Global collision avoidance (all robots) ===
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

    # === Unicycle projection + saturation ===
    yaw = H[idx][2] if is_herder else T[idx][2]
    projection_distance = 0.15 if is_herder else 0.05

    cs, ss = np.cos(yaw), np.sin(yaw)
    linear_vel = cs * dX[0] + ss * dX[1]
    angular_vel = (1.0 / projection_distance) * (-ss * dX[0] + cs * dX[1])

    if is_herder:
        d = TURTLEBOT_RADIUS * angular_vel
        den1 = abs(linear_vel + d)
        den2 = abs(linear_vel - d)
        s = 1.0
        if den1 > 0:
            s = min(s, V_H / den1)
        if den2 > 0:
            s = min(s, V_H / den2)
        s = max(0.0, min(1.0, s))
        linear_vel *= s
        angular_vel *= s

    cmd = Twist()
    cmd.linear.x = float(linear_vel)
    cmd.angular.z = float(angular_vel)

    print('is herder:', is_herder, 'linear_vel:', linear_vel, 'angular_vel:', angular_vel)
    return cmd

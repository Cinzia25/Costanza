import numpy as np
import torch

from shepherding_control.my_control_library.utils import select_targets, select_targets_learning2, create_single_integrator_barrier_certificate_with_boundary, create_si_to_uni_dynamics, create_si_to_uni_mapping
from shepherding_control.my_control_library.actor_critic_continuous import ActorCriticContinuous
from shepherding_control.my_control_library.actor_critic_mappo_discrete import ActorCriticMAPPODiscrete

use_learned_target_selection = False

# Arena parameters
scaling_rate = 2 / 40 / 1.6
rho_g = 5 * scaling_rate    # Radius of goal region

dt = 0.02

alpha = 0.1
damping = 60
diffusion = 0.0

# Herder dynamics
herder_max_vel = 8  # Maximum velocity (in scaled units)

# Barrier certificate to avoid inter-agent and boundary collisions
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary(
    safety_radius=0.20, boundary_points=np.array([-16, 16, -10, 10])
)

# Conversion between dynamics
_, uni_to_si_states = create_si_to_uni_mapping(projection_distance=0.05, angular_velocity_limit=1)
si_to_uni_dyn = create_si_to_uni_dynamics(angular_velocity_limit=1)

# === Load Actor-Critic Models ===
network_config = {
    'hidden_sizes':  [64, 64, 64, 64, 64],
    'activation': "relu"
}
network_config_HL = {
    'hidden_sizes':  [256, 128],
    'activation': "relu"
}
model = ActorCriticContinuous(network_config=network_config, max_speed=0.2)
model_HL = ActorCriticMAPPODiscrete(network_config=network_config_HL)


def learning_controller(H: np.ndarray, T: np.ndarray, logger):
#   H: shape (n_herder, 3) rows are [x, y, yaw]
#   T: shape (n_target, 3) rows are [x, y, yaw]
#   returns:
#     herder_cmds: shape (n_herder, 2) rows are [v, omega]
#     target_cmds: shape (n_target, 2) rows are [v, omega]

    num_herders = np.shape(H)[0]
    num_targets = np.shape(T)[0]

    num_agents = num_targets + num_herders

    target_vel = np.zeros((num_targets, 2))
    herder_vel = np.zeros((num_herders, 2))

    
    # Get current agent positions
    target_pos = T[:, :2]
    herder_pos = H[:, :2]

    x_si = np.vstack((target_pos, herder_pos)).T
    x = np.vstack((T, H)).T

    

    # === TARGET DYNAMICS ===
    noise = np.random.normal(0, 1, size=(num_targets, 2))

    # Calculate the relative positions
    relative_positions = target_pos[:, np.newaxis, :] - herder_pos[np.newaxis, :, :]  # Shape (N, M, 2)

    

    # Compute the Euclidean distances between herders and targets
    distances = np.linalg.norm(relative_positions, axis=2)  # Shape (N, M)
    # Compute the force kernel using power-law repulsion
    kernel = alpha / (distances ** 3)
    repulsion = np.sum(kernel[:, :, np.newaxis] * relative_positions, axis=1)


    target_vel += ( - damping * target_vel + repulsion) * dt + diffusion * noise * np.sqrt(dt)
    # === HERDER DYNAMICS ===

    # if step % 1 == 0:
    if not use_learned_target_selection:
        selected_targets = select_targets(herder_pos, target_pos, num_herders)
    else:
        selected_targets = select_targets_learning2(herder_pos, target_pos, num_herders, num_targets, model_HL)
        pass

    selected_targets = np.array(selected_targets)

    

    # Prepare inputs for the ActorCritic model
    valid_mask = selected_targets < num_targets
    target_positions = np.zeros((num_herders, 2), dtype=np.float32)
    target_positions[valid_mask] = target_pos[selected_targets[valid_mask]]
    relative_positions = target_positions - herder_pos
    batch_inputs = np.hstack((relative_positions, target_positions)).astype(np.float32)
    batch_tensor = torch.tensor(batch_inputs * (1 / 50) * 20 * 0.1)

    

    actions = model.get_action(batch_tensor).cpu().numpy()
    herder_vel = actions  # Scale back velocity

    # === COMBINE VELOCITIES ===
    dxi = np.vstack((target_vel, herder_vel)).T
    dxi = si_barrier_cert(dxi, x_si)
    dxu = si_to_uni_dyn(dxi, x)

    dxu = dxu.T


    target_uni_vel = dxu[:num_targets, :]
    herder_uni_vel = dxu[num_targets:num_agents, :]

    print("herder.shape:", herder_uni_vel.shape)
    print("target.shape:", target_uni_vel.shape)

    return herder_uni_vel, target_uni_vel


import numpy as np
import torch
import torch.nn as nn
from torch.distributions.normal import Normal
import os

import scipy

from shepherding_control.my_control_library.actor_critic_base import ActorCriticBase, layer_init


class ActorCriticContinuous(ActorCriticBase):
    """
    PPO actor-critic for continuous actions.

    * With a normal Gym env the action bounds come from the env’s Box space.
    * With `driving=True` the network outputs (speed, heading) in
      [0, max_speed] × [−π, π].

    For PPO_polar simply build the env’s action_space as
    Box([0,-π], [V_MAX, π]) – this class will detect those bounds.
    """

    def __init__(self, network_config, *, max_speed=12.0):

        # ---------------------------------------------------------- #
        # Infer observation / action dimensions and bounds
        # ---------------------------------------------------------- #

        obs_shape, action_shape = 4, 2
        self.action_min = torch.tensor(np.array([-max_speed, -max_speed]), dtype=torch.float32)
        self.action_max = -self.action_min


        super().__init__(obs_shape, action_shape, network_config)

        # ---------------------------------------------------------- #
        # Network heads
        # ---------------------------------------------------------- #
        last_hidden = network_config["hidden_sizes"][-1]

        self.critic = nn.Sequential(
            self.critic_base_net,
            layer_init(nn.Linear(last_hidden, 1), std=1.0)
        )

        self.actor_mean = nn.Sequential(
            self.actor_base_net,
            layer_init(nn.Linear(last_hidden, action_shape), std=0.01),
            nn.Tanh()
        )

        self.actor_log_std = nn.Parameter(torch.zeros(1, action_shape))

        # Get the current directory
        current_dir = os.path.dirname(os.path.abspath(__file__))

        # Load the .npy file instead of the .pt file
        pt_path = os.path.join(current_dir, 'PPO_LL_agent.pt')

        pt_path = f'/home/stefano/dev/Costanza/ros2_ws/src/shepherding_control/shepherding_control/my_control_library/PPO_LL_agent.pt'

        # Load the trained agent
        self.load_state_dict(
            torch.load(pt_path, map_location="cpu"))


    # ------------------------- forward helpers -------------------------- #
    def get_value(self, x):
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        mean = self.scale_action(self.actor_mean(x))
        std  = torch.exp(self.actor_log_std.expand_as(mean))
        dist = Normal(mean, std)

        if action is None:
            action = dist.sample()

        log_prob = dist.log_prob(action).sum(-1)
        entropy  = dist.entropy().sum(-1)
        value    = self.get_value(x)
        return action, log_prob, entropy, value

    @torch.jit.export
    def get_action(self, x):
        with torch.no_grad():
            return self.scale_action(self.actor_mean(x))

    @torch.jit.export
    # map tanh output (-1,1) → physical bounds
    def scale_action(self, a):
        half_range = (self.action_max - self.action_min) / 2.0
        center     = (self.action_max + self.action_min) / 2.0
        return a * half_range + center

    @torch.jit.ignore
    # make sure bounds move with the model to GPU if needed
    def to(self, device):
        self.action_min = self.action_min.to(device)
        self.action_max = self.action_max.to(device)
        return super().to(device)
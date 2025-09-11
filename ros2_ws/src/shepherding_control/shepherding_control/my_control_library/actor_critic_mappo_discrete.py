import torch
import os
import torch.nn as nn
import numpy as np
from torch.distributions.categorical import Categorical

from shepherding_control.my_control_library.actor_critic_base import ActorCriticBase, layer_init


class ActorCriticMAPPODiscrete(ActorCriticBase):
    def __init__(self, network_config):
        # Assuming observations are per agent
        obs_shape = 14
        action_shape = 5

        super(ActorCriticMAPPODiscrete, self).__init__(obs_shape, action_shape, network_config)

        # Build actor and critic heads
        last_hidden_size = network_config['hidden_sizes'][-1]

        # Critic head
        self.critic = nn.Sequential(
            self.critic_base_net,
            layer_init(nn.Linear(last_hidden_size, 1), std=1.0)
        )

        # Actor head
        self.actor = nn.Sequential(
            self.actor_base_net,
            layer_init(nn.Linear(last_hidden_size, action_shape), std=0.01)
        )

        self.num_agents = 2

        # Get the current directory
        current_dir = os.path.dirname(os.path.abspath(__file__))

        # Load the .npy file instead of the .pt file
        pt_path = os.path.join(current_dir, 'PPO_HL_agent.pt')

        pt_path = f'/home/stefano/dev/Costanza/ros2_ws/src/shepherding_control/shepherding_control/my_control_library/PPO_HL_agent.pt'

        # Load the trained agent
        self.load_state_dict(
            torch.load(pt_path, map_location="cpu"))

    def get_value(self, x):
        # x shape: (batch_size, obs_shape)
        return self.critic(x).squeeze(-1)  # Output shape: (batch_size,)

    def get_action_and_value(self, x, action=None):
        # x shape: (batch_size, obs_shape)
        logits = self.actor(x)  # Output shape: (batch_size, action_shape)
        dist = Categorical(logits=logits)  # Assuming single categorical per agent
        if action is None:
            action = dist.sample()
        logprob = dist.log_prob(action)
        entropy = dist.entropy()
        value = self.get_value(x)
        return action, logprob, entropy, value

    @torch.jit.export
    def get_action(self, x):
        logits = self.actor(x)
        return logits.argmax(dim=-1)

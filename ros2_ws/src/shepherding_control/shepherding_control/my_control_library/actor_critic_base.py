import numpy as np
import torch
import torch.nn as nn


def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


def get_activation(activation_str):
    activation_str = activation_str.lower()
    if activation_str == 'relu':
        return nn.ReLU
    elif activation_str == 'tanh':
        return nn.Tanh
    elif activation_str == 'leakyrelu':
        return nn.LeakyReLU
    else:
        raise ValueError(f"Unsupported activation function: {activation_str}")


def build_network(input_size, hidden_sizes, activation):
    layers = []
    activation_function = get_activation(activation)
    last_size = input_size
    for size in hidden_sizes:
        layers.append(layer_init(nn.Linear(last_size, size)))
        layers.append(activation_function())
        last_size = size
    return nn.Sequential(*layers)


class ActorCriticBase(nn.Module):
    def __init__(self, obs_shape, action_shape, network_config):
        super(ActorCriticBase, self).__init__()
        self.obs_shape = obs_shape
        self.action_shape = action_shape
        self.network_config = network_config

        # Build identical feature extractor for both actor and critic
        self.actor_base_net = build_network(obs_shape, network_config['hidden_sizes'], network_config['activation'])
        self.critic_base_net = build_network(obs_shape, network_config['hidden_sizes'], network_config['activation'])

    def get_value(self, x):
        raise NotImplementedError

    def get_action_and_value(self, x, action=None):
        raise NotImplementedError

    def get_action(self, x):
        raise NotImplementedError

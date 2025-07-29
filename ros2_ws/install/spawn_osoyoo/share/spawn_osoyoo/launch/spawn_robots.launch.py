from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
import numpy as np
import random

def create_spawn_node(model_path, robot_name, x, y, z):
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', model_path,
            '-name', robot_name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ]
    )

def create_bridge_node(robot_name):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
    )

def create_target(model_path, robot_name, x, y, z):
    spawn_node = create_spawn_node(model_path, robot_name, x, y, z)
    bridge_node = create_bridge_node(robot_name)

    ros_nodes = GroupAction([
        PushRosNamespace(robot_name),
        # Aggiungi altri nodi nel namespace se necessario
        bridge_node
    ])

    return [spawn_node, ros_nodes]

def generate_launch_description():
    ld = LaunchDescription()

    robot_count = 10
    model_path = '/root/data/ros2_ws/models/osoyoo/model.sdf'
    positions = []

    k = 0
    while k < robot_count:
        x_pos = random.uniform(-5, 5)
        y_pos = random.uniform(-5, 5)
        z_pos = 0.07
        new_point = np.array([x_pos, y_pos])

        if not positions:
            valid = True
        else:
            dists = np.linalg.norm(np.array(positions) - new_point, axis=1)
            valid = np.all(dists > 1.0)

        if valid:
            robot_name = f'target{k+1}'
            positions.append(new_point)
            nodes = create_target(model_path, robot_name, x_pos, y_pos, z_pos)
            for n in nodes:
                ld.add_action(n)
            k += 1  # Solo incremento se il robot è stato accettato

    return ld


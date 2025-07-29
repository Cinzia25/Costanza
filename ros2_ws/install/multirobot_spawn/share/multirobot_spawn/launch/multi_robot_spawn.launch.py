from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
import numpy as np
import random
import os
import xacro
import tempfile
import subprocess

def generate_custom_sdf(original_path, robot_name):
    with open(original_path, 'r') as f:
        sdf_content = f.read()

    sdf_content = sdf_content.replace(
        '<topic>cmd_vel</topic>',
        f'<topic>/model/{robot_name}/cmd_vel</topic>'
    ).replace(
        '<odom_topic>odom</odom_topic>',
        f'<odom_topic>/model/{robot_name}/odom</odom_topic>'
    ).replace(
        '<tf_topic>tf</tf_topic>',
        f'<tf_topic>/model/{robot_name}/tf</tf_topic>'
    )

    

    temp_sdf_path = f'/tmp/{robot_name}.sdf'
    with open(temp_sdf_path, 'w') as f:
        f.write(sdf_content)

    return temp_sdf_path


def create_spawn_node(model_path, robot_name, x, y, z):
    args = ['-name', robot_name, '-x', str(x), '-y', str(y), '-z', str(z)]
    args += ['-file', model_path]  # sempre passiamo il file
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=args,
    )
    
def create_bridge_node(robot_name):
    bridges = []

    bridges = [
        f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        f'/model/{robot_name}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        f'/model/{robot_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    ]

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridges,
    )

def create_osoyoo(index, model_path, x, y, z):
    robot_name = f'target{index+1}'
    custom_model_path = generate_custom_sdf(model_path, robot_name)
    spawn_node = create_spawn_node(custom_model_path, robot_name, x, y, z)
    bridge_node = create_bridge_node(robot_name)

    return [spawn_node, GroupAction([
        PushRosNamespace(robot_name),
        bridge_node
    ])]
    
    
def create_turtlebot(index, x, y, z):
    robot_name = f'herder{index+1}'
    xacro_path = '/root/data/ros2_ws/src/ionic_demo/ionic_demo/models/ionic_tb4.urdf.xacro'

    xacro_cmd = [
        'xacro', xacro_path,
        f'gz_namespace:={robot_name}'
    ]
    urdf_xml = subprocess.check_output(xacro_cmd).decode('utf-8')

    urdf_path = f'/tmp/{robot_name}.urdf'

    with open(urdf_path, 'w') as f:
        f.write(urdf_xml)
        
        
    # robot_state_publisher con prefisso tf
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='node_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': urdf_xml},
        ]
    )
    

    spawn_node = create_spawn_node(urdf_path, robot_name, x, y, z)
    bridge_node = create_bridge_node(robot_name)

    return [spawn_node, GroupAction([
        PushRosNamespace(robot_name),
        robot_state_pub,
        bridge_node
    ])]

def generate_launch_description():
    ld = LaunchDescription()

    osoyoo_count = 10
    turtlebot_count = 5
    model_path_osoyoo = '/root/data/ros2_ws/models/osoyoo/model.sdf'
    
    positions = []

    def is_valid(x, y):
        pt = np.array([x, y])
        if not positions:
            return True
        dists = np.linalg.norm(np.array(positions) - pt, axis=1)
        return np.all(dists > 1.5)

    # Spawn Osoyoo
    i = 0
    while i < osoyoo_count:
        x, y, z = random.uniform(-5, 5), random.uniform(-5, 5), 0.07
        if is_valid(x, y):
            positions.append([x, y])
            nodes = create_osoyoo(i, model_path_osoyoo, x, y, z)
            for node in nodes:
                ld.add_action(node)
            i += 1

    # Spawn TurtleBot4
    j = 0
    while j < turtlebot_count:
        x, y, z = random.uniform(-5, 5), random.uniform(-5, 5), 0.0
        if is_valid(x, y):
            positions.append([x, y])
            nodes = create_turtlebot(j, x, y, z)
            for node in nodes:
                ld.add_action(node)
            j += 1

    return ld

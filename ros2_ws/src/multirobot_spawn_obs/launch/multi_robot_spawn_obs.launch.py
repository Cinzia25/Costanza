from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from shapely.geometry import Point, Polygon
from shapely import affinity
import numpy as np
import random
import os
import xacro
import tempfile
import subprocess


def rotated_box(center, size, angle_rad):
    """
    Build a rotated rectangle (Shapely Polygon), centered at `center`,
    axis lengths `size=(lx, ly)`, rotated by `angle_deg` (NOTE: radians).
    """
    cx, cy = center
    lx, ly = size
    # Base rectangle centered at origin
    box = Polygon([[-lx/2, -ly/2], [lx/2, -ly/2], [lx/2, ly/2], [-lx/2, ly/2]])
    # Rotate around (0,0); use_radians=True means `angle_rad` must already be in radians
    box = affinity.rotate(box, angle_rad, origin=(0, 0), use_radians=True)
    # Translate to the requested center
    box = affinity.translate(box, cx, cy)
    return box
    
def is_valid(x, y, positions, rect):
    """
    Validate a spawn position:
    - Keep at least 1 meter from previously accepted positions.
    - Keep at least 0.3 m from the obstacle polygon `rect`.
    """
    pt = np.array([x, y])
    point = Point(x, y)

    # Minimum distance from other robots
    if positions:
        dists = np.linalg.norm(np.array(positions) - pt, axis=1)
        if not np.all(dists > 1):
            return False

    # Minimum distance from the obstacle
    if point.distance(rect) < 0.3:
        return False

    return True


def generate_custom_sdf(original_path, robot_name):
    """
    Read an SDF file and rewrite topics so each model has namespaced
    cmd_vel, odom, and tf topics under /model/{robot_name}/...
    Returns a path to a temp SDF written under /tmp.
    """
    with open(original_path, 'r') as f:
        sdf_content = f.read()

    # Replace generic topics with per-model topics
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

    # Write customized SDF to /tmp for ros_gz_sim create
    temp_sdf_path = f'/tmp/{robot_name}.sdf'
    with open(temp_sdf_path, 'w') as f:
        f.write(sdf_content)

    return temp_sdf_path


def create_spawn_node(model_path, robot_name, x, y, z):
    """
    Create a ros_gz_sim 'create' node to spawn a model from file at pose (x,y,z).
    """
    args = ['-name', robot_name, '-x', str(x), '-y', str(y), '-z', str(z)]
    args += ['-file', model_path]  # always supply file path
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=args,
    )
    
def create_bridge_node(robot_name):
    """
    Create a bridge node mapping per-model Gazebo topics to ROS 2 types.
    Assumes model-scoped Gazebo topics under /model/{robot_name}/...
    """
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
    """
    Spawn one Osoyoo 'target' model and bridge its topics.
    Also push a ROS namespace equal to the robot name.
    """
    robot_name = f'target{index+1}'
    custom_model_path = generate_custom_sdf(model_path, robot_name)
    spawn_node = create_spawn_node(custom_model_path, robot_name, x, y, z)
    bridge_node = create_bridge_node(robot_name)

    return [spawn_node, GroupAction([
        PushRosNamespace(robot_name),
        bridge_node
    ])]
    
    
def create_turtlebot(index, x, y, z):
    """
    Spawn one TurtleBot4 'herder' model from a xacro -> URDF pipeline,
    publish its TF via robot_state_publisher, and bridge topics.
    """
    robot_name = f'herder{index+1}'
    xacro_path = '/root/data/ros2_ws/src/ionic_demo/ionic_demo/models/ionic_tb4.urdf.xacro'
    turtlebot_model_path = '/root/data/ros2_ws/models/turtlebot4/nav2_minimal_tb4_description/urdf'

    # Generate URDF text from xacro with a model-specific gz namespace
    xacro_cmd = [
        'xacro', xacro_path,
        f'gz_namespace:={robot_name}',
        f'tb4_urdf_dir:={turtlebot_model_path}'
    ]
    urdf_xml = subprocess.check_output(xacro_cmd).decode('utf-8')
    
    urdf_path = f'/tmp/{robot_name}.urdf'
    with open(urdf_path, 'w') as f:
        f.write(urdf_xml)
        
    # TF publisher; uses model URDF and simulation time
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
    
    # Spawn the URDF as a model in Gazebo
    spawn_node = create_spawn_node(urdf_path, robot_name, x, y, z)
    bridge_node = create_bridge_node(robot_name)

    return [spawn_node, GroupAction([
        PushRosNamespace(robot_name),
        robot_state_pub,
        bridge_node
    ])]

def generate_launch_description():
    """
    Build a LaunchDescription that:
    - Creates one rotated rectangular obstacle (Shapely, used only for placement checks).
    - Randomly spawns M Osoyoo targets and N TurtleBot4 herders,
      enforcing min distance to obstacle and between robots.
    - Sets up per-model ROS<->Gazebo bridges and namespaces.
    """
    ld = LaunchDescription()

    osoyoo_count = 5
    turtlebot_count = 2
    model_path_osoyoo = '/root/data/ros2_ws/models/osoyoo/model.sdf'

    # Obstacle parameters (angle already in radians)
    xc = 2
    yc = 2
    lx = 1 
    ly = 2
    theta_rad = 0.7854
    
    rect = rotated_box(center=(xc, yc), size=(lx, ly), angle_rad=theta_rad)
    
    positions = []  # accepted (x,y) positions to enforce spacing

    # Spawn Osoyoo targets
    i = 0
    while i < osoyoo_count:
        x, y, z = random.uniform(-5, 5), random.uniform(-5, 5), 0.07
        # Deterministic alternative for debugging:
        # x, y, z = 4, 4, 0.07
        if is_valid(x, y, positions, rect):
            positions.append([x, y])
            nodes = create_osoyoo(i, model_path_osoyoo, x, y, z)
            for node in nodes:
                ld.add_action(node)
            i += 1

    # Spawn TurtleBot4 herders
    j = 0
    while j < turtlebot_count:
        x, y, z = random.uniform(-5, 5), random.uniform(-5, 5), 0.0
        # Deterministic alternative for debugging:
        # x, y, z = -1, -1, 0.00
        if is_valid(x, y, positions, rect):
            positions.append([x, y])
            nodes = create_turtlebot(j, x, y, z)
            for node in nodes:
                ld.add_action(node)
            j += 1

    return ld


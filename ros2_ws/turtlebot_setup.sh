#!/bin/bash

source /opt/ros/jazzy/setup.bash

MODE="real"  # sim or real

DISCOVERY_IP=134.34.225.196 # insert TurtleBot 4 IP address (check the LED screen). no space after equals
ROBOT_ID=5     # insert your robot identifier (check the top plate). no space after equals

if [ "$MODE" == "real" ]; then
    # Check that DISCOVERY_IP and ROBOT_ID are not empty
    if [ -z "$DISCOVERY_IP" ] || [ -z "$ROBOT_ID" ]; then
        echo "[ERROR] DISCOVERY_IP or ROBOT_ID is not set."
        echo "Please edit the script or export the variables manually before running:"
        echo ""
        echo "Example:"
        echo "  export DISCOVERY_IP=192.168.0.10"
        echo "  export ROBOT_ID=5"
        echo "  source ./ros_env.sh real"
        return 1 2>/dev/null || exit 1
    fi

    # Set environment variables
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    [ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
    export ROS_DOMAIN_ID=$ROBOT_ID
    export ROS_DISCOVERY_SERVER="${DISCOVERY_IP}:11811;"

elif [ "$MODE" == "sim" ]; then
    # Clear all environment variables
    unset RMW_IMPLEMENTATION
    unset ROS_SUPER_CLIENT
    unset ROS_DOMAIN_ID
    unset ROS_DISCOVERY_SERVER
else
    echo "Usage: source $0 [sim | real]"
    return 1 2>/dev/null || exit 1
fi

ros2 daemon stop
ros2 daemon start

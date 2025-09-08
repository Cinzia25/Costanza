#!/usr/bin/env bash
#set -Eeuo pipefail

REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_PASS="${REMOTE_PASS:-turtlebot4}"
PORT="${PORT:-11811}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-5}"

# Robot e loro ID
TURTLEBOTS=(134.34.225.157 134.34.225.196)
TURTLEBOT_IDS=(turtlebot4_4 turtlebot4_5)

PC_IP="$(hostname -I | awk '{print $1}')"

# PC_DISCOVERY=";${PC_IP}:${PORT};"
PC_DISCOVERY=";127.0.0.1:${PORT};"  
for ip in "${TURTLEBOTS[@]}"; do PC_DISCOVERY+="${ip}:${PORT};"; done
# PC_DISCOVERY+="${PC_IP}:${PORT}"_
export ROS_SUPER_CLIENT=True 
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=${PC_DISCOVERY}

printenv | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_LOCALHOST_ONLY'


# SSH_OPTS=(
#   -o ConnectTimeout=5
#   -o StrictHostKeyChecking=accept-new
#   -o PreferredAuthentications=password
#   -o PubkeyAuthentication=no
#   -o KbdInteractiveAuthentication=no
# )



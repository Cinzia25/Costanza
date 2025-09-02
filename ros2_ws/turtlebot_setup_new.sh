#!/usr/bin/env bash
REMOTE_USER="${REMOTE_USER:-ubuntu}"
PORT="${PORT:-11811}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-5}"

TURTLEBOTS=(134.34.225.196)
PC_IP=$(hostname -I | awk '{print $1}')

# ---- PC ----
PC_DISCOVERY=""
for ip in "${TURTLEBOTS[@]}"; do PC_DISCOVERY+="${ip}:${PORT};"; done
PC_DISCOVERY+="${PC_IP}:${PORT}"

export ROS_DISCOVERY_SERVER="${PC_DISCOVERY}"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"

# ---- ROBOT ----
for TB in "${TURTLEBOTS[@]}"; do
  sshpass -p 'turtlebot4' ssh -o ConnectTimeout=5 "${REMOTE_USER}@${TB}" "
  set -euo pipefail
  
  TB_DISCOVERY=\"${TB}:${PORT};${PC_IP}:${PORT}\"

  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DISCOVERY_SERVER=\"\${TB_DISCOVERY}\"
  export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID_VALUE}\"
  "
done
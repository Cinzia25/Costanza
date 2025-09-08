#!/usr/bin/env bash
#set -Eeuo pipefail

REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_PASS="${REMOTE_PASS:-turtlebot4}"
PORT="${PORT:-11811}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-5}"

# Robot e loro ID (order by increasing server-id)
TURTLEBOTS=(134.34.225.157 134.34.225.196)
TURTLEBOT_IDS=(turtlebot4_4 turtlebot4_5)
TURTLEBOT_SERVER_IDS=(2 3)

PC_IP="$(hostname -I | awk '{print $1}')"

# PC_DISCOVERY=";${PC_IP}:${PORT};"
PC_DISCOVERY=";127.0.0.1:${PORT};"  
for ip in "${TURTLEBOTS[@]}"; do PC_DISCOVERY+="${ip}:${PORT};"; done
export ROS_SUPER_CLIENT=True 
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=${PC_DISCOVERY}

printenv | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_LOCALHOST_ONLY'


SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
)


for idx in "${!TURTLEBOTS[@]}"; do
  host="${TURTLEBOTS[$idx]}"
  tb_id="${TURTLEBOT_IDS[$idx]}"
  tb_discovery=";${PC_IP}:${PORT};"

  counter=2
  while [ "$counter" -ne "${TURTLEBOT_SERVER_IDS[$idx]}" ]; do
    tb_discovery+=";"
    ((counter++))
  done

  tb_discovery+="${host}:${PORT};"



  echo "[LOCAL] → ${host} (tb_id=${tb_id}, PC_IP=${PC_IP})"
  

  sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${host}" /bin/bash -s <<EOF
set -Eeuo pipefail
T_IP="$(hostname -I | awk '{print $1}')"

  echo $T_IP
# Config Fast-DDS / ROS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="${tb_discovery}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"


# Source ROS
set +u
source /opt/ros/jazzy/setup.bash
set -u

ros2 daemon stop
ros2 daemon start
ros2 param set /${tb_id}/_do_not_use/motion_control safety_override full || echo "[WARN] safety_override non impostato"
ros2 param set /${tb_id}/_do_not_use/motion_control reflexes_enabled False || echo "[WARN] reflexes_enabled non impostato"

printenv | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_LOCALHOST_ONLY'

echo "[OK] Parametri inviati a /${tb_id}"
EOF

done

echo "[LOCAL] Fine."

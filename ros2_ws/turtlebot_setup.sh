#!/usr/bin/env bash
#set -Eeuo pipefail

CSV_FILE="robots_sheet.csv"        # <-- adjust path if needed
REMOTE_USER="${REMOTE_USER:-ubuntu}"
REMOTE_PASS="${REMOTE_PASS:-turtlebot4}"
PORT="${PORT:-11811}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-5}"

# Turtlebot IDs as numbers only
TURTLEBOT_IDS=(3 4)

PC_IP="$(hostname -I | awk '{print $1}')"

### ---- Helper: build discovery string ----
# Args:
#   $1 = max index (for PC) OR robot index (for TB)
#   $2 = robot host IP (for TB, empty for PC)
#   $3 = mode: "pc" or "tb"
build_discovery() {
  local max_idx=$1
  local host_ip=$2
  local mode=$3
  local str=""

  for ((slot=1; slot<=max_idx; slot++)); do
    if (( slot == 1 )); then
      str+=";${PC_IP}:${PORT};"
    elif [[ "$mode" == "tb" && $slot -eq $max_idx ]]; then
      str+="${host_ip}:${PORT};"
    elif [[ "$mode" == "pc" ]]; then
      local robot_id="TURTLEBOT4_${slot}"
      local ip
      ip=$(awk -F, -v target="$robot_id" 'NR>1 && $1==target {print $2}' "$CSV_FILE")
      if [[ -n "$ip" ]]; then
        str+="${ip}:${PORT};"
      else
        str+=";"
      fi
    else
      str+=";"
    fi
  done

  echo "$str"
}

### ---- Build ROS_DISCOVERY_SERVER for PC (includes ALL TBs) ----
max_id=${TURTLEBOT_IDS[-1]}   # largest ID in the list
PC_DISCOVERY=$(build_discovery "$max_id" "" "pc")

export ROS_SUPER_CLIENT=True 
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=${PC_DISCOVERY}

echo "[LOCAL] ROS_DISCOVERY_SERVER for PC:"
echo "${ROS_DISCOVERY_SERVER}"

printenv | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_LOCALHOST_ONLY'


SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
)

### ---- Configure Each Turtlebot ----
for idx in "${TURTLEBOT_IDS[@]}"; do
  robot_id="TURTLEBOT4_${idx}"
  tb_id=$(echo "$robot_id" | tr '[:upper:]' '[:lower:]')

  host=$(awk -F, -v target="$robot_id" 'NR>1 && $1==target {print $2}' "$CSV_FILE")

  if [[ -z "$host" ]]; then
    echo "[LOCAL] Skipping ${robot_id}: no IP in CSV"
    continue
  fi

  tb_discovery=$(build_discovery "$idx" "$host" "tb")

  echo "[LOCAL] → ${host} (tb_id=${tb_id}, PC_IP=${PC_IP})"
  echo "[LOCAL] ROS_DISCOVERY_SERVER for ${robot_id}: ${tb_discovery}"

  sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${host}" /bin/bash -s <<EOF
set -Eeuo pipefail
T_IP="\$(hostname -I | awk '{print \$1}')"
echo "[REMOTE] Host IP: \$T_IP"

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

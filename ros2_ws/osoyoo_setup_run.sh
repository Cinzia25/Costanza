#!/usr/bin/env bash
set -euo pipefail

CSV_FILE="robots_sheet.csv"           # adjust path
REMOTE_USER="${REMOTE_USER:-pi}"
REMOTE_PASS="${REMOTE_PASS:-raspberry}"

# OSOYOO robot indices (numbers only)
OSOYOO_IDS=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27)

PORT="${PORT:-11811}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-5}"
RMW_IMPL="${RMW_IMPL:-rmw_fastrtps_cpp}"

RESERVED_MAX=100
BASE_OSOYOO_SLOT=$((RESERVED_MAX + 1))

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
  -n
)

FAILED=()
REACH_FAIL=()

# --- helpers ---
get_ip_from_csv() {
  local id="$1"
  awk -F, -v target="$id" 'NR>1 && $1==target {print $2}' "$CSV_FILE"
}

build_robot_discovery() {
  local pc_ip="$1" r_ip="$2" idx="$3"
  local robot_slot=$((BASE_OSOYOO_SLOT + idx))
  local s=""
  for ((slot=1; slot<=robot_slot; slot++)); do
    if ((slot == 1)); then
      s+=";${pc_ip}:${PORT};"
    elif ((slot == robot_slot)); then
      s+="${r_ip}:${PORT};"
    else
      s+=";"
    fi
  done
  echo "$s"
}

# --- main ---

PC_IP="$(hostname -I | awk '{print $1}')"
[[ -z "${PC_IP:-}" ]] && { echo "[LOCAL] ERROR: could not detect PC_IP"; exit 1; }

echo "[LOCAL] Assuming PC discovery server is already running with:"
echo "  fastdds discovery -i 1   (listening on $PC_IP:$PORT)"
echo

for idx in "${OSOYOO_IDS[@]}"; do
  rid="OSOYOO_${idx}"
  node_name="osoyoo_${idx}_onboard_controller"
  RIP="$(get_ip_from_csv "$rid")"

  if [[ -z "$RIP" ]]; then
    echo "[$rid] No IP in CSV — skipping"
    FAILED+=("$rid")
    continue
  fi

  ROBOT_DISCOVERY="$(build_robot_discovery "$PC_IP" "$RIP" "$idx")"

  {
    echo "[$rid] Checking connectivity at $RIP..."
    if ! ping -c1 -W1 "$RIP" >/dev/null 2>&1; then
      echo "[$rid] Unreachable"
      REACH_FAIL+=("$rid")
      exit 1
    fi

    echo "[$rid] Connecting..."
    sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${RIP}" "bash -lc '
      set -Eeuo pipefail

      # --- export discovery environment ---
      export RMW_IMPLEMENTATION=\"$RMW_IMPL\"
      export ROS_DOMAIN_ID=\"$ROS_DOMAIN_ID_VALUE\"
      export ROS_DISCOVERY_SERVER=\"$ROBOT_DISCOVERY\"

      # --- debug prints ---
      echo \"[REMOTE][$rid] ROS_DISCOVERY_SERVER=\$ROS_DISCOVERY_SERVER\"
      echo \"[REMOTE][$rid] ROS_DOMAIN_ID=\$ROS_DOMAIN_ID\"
      echo \"[REMOTE][$rid] RMW_IMPLEMENTATION=\$RMW_IMPLEMENTATION\"

      # --- ROS setup ---
      set +u
      source /opt/ros/humble/setup.bash
      [ -f /home/pi/ssm_ws/install/setup.sh ] && source /home/pi/ssm_ws/install/setup.sh
      set -u

      ros2 daemon stop || true
      ros2 daemon start

      if ros2 node list | grep -qx \"/$node_name\"; then
        echo \"[REMOTE][$rid] Node /$node_name already running\"
      else
        echo \"[REMOTE][$rid] Starting /$node_name ...\"
        nohup ros2 run robot_controller controller \\
          --ros-args -r __node:=$node_name \\
          > /home/pi/controller.out 2>&1 < /dev/null &
        sleep 3
        if pgrep -f \"ros2 run robot_controller controller\" >/dev/null 2>&1; then
          echo \"[REMOTE][$rid] Process for /$node_name is running (last 5 log lines):\"
          tail -n 5 /home/pi/controller.out || true
        else
          echo \"[REMOTE][$rid] ERROR: Controller process not found\" >&2
          tail -n 50 /home/pi/controller.out || true
          exit 1
        fi
      fi
    '"
    echo "[$rid] OK"
  } || {
    echo "[$rid] FAILED"
    FAILED+=("$rid")
  } &
done

wait

echo
echo "[LOCAL] Completed setup."
[[ ${#REACH_FAIL[@]} -gt 0 ]] && {
  echo "[LOCAL] Unreachable robots:"; printf ' - %s\n' "${REACH_FAIL[@]}"
}
[[ ${#FAILED[@]} -gt 0 ]] && {
  echo "[LOCAL] Robots failed to configure:"; printf ' - %s\n' "${FAILED[@]}"
}
[[ ${#REACH_FAIL[@]} -eq 0 && ${#FAILED[@]} -eq 0 ]] && echo "[LOCAL] All robots configured successfully."

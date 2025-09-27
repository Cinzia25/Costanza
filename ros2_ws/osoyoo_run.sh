#!/usr/bin/env bash
set -euo pipefail

CSV_FILE="robots_sheet.csv"       # <-- adjust path if needed
REMOTE_USER="${REMOTE_USER:-pi}"
REMOTE_PASS="${REMOTE_PASS:-raspberry}"
OSOYOOS_IDS=(3 7 9 10)
# OSOYOOS_IDS=(7)
# OSOYOOS_IDS=(11 12 13 14 15 16 17 18 19 20 21 22 23 24 25)

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
  -n
)

# Function: get IP from CSV by robot ID
get_ip_from_csv() {
  local id="$1"
  awk -F, -v target="$id" 'NR>1 && $1==target {print $2}' "$CSV_FILE"
}

FAILED_ROBOTS=()

for idx in "${OSOYOOS_IDS[@]}"; do
  robot_id="OSOYOO_${idx}"
  node_name="osoyoo_${idx}_onboard_controller"
  host=$(get_ip_from_csv "$robot_id")

  if [[ -z "$host" ]]; then
    echo "[$robot_id] No IP found in $CSV_FILE"
    FAILED_ROBOTS+=("$robot_id")
    continue
  fi

  {
    echo "[$robot_id] Connecting to $host ..."
    sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${host}" "bash -lc '
      set -Eeuo pipefail
      export LANG=C.UTF-8 LC_ALL=C.UTF-8

      echo \"===== \$(hostname) @ \$(date -Is) =====\"

      cd /home/pi || exit 1
      source ./setup-ros2-discovery.sh || true

      set +u
      source /opt/ros/humble/setup.sh
      set -u

      if [ -d /home/pi/ssm_ws ]; then
        cd /home/pi/ssm_ws || exit 1
        set +u
        [ -f install/setup.sh ] && source install/setup.sh || true
        set -u
      fi

      # Check if node is already running
      if ros2 node list | grep -qx \"/$node_name\"; then
        echo \"[REMOTE][$robot_id] Node /$node_name already running\"
      else
        echo \"[REMOTE][$robot_id] Starting /$node_name ...\"
        nohup ros2 run robot_controller controller \\
          --ros-args -r __node:=$node_name \\
          > /home/pi/controller.out 2>&1 < /dev/null &
        echo \$! > /home/pi/controller.pid
        sleep 2

        if ros2 node list | grep -qx \"/$node_name\"; then
          echo \"[REMOTE][$robot_id] Node /$node_name started successfully\"
        else
          echo \"[REMOTE][$robot_id] ERROR: Failed to start /$node_name\" >&2
          exit 1
        fi
      fi
    '"
  } || {
    echo "[$robot_id] FAILED"
    FAILED_ROBOTS+=("$robot_id")
  } &
done

wait   # wait for all parallel jobs to finish

echo
echo "[LOCAL] Finished processing all Osoyoo robots."
if [[ ${#FAILED_ROBOTS[@]} -gt 0 ]]; then
  echo "[LOCAL] The following robots FAILED:"
  printf ' - %s\n' "${FAILED_ROBOTS[@]}"
else
  echo "[LOCAL] All robots succeeded."
fi

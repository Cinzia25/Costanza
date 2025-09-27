#!/usr/bin/env bash
set -euo pipefail

CSV_FILE="robots_sheet.csv"   # <-- adjust path to your CSV
REMOTE_USER="${REMOTE_USER:-pi}"
REMOTE_PASS="${REMOTE_PASS:-raspberry}"
OSOYOO_IDS=(3 7 9 10)
# OSOYOO_IDS=(7)

# OSOYOO_IDS=(11 12 13 14 15 16 17 18 19 20 21 22 23 24 25)

PC_IP=$(hostname -I | awk '{print $1}')

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
)

# Function: get IP from CSV for a given ID
get_ip_from_csv() {
  local id="$1"
  awk -F, -v target="$id" 'NR>1 && $1==target {print $2}' "$CSV_FILE"
}

# Track failed robots
FAILED_ROBOTS=()

for idx in "${OSOYOO_IDS[@]}"; do
  robot_id="OSOYOO_${idx}"
  ip=$(get_ip_from_csv "$robot_id")

  if [[ -z "$ip" ]]; then
    echo "[$robot_id] No IP found in $CSV_FILE"
    FAILED_ROBOTS+=("$robot_id")
    continue
  fi

  {
    echo "[$robot_id] Connecting to $ip ..."
    sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${ip}" "
      set -euo pipefail
      cd /home/pi

      # Update <address> tag in super_client.xml
      sed -i.bak -E 's#(<address>)[^<]+(</address>)#\1${PC_IP}\2#' super_client.xml || true

      # Source setup script
      source ./setup-ros2-discovery.sh

      # Print relevant ROS environment
      printenv | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY'
    "
  } || {
    echo "[$robot_id] FAILED"
    FAILED_ROBOTS+=("$robot_id")
  } &
done

wait   # Wait for all parallel SSH jobs

echo
echo "[LOCAL] All Osoyoo robots processed."
if [[ ${#FAILED_ROBOTS[@]} -gt 0 ]]; then
  echo "[LOCAL] The following robots FAILED:"
  printf ' - %s\n' "${FAILED_ROBOTS[@]}"
else
  echo "[LOCAL] All robots succeeded."
fi

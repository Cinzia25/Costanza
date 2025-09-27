#!/usr/bin/env bash
set -euo pipefail

CSV_FILE="robots_sheet.csv"           # adjust path
REMOTE_USER="${REMOTE_USER:-pi}"
REMOTE_PASS="${REMOTE_PASS:-raspberry}"

# OSOYOO robot indices (numbers only)
OSOYOO_IDS=(3 7 9 10)

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
  -n
)

# Get IP from CSV for a given robot ID
get_ip_from_csv() {
  local id="$1"
  awk -F, -v target="$id" 'NR>1 && $1==target {print $2}' "$CSV_FILE"
}

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
    echo "[$robot_id] Restarting at $ip ..."
    sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${ip}" "
      echo '$REMOTE_PASS' | sudo -S reboot
    "
    echo "[$robot_id] Reboot command sent (connection will close)."
  } || {
    echo "[$robot_id] FAILED to send reboot"
    FAILED_ROBOTS+=("$robot_id")
  } &
done

wait   # wait for all parallel jobs to finish

echo
echo "[LOCAL] All reboot commands sent."
if [[ ${#FAILED_ROBOTS[@]} -gt 0 ]]; then
  echo "[LOCAL] The following robots FAILED:"
  printf ' - %s\n' "${FAILED_ROBOTS[@]}"
else
  echo "[LOCAL] All robots processed successfully."
fi

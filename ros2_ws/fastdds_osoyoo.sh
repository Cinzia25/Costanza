#!/usr/bin/env bash
set -euo pipefail

CSV_FILE="robots_sheet.csv"
REMOTE_USER="pi"
REMOTE_PASS="raspberry"
OSOYOO_IDS=(2 4 5 6 7 9 10)   # <-- personalizza i robot da avviare
PC_IP="10.111.21.112"         # IP del discovery server (PC centrale)
PORT="11811"
ROS_DOMAIN_ID_VALUE="5"

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

for idx in "${OSOYOO_IDS[@]}"; do
  robot_id="OSOYOO_${idx}"
  node_name="osoyoo_${idx}_onboard_controller"
  host=$(get_ip_from_csv "$robot_id")

  if [[ -z "$host" ]]; then
    echo "[$robot_id] Nessun IP trovato in $CSV_FILE"
    FAILED_ROBOTS+=("$robot_id")
    continue
  fi

  {
    echo "[$robot_id] Connessione a $host ..."
    sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${host}" "bash -lc '
      set -Eeuo pipefail
      cd /home/pi || exit 1

      # Crea/aggiorna setup-ros2-osoyoo.sh
      cat > setup-ros2-osoyoo.sh <<EOF
#!/usr/bin/env bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
export ROS_SUPER_CLIENT=1
export ROS_DISCOVERY_SERVER=\";${PC_IP}:${PORT};\"
EOF
      chmod +x setup-ros2-osoyoo.sh

      # Source configurazione
      source ./setup-ros2-osoyoo.sh

      # Source ROS 2
      set +u
      source /opt/ros/humble/setup.sh
      set -u

      # Source workspace, se presente
      if [ -d /home/pi/ssm_ws ]; then
        cd /home/pi/ssm_ws
        set +u
        [ -f install/setup.sh ] && source install/setup.sh || true
        set -u
      fi

      # Controlla se il nodo è già attivo
      if ros2 node list | grep -qx \"/$node_name\"; then
        echo \"[REMOTE][$robot_id] Nodo /$node_name già attivo\"
      else
        echo \"[REMOTE][$robot_id] Avvio /$node_name...\"
        nohup ros2 run robot_controller controller \\
          --ros-args -r __node:=$node_name \\
          > /home/pi/controller.out 2>&1 < /dev/null &
        echo \$! > /home/pi/controller.pid
        sleep 2

        if ros2 node list | grep -qx \"/$node_name\"; then
          echo \"[REMOTE][$robot_id] Nodo /$node_name avviato correttamente\"
        else
          echo \"[REMOTE][$robot_id] ERRORE: Impossibile avviare /$node_name\" >&2
          exit 1
        fi
      fi
    '"
  } || {
    echo "[$robot_id] FAILED"
    FAILED_ROBOTS+=("$robot_id")
  } &
done

wait   # wait for all parallel jobs

echo
echo "[LOCAL] Fine deploy Osoyoo."
if [[ ${#FAILED_ROBOTS[@]} -gt 0 ]]; then
  echo "[LOCAL] I seguenti robot hanno FALLITO:"
  printf ' - %s\n' "${FAILED_ROBOTS[@]}"
else
  echo "[LOCAL] Tutti i robot Osoyoo avviati correttamente."
fi

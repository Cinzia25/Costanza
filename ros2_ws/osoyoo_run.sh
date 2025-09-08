#!/usr/bin/env bash
REMOTE_USER="${REMOTE_USER:-pi}"
REMOTE_PASS="${REMOTE_PASS:-raspberry}"
OSOYOOS=(134.34.231.94 134.34.231.57 134.34.231.174)

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
  -n
)

for host in "${OSOYOOS[@]}"; do
  sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${host}" 'bash -lc "
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

    # Avvia solo se non è già presente il nodo
    if ros2 node list | grep -qx \"/onboard_controller\"; then
      echo \"Node already running\"
    else
      nohup ros2 run robot_controller controller \
        > /home/pi/controller.out 2>&1 < /dev/null &
      echo \$! > /home/pi/controller.pid
      sleep 2
    fi

    # Controllo salute
    if [ -f /home/pi/controller.pid ] && ps -p \$(cat /home/pi/controller.pid) > /dev/null 2>&1; then
      echo \"[REMOTE] controller attivo con PID \$(cat /home/pi/controller.pid)\"
    else
      if ros2 node list | grep -qx \"/onboard_controller\"; then
        echo \"[REMOTE] controller attivo (nodo visibile), PID non disponibile\"
      else
        echo \"[REMOTE] ERRORE: controller non in esecuzione\" >&2
        tail -n 100 /home/pi/controller.out || true
        exit 1
      fi
    fi

    ros2 node list || true
  "'
done

echo "[LOCAL] Fine."

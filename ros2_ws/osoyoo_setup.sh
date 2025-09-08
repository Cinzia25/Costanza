#!/usr/bin/env bash
REMOTE_USER="${REMOTE_USER:-pi}"
OSOYOOS=(134.34.231.94 134.34.231.57 134.34.231.174)
PC_IP=$(hostname -I | awk '{print $1}')

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
)

for osoyoo in "${OSOYOOS[@]}"; do
  sshpass -p 'raspberry' ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${osoyoo}" "
    set -euo pipefail
    cd /home/pi
    # Se il file è scrivibile da pi:
    sed -i.bak -E 's#(<address>)[^<]+(</address>)#\1'"$PC_IP"'\2#' super_client.xml
    # Se vedi 'Permission denied' su sed, usa questa riga al posto della precedente:
    # echo 'raspberry' | sudo -S sed -i.bak -E 's#(<address>)[0-9]{1,3}(\.[0-9]{1,3}){3}(</address>)#\1$PC_IP\3#' super_client.xml

    # Assicurati che sia leggibile da pi (exec non serve per source)
    # chmod 644 setup-ros2-discovery.sh  # solo se necessario
    source ./setup-ros2-discovery.sh

    printenv | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY'
  "
done

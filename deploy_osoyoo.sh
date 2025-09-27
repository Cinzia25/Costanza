#!/bin/bash

CSV_FILE="./ros2_ws/robots_sheet.csv"
SRC_DIR="./ssm_ws"
DEST_PARENT="/home/pi"     # dove verrà copiata la cartella ssm_ws
USER="pi"
PASSWORD="raspberry"

# Controlla che sshpass sia installato
if ! command -v sshpass &> /dev/null; then
    echo "Errore: sshpass non installato. Installa con: sudo apt-get install sshpass"
    exit 1
fi

# Controlla file CSV
if [ ! -f "$CSV_FILE" ]; then
    echo "Errore: file CSV non trovato: $CSV_FILE"
    exit 1
fi

# Controlla cartella sorgente
if [ ! -d "$SRC_DIR" ]; then
    echo "Errore: cartella sorgente non trovata: $SRC_DIR"
    exit 1
fi

echo "Distribuzione e build di $SRC_DIR su tutti i robot OSOYOO..."

# File temporanei per i report
SUCCESS_FILE=/tmp/deploy_success.txt
FAIL_FILE=/tmp/deploy_fail.txt
> "$SUCCESS_FILE"
> "$FAIL_FILE"

# Funzione che copia e builda su un robot
deploy_robot() {
    local ROBOT_ID=$1
    local IP_ADDRESS=$2

    echo ">>> Inizio deploy su $ROBOT_ID ($IP_ADDRESS)..."

    # Rimuove vecchia cartella
    sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no $USER@"$IP_ADDRESS" "rm -rf $DEST_PARENT/ssm_ws" 2>/dev/null

    # Copia la nuova cartella
    if sshpass -p "$PASSWORD" scp -o StrictHostKeyChecking=no -r "$SRC_DIR" "$USER@$IP_ADDRESS:$DEST_PARENT" 2>/dev/null; then
        echo "   ✅ Copia riuscita su $ROBOT_ID"

        # Esegue colcon build sul robot
        if sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no $USER@"$IP_ADDRESS" "cd $DEST_PARENT/ssm_ws && colcon build --symlink-install" 2>/dev/null; then
            echo "   ✅ Build completata su $ROBOT_ID"
            echo "$ROBOT_ID,$IP_ADDRESS" >> "$SUCCESS_FILE"
        else
            echo "   ❌ Build FALLITA su $ROBOT_ID ($IP_ADDRESS)"
            echo "$ROBOT_ID,$IP_ADDRESS (build fail)" >> "$FAIL_FILE"
        fi
    else
        echo "   ❌ Copia FALLITA su $ROBOT_ID ($IP_ADDRESS)"
        echo "$ROBOT_ID,$IP_ADDRESS (copy fail)" >> "$FAIL_FILE"
    fi
}

# Legge il CSV, salta intestazione, filtra OSOYOO
tail -n +2 "$CSV_FILE" | grep "^OSOYOO_" | while IFS=, read -r ROBOT_ID IP_ADDRESS MAC; do
    if [ -n "$IP_ADDRESS" ]; then
        deploy_robot "$ROBOT_ID" "$IP_ADDRESS" &
    fi
done

# Attende la fine dei processi paralleli
wait

# Report finale
echo
echo "========= REPORT FINALE ========="
if [ -s "$SUCCESS_FILE" ]; then
    echo "Robot con copia + build riuscita:"
    cat "$SUCCESS_FILE" | sed 's/^/  /'
else
    echo "Nessun robot con copia + build riuscita."
fi

echo
if [ -s "$FAIL_FILE" ]; then
    echo "Robot con errore:"
    cat "$FAIL_FILE" | sed 's/^/  /'
else
    echo "Nessun errore."
fi
echo "================================="

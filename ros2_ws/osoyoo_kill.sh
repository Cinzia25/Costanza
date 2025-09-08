#!/usr/bin/env bash
REMOTE_USER="${REMOTE_USER:-pi}"
REMOTE_PASS="${REMOTE_PASS:-raspberry}"
OSOYOOS=(134.34.231.94)

SSH_OPTS=(
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=accept-new
  -o PreferredAuthentications=password
  -o PubkeyAuthentication=no
  -o KbdInteractiveAuthentication=no
)

for host in "${OSOYOOS[@]}"; do
  sshpass -p "$REMOTE_PASS" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${host}" /bin/bash -s <<'EOF'
    set -euo pipefail
    PIDFILE="$HOME/run/controller.pid"
    if [ ! -f "$PIDFILE" ]; then
      echo "Pidfile non trovato: $PIDFILE"; exit 0
    fi
    PID=$(<"$PIDFILE")
    if ! kill -0 "$PID" 2>/dev/null; then
      echo "Processo non in esecuzione (PID $PID)"; exit 0
    fi
    kill -2 "$PID" || true    # SIGINT
    sleep 2
    kill -0 "$PID" 2>/dev/null || exit 0
    kill -15 "$PID" || true   # SIGTERM
    sleep 2
    kill -0 "$PID" 2>/dev/null && kill -9 "$PID" || true  # SIGKILL se ancora vivo

 EOF

done


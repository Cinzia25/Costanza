#!/usr/bin/env bash
set -euo pipefail

# Defaults
N=10
RATE=10
ONCE=0

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--num) N="$2"; shift 2;;
    -r|--rate) RATE="$2"; shift 2;;
    --once) ONCE=1; shift;;
    *) echo "Usage: $0 [-n NUM] [-r RATE] [--once]"; exit 1;;
  esac
done

# Message to publish
TWIST_MSG='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}'

if [[ $ONCE -eq 1 ]]; then
  echo "[LOCAL] Sending one Twist command to $N Osoyoo robots..."
  for ((i=1; i<=N; i++)); do
    topic="/osoyoo_${i}/cmd_vel"
    echo "[LOCAL] → Publishing once to $topic"
    ros2 topic pub --once "$topic" geometry_msgs/msg/Twist "$TWIST_MSG"
  done
else
  echo "[LOCAL] Sending Twist commands to $N Osoyoo robots at ${RATE} Hz..."
  echo "[LOCAL] Press Ctrl+C to stop."
  for ((i=1; i<=N; i++)); do
    topic="/osoyoo_${i}/cmd_vel"
    echo "[LOCAL] → Publishing continuously to $topic"
    ros2 topic pub -r "$RATE" "$topic" geometry_msgs/msg/Twist "$TWIST_MSG" &
  done
  wait
fi

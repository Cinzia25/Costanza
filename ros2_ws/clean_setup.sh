#!/bin/bash

echo "Killing ros_gz_bridge nodes..."
pkill -f ros_gz_bridge || echo "No ros_gz_bridge processes found."

echo "Killing robot_state_publisher nodes..."
pkill -f robot_state_publisher || echo "No robot_state_publisher processes found."

echo "Killing create nodes..."
pkill -f create || echo "No create processes found."

pkill -f gz
echo "Done."


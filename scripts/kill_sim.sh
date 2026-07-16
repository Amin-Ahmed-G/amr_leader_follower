#!/bin/bash
# scripts/kill_sim.sh
# Idempotent cleanup script for leader-follower simulation processes

echo "Cleaning up simulation processes..."

# Option to toggle docker0 interface as fallback
if [ "$1" = "--disable-docker0" ]; then
    echo "Attempting to disable docker0 interface as fallback..."
    sudo -n ip link set docker0 down 2>/dev/null || echo "Warning: Could not disable docker0 (requires passwordless sudo)"
elif [ "$1" = "--restore-docker0" ]; then
    echo "Attempting to restore docker0 interface..."
    sudo -n ip link set docker0 up 2>/dev/null || echo "Warning: Could not restore docker0 (requires passwordless sudo)"
fi

# 1. Kill rosbridge / websocket nodes holding port 9090
if command -v lsof >/dev/null 2>&1; then
    PIDS=$(lsof -t -i :9090)
    if [ -n "$PIDS" ]; then
        echo "Killing processes holding port 9090: $PIDS"
        kill -9 $PIDS 2>/dev/null || true
    fi
else
    echo "lsof not found, using fuser..."
    fuser -k -9 9090/tcp 2>/dev/null || true
fi

# 2. Kill specific rosbridge and bridge processes
pkill -9 -f "rosbridge_websocket" || true
pkill -9 -f "rosapi_node" || true
pkill -9 -f "parameter_bridge" || true
pkill -9 -f "ros_gz_bridge" || true

# 3. Kill Gazebo and ruby simulation processes
killall -9 ruby 2>/dev/null || true
killall -9 gz 2>/dev/null || true
pkill -9 -f "gz sim" || true
pkill -9 -f "gzserver" || true
pkill -9 -f "gzclient" || true

# 4. Kill Nav2 server nodes
pkill -9 -f "nav2_" || true
pkill -9 -f "lifecycle_manager" || true
pkill -9 -f "planner_server" || true
pkill -9 -f "controller_server" || true
pkill -9 -f "behavior_server" || true
pkill -9 -f "bt_navigator" || true
pkill -9 -f "waypoint_follower" || true
pkill -9 -f "velocity_smoother" || true
pkill -9 -f "collision_monitor" || true
pkill -9 -f "opennav_docking" || true
pkill -9 -f "smoother_server" || true
pkill -9 -f "route_server" || true
pkill -9 -f "robot_state_publisher" || true

# 5. Kill ros2 launch / package executables for this sim.
# Do NOT match the bare package name "amr_leader_follower" — that also
# matches this script's path (.../amr_leader_follower/scripts/kill_sim.sh)
# and self-kills before cleanup finishes.
pkill -9 -f "ros2 launch amr_leader_follower" || true
pkill -9 -f "lib/amr_leader_follower/" || true
pkill -9 -f "ground_truth_bridge.py" || true

echo "Cleanup completed successfully."

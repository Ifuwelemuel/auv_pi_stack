#!/bin/bash
# Kill every stack process and confirm the graph is empty (D10).
pkill -9 -f "lib/auv_control/" ; pkill -9 -f "lib/auv_drivers/"
pkill -9 -f joy_node ; pkill -9 -f mavros_node ; pkill -9 -f "ros2 bag record"
sleep 2
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
echo "--- remaining nodes (should be empty) ---"
ros2 node list

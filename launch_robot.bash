#!/bin/bash

# Array to store all background process IDs
PIDS=()

cleanup() {
    echo -e "\nCtrl+C detected! Cleaning up all ROS2 and Gazebo processes..."
    # Gracefully terminate everything
    ros2 daemon stop
    pkill -f rviz2
    pkill -f gz
    kill "${PIDS[@]}" 2>/dev/null
    
    sleep 2
    
    kill -9 "${PIDS[@]}" 2>/dev/null
    
    echo "All processes cleaned up."
    pkill -f ros2
    ros2 daemon start
}

trap cleanup SIGINT
ros2 launch mm_behaviors robot_state_publisher.launch.py &
PIDS+=($!) # Capture PID 1
sleep 1
ros2 launch mm_behaviors gazebo_only.launch.py &
PIDS+=($!) # Capture PID 2
sleep 4
ros2 launch mm_behaviors controllers.launch.py &
PIDS+=($!) # Capture PID 3
sleep 5
ros2 launch mm_nav slam_nav2.launch.py &
PIDS+=($!) # Capture PID 3
sleep 10
ros2 launch mm_behaviors bt_node_only.launch.py &
PIDS+=($!) # Capture PID 4
sleep 2
wait "${PIDS[@]}" 


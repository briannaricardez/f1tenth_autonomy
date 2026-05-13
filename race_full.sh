#!/bin/bash
source ~/f1tenth_autonomy/install/setup.bash

echo "=== STEP 1: Starting SLAM + FTG mapping ==="
ros2 launch team_planning slam_ftg_launch.py &
SLAM_PID=$!
sleep 10

echo "=== STEP 2: Recording waypoints ==="
ros2 run team_planning record_waypoints --ros-args \
  -p out_csv:=/home/team2/f1tenth_autonomy/maps/my_track.csv

echo "=== STEP 3: Saving map ==="
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/team2/f1tenth_autonomy/src/team_planning/maps/my_track_map'}"
sleep 2

echo "=== STEP 4: Stopping SLAM + FTG ==="
kill $SLAM_PID
sleep 3

echo "=== STEP 5: Launching full autonomy stack SLAM + MPP + PP + FTG ==="
ros2 launch team_planning localization_mpp_pp_launch.py \
  waypoints_csv:=/home/team2/f1tenth_autonomy/maps/my_track.csv

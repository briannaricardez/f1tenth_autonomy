#!/bin/bash
source ~/f1tenth_autonomy/install/setup.bash
ros2 launch team_planning localization_mpp_pp_launch.py \
  waypoints_csv:=~/f1tenth_autonomy/maps/my_track.csv

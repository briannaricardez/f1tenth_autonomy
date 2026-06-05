# F1TENTH Autonomous Racing Stack

A full autonomous F1TENTH racing stack built in ROS2 Humble, integrating map-free reactive centerline following, curvature-adaptive Pure Pursuit control, Follow-The-Gap emergency avoidance, and continuous controller arbitration.

No SLAM, no pre-recorded waypoints, no map. The car extracts a local track centerline from each LiDAR scan and follows it using Pure Pursuit, with FTG as a safety fallback.

This project supports both:
- Simulation validation in f1tenth_gym_ros
- Physical deployment on the F1TENTH vehicle

---

## Stack Architecture

```
LiDAR Scan
    └─> Centerline Follower     (rolling local path from wall midpoints)
              │
              ▼
        Pure Pursuit             (primary controller, 0.8–4.0 m/s)
              │
              ▼
        Follow-The-Gap           (safety fallback, 0.5–2.5 m/s)
              │
              ▼
        Drive Arbiter            (continuous PP/FTG blend)
              │
              ▼
           /drive
````

---

## TF Tree

map -> ego_racecar/odom (static identity, no SLAM)
```
ego_racecar/odom
    └─> ego_racecar/base_link   (published by vesc_to_odom)
              └─> ego_racecar/laser  (static identity)
```

Pure Pursuit runs with map_frame = base_frame = ego_racecar/base_link so all path following is in the car's local frame. No localization required.

---

## Speed Hierarchy

| Mode | m/s | MPH | Condition |
| --- | --- | --- | --- |
| PP straight | 4.0 | 8.9 | Open corridor |
| PP corner | 0.8 | 1.8 | Tight curvature |
| FTG max | 2.5 | 5.6 | Obstacle avoidance |
| FTG min | 0.5 | 1.1 | Tightest avoidance |

---

## Drive Arbiter Blend Thresholds

| Threshold | Distance | Notes |
| --- | --- | --- |
| Blend start (blend_far) | 1.5m | Alpha begins rising toward FTG |
| Full FTG (blend_near) | 0.8m | Alpha reaches 1.0, pure FTG |
| Smoothing bypass | alpha >= 0.85 | Output slew disabled for instant emergency steer |
| FTG danger lock | 1.0m | FTG locks direction and snaps to max steer |
| Startup lockout | 1.5s | Arbiter holds output for 1.5s on node start |

---

## FTG Obstacle Avoidance Behavior

- On front danger detection, FTG measures average open space on left vs right side of scan
- Direction is locked on first detection frame and held until obstacle clears
- Steering snaps immediately to max angle with no slew rate limiting
- Raw (unsmoothed) LiDAR ranges used for danger detection to preserve narrow obstacle returns
- Gap target uses angular midpoint of gap opening, not deepest point

---

## Servo Calibration

| Parameter | Value |
| --- | --- |
| steering_angle_to_servo_gain | -1.2135 |
| steering_angle_to_servo_offset | 0.5530 |
| max_steering_angle | 0.36 rad (~21 degrees) |

The max_steering_angle is set to 0.36 rad to keep the servo within [0, 1] with this gain/offset combination. Full left servo lands at ~0.990, full right at ~0.116.

---

## Dependencies

- ROS 2 Humble
- Python 3
- colcon build tool
- numpy
- slam_toolbox: sudo apt install ros-humble-slam-toolbox (optional, only needed for the legacy SLAM workflow)

---

## Setup

git clone https://github.com/briannaricardez/f1tenth_autonomy.git
cd f1tenth_autonomy
vcs import src < deps/f1tenth.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

---

## Competition Day Workflow

### Map-free racing (primary, use this)

No setup required. Launch and drive.

Real car:
ros2 launch team_planning centerline_pp_launch.py

Simulation (launch gym bridge first):
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 launch team_planning centerline_pp_sim_launch.py

Key tuning parameters in centerline_pp_launch.py:

| Parameter | Default | Effect |
| --- | --- | --- |
| lookahead_samples | [0.4, 0.8, 1.2, 1.6, 2.0, 2.5, 3.0] | Local horizon length |
| slab_half_width | 0.20 m | Wall detection robustness |
| smoothing_window | 3 | Lateral noise rejection |
| max_speed | 4.0 m/s | PP top speed |
| min_speed | 0.8 m/s | PP corner speed |
| lookahead_distance | 1.0 m | PP lookahead |

---

---

## Nodes

| Node | Package | Description |
| --- | --- | --- |
| centerline_follower | team_planning | Map-free reactive planner, extracts local track centerline from each LiDAR scan |
| pure_pursuit | team_planning | Pure Pursuit with curvature-adaptive speed scaling |
| ftg | team_planning | Follow-The-Gap reactive obstacle avoidance with direction lock and danger snap steer |
| noise_proxy | team_planning | Sensor noise injection for robustness testing |
| drive_arbiter | team_control | Continuous PP/FTG blend with speed-dependent output smoothing and danger bypass |
| keyboard_teleop | team_control | Manual keyboard control |

---

## Hardware

| Component | Detail |
| --- | --- |
| Computer | Jetson Orin Nano (Ubuntu 22.04) |
| Motor controller | VESC 6 MK6 at /dev/ttyACM0 |
| LiDAR | Hokuyo ethernet at 192.168.0.10:10940 |
| WiFi | BrosTrend AXE5400 USB antenna |
| SSH | team2@192.168.1.119 (primary), 192.168.1.120 (fallback) |

---

## License

MIT

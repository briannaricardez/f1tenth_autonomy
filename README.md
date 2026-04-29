# F1TENTH Autonomous Racing Stack

A full autonomous F1TENTH racing stack built in **ROS2 Humble**, integrating **SLAM-based localization, local motion path planning, curvature-adaptive Pure Pursuit control, Follow-The-Gap emergency avoidance, and automatic controller arbitration**.

This project supports both:
- Simulation validation in f1tenth_gym_ros
- Physical deployment on the F1TENTH vehicle

---

## Stack Architecture---

## Dependencies

- ROS 2 Humble
- Python 3
- colcon build tool
- numpy
- slam_toolbox: `sudo apt install ros-humble-slam-toolbox`
- nav2_amcl: `sudo apt install ros-humble-nav2-amcl`
- nav2_map_server: `sudo apt install ros-humble-nav2-map-server`
- nav2_lifecycle_manager: `sudo apt install ros-humble-nav2-lifecycle-manager`

---

## Setup

```bash
git clone https://github.com/briannaricardez/f1tenth_autonomy.git
cd f1tenth_autonomy
vcs import src < deps/f1tenth.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Competition Day Workflow

### Step 1 — Mapping lap (do this fresh at every new venue)

```bash
# Terminal 1 — simulator (skip for real car)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2 — SLAM + FTG mapping
ros2 launch team_planning slam_ftg_launch.py

# Terminal 3 — serialize map immediately after lap completes
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$HOME/f1tenth_autonomy/src/team_planning/maps/my_track_map'}"

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$HOME/f1tenth_autonomy/src/team_planning/maps/my_track_map'}}"
```

### Step 2 — Update map path for your machine

```bash
sed -i "s|/home/team2|$HOME|g" \
    ~/f1tenth_autonomy/src/team_planning/config/slam_toolbox_localization_params.yaml

colcon build --packages-select team_planning
source install/setup.bash
```

### Step 3 — Race

```bash
ros2 launch team_planning localization_mpp_pp_launch.py \
    waypoints_csv:=$HOME/f1tenth_autonomy/src/team_planning/waypoints/my_track.csv
```

---

## Speed Hierarchy

| Mode | m/s | MPH | Condition |
| --- | --- | --- | --- |
| PP straight | 3.5 | 7.8 | Open track |
| PP corner | 1.5 | 3.4 | Tight curvature |
| FTG max | 1.4 | 3.1 | Obstacle avoidance |
| FTG min | 1.0 | 2.2 | Tightest avoidance |

---

## Safety Supervisor Thresholds

| Threshold | Distance | Feet |
| --- | --- | --- |
| FTG trigger | 0.8m | 2.6 ft |
| PP return | 1.0m | 3.3 ft |

---

## Nodes

| Node | Package | Description |
| --- | --- | --- |
| `ftg` | team_planning | Follow-The-Gap reactive obstacle avoidance |
| `pure_pursuit` | team_planning | Pure Pursuit with curvature-adaptive speed scaling |
| `mpp` | team_planning | Model Predictive Planner — local horizon from global waypoints |
| `record_waypoints` | team_planning | Waypoint recorder with auto loop-closure detection |
| `noise_proxy` | team_planning | Sensor noise injection for robustness testing |
| `drive_mux` | team_control | Switches between PP and FTG based on /control_mode |
| `safety_supervisor` | team_control | Publishes control mode based on front obstacle distance |
| `keyboard_teleop` | team_control | Manual keyboard control |

---

## TF Tree---

## License

MIT

# F1Tenth Autonomy

ROS 2 workspace for autonomous racing on the F1Tenth platform. Includes SLAM (via slam_toolbox), reactive obstacle avoidance (Follow-The-Gap), path tracking (Pure Pursuit), and manual control (keyboard teleop).

## Prerequisites

- ROS 2 (Humble or later)
- Python 3
- [colcon](https://colcon.readthedocs.io/) build tool
- [vcs](https://github.com/dirk-thomas/vcstool) for dependency management
- numpy
- slam_toolbox (`sudo apt install ros-humble-slam-toolbox`)

## Setup

```bash
# Clone the repository
git clone <repo-url> f1tenth_autonomy
cd f1tenth_autonomy

# Import simulator dependencies
vcs import src < deps/f1tenth.repos

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

## Running

### SLAM + Follow-The-Gap (recommended)

Build a map while driving autonomously with FTG:

```bash
ros2 launch team_planning slam_ftg_launch.py
```

### SLAM only (map building with teleop)

Drive manually to build a map before racing:

```bash
# Terminal 1: simulator
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: SLAM
ros2 launch team_planning slam_only_launch.py

# Terminal 3: drive manually
ros2 run team_control keyboard_teleop
```

Visualize in RViz2 by adding `/map` and TF displays.

### SLAM + Pure Pursuit

Record waypoints in the map frame, then race with SLAM-corrected tracking:

```bash
# Step 1: Record waypoints with SLAM running
ros2 run team_planning record_waypoints --ros-args \
  -p use_slam_pose:=true -p out_csv:=my_track.csv

# Step 2: Race with SLAM + Pure Pursuit
ros2 launch team_planning slam_pp_launch.py waypoints_csv:=my_track.csv
```

### Localization mode (reuse a saved map)

After building a map, set `map_file_name` in `config/slam_toolbox_localization_params.yaml`, then:

```bash
ros2 launch team_planning localization_pp_launch.py waypoints_csv:=my_track.csv
```

### Follow-The-Gap (without SLAM)

```bash
ros2 launch team_planning ftg_launch.py
```

### Pure Pursuit (without SLAM)

```bash
ros2 run team_planning pure_pursuit --ros-args -p waypoints_csv:=/path/to/waypoints.csv
```

### Record Waypoints

```bash
# With SLAM (map frame, recommended):
ros2 run team_planning record_waypoints --ros-args \
  -p use_slam_pose:=true -p out_csv:=my_track.csv

# Without SLAM (odometry frame):
ros2 run team_planning record_waypoints --ros-args -p out_csv:=my_track.csv
```

### Keyboard Teleop

```bash
ros2 run team_control keyboard_teleop
```

Controls: `W/S` speed, `A/D` steer, `Space` stop, `R` reset, `Q` quit.

## Architecture

```
                        slam_toolbox
                     (map→odom TF + /map)
                            ↑
/scan ────────┬───→ [FTG Node] ──→ /drive_ftg ──→ [Drive Mux] ──→ /drive
              │
/odom + /tf ──┤
              └───→ [Pure Pursuit] ──→ /drive_pp
                    (TF2: map→base_link)

Keyboard ──→ [Teleop Node] ──→ /drive
```

**TF tree:** `map → ego_racecar/odom → ego_racecar/base_link → ego_racecar/laser`

## Packages

| Package | Description |
|---------|-------------|
| `team_planning` | Autonomous navigation: SLAM config, FTG, Pure Pursuit, waypoint recorder |
| `team_control` | Vehicle control: keyboard teleop, drive multiplexer |
| `f1tenth_gym` | Simulation environment (external, via vcs) |
| `f1tenth_gym_ros` | ROS 2 bridge for the simulator (external, via vcs) |

## License

MIT

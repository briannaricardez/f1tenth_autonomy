# F1Tenth Autonomy

ROS 2 workspace for autonomous racing on the F1Tenth platform. The stack uses SLAM for localization, a safety supervisor to switch between reactive obstacle avoidance (Follow-The-Gap) and path tracking (Pure Pursuit), and a Model Predictive Planner for local path generation.

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

## How It Works

The system operates in two phases:

1. **Mapping** -- On the first lap, slam_toolbox builds an occupancy grid map from lidar scans (saved as `maps/my_track_map.pgm`). Waypoints are recorded during this lap.
2. **Localization + Racing** -- On subsequent runs, slam_toolbox matches live scans against the saved map to continuously correct the car's pose. Pure Pursuit uses this corrected pose to follow the recorded waypoints.

A safety supervisor monitors the front lidar cone and automatically switches from Pure Pursuit to Follow-The-Gap when an obstacle is too close, then switches back once the path is clear.

## Running

### Step 1: Build a map (first time only)

Drive manually or with FTG to build and save a map:

```bash
# Terminal 1: simulator
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: SLAM in mapping mode
ros2 launch team_planning slam_only_launch.py

# Terminal 3: drive manually
ros2 run team_control keyboard_teleop
```

Visualize in RViz2 by adding `/map` and TF displays.

### Step 2: Record waypoints

```bash
ros2 run team_planning record_waypoints --ros-args -p out_csv:=/path/to/my_track.csv
```

The recorder auto-stops when it detects loop closure (car returns near its starting point after sufficient distance).

### Step 3: Race with localization

Set the `map_file_name` in `config/slam_toolbox_localization_params.yaml` to your saved map, then:

```bash
ros2 launch team_planning localization_pp_launch.py waypoints_csv:=/path/to/my_track.csv
```

### SLAM + FTG (mapping while driving autonomously)

```bash
ros2 launch team_planning slam_ftg_launch.py
```

### SLAM + Pure Pursuit (mapping while tracking waypoints)

```bash
ros2 launch team_planning slam_pp_launch.py waypoints_csv:=/path/to/my_track.csv
```

### Follow-The-Gap only (no SLAM)

```bash
ros2 launch team_planning ftg_launch.py
```

### Keyboard Teleop

```bash
ros2 run team_control keyboard_teleop
```

Controls: `W/S` speed, `A/D` steer, `Space` stop, `R` reset, `Q` quit.

## Architecture

```
                       slam_toolbox
                    (map->odom TF + /map)
                           |
/scan ---+--> [FTG Node] ---------> /drive_ftg --+
         |                                       |
         +--> [Safety Supervisor] -> /control_mode --> [Drive Mux] --> /drive
         |                          (pp / ftg)   |
         |   [MPP Node] -------> /local_path     |
         |        |                              |
         |   [Pure Pursuit] ---> /drive_pp ------+
         |   (TF2: map->base_link)
         |
/odom ---+
```

**Safety supervisor** watches the front lidar cone (+-20 deg). When an obstacle is closer than 1.0m, it switches the mux to FTG. When the front clears past 1.3m, it switches back to PP.

**MPP (Model Predictive Planner)** slices the global waypoints into a local horizon and publishes it as `nav_msgs/Path`. Pure Pursuit can optionally follow this local path instead of the full global waypoints.

**TF tree:** `map -> ego_racecar/odom -> ego_racecar/base_link -> ego_racecar/laser`

## Nodes

| Node | Package | Description |
|------|---------|-------------|
| `ftg` | team_planning | Follow-The-Gap reactive obstacle avoidance |
| `pure_pursuit` | team_planning | Pure Pursuit path tracker (TF2 pose, global/local path) |
| `mpp` | team_planning | Model Predictive Planner (local horizon from global waypoints) |
| `record_waypoints` | team_planning | Waypoint recorder with auto loop-closure detection |
| `drive_mux` | team_control | Switches between PP and FTG based on `/control_mode` |
| `safety_supervisor` | team_control | Publishes control mode based on front obstacle distance |
| `keyboard_teleop` | team_control | Manual keyboard control |

## Packages

| Package | Description |
|---------|-------------|
| `team_planning` | SLAM config, FTG, Pure Pursuit, MPP, waypoint recorder |
| `team_control` | Keyboard teleop, drive multiplexer, safety supervisor |
| `f1tenth_gym` | Simulation environment (external, via vcs) |
| `f1tenth_gym_ros` | ROS 2 bridge for the simulator (external, via vcs) |

## License

MIT

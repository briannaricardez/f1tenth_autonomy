# F1Tenth Autonomy

ROS 2 workspace for autonomous racing on the F1Tenth platform. Includes reactive obstacle avoidance (Follow-The-Gap), path tracking (Pure Pursuit), and manual control (keyboard teleop).

## Prerequisites

- ROS 2 (Humble or later)
- Python 3
- [colcon](https://colcon.readthedocs.io/) build tool
- [vcs](https://github.com/dirk-thomas/vcstool) for dependency management
- numpy

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

### Follow-The-Gap (reactive obstacle avoidance)

Launch FTG with the drive multiplexer:

```bash
ros2 launch team_planning ftg_launch.py
```

Or run individually:

```bash
ros2 run team_planning ftg
ros2 run team_control drive_mux
```

### Pure Pursuit (waypoint tracking)

```bash
ros2 run team_planning pure_pursuit --ros-args -p waypoints_csv:=/path/to/waypoints.csv
```

### Record Waypoints

Drive a lap manually and record waypoints for Pure Pursuit:

```bash
# In one terminal, start teleop
ros2 run team_control keyboard_teleop

# In another terminal, record waypoints
ros2 run team_planning record_waypoints --ros-args -p out_csv:=my_track.csv
```

Press Ctrl+C on the recorder when done to save.

### Keyboard Teleop

```bash
ros2 run team_control keyboard_teleop
```

Controls: `W/S` speed, `A/D` steer, `Space` stop, `R` reset, `Q` quit.

## Architecture

```
LaserScan (/scan) --> [FTG Node] --> /drive_ftg --> [Drive Mux] --> /drive
Odometry  (/ego_racecar/odom) --> [Pure Pursuit] --> /drive_pp
Keyboard  --> [Teleop Node] --> /drive
```

## Packages

| Package | Description |
|---------|-------------|
| `team_planning` | Autonomous navigation: FTG, Pure Pursuit, waypoint recorder |
| `team_control` | Vehicle control: keyboard teleop, drive multiplexer |
| `f1tenth_gym` | Simulation environment (external, via vcs) |
| `f1tenth_gym_ros` | ROS 2 bridge for the simulator (external, via vcs) |

## License

MIT

# F1TENTH Autonomous Racing Stack
A full autonomous F1TENTH racing stack built in **ROS2 Humble**, integrating **SLAM-based localization, local motion path planning, Pure Pursuit control, Follow-The-Gap emergency avoidance, and automatic controller arbitration**.

This project supports both:
- **simulation validation in f1tenth_gym_ros**
- **future deployment on the physical F1TENTH vehicle**

---

# Full Stack Architecture
```text
SLAM Localization
    ↓
Motion Path Planning (MPP)
    ↓
Pure Pursuit (Primary Controller)
    ↓
Follow-The-Gap (Emergency Controller)
    ↓
Safety Supervisor
    ↓
DriveMux
    ↓
/drive

# Jetson Hexapod: ROS 2 Packages

ROS 2 workspace packages for an 18-DOF hexapod robot running on an NVIDIA Jetson Nano and used as a competition robot at Unibots UK 2025.

## Packages

| Package | Purpose |
|---|---|
| `hexa2` | Robot description: URDF, meshes, configuration and launch files |
| `hexa2_control` | Joint control, launch files and RViz configuration |
| `hexa2_gait` | Gait generation (Python) |
| `hexa2_teleop` | Teleoperation nodes and scripts |
| `unibots` | Competition-specific nodes and launch files |
| `hexapod_controller2.py` | Standalone controller script |

## Build

```bash
mkdir -p ~/ws/src && cp -r hexa2* unibots ~/ws/src/
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

Tested with ROS 2 Humble on Ubuntu. See the companion repository `hexapod_ros2` for the full control stack and architecture notes.

## Author

Baisel Alen Kuruppassery, [LinkedIn](https://www.linkedin.com/in/baisel-alen-kuruppassery-75731a222)

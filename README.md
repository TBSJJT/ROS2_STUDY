# GasRobot ROS 2 Workspace

ROS 2 Humble workspace for the GasRobot mobile gas-sensing platform.

## Package layout

```text
gasrobot_ws/
└── src/
    ├── gasrobot_interfaces   # Project-owned ROS interfaces
    ├── gasrobot_description  # URDF, Xacro, and RViz resources
    ├── gasrobot_base         # STM32 bridge, odometry, and IMU
    ├── gasrobot_gas          # Gas sensing and processing
    ├── gasrobot_gas_mapping  # SLAM configuration and maps
    ├── gasrobot_navigation   # Lidar safety and Nav2 configuration
    ├── gasrobot_bringup      # Top-level launch orchestration
    └── vendor                # Third-party ROS packages
```

The `vendor` directory currently contains the LS lidar driver and its vendor
interfaces. Project packages must not modify or duplicate vendor package names.

## Build

```bash
cd /home/book/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Hardware only:

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=hardware enable_rviz:=false
```

SLAM:

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=slam
```

Navigation with the default installed map:

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=nav
```

Common launch arguments include `serial_port`, `baud`, `enable_lidar`,
`enable_safety`, `enable_rviz`, `use_sim_time`, and `map`.

## Ownership rules

- Hardware communication and chassis state belong in `gasrobot_base`.
- Gas sensing algorithms belong in `gasrobot_gas`.
- SLAM, map assets, and gas-map integration belong in `gasrobot_gas_mapping`.
- Navigation and collision-safety behavior belong in `gasrobot_navigation`.
- Cross-package startup belongs in `gasrobot_bringup`.
- Third-party source stays isolated in `src/vendor`.

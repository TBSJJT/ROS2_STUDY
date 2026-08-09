# gasrobot_gas_mapping

GasRobot 的 SLAM、地图资源以及后续气体浓度空间融合包。

## 内容

- `config/slam_toolbox.yaml`：当前使用的实车 SLAM Toolbox 参数。
- `config/slam_toolbox_legacy.yaml`：迁移时保留的旧版参数，用于对照。
- `maps/`：已保存的 PGM/YAML 地图。
- `launch/mapping.launch.py`：SLAM Toolbox 启动封装。

## 使用

单独启动建图：

```bash
ros2 launch gasrobot_gas_mapping mapping.launch.py
```

整机建图：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=slam
```

本包负责地图和气体空间数据融合；底盘里程计由 `gasrobot_base` 提供，导航
参数与避障行为属于 `gasrobot_navigation`。

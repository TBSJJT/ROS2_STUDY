# gasrobot_gas_mapping

GasRobot 的 SLAM、地图资源、气体数据空间关联和风险事件历史位姿补偿包。

## 内容

- `config/slam_toolbox.yaml`：当前使用的实车 SLAM Toolbox 参数。
- `config/slam_toolbox_legacy.yaml`：迁移时保留的旧版参数，用于对照。
- `maps/`：已保存的 PGM/YAML 地图。
- `launch/mapping.launch.py`：SLAM Toolbox 启动封装。

当前 PicoPC 导航和正常巡检默认使用 `maps/picopc_1.yaml`。该地图分辨率为
0.05 米/栅格，地图范围约为 20.90 米 × 16.65 米；`free_thresh` 固定为 0.196，
确保灰度 205 的未探索区域不会被 Nav2 或巡检路线校验器误判为自由区域。

`maps/gasrobot_map.yaml` 是较早保存的对照地图。两张地图的图像结构相近，但地图
原点和裁剪尺寸不同，因此二者的 `map` 坐标不能直接混用。切换地图后必须重新校验
AMCL 初始位姿和全部巡检航点。

后续风险补偿节点将使用气体读数的采样时间与传感器等效响应延迟，查询
`map -> gas_sensor_link` 的历史 TF，同时保留报警时刻原始位置和补偿位置，
便于开展不同速度下 `E_raw` 与 `E_corr` 的对比实验。

## 使用

单独启动建图：

```bash
ros2 launch gasrobot_gas_mapping mapping.launch.py
```

整机建图：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=slam
```

本包负责地图、气体空间数据融合和风险位置补偿；底盘里程计由
`gasrobot_base` 提供，AMCL 与 Nav2 参数属于 `gasrobot_navigation`。

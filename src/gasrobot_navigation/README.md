# gasrobot_navigation

GasRobot 已知地图定位、路径规划、动态避障和自主巡检使用的 Nav2 配置包。

## 软件包边界

本包是纯配置包，只保存 `nav2_params.yaml` 和项目级 Launch，不实现自定义导航
或感知算法，也不额外实现激光雷达速度过滤节点。

Nav2 已经提供全局规划、局部控制、代价地图、行为树和恢复行为。本课题的研究重点是
移动气体检测中的风险事件历史位姿补偿，因此导航部分优先复用成熟框架。

## 文件

- `config/nav2_params.yaml`：AMCL、规划器、控制器、代价地图和行为树参数。
- `launch/navigation.launch.py`：对 Nav2 官方 Bringup 的项目级封装。

## 运行

```bash
source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash
ros2 launch gasrobot_navigation navigation.launch.py \
  map:=/home/book/ros2_study/gasrobot_ws/src/gasrobot_gas_mapping/maps/gasrobot_map.yaml
```

实车运行时 `use_sim_time` 必须保持 `false`。在 RViz 中设置初始位姿后，可使用 Nav2
单目标点或多目标点能力验证定位与路径跟踪，再由后续巡检任务节点调用相同的 Nav2 Action。

## 与气体巡检的关系

巡检任务节点负责提交路线、监听气体数据和生成风险事件；本包只保证机器人能够按路线
运动。气体读数、历史位姿关联和风险位置补偿接口统一定义在
`gasrobot_interfaces` 中，具体算法归入 `gasrobot_gas` 或 `gasrobot_gas_mapping`。

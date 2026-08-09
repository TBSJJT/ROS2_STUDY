# gasrobot_base

GasRobot 底盘硬件抽象包，负责 ROS 2 与 STM32 麦克纳姆底盘之间的通信。

## 功能

- 订阅速度指令并通过串口发送到底盘控制器。
- 发布里程计 `/odom` 和原始 IMU `/imu/data_raw`。
- 发布 `odom -> base_footprint` TF。
- 提供速度限幅、指令超时停车、串口断线重连和反馈帧校验。

## 节点

```bash
ros2 run gasrobot_base stm32_bridge --ros-args \
  --params-file $(ros2 pkg prefix gasrobot_base)/share/gasrobot_base/config/stm32_bridge.yaml
```

主要参数位于 `config/stm32_bridge.yaml`，包括串口、波特率、话题、坐标系、速度限制和 IMU 换算参数。

## 边界

本包只处理底盘、里程计和 IMU。雷达驱动属于 `vendor`，避障策略属于
`gasrobot_navigation`，跨包启动由 `gasrobot_bringup` 负责。

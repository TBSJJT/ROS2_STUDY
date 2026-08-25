# gasrobot_base

GasRobot 底盘硬件抽象软件包，负责 ROS 2 与 STM32 麦克纳姆底盘之间的通信。

## 职责边界

- 接收速度指令并通过串口发送到底盘控制器。
- 发布里程计 `/odom`、原始 IMU `/imu/data_raw` 和里程计 TF。
- 处理速度限幅、指令超时停车、串口断线重连和反馈帧校验。
- 不包含雷达驱动、导航避障、气体检测或跨软件包启动编排。

## 内部结构

```text
gasrobot_base/
├── models.py            # 跨模块不可变数据模型
├── protocol.py          # 9/23 字节协议、校验和与流式拆帧
├── serial_transport.py  # pyserial 连接和非阻塞读写
├── imu.py               # ICM20602 单位换算、方向与死区处理
├── odometry.py          # 使用 STM32 航向的二维位置积分
├── parameters.py        # 参数声明、类型转换与安全校验
├── node.py              # ROS 发布订阅、定时器、TF 与模块编排
└── stm32_bridge.py      # 稳定的可执行入口与兼容类名
```

`protocol.py`、`imu.py` 和 `odometry.py` 不依赖 ROS，能够使用普通 `pytest`
完成确定性测试。`node.py` 只负责将这些模块接入 ROS 2，不再保存协议实现细节。

## 串口协议

控制帧固定为 9 字节：帧头 `0x7B`、三轴有符号大端 `int16` 速度、校验和及
帧尾 `0x7D`。平移速度单位为 `mm/s`，旋转速度单位为 `mrad/s`。

反馈帧固定为 23 字节。各字段布局如下：

| 字节 | 字段 | 编码与单位 |
| --- | --- | --- |
| 0 | 帧头 | `0x7B` |
| 1～2 | `vx` | 有符号大端 `int16`，`mm/s` |
| 3～4 | `vy` | 有符号大端 `int16`，`mm/s` |
| 5～6 | `wz` | 有符号大端 `int16`，`mrad/s` |
| 7～12 | 加速度 XYZ | 三个有符号大端 `int16`，传感器原始计数 |
| 13～18 | 角速度 XYZ | 三个有符号大端 `int16`，传感器原始计数 |
| 19～20 | `yaw` | 有符号大端 `int16`，单位 `0.01°` |
| 21 | 校验和 | 字节 1～20 的无符号累加低八位 |
| 22 | 帧尾 | `0x7D` |

`yaw` 采用有符号百分之一度编码，`-18000～18000` 对应
`-180.00°～180.00°`，分辨率为 `0.01°`，最大量化误差约为 `0.005°`。
ROS 解码后转换为弧度，不在上位机重新积分角速度。

协议层负责字节序、单位和坐标方向差异，ROS 节点只接触已解码数据。23 字节
反馈与旧的 21/22 字节反馈不兼容，必须同步更新并烧录 STM32 固件。

## 里程计航向策略

STM32 已经使用 IMU 姿态算法积分得到 `Yaw`，因此 `/odom` 的航向直接采用反馈帧
中的绝对值。ROS 端不再对 IMU 或轮速角速度重复积分。这样可以避免同一物理量在
上下位机各积分一次，并确保 OLED、下位机控制和 ROS 里程计使用同一个航向状态。

平面位置 `x/y` 仍需由底盘平移速度积分。积分时使用相邻两帧 STM32 航向的最短
角差中点完成机体系到里程计系的转换，能够正确处理 `+180°/-180°` 回绕。
参数 `use_imu_wz_for_twist` 只决定 `/odom.twist.twist.angular.z` 发布 IMU 角速度
还是轮速解算角速度，不参与航向计算。

需要注意，当前六轴 IMU 没有磁力计，STM32 的 `Yaw` 是相对于上电初始方向的航向，
并非地理北向，长期运行仍可能漂移。SLAM 或定位系统应通过 `map→odom` 外部观测
修正全局位姿。

## 运行

```bash
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash
ros2 run gasrobot_base stm32_bridge --ros-args \
  --params-file /userdata/iceice/gasrobot_ws/install/gasrobot_base/share/gasrobot_base/config/stm32_bridge.yaml
```

主要参数位于 `config/stm32_bridge.yaml`，包括串口、话题、坐标系、速度限幅、
超时策略和 IMU 换算参数。

## 测试

```bash
cd /userdata/iceice/gasrobot_ws
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash
colcon test --packages-select gasrobot_base
colcon test-result --verbose
```

测试覆盖控制帧编码、23 字节反馈帧校验、双字节有符号航向解码、串口数据分片与
坏帧恢复、IMU 单位换算、航向回绕、里程计位置积分和参数校验。涉及实物串口的
联调应在硬件测试环境单独进行。

## 扩展规则

- 协议字段变化只修改 `protocol.py` 并同步新增协议测试。
- 更换 IMU 型号或标定算法只修改 `imu.py`。
- 更换里程计积分策略只修改 `odometry.py`。
- 新增 ROS 话题或 TF 才修改 `node.py`。
- 禁止再次把协议、串口和运动学逻辑写回入口文件。

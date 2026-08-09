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
├── protocol.py          # 9/21 字节协议、校验和与流式拆帧
├── serial_transport.py  # pyserial 连接和非阻塞读写
├── imu.py               # ICM20602 单位换算、方向与死区处理
├── odometry.py          # 与 ROS 无关的二维里程计积分
├── parameters.py        # 参数声明、类型转换与安全校验
├── node.py              # ROS 发布订阅、定时器、TF 与模块编排
└── stm32_bridge.py      # 稳定的可执行入口与兼容类名
```

`protocol.py`、`imu.py` 和 `odometry.py` 不依赖 ROS，能够使用普通 `pytest`
完成确定性测试。`node.py` 只负责将这些模块接入 ROS 2，不再保存协议实现细节。

## 串口协议

控制帧固定为 9 字节：帧头 `0x7B`、三轴有符号大端 `int16` 速度、校验和及
帧尾 `0x7D`。平移速度单位为 `mm/s`，旋转速度单位为 `mrad/s`。

反馈帧固定为 21 字节：三轴底盘速度、三轴加速度计、三轴陀螺仪、校验和与
帧尾。协议层负责字节序、单位和下位机旋转方向差异，ROS 节点只接触已解码数据。

## 运行

```bash
source /opt/ros/humble/setup.bash
source /home/book/ros2_study/gasrobot_ws/install/setup.bash
ros2 run gasrobot_base stm32_bridge --ros-args \
  --params-file /home/book/ros2_study/gasrobot_ws/install/gasrobot_base/share/gasrobot_base/config/stm32_bridge.yaml
```

主要参数位于 `config/stm32_bridge.yaml`，包括串口、话题、坐标系、速度限幅、
超时策略和 IMU 换算参数。

## 测试

```bash
cd /home/book/ros2_study/gasrobot_ws
colcon test --packages-select gasrobot_base
colcon test-result --verbose
```

测试覆盖控制帧编码、反馈帧校验、串口数据分片与坏帧恢复、IMU 单位换算、
Z 轴修正、里程计积分和参数校验。涉及实物串口的联调应在硬件测试环境单独进行。

## 扩展规则

- 协议字段变化只修改 `protocol.py` 并同步新增协议测试。
- 更换 IMU 型号或标定算法只修改 `imu.py`。
- 更换里程计积分策略只修改 `odometry.py`。
- 新增 ROS 话题或 TF 才修改 `node.py`。
- 禁止再次把协议、串口和运动学逻辑写回入口文件。

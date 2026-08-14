# gasrobot_description

GasRobot 的机器人模型与可视化资源包。

## 内容

- `urdf/gasrobot/gas_robot.urdf.xacro`：主 Xacro 模型。
- `urdf/gasrobot/base.urdf.xacro`：底盘模型。
- `urdf/gasrobot/wheel/`：轮组模型。
- `urdf/gasrobot/sensor/`：相机、IMU 和雷达模型。
- `launch/description.launch.py`：启动 `robot_state_publisher` 和可选的
  `joint_state_publisher`。
- `config/show_robot_model.rviz`：模型显示配置。

## 使用

```bash
ros2 launch gasrobot_description description.launch.py
```

检查模型：

```bash
xacro src/gasrobot_description/urdf/gasrobot/gas_robot.urdf.xacro > /tmp/gasrobot.urdf
check_urdf /tmp/gasrobot.urdf
```

传感器或机械结构变更应在本包修改，运行算法和硬件通信不应放入本包。

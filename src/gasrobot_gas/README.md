# gasrobot_gas

GasRobot 的气体传感器驱动、标定、滤波和浓度处理包。

当前旧工程中没有可迁移的气体传感节点，因此本包暂时是可编译的工程骨架，
没有虚构未确定的设备协议或 ROS API。

## 后续代码约定

- 设备通信和原始采样节点放在本包。
- 稳定的项目消息、服务和动作定义放在 `gasrobot_interfaces`。
- 气体浓度与位姿/地图融合放在 `gasrobot_gas_mapping`。
- 启动文件由功能包提供，整机组合入口放在 `gasrobot_bringup`。

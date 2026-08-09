# gasrobot_interfaces

GasRobot 自有 ROS 消息、服务和动作的集中定义包。

当前旧工程没有稳定的自定义接口可迁移，因此本包保持为可编译骨架。第一版
气体传感器协议确定后，再引入 `rosidl_default_generators` 并添加 `msg/`、
`srv/` 或 `action/` 定义。

## 约定

- 只放跨包共享且已经稳定的接口。
- 不复制标准 ROS 消息。
- 不复制 `src/vendor` 中的第三方接口。
- 修改既有接口时应同步版本号并检查所有消费者。

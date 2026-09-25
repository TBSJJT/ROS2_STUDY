# gasrobot_interfaces：保留的自定义接口包

本包保留已有消息、服务和 Action 定义，以维持现有源码的构建兼容性。
当前固定顺序配送使用以下标准 ROS 2 接口，不依赖本包的自定义业务接口。

| 用途 | 标准接口 |
|---|---|
| 单目标导航 | `nav2_msgs/action/NavigateToPose` |
| 开始、暂停、继续、取消 | `std_srvs/srv/Trigger` |
| 配送状态 | `std_msgs/msg/String`，内容为 JSON |
| 工位可视化 | `visualization_msgs/msg/MarkerArray` |
| 里程计 | `nav_msgs/msg/Odometry` |
| 速度指令 | `geometry_msgs/msg/Twist` |

实际话题、服务名称及日志格式见 [配送包说明](../gasrobot_delivery/README.md)。
本次文档整理没有修改已有接口定义或调用方。

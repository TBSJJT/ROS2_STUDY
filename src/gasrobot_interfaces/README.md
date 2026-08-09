# gasrobot_interfaces

GasRobot 跨软件包共享的 ROS 2 消息、服务和动作接口。

## 数据链

```text
气体传感器
  → GasReading / GasSensorArray
  → LocatedGasReading（采样时间与地图位姿关联）
  → RiskEvent（异常检测、原始位置与历史位姿补偿位置）
  → ReportRiskEvent（后端接收确认）
```

## 消息

- `GasReading`：单个传感器带时间戳的浓度、单位、环境量与状态。
- `GasSensorArray`：同一采样周期内的多传感器读数。
- `LocatedGasReading`：气体读数及其在地图中的传感器位姿。
- `RiskEvent`：风险编号、等级、报警时间、原始/补偿位置、补偿参数及现场图像。

## 服务与动作

- `CalibrateGasSensor`：请求零点或已知浓度标定。
- `ReportRiskEvent`：向后端提交风险事件并获取接收确认。
- `ExecuteInspection`：执行一组巡检点，持续反馈进度、气体值和风险计数。

## 使用原则

- `header.stamp` 必须表示实际采样时刻，不能使用发布时刻替代。
- 风险补偿使用 `alarm_time - response_delay` 查询 TF2 历史位姿。
- `raw_pose` 保存报警时刻位置，`corrected_pose` 保存补偿位置，两者都不得覆盖。
- 图像使用压缩消息，后端节点可按带宽策略另存文件或上传对象存储。
- 修改既有字段属于接口变更，必须同步所有生产者、消费者和实验数据解析程序。

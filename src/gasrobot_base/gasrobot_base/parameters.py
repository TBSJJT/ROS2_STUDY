#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘桥接节点的参数声明、读取与校验.

BridgeConfig 的设计

BridgeConfig 是一个 "frozen dataclass" (不可变数据类).
所有配置值在创建后不能修改, 这确保了:
1. 线程安全 (多个线程读取同一个配置不会有竞态条件)
2. 可预测性 (配置在运行时不会悄悄改变)
3. 可测试性 (测试可以创建自定义配置而不影响全局状态)

from_node(classmethod) 是工厂方法:
  输入: ROS 2 Node 对象
  输出: BridgeConfig 实例
  过程: 从 Node 读取参数 -> 类型转换 -> 校验 -> 返回

================================================================================
参数校验

validate() 方法在配置创建后检查所有参数的合法性:
- 正数检查: 波特率、频率、超时、限幅等必须 > 0
- 非负检查: 启动延时、死区等必须 >= 0
- 字符串检查: 设备路径、话题名等不能为空

如果校验失败, 抛出 ValueError 并给出清晰的错误消息.
这确保了节点不会以错误的配置启动.

"""

# dataclasses 模块:
#   dataclass: 数据类装饰器
#   asdict: 把数据类实例转换为普通字典
from dataclasses import asdict, dataclass

# VelocityLimits: 速度限幅数据类 (在 protocol.py 中定义)
from gasrobot_base.protocol import VelocityLimits


@dataclass(frozen=True)
class BridgeConfig:
    """
    STM32 桥接节点的不可变配置.

    使用 @dataclass(frozen=True) 创建不可变对象.
    每个字段的默认值都是合理的安全值.

    """
    # port: 串口设备文件路径
    port: str = "/dev/ttyUSB0"
    # baud: 串口通信波特率 (bits per second)
    baud: int = 115200
    # reconnect_period: 断线重连间隔 (秒)
    reconnect_period: float = 1.0
    # startup_delay: 串口打开后的稳定延时 (秒)
    startup_delay: float = 0.2
    # startup_stop_frames: 启动时发送的停车帧数量
    startup_stop_frames: int = 5
    # 默认 "/cmd_vel" 是 ROS 导航栈的标准话题名
    cmd_vel_topic: str = "/cmd_vel"
    # odom_topic: 发布的里程计话题名
    odom_topic: str = "/odom"
    # imu_topic: 发布的 IMU 数据话题名
    imu_topic: str = "/imu/data_raw"
    # odom_frame: 里程计坐标系的 TF 名称
    odom_frame: str = "odom"
    # base_frame: 机器人本体坐标系的 TF 名称
    # ROS 导航栈期望这个名称固定为 "base_link" 或 "base_footprint"
    base_frame: str = "base_link"
    # imu_frame: IMU 传感器坐标系的 TF 名称
    imu_frame: str = "imu_link"
    # publish_odom_tf: 是否发布 odom -> base_link 的 TF 变换
    # True: 发布 TF (推荐, 导航栈需要这个 TF 来知道机器人位姿)
    # False: 不发布 (如果其他节点负责发布)
    publish_odom_tf: bool = True

    # =========================================================================
    # 发送频率、通信超时和速度安全限幅
    # =========================================================================
    tx_rate: float = 50.0
    cmd_timeout: float = 0.3
    feedback_timeout: float = 0.5
    max_linear_x: float = 0.5
    max_linear_y: float = 0.5
    # 1.2 rad/s ≈ 69 度/秒
    max_angular_z: float = 1.2
    # =========================================================================
    # IMU 量程换算以及 Z 轴角速度修正参数
    # =========================================================================
    # accel_lsb_per_g: 加速度计量程系数 (LSB/g)
    # 4096 对应 ICM20602 的 ±8g 量程配置
    # 值越小 -> 量程越大 -> 精度越低
    accel_lsb_per_g: float = 4096.0

    # gyro_lsb_per_dps: 陀螺仪量程系数 (LSB/dps)
    # 131 对应 ICM20602 的 ±250 dps 量程配置
    gyro_lsb_per_dps: float = 131.0

    # use_imu_wz_for_twist: 是否使用 IMU 的 Z 轴角速度作为 twist.angular.z
    # True: 使用 IMU (通常更精确, 因为有更高的采样率)
    # False: 使用底盘反馈的原始角速度
    use_imu_wz_for_twist: bool = True

    # imu_z_sign: IMU Z 轴的方向修正
    # 1.0: 不反转方向
    # -1.0: 反转方向 (IMU 安装方向与底盘 Z 轴相反时使用)
    imu_z_sign: float = 1.0

    # gyro_z_offset_radps: 陀螺仪 Z 轴零偏补偿 (rad/s)
    # 静止时陀螺仪可能输出的微小非零值
    # 正值表示减去一个正的漂移
    gyro_z_offset_radps: float = 0.0

    # gyro_z_deadband: 陀螺仪 Z 轴死区 (rad/s)
    # 绝对值小于此阈值的信号强制为 0
    # 0.02 rad/s ≈ 1.1 度/秒
    gyro_z_deadband: float = 0.02

    # =========================================================================
    # 运行状态输出和串口逐帧调试开关
    # =========================================================================
    # status_period: 状态日志输出周期 (秒)
    # 1.0 表示每秒打印一次综合状态
    status_period: float = 1.0

    # print_imu_raw: 状态日志中是否打印 IMU 原始计数值
    # True: 打印原始值 (方便标定和调试)
    # False: 不打印 (减少日志量)
    print_imu_raw: bool = True

    # debug_tx: 是否打印每帧发送的速度指令
    # True: 打印 (50Hz 下日志量非常大, 仅调试时开启)
    debug_tx: bool = False

    # debug_rx: 是否打印每帧收到的底盘反馈
    debug_rx: bool = False

    # =====================================================================
    # from_node: 工厂类方法
    # =====================================================================
    @classmethod
    def from_node(cls, node) -> "BridgeConfig":
        """
        在 ROS 节点上声明全部参数, 读取最终值, 并返回校验后的配置.

        这是创建 BridgeConfig 的标准方式.
        它会:
        1. 遍历 BridgeConfig 的所有默认值
        2. 在 ROS 节点上声明每个参数 (declare_parameter)
        3. 从 ROS 节点读取每个参数的最终值
           (可能被 YAML 文件或命令行覆盖)
        4. 做类型转换 (ROS 参数系统返回动态类型)
        5. 创建 BridgeConfig 实例
        6. 调用 validate() 校验所有参数

        参数:
            node: rclpy.node.Node 实例 (STM32BridgeNode)

        返回:
            BridgeConfig: 不可变的、已校验的配置对象

        """
        # --- 步骤 1: 获取默认值字典 ---
        # cls() 创建一个全默认值的 BridgeConfig 实例
        # asdict() 把它转为普通字典
        # 例如: {"port": "/dev/ttyUSB0", "baud": 115200, ...}
        defaults = asdict(cls())

        # --- 步骤 2: 声明参数 ---
        # 遍历每个参数, 用默认值调用 declare_parameter
        # node.declare_parameter(name, value):
        #   - 在 ROS 参数系统中注册这个参数
        #   - 设置默认值 (可被 YAML 文件或命令行覆盖)
        for name, value in defaults.items():
            node.declare_parameter(name, value)

        # --- 步骤 3: 读取最终参数值 ---
        # 字典推导式: 一次性读取所有参数
        # node.get_parameter(name).value 返回的结果类型取决于:
        #   - YAML 文件中的类型
        #   - 命令行参数的类型
        # 注意: rclpy 返回的是动态类型 (可能是 int, float, str, bool 等)
        # 所以下一步需要显式类型转换
        values = {
            name: node.get_parameter(name).value
            for name in defaults
        }

        # --- 步骤 4-5: 类型转换并创建配置 ---
        # rclpy 参数系统返回动态类型, 显式转换确保下游代码安全
        config = cls(
            port=str(values["port"]),
            baud=int(values["baud"]),
            reconnect_period=float(values["reconnect_period"]),
            startup_delay=float(values["startup_delay"]),
            startup_stop_frames=int(values["startup_stop_frames"]),
            cmd_vel_topic=str(values["cmd_vel_topic"]),
            odom_topic=str(values["odom_topic"]),
            imu_topic=str(values["imu_topic"]),
            odom_frame=str(values["odom_frame"]),
            base_frame=str(values["base_frame"]),
            imu_frame=str(values["imu_frame"]),
            publish_odom_tf=bool(values["publish_odom_tf"]),
            tx_rate=float(values["tx_rate"]),
            cmd_timeout=float(values["cmd_timeout"]),
            feedback_timeout=float(values["feedback_timeout"]),
            max_linear_x=float(values["max_linear_x"]),
            max_linear_y=float(values["max_linear_y"]),
            max_angular_z=float(values["max_angular_z"]),
            accel_lsb_per_g=float(values["accel_lsb_per_g"]),
            gyro_lsb_per_dps=float(values["gyro_lsb_per_dps"]),
            use_imu_wz_for_twist=bool(values["use_imu_wz_for_twist"]),
            # imu_z_sign 强制二值化: 正为 1.0, 负为 -1.0
            imu_z_sign=(
                -1.0 if float(values["imu_z_sign"]) < 0.0 else 1.0
            ),
            gyro_z_offset_radps=float(
                values["gyro_z_offset_radps"]
            ),
            gyro_z_deadband=float(values["gyro_z_deadband"]),
            status_period=float(values["status_period"]),
            print_imu_raw=bool(values["print_imu_raw"]),
            debug_tx=bool(values["debug_tx"]),
            debug_rx=bool(values["debug_rx"]),
        )

        # --- 步骤 6: 校验 ---
        config.validate()
        return config

    # =====================================================================
    # velocity_limits: 计算属性
    # =====================================================================
    @property
    def velocity_limits(self) -> VelocityLimits:
        """
        返回协议编码器需要的三轴速度限幅 (VelocityLimits 对象).

        @property 装饰器让此方法像属性一样访问:
            config.velocity_limits  (不需要加括号)

        这只是一个便利方法, 把三个独立的限幅值打包为编码器需要的格式.
        """
        return VelocityLimits(
            linear_x=self.max_linear_x,
            linear_y=self.max_linear_y,
            angular_z=self.max_angular_z,
        )

    # =====================================================================
    # validate: 参数校验
    # =====================================================================
    def validate(self) -> None:
        """
        拒绝会导致节点失效或产生危险行为的参数配置.

        校验分三类:
        1. 正数检查: 值必须 > 0 (如波特率、频率、超时、限幅)
        2. 非负检查: 值必须 >= 0 (如延时、死区)
        3. 非空字符串检查: 值不能为空或全空白 (如设备名、话题名)

        异常:
            ValueError: 如果有任何参数校验失败

        """
        # ---- 正数检查 ----
        # 这些参数如果 <= 0, 节点行为没有物理意义或会产生危险
        positive_values = {
            "baud": self.baud,                           # 波特率为 0 无法通信
            "reconnect_period": self.reconnect_period,    # 重连间隔为 0 会高频重试
            "startup_stop_frames": self.startup_stop_frames,  # 启动停车帧为 0 不安全
            "tx_rate": self.tx_rate,                     # 发送频率为 0 不发送指令
            "cmd_timeout": self.cmd_timeout,              # 超时为 0 立即停车
            "feedback_timeout": self.feedback_timeout,    # 超时为 0 永远显示超时
            "max_linear_x": self.max_linear_x,            # 限幅为 0 底盘不动
            "max_linear_y": self.max_linear_y,
            "max_angular_z": self.max_angular_z,
            "accel_lsb_per_g": self.accel_lsb_per_g,      # 量程为 0 换算除零
            "gyro_lsb_per_dps": self.gyro_lsb_per_dps,    # 量程为 0 换算除零
            "status_period": self.status_period,          # 周期为 0 日志刷屏
        }

        for name, value in positive_values.items():
            if value <= 0:
                raise ValueError(f"{name} 必须大于 0")

        # ---- 非负检查 ----
        # 这些参数可以为 0 (表示不需要), 但不能为负 (没有物理意义)
        if self.startup_delay < 0.0:
            raise ValueError("startup_delay 不能小于 0")
        if self.gyro_z_deadband < 0.0:
            raise ValueError("gyro_z_deadband 不能小于 0")

        # ---- 非空字符串检查 ----
        # 空的话题名或坐标系名会导致通信不可用但不会报错
        required_text = {
            "port": self.port,
            "cmd_vel_topic": self.cmd_vel_topic,
            "odom_topic": self.odom_topic,
            "imu_topic": self.imu_topic,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
        }

        for name, value in required_text.items():
            # .strip() 去除首尾空白字符
            # 如果去除后为空字符串, 说明参数值是 "" 或 "   " 等
            if not value.strip():
                raise ValueError(f"{name} 不能为空")

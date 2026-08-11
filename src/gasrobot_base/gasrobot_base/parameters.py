"""底盘桥接节点的参数声明、读取与校验。"""

from dataclasses import asdict, dataclass

from gasrobot_base.protocol import VelocityLimits


@dataclass(frozen=True)
class BridgeConfig:
    """STM32 桥接节点的不可变配置。"""

    # 串口连接与重连策略。
    port: str = "/dev/ttyUSB0"
    baud: int = 115200
    reconnect_period: float = 1.0
    startup_delay: float = 0.2
    startup_stop_frames: int = 5

    # ROS 话题名称和 TF 坐标系名称。
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    imu_topic: str = "/imu/data_raw"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    imu_frame: str = "imu_link"
    publish_odom_tf: bool = True

    # 发送频率、通信超时和速度安全限幅。
    tx_rate: float = 50.0
    cmd_timeout: float = 0.3
    feedback_timeout: float = 0.5
    max_linear_x: float = 0.5
    max_linear_y: float = 0.5
    max_angular_z: float = 1.2

    # IMU 量程换算以及 Z 轴角速度修正参数。
    accel_lsb_per_g: float = 4096.0
    gyro_lsb_per_dps: float = 131.0
    use_imu_wz_for_twist: bool = True
    imu_z_sign: float = 1.0
    gyro_z_offset_radps: float = 0.0
    gyro_z_deadband: float = 0.02

    # 运行状态输出和串口逐帧调试开关。
    status_period: float = 1.0
    print_imu_raw: bool = True
    debug_tx: bool = False
    debug_rx: bool = False

    @classmethod
    def from_node(cls, node) -> "BridgeConfig":
        """在 ROS 节点上声明全部参数并读取最终值。"""

        defaults = asdict(cls())

        # 以数据类默认值为唯一参数清单，避免声明和读取两处手工维护。
        for name, value in defaults.items():
            node.declare_parameter(name, value)

        values = {
            name: node.get_parameter(name).value
            for name in defaults
        }
        # rclpy 返回动态类型，集中转换后其余模块只使用确定的配置类型。
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
        config.validate()
        return config

    @property
    def velocity_limits(self) -> VelocityLimits:
        """返回协议编码器需要的三轴速度限幅。"""

        return VelocityLimits(
            linear_x=self.max_linear_x,
            linear_y=self.max_linear_y,
            angular_z=self.max_angular_z,
        )

    def validate(self) -> None:
        """拒绝会导致节点失效或产生危险行为的参数。"""

        positive_values = {
            "baud": self.baud,
            "reconnect_period": self.reconnect_period,
            "startup_stop_frames": self.startup_stop_frames,
            "tx_rate": self.tx_rate,
            "cmd_timeout": self.cmd_timeout,
            "feedback_timeout": self.feedback_timeout,
            "max_linear_x": self.max_linear_x,
            "max_linear_y": self.max_linear_y,
            "max_angular_z": self.max_angular_z,
            "accel_lsb_per_g": self.accel_lsb_per_g,
            "gyro_lsb_per_dps": self.gyro_lsb_per_dps,
            "status_period": self.status_period,
        }

        # 频率、超时、量程和限幅必须为正，否则节点行为没有物理意义。
        for name, value in positive_values.items():
            if value <= 0:
                raise ValueError(f"{name} 必须大于 0")

        if self.startup_delay < 0.0:
            raise ValueError("startup_delay 不能小于 0")
        if self.gyro_z_deadband < 0.0:
            raise ValueError("gyro_z_deadband 不能小于 0")

        required_text = {
            "port": self.port,
            "cmd_vel_topic": self.cmd_vel_topic,
            "odom_topic": self.odom_topic,
            "imu_topic": self.imu_topic,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
        }

        # 空设备名、话题名或坐标系名会导致启动成功但通信不可用。
        for name, value in required_text.items():
            if not value.strip():
                raise ValueError(f"{name} 不能为空")
